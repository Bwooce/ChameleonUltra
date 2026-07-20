#include "usb_main.h"
#include "syssleep.h"
#include "dataframe.h"

#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_core.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_string_desc.h"
#include "usb/app_usbd_ccid.h"
#include "usb/ccid_defs.h"
#include "settings.h"
#include "app_timer.h"

#define NRF_LOG_MODULE_NAME usb_cdc
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// USB DEFINES START
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// CCID composite interface (interface 2) — Ultra only (Lite has no HF reader).
// Endpoints from the free pool (CDC uses EPIN1/EPIN2/EPOUT1): bulk-IN EPIN3,
// bulk-OUT EPOUT2, intr-IN EPIN4.
#if defined(PROJECT_CHAMELEON_ULTRA)
#define CCID_INTERFACE          2
APP_USBD_CCID_GLOBAL_DEF(m_app_ccid,
                         CCID_INTERFACE,
                         NRF_DRV_USBD_EPIN3,
                         NRF_DRV_USBD_EPOUT2,
                         NRF_DRV_USBD_EPIN4);

/* Set from the USB event handler, read by ccid_periodic_run(): the SDK leaves
 * app_usbd_core_state_get() == Configured across a suspend, so we track it. */
static volatile bool m_usb_suspended = false;
#endif

// USB DEFINES END

// USB CODE START
volatile bool g_usb_connected = false;
volatile bool g_usb_port_opened = false;
volatile bool g_usb_led_marquee_enable = true;
static uint8_t cdc_data_buffer[NRF_DRV_USBD_EPSIZE];

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event) {

    // app_usbd_cdc_acm_t const *p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
            // Setup first transfer
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, cdc_data_buffer, sizeof(cdc_data_buffer));
            UNUSED_VARIABLE(ret);

            NRF_LOG_INFO("CDC ACM port opened");
            g_usb_port_opened = true;
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            g_usb_port_opened = false;
            g_usb_led_marquee_enable = true;
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
            // Get amount of data transfered to process data
            size_t size = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
            data_frame_receive(cdc_data_buffer, size);

            // Setup next transfer
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, cdc_data_buffer, sizeof(cdc_data_buffer));
            UNUSED_VARIABLE(ret);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
    switch (event) {
        case APP_USBD_EVT_DRV_SUSPEND:
            NRF_LOG_INFO("USB SUSPEND");
#if defined(PROJECT_CHAMELEON_ULTRA)
            /* The SDK does not change app_usbd_core_state_get() on suspend, so
             * the CCID scan gate would stay open: without this we keep driving
             * the RF field (orders of magnitude over the USB suspend current
             * budget) and any presence change queues an interrupt-IN transfer
             * that cannot complete, latching notify_pending.
             *
             * ONLY set the flag here. This handler can run before the RC522 is
             * initialised, and the antenna calls are raw SPI writes with no
             * init check -- touching the radio here hardfaults the device. The
             * field is dropped from the main loop instead, see ccid_periodic_run. */
            m_usb_suspended = true;
            /* Drop the field here rather than on the next ccid_periodic_run():
             * this event is dispatched from app_usbd_event_queue_process(),
             * which the main loop calls AFTER ccid_periodic_run() and just
             * before sleep_system_run(), so the device can enter its sleep path
             * before the gate gets another iteration.
             *
             * Safe despite the rc522 init hazard that has bitten this code
             * repeatedly: the USB event queue is drained from the main loop, not
             * an ISR, and this is a no-op unless a poll actually drove the
             * antenna, with hf14a_4_field_off() additionally guarded on
             * get_device_mode().
             *
             * HONEST STATUS: on macOS this is belt-and-braces, not the thing
             * that actually saves the battery. Measured across a host sleep, the
             * field LED goes out only AFTER the power-off animation -- i.e. it
             * is system_off_enter() pulling READER_POWER low that kills the
             * carrier, not this call. Why this path does not visibly drop it
             * first was not chased, since the field does end up off either way.
             * Retained because a host that suspends the bus WITHOUT the device
             * deep-sleeping would otherwise leave the carrier driven, and that
             * case is untested (Linux/Windows). */
            ccid_slot_radio_shutdown();
#endif
            break;

        case APP_USBD_EVT_DRV_RESUME:
            NRF_LOG_INFO("USB RESUME");
#if defined(PROJECT_CHAMELEON_ULTRA)
            m_usb_suspended = false;
#endif
            break;

        case APP_USBD_EVT_STARTED:
            NRF_LOG_INFO("USB STARTED");
            break;

        case APP_USBD_EVT_STOPPED:
            NRF_LOG_INFO("USB STOPPED");
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            sleep_timer_stop();
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled()) {
                app_usbd_enable();
            }
            g_usb_led_marquee_enable = true;
            break;

        case APP_USBD_EVT_POWER_REMOVED:
            sleep_timer_start(SLEEP_DELAY_MS_USB_POWER_DISCONNECTED);
            NRF_LOG_INFO("USB power removed");
            g_usb_connected = false;
            g_usb_led_marquee_enable = false;
            app_usbd_stop();
            break;

        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            g_usb_connected = true;
            app_usbd_start();
            break;

        default:
            // NRF_LOG_INFO("Other usb event: %d", event);
            break;
    }
}

// USB CODE END

void usb_cdc_init(void) {
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

#if defined(PROJECT_CHAMELEON_ULTRA)
    // Composite: append the CCID smart-card-reader interface alongside CDC.
    ret = app_usbd_class_append(app_usbd_ccid_class_inst_get(&m_app_ccid));
    APP_ERROR_CHECK(ret);

    // CCID reader is off by default (normal Chameleon behavior); the user opts
    // in via DATA_CMD_SET_CCID_ENABLE, persisted in settings. Settings are
    // loaded before usb_cdc_init() (see app_main), so this reflects saved state.
    ccid_slot_set_enabled(settings_get_ccid_enable());
#endif
}

#define CCID_POLL_INTERVAL_MS  150

// Interrupt-driven CCID presence: scan the field on an interval (main-loop
// context, so the blocking RF scan is safe) and push RDR_to_PC_NotifySlotChange
// only when presence changes. Called from the main loop.
void ccid_periodic_run(void) {
#if defined(PROJECT_CHAMELEON_ULTRA)
    /* Gate on ENUMERATION COMPLETE, not merely USB power. g_usb_connected is set
     * from APP_USBD_EVT_POWER_READY, i.e. before app_usbd_start() has finished
     * enumerating. Scanning that early runs blocking RF work (antenna cycling,
     * bsp_delay_ms, scan timeouts) in the main loop, which starves the USB event
     * queue so the device misses the host's SETUP requests and never enumerates.
     * With ccid_enable persisted true that made the device invisible on USB from
     * boot. Only scan once the host has actually configured us. */
    /* ONE predicate, ONE exit. Previously only the suspend path dropped the
     * field, so a cable pull (POWER_REMOVED clears g_usb_connected with no
     * SUSPEND event) or any exit from Configured returned with the RF carrier
     * still energised -- permanently, off the battery -- and any open session
     * still active. */
    bool may_scan = !m_usb_suspended && g_usb_connected &&
                    (app_usbd_core_state_get() == APP_USBD_STATE_Configured);
    if (!may_scan) {
        /* Idempotent, and a no-op if no poll ever drove the antenna -- the slot
         * owns that state, because it owns the RF. */
        ccid_slot_radio_shutdown();
        return;
    }
    static uint32_t last_tick = 0;
    uint32_t now = app_timer_cnt_get();
    /* 150 ms, not 300. This interval is the dominant term in BOTH detection
     * latencies: arrival waits on average half of it, and removal needs two
     * consecutive misses so it costs two full intervals. An idle poll is now
     * only the WUPA probe (~18 ms), so the duty cycle is ~12% -- nowhere near
     * the ~68% that caused the original main-loop starvation, when a single
     * poll ran ~2 s. */
    if (app_timer_cnt_diff_compute(now, last_tick) < APP_TIMER_TICKS(CCID_POLL_INTERVAL_MS)) return;
    last_tick = now;

    bool present;
    if (ccid_slot_presence_changed(&present)) {
        /* Only record it as notified if the notification actually went out --
         * otherwise a dropped change is never resent and the host's view of the
         * slot stays wrong for good. */
        if (app_usbd_ccid_notify_slot_change(&m_app_ccid, present)) {
            ccid_slot_mark_notified();
        }
    }
#endif
}

void usb_cdc_write(const void *p_buf, uint16_t length) {
    ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_buf, length);
    APP_ERROR_CHECK(err_code);
}

// override fputc to printf to cdc serial
/* dont't enable
int fputc(int ch, FILE *f){
    static int ch_static;
    ch_static = ch;

    // must cdc is available
    if (g_usb_port_opened && g_usb_connected) {
        // send and wait done.
        ret_code_t ret;
        do {
            ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &ch_static, 1);
        } while(ret == NRF_ERROR_BUSY);

        // error log
        if (ret != NRF_SUCCESS) {
            NRF_LOG_ERROR("CDC ACM Unavailable, fputc: %c, return code: %d", ch_static, ret);
        }
    }

    return ch;
}
*/

bool is_usb_working(void) {
    return g_usb_port_opened;
}
