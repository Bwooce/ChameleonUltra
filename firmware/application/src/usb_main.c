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
            break;

        case APP_USBD_EVT_DRV_RESUME:
            NRF_LOG_INFO("USB RESUME");
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
    if (app_usbd_core_state_get() != APP_USBD_STATE_Configured) return;
    if (!g_usb_connected) return;
    static uint32_t last_tick = 0;
    uint32_t now = app_timer_cnt_get();
    if (app_timer_cnt_diff_compute(now, last_tick) < APP_TIMER_TICKS(300)) return;
    last_tick = now;

    bool present;
    if (ccid_slot_presence_changed(&present)) {
        app_usbd_ccid_notify_slot_change(&m_app_ccid, present);
        ccid_slot_mark_notified();
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
