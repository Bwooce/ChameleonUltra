/**
 * @file app_usbd_ccid.c
 * @brief Custom app_usbd CCID class implementation (nRF5 SDK v17.1.0).
 *
 * See app_usbd_ccid.h. Bulk-OUT messages are reassembled into one CCID block
 * then handed to ccid_slot_process(); its reply is sent on Bulk-IN. Descriptor
 * layout: Interface(0x0B) + CCID functional descriptor(0x21,54B) + 3 endpoints.
 *
 * Written against the SDK class contract; requires an on-target build to verify.
 */
#include "app_usbd_ccid.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_descriptor.h"
#include "ccid_defs.h"
#include <string.h>

#define NRF_LOG_MODULE_NAME usb_ccid
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* ccid_slot_process()/ccid_slot_card_present() are declared in ccid_defs.h. */

/* ---- instance accessors --------------------------------------------- */
static inline app_usbd_ccid_t const *ccid_get(app_usbd_class_inst_t const *p_inst) {
    return (app_usbd_ccid_t const *)p_inst;
}
static inline app_usbd_ccid_ctx_t *ccid_ctx_get(app_usbd_ccid_t const *p_ccid) {
    return &p_ccid->specific.p_data->ctx;
}

static nrf_drv_usbd_ep_t ep_addr_get(app_usbd_class_inst_t const *p_inst, uint8_t ep_idx) {
    app_usbd_class_iface_conf_t const *p_if =
        app_usbd_class_iface_get(p_inst, APP_USBD_CCID_IFACE_IDX);
    return app_usbd_class_ep_address_get(app_usbd_class_iface_ep_get(p_if, ep_idx));
}
#define BULK_IN_EP(p)   ep_addr_get((p), APP_USBD_CCID_BULK_IN_IDX)
#define BULK_OUT_EP(p)  ep_addr_get((p), APP_USBD_CCID_BULK_OUT_IDX)
#define INTR_IN_EP(p)   ep_addr_get((p), APP_USBD_CCID_INTR_IN_IDX)

/* ---- CCID class functional descriptor (54 bytes, CCID 1.1 sec 5.1) --- */
/* dwFeatures v1 = short-APDU level + auto voltage/freq/baud/param-config/PPS.
 * (FU-04 switches to extended-APDU level 0x00040000 and raises the message
 * length ceiling.) Little-endian dwords. */
static const uint8_t m_ccid_func_desc[CCID_FUNC_DESC_LENGTH] = {
    CCID_FUNC_DESC_LENGTH,          /* bLength = 54                          */
    CCID_DESC_TYPE_FUNCTIONAL,      /* bDescriptorType = 0x21                */
    0x10, 0x01,                     /* bcdCCID = 1.10                        */
    0x00,                           /* bMaxSlotIndex = 0 (1 slot)            */
    0x01,                           /* bVoltageSupport = 5V.                  */
                                    /* Contactless has no Vcc, so the field is */
                                    /* nominally meaningless -- but strict     */
                                    /* libccid (Linux) and usbccid.sys         */
                                    /* (Windows) sanity-check it, and 0x00     */
                                    /* ("supports no voltage") is what they    */
                                    /* reject. macOS accepts either, which is  */
                                    /* exactly why the problem cannot be found */
                                    /* here. 5V is the safe declaration.       */
    0x02, 0x00, 0x00, 0x00,         /* dwProtocols = T=1                     */
    0xFC, 0x0D, 0x00, 0x00,         /* dwDefaultClock = 3580 kHz             */
    0xFC, 0x0D, 0x00, 0x00,         /* dwMaximumClock = 3580 kHz             */
    0x00,                           /* bNumClockSupported = 0                */
    0x80, 0x25, 0x00, 0x00,         /* dwDataRate = 9600 bps                 */
    0x80, 0x25, 0x00, 0x00,         /* dwMaxDataRate = 9600 bps              */
    0x00,                           /* bNumDataRatesSupported = 0            */
    0xFE, 0x00, 0x00, 0x00,         /* dwMaxIFSD = 254                       */
    0x00, 0x00, 0x00, 0x00,         /* dwSynchProtocols = 0                  */
    0x00, 0x00, 0x00, 0x00,         /* dwMechanical = 0                      */
    0xBA, 0x00, 0x02, 0x00,         /* dwFeatures = 0x000200BA               */
    0x0F, 0x01, 0x00, 0x00,         /* dwMaxCCIDMessageLength = 271          */
    0xFF,                           /* bClassGetResponse = echo              */
    0xFF,                           /* bClassEnvelope = echo                 */
    0x00, 0x00,                     /* wLcdLayout = none                     */
    0x00,                           /* bPINSupport = none                    */
    0x01                            /* bMaxCCIDBusySlots = 1                 */
};

/* ---- endpoint I/O ---------------------------------------------------- */

/* Full CCID message length (10-byte header + dwLength), or 0 if the header
 * hasn't been received yet. */
static uint32_t ccid_msg_total(const uint8_t *b, uint16_t len) {
    if (len < CCID_BULK_HEADER_LEN) return 0;
    uint32_t dlen = (uint32_t)b[CCID_OFF_LENGTH] |
                    ((uint32_t)b[CCID_OFF_LENGTH + 1] << 8) |
                    ((uint32_t)b[CCID_OFF_LENGTH + 2] << 16) |
                    ((uint32_t)b[CCID_OFF_LENGTH + 3] << 24);
    return CCID_BULK_HEADER_LEN + dlen;
}

/* Arm a bulk-OUT receive of exactly `want` bytes at the accumulation point.
 * Sizing to the known remainder means the transfer completes on buffer-full,
 * so messages that are an exact multiple of the 64-byte max packet (no short
 * terminator / ZLP) still complete. `want` is clamped to the buffer. */
static bool ccid_rx_consumer(nrf_drv_usbd_ep_transfer_t *p_next, void *p_context,
                             size_t ep_size, size_t data_size) {
    app_usbd_ccid_ctx_t *p_ctx = (app_usbd_ccid_ctx_t *)p_context;
    size_t space = sizeof(p_ctx->rx_buf) - p_ctx->rx_len;
    if (data_size > space) data_size = space;
    /* dwLength from the bytes ALREADY stored (this packet not copied yet). */
    uint32_t total = ccid_msg_total(p_ctx->rx_buf, p_ctx->rx_len);
    p_next->p_data.rx = p_ctx->rx_buf + p_ctx->rx_len;
    p_next->size      = data_size;
    p_ctx->rx_len     = (uint16_t)(p_ctx->rx_len + data_size);
    (void)ep_size;
    if (total == 0) return false;   /* header not readable yet: stop, handler re-checks */
    if (p_ctx->rx_len >= total)                 return false;  /* message complete */
    if (p_ctx->rx_len >= sizeof(p_ctx->rx_buf)) return false;  /* buffer full      */
    return true;   /* header known and more to come */
}

/* Arm a handled bulk-OUT transfer WITHOUT resetting rx_len (used to read the
 * next packet(s) of a message whose header we've already seen). */
static ret_code_t ccid_rx_arm_handled(app_usbd_class_inst_t const *p_inst) {
    app_usbd_ccid_ctx_t *p_ctx = ccid_ctx_get(ccid_get(p_inst));
    nrf_drv_usbd_handler_desc_t const handler_desc = {
        .handler.consumer = ccid_rx_consumer,
        .p_context        = p_ctx,
    };
    return app_usbd_ep_handled_transfer(BULK_OUT_EP(p_inst), &handler_desc);
}

static ret_code_t ccid_rx_start(app_usbd_class_inst_t const *p_inst) {
    ccid_ctx_get(ccid_get(p_inst))->rx_len = 0;
    return ccid_rx_arm_handled(p_inst);
}
static ret_code_t ccid_tx_send(app_usbd_class_inst_t const *p_inst,
                               const uint8_t *buf, size_t len) {
    NRF_DRV_USBD_TRANSFER_IN(xfer, buf, len);
    return app_usbd_ep_transfer(BULK_IN_EP(p_inst), &xfer);
}

/* One complete CCID message has been reassembled in rx_buf (by the consumer);
 * process it and reply on bulk-IN (or re-arm if there is nothing to send). */
static void ccid_on_rx_done(app_usbd_class_inst_t const *p_inst, size_t rx_size) {
    (void)rx_size;
    app_usbd_ccid_ctx_t *p_ctx = ccid_ctx_get(ccid_get(p_inst));

    /* The consumer stops after the first packet (before dwLength is readable) and
     * whenever more of the message remains; continue reading the exact remainder
     * without resetting rx_len until the full CCID message is in. */
    uint32_t total = ccid_msg_total(p_ctx->rx_buf, p_ctx->rx_len);
    if ((total == 0 || p_ctx->rx_len < total) && p_ctx->rx_len < sizeof(p_ctx->rx_buf)) {
        (void)ccid_rx_arm_handled(p_inst);
        return;
    }

    uint16_t out_len = ccid_slot_process(p_ctx->rx_buf, p_ctx->rx_len,
                                         p_ctx->tx_buf, sizeof(p_ctx->tx_buf));
    if (out_len > 0) {
        p_ctx->tx_in_flight = true;
        (void)ccid_tx_send(p_inst, p_ctx->tx_buf, out_len);
    } else {
        (void)ccid_rx_start(p_inst); /* nothing to send, re-arm receive */
    }
}

/* ---- descriptor feeder ---------------------------------------------- */
static bool ccid_feed_descriptors(app_usbd_class_descriptor_ctx_t *p_ctx,
                                  app_usbd_class_inst_t const *p_inst,
                                  uint8_t *p_buff, size_t max_size) {
    static app_usbd_class_iface_conf_t const *p_iface;
    static app_usbd_class_ep_conf_t    const *p_ep;
    static uint8_t i;
    p_iface = app_usbd_class_iface_get(p_inst, APP_USBD_CCID_IFACE_IDX);

    APP_USBD_CLASS_DESCRIPTOR_BEGIN(p_ctx, p_buff, max_size);

    /* Interface descriptor (9 bytes) */
    APP_USBD_CLASS_DESCRIPTOR_WRITE(0x09);
    APP_USBD_CLASS_DESCRIPTOR_WRITE(APP_USBD_DESCRIPTOR_INTERFACE);
    APP_USBD_CLASS_DESCRIPTOR_WRITE(app_usbd_class_iface_number_get(p_iface));
    APP_USBD_CLASS_DESCRIPTOR_WRITE(0x00); /* bAlternateSetting */
    APP_USBD_CLASS_DESCRIPTOR_WRITE(app_usbd_class_iface_ep_count_get(p_iface)); /* =3 */
    APP_USBD_CLASS_DESCRIPTOR_WRITE(CCID_INTERFACE_CLASS);
    APP_USBD_CLASS_DESCRIPTOR_WRITE(CCID_INTERFACE_SUBCLASS);
    APP_USBD_CLASS_DESCRIPTOR_WRITE(CCID_INTERFACE_PROTOCOL);
    APP_USBD_CLASS_DESCRIPTOR_WRITE(0x00); /* iInterface */

    /* CCID functional descriptor (54 bytes) */
    for (i = 0; i < CCID_FUNC_DESC_LENGTH; i++) {
        APP_USBD_CLASS_DESCRIPTOR_WRITE(m_ccid_func_desc[i]);
    }

    /* Endpoint descriptors (2 bulk + 1 interrupt, 7 bytes each) */
    for (i = 0; i < app_usbd_class_iface_ep_count_get(p_iface); i++) {
        p_ep = app_usbd_class_iface_ep_get(p_iface, i);
        APP_USBD_CLASS_DESCRIPTOR_WRITE(0x07);
        APP_USBD_CLASS_DESCRIPTOR_WRITE(APP_USBD_DESCRIPTOR_ENDPOINT);
        APP_USBD_CLASS_DESCRIPTOR_WRITE(app_usbd_class_ep_address_get(p_ep));
        APP_USBD_CLASS_DESCRIPTOR_WRITE((i == APP_USBD_CCID_INTR_IN_IDX)
            ? APP_USBD_DESCRIPTOR_EP_ATTR_TYPE_INTERRUPT
            : APP_USBD_DESCRIPTOR_EP_ATTR_TYPE_BULK);
        APP_USBD_CLASS_DESCRIPTOR_WRITE(LSB_16(NRF_DRV_USBD_EPSIZE));
        APP_USBD_CLASS_DESCRIPTOR_WRITE(MSB_16(NRF_DRV_USBD_EPSIZE));
        APP_USBD_CLASS_DESCRIPTOR_WRITE((i == APP_USBD_CCID_INTR_IN_IDX) ? 0x10 : 0x00); /* bInterval */
    }

    APP_USBD_CLASS_DESCRIPTOR_END();
}

/* ---- class-specific SETUP requests (bmRequestType recipient=interface) - */
#define CCID_REQ_ABORT                  0x01
#define CCID_REQ_GET_CLOCK_FREQUENCIES  0x02
#define CCID_REQ_GET_DATA_RATES         0x03

static ret_code_t ccid_setup_class_in(app_usbd_class_inst_t const *p_inst,
                                      app_usbd_setup_evt_t const *p_setup) {
    switch (p_setup->setup.bRequest) {
    case CCID_REQ_GET_CLOCK_FREQUENCIES: /* fallthrough */
    case CCID_REQ_GET_DATA_RATES:
        /* CCID 1.1 5.1: with bNumClockSupported and bNumDataRatesSupported both
         * zero these requests do not apply, and the reader supports only the
         * descriptor's dwDefaultClock/dwDataRate. Stalling says that. Returning
         * four zero bytes instead advertised a 0 kHz clock and 0 bps rate --
         * a wrong answer where a refusal was wanted. */
        return NRF_ERROR_NOT_SUPPORTED;
    default:
        return NRF_ERROR_NOT_SUPPORTED;
    }
}

static ret_code_t ccid_setup_class_out(app_usbd_class_inst_t const *p_inst,
                                       app_usbd_setup_evt_t const *p_setup) {
    switch (p_setup->setup.bRequest) {
    case CCID_REQ_ABORT: {
        /* CCID CLASS ABORT (control): wValue = (bSeq << 8) | bSlot. This is a
         * single-slot reader, so only slot 0 is valid; reject anything else.
         * We resync both bulk pipes and re-arm RX so the reader recovers. The
         * bSeq match against the paired PC_to_RDR_Abort bulk message is not
         * tracked, as no supported host issues the abort pair in normal use. */
        if (p_setup->setup.wValue.lb != 0) return NRF_ERROR_NOT_SUPPORTED;
        nrf_drv_usbd_ep_abort(BULK_OUT_EP(p_inst));
        nrf_drv_usbd_ep_abort(BULK_IN_EP(p_inst));
        (void)ccid_rx_start(p_inst);
        return NRF_SUCCESS;
    }
    default:
        return NRF_ERROR_NOT_SUPPORTED;
    }
}

static ret_code_t ccid_setup_handler(app_usbd_class_inst_t const *p_inst,
                                     app_usbd_setup_evt_t const *p_setup) {
    if (app_usbd_setup_req_rec(p_setup->setup.bmRequestType) != APP_USBD_SETUP_REQREC_INTERFACE)
        return NRF_ERROR_NOT_SUPPORTED;
    if (app_usbd_setup_req_typ(p_setup->setup.bmRequestType) != APP_USBD_SETUP_REQTYPE_CLASS)
        return NRF_ERROR_NOT_SUPPORTED;
    if (app_usbd_setup_req_dir(p_setup->setup.bmRequestType) == APP_USBD_SETUP_REQDIR_IN)
        return ccid_setup_class_in(p_inst, p_setup);
    return ccid_setup_class_out(p_inst, p_setup);
}

/* ---- endpoint transfer completion ----------------------------------- */
static ret_code_t ccid_endpoint_ev(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_complex_evt_t const *p_event) {
    app_usbd_ccid_t const *p_ccid = ccid_get(p_inst);
    app_usbd_ccid_ctx_t   *p_ctx  = ccid_ctx_get(p_ccid);
    nrf_drv_usbd_ep_t      ep      = p_event->drv_evt.data.eptransfer.ep;
    nrf_drv_usbd_ep_status_t st    = p_event->drv_evt.data.eptransfer.status;

    if (ep == BULK_OUT_EP(p_inst)) {
        if (st == NRF_USBD_EP_OK) {
            ccid_on_rx_done(p_inst, nrf_drv_usbd_epout_size_get(ep));
        } else if (p_ctx->rx_len == 0) {
            /* WAITING/ABORTED while idle (no message in progress): (re)arm the
             * receive so a NAK'd first packet is picked up. Crucially, do NOT
             * re-arm when rx_len > 0: with the app_usbd event queue enabled, a
             * WAITING queued from the tail of the previous message can arrive
             * part-way through the next one and would reset rx_len mid-message. */
            (void)ccid_rx_start(p_inst);
        }
        return NRF_SUCCESS;
    }
    if (ep == BULK_IN_EP(p_inst)) {
        p_ctx->tx_in_flight = false;
        (void)ccid_rx_start(p_inst); /* ready for the next command */
        return NRF_SUCCESS;
    }
    if (ep == INTR_IN_EP(p_inst)) {
        p_ctx->notify_pending = false;   /* slot-change notification delivered */
        return NRF_SUCCESS;
    }
    return NRF_ERROR_NOT_SUPPORTED;
}

/* ---- top-level event handler ---------------------------------------- */
static ret_code_t ccid_event_handler(app_usbd_class_inst_t const *p_inst,
                                     app_usbd_complex_evt_t const *p_event) {
    app_usbd_ccid_t const *p_ccid = ccid_get(p_inst);
    app_usbd_ccid_ctx_t   *p_ctx  = ccid_ctx_get(p_ccid);

    switch (p_event->app_evt.type) {
    case APP_USBD_EVT_INST_APPEND:
        memset(p_ctx, 0, sizeof(*p_ctx));
        return NRF_SUCCESS;
    case APP_USBD_EVT_DRV_RESET:
        p_ctx->tx_in_flight  = false;
        p_ctx->notify_pending = false;  /* in-flight IN transfers are gone */
        p_ctx->rx_len = 0;
        /* The host's slot state is reset too, so forget what we think we told
         * it -- otherwise a card already on the reader is never announced. */
        ccid_slot_invalidate_notify();
        return NRF_SUCCESS;
    case APP_USBD_EVT_DRV_SETUP:
        return ccid_setup_handler(p_inst, (app_usbd_setup_evt_t const *)p_event);
    case APP_USBD_EVT_DRV_EPTRANSFER:
        return ccid_endpoint_ev(p_inst, p_event);
    case APP_USBD_EVT_STARTED:
        /* BRING-UP NOTE (IM-11): arming the first bulk-OUT read here assumes the
         * OUT endpoint is already enabled. If the host never delivers the first
         * PC_to_RDR message, move this to an iface_select method (which fires on
         * SET_CONFIGURATION after endpoints are enabled) per the SDK contract. */
        (void)ccid_rx_start(p_inst); /* arm first bulk-OUT receive */
        return NRF_SUCCESS;
    case APP_USBD_EVT_STOPPED:
    case APP_USBD_EVT_POWER_REMOVED:
        p_ctx->tx_in_flight  = false;
        p_ctx->notify_pending = false;  /* else slot-change notifies stop forever */
        return NRF_SUCCESS;
    default:
        return NRF_ERROR_NOT_SUPPORTED;
    }
}

const app_usbd_class_methods_t app_usbd_ccid_class_methods = {
    .event_handler    = ccid_event_handler,
    .feed_descriptors = ccid_feed_descriptors,
};

/* ---- interrupt-IN slot-change notification -------------------------- */
bool app_usbd_ccid_notify_slot_change(app_usbd_ccid_t const *p_ccid, bool card_present) {
    app_usbd_ccid_ctx_t *p_ctx = ccid_ctx_get(p_ccid);
    app_usbd_class_inst_t const *p_inst = &p_ccid->base;
    /* Report failure rather than swallowing it: the caller must NOT record the
     * state as notified, or a change dropped here is never resent and the host
     * keeps the opposite view of the slot forever. */
    if (p_ctx->notify_pending) return false;   /* previous notification in flight */
    /* RDR_to_PC_NotifySlotChange: bMessageType + bmSlotICCState (slot 0).
     * bit0 = slot 0 present, bit1 = state changed since last notify. */
    static uint8_t note[2];
    note[0] = RDR_TO_PC_NOTIFYSLOTCHANGE;
    note[1] = (uint8_t)((card_present ? 0x01 : 0x00) | 0x02);
    p_ctx->notify_pending = true;
    NRF_DRV_USBD_TRANSFER_IN(xfer, note, sizeof(note));
    if (app_usbd_ep_transfer(INTR_IN_EP(p_inst), &xfer) != NRF_SUCCESS) {
        p_ctx->notify_pending = false;   /* let the caller retry next cycle */
        return false;
    }
    return true;
}
