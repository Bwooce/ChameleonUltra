/**
 * @file app_usbd_ccid.h
 * @brief Custom app_usbd USB CCID (smart-card reader) class for nRF5 SDK.
 *
 * Composite sibling of the existing CDC-ACM class. One interface (class 0x0B)
 * with three endpoints: Bulk-IN, Bulk-OUT, Interrupt-IN. Bulk messages are
 * serviced by ccid_slot_process() (see ccid_slot.c). Modelled on
 * app_usbd_cdc_acm per the SDK class contract.
 *
 * NOTE: SDK-macro glue below is written against nRF5 SDK v17.1.0 and must be
 * verified by an on-target build (Bruce) — it cannot be compiled on the host.
 */
#ifndef APP_USBD_CCID_H
#define APP_USBD_CCID_H

#include "app_usbd.h"
#include "app_usbd_class_base.h"
#include "nrf_drv_usbd.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CCID class methods (defined in app_usbd_ccid.c). */
extern const app_usbd_class_methods_t app_usbd_ccid_class_methods;

/* Endpoint indices within the single CCID interface — MUST match the order
 * endpoints are listed in APP_USBD_CCID_CONFIG below. */
#define APP_USBD_CCID_IFACE_IDX         0
#define APP_USBD_CCID_BULK_IN_IDX       0
#define APP_USBD_CCID_BULK_OUT_IDX      1
#define APP_USBD_CCID_INTR_IN_IDX       2

/* Interrupt-IN endpoint carries RDR_to_PC_NotifySlotChange: a firmware timer
 * scans the field and pushes presence changes to the host (interrupt-driven).
 * GetSlotStatus still returns the cached state as a fallback for pollers. */

/** @brief Interface/endpoint shape: 1 interface, 2 bulk + 1 interrupt-IN. */
#define APP_USBD_CCID_CONFIG(iface, ep_bulk_in, ep_bulk_out, ep_intr_in) \
    ((iface, ep_bulk_in, ep_bulk_out, ep_intr_in))

/** @brief Const (flash) instance config. */
typedef struct {
    uint8_t iface_number;
} app_usbd_ccid_inst_t;

/** @brief RAM context/state for the class. */
typedef struct {
    uint8_t  seq;                 /* last bSeq seen (echoed)                 */
    bool     tx_in_flight;        /* a bulk-IN transfer is pending            */
    bool     notify_pending;      /* a slot-change interrupt is queued        */
    uint16_t rx_len;              /* bytes accumulated in rx_buf              */
    uint8_t  rx_buf[10 + 261];    /* one CCID bulk-OUT message (short APDU)   */
    uint8_t  tx_buf[10 + 261];    /* one CCID bulk-IN message                 */
} app_usbd_ccid_ctx_t;

/* These SPECIFIC_DEC macros must be defined BEFORE the TYPEDEF that consumes
 * them (the preprocessor expands them at the point of the TYPEDEF call). */
#define APP_USBD_CCID_INSTANCE_SPECIFIC_DEC app_usbd_ccid_inst_t inst;
#define APP_USBD_CCID_DATA_SPECIFIC_DEC     app_usbd_ccid_ctx_t ctx;

/*lint -save -e10 -e26 -e64 -e123 -e505 */
APP_USBD_CLASS_TYPEDEF(app_usbd_ccid,
                       APP_USBD_CCID_CONFIG(0, 0, 0, 0),
                       APP_USBD_CCID_INSTANCE_SPECIFIC_DEC,
                       APP_USBD_CCID_DATA_SPECIFIC_DEC);
/*lint -restore */

/**
 * @brief Define a CCID class instance globally.
 *
 * @param instance_name  symbol name for the instance
 * @param iface          interface number
 * @param ep_bulk_in     bulk-IN endpoint  (e.g. NRF_DRV_USBD_EPIN3)
 * @param ep_bulk_out    bulk-OUT endpoint (e.g. NRF_DRV_USBD_EPOUT2)
 * @param ep_intr_in     interrupt-IN endpoint (e.g. NRF_DRV_USBD_EPIN4)
 */
#define APP_USBD_CCID_GLOBAL_DEF(instance_name, iface, ep_bulk_in, ep_bulk_out, ep_intr_in) \
    APP_USBD_CLASS_INST_GLOBAL_DEF(                                                          \
        instance_name,                                                                      \
        app_usbd_ccid,                                                                      \
        &app_usbd_ccid_class_methods,                                                        \
        APP_USBD_CCID_CONFIG(iface, ep_bulk_in, ep_bulk_out, ep_intr_in),                    \
        (.inst = { .iface_number = iface }))

/** @brief Get the base class instance for app_usbd_class_append(). */
static inline app_usbd_class_inst_t const *
app_usbd_ccid_class_inst_get(app_usbd_ccid_t const *p_ccid) {
    return &p_ccid->base;
}

/** @brief Push RDR_to_PC_NotifySlotChange on the interrupt-IN endpoint. */
void app_usbd_ccid_notify_slot_change(app_usbd_ccid_t const *p_ccid, bool card_present);

#ifdef __cplusplus
}
#endif

#endif /* APP_USBD_CCID_H */
