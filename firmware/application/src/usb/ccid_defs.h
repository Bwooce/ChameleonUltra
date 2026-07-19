/**
 * @file ccid_defs.h
 * @brief USB CCID (Chip/Smart Card Interface Device) protocol definitions.
 *
 * SDK-independent constants and structures from the USB Device Class
 * Specification for USB CCID devices, revision 1.1. Used by the CCID
 * app_usbd class and the contactless slot relay. No Nordic SDK types here.
 */
#ifndef CCID_DEFS_H
#define CCID_DEFS_H

#include <stdint.h>

/* USB interface class code for CCID (bInterfaceClass). */
#define CCID_INTERFACE_CLASS            0x0B
#define CCID_INTERFACE_SUBCLASS         0x00
#define CCID_INTERFACE_PROTOCOL         0x00  /* 0x00 = bulk, per CCID 1.1 */

/* CCID class-specific functional descriptor. */
#define CCID_DESC_TYPE_FUNCTIONAL       0x21
#define CCID_FUNC_DESC_LENGTH           54    /* fixed size, CCID 1.1 sec 5.1 */
#define CCID_VERSION_BCD                0x0110 /* bcdCCID = 1.10 */

/* dwFeatures bits (CCID 1.1 Table 5.1-1). */
#define CCID_FEAT_AUTO_PARAM_CONFIG     0x00000002UL /* auto config by ATR */
#define CCID_FEAT_AUTO_ACTIVATION       0x00000004UL
#define CCID_FEAT_AUTO_VOLTAGE          0x00000008UL
#define CCID_FEAT_AUTO_FREQ             0x00000010UL
#define CCID_FEAT_AUTO_BAUD             0x00000020UL
#define CCID_FEAT_AUTO_PARAM_NEG        0x00000040UL /* auto PPS made by CCID */
#define CCID_FEAT_AUTO_PPS              0x00000080UL /* auto PPS from params   */
#define CCID_FEAT_LEVEL_TPDU            0x00010000UL /* exchange at TPDU level */
#define CCID_FEAT_LEVEL_SHORT_APDU      0x00020000UL /* short APDU level        */
#define CCID_FEAT_LEVEL_EXT_APDU        0x00040000UL /* short+extended APDU     */

/* dwProtocols bits. */
#define CCID_PROTO_T0                   0x00000001UL
#define CCID_PROTO_T1                   0x00000002UL

/* bulk-OUT message types (host -> reader). */
#define PC_TO_RDR_ICCPOWERON            0x62
#define PC_TO_RDR_ICCPOWEROFF           0x63
#define PC_TO_RDR_GETSLOTSTATUS         0x65
#define PC_TO_RDR_XFRBLOCK              0x6F
#define PC_TO_RDR_GETPARAMETERS         0x6C
#define PC_TO_RDR_RESETPARAMETERS       0x6D
#define PC_TO_RDR_SETPARAMETERS         0x61
#define PC_TO_RDR_ESCAPE                0x6B
#define PC_TO_RDR_ICCCLOCK              0x6E
#define PC_TO_RDR_ABORT                 0x72

/* bulk-IN message types (reader -> host). */
#define RDR_TO_PC_DATABLOCK             0x80
#define RDR_TO_PC_SLOTSTATUS            0x81
#define RDR_TO_PC_PARAMETERS            0x82
#define RDR_TO_PC_ESCAPE                0x83

/* interrupt-IN message type. */
#define RDR_TO_PC_NOTIFYSLOTCHANGE      0x50

/* bStatus field: bmICCStatus (bits 0-1) | bmCommandStatus (bits 6-7). */
#define CCID_ICC_STATUS_ACTIVE          0x00 /* present, activated   */
#define CCID_ICC_STATUS_INACTIVE        0x01 /* present, not active  */
#define CCID_ICC_STATUS_ABSENT          0x02 /* no ICC present       */
#define CCID_CMD_STATUS_OK              0x00
#define CCID_CMD_STATUS_FAILED          0x40
#define CCID_CMD_STATUS_TIME_EXT        0x80

/* bError values (subset; used with CMD_STATUS_FAILED). */
#define CCID_ERROR_NONE                 0x00
#define CCID_ERROR_ICC_MUTE             0xFE /* card did not answer  */
#define CCID_ERROR_XFR_OVERRUN          0xFC
#define CCID_ERROR_HW_ERROR             0xFB
#define CCID_ERROR_CMD_SLOT_BUSY        0xE0
#define CCID_ERROR_CMD_UNSUPPORTED      0x00 /* with slot busy check */

/* Message sizing. Short-APDU v1: FSC-bounded APDUs. */
#define CCID_MAX_APDU_LEN               261  /* short APDU: 5 + 255 + 1 */
#define CCID_BULK_HEADER_LEN            10
#define CCID_MAX_MESSAGE_LEN            (CCID_BULK_HEADER_LEN + CCID_MAX_APDU_LEN)

/**
 * @brief Common 10-byte CCID bulk message header (bulk-OUT and bulk-IN share layout).
 *
 * Wire layout is little-endian and byte-packed; parse/emit via offsets rather
 * than casting a struct over the buffer to stay alignment-safe on Cortex-M.
 */
#define CCID_OFF_MSGTYPE                0  /* bMessageType (1)             */
#define CCID_OFF_LENGTH                 1  /* dwLength (4, LE)             */
#define CCID_OFF_SLOT                   5  /* bSlot (1)                    */
#define CCID_OFF_SEQ                    6  /* bSeq (1)                     */
/* bulk-OUT bytes 7-9 are message-specific parameters;                    */
/* bulk-IN  byte 7 = bStatus, byte 8 = bError, byte 9 = message-specific. */
#define CCID_OFF_IN_STATUS              7
#define CCID_OFF_IN_ERROR              8
#define CCID_OFF_IN_CHAINPARAM         9
#define CCID_OFF_DATA                   10

/**
 * @brief Build the PC/SC Part 3 contactless pseudo-ATR from a card ATS.
 *
 * Extracts the historical bytes from @p ats (skipping TL and TA/TB/TC as
 * indicated by T0) and wraps them as 3B 8n 80 01 <hist> TCK.
 *
 * @param ats      pointer to ATS bytes (as returned by RATS), may be NULL/empty
 * @param ats_len  length of @p ats
 * @param out      output buffer, must hold at least 5 + hist_len bytes (<= 22)
 * @return number of bytes written to @p out
 */
uint8_t ccid_pseudo_atr_from_ats(const uint8_t *ats, uint8_t ats_len, uint8_t *out);

/* ---- CCID slot control API (ccid_slot.c) ---------------------------- */
#include <stdbool.h>

/** @brief Process one CCID bulk-OUT message; write the bulk-IN reply to @p resp.
 *  @return length of the reply, or 0 to send nothing. */
uint16_t ccid_slot_process(const uint8_t *msg, uint16_t msg_len,
                           uint8_t *resp, uint16_t resp_max);

/** @brief Enable/disable the reader (Q4 runtime toggle; disabled = empty slot). */
void ccid_slot_set_enabled(bool en);

/* Radio-hold owners (IM-33). CCID yields while any are held. */
#define CCID_HOLD_CDC     0x01   /* CDC reader/attack command using the radio */
#define CCID_HOLD_BUTTON  0x02   /* human button activity                     */

/** @brief Hold/release the radio for another consumer (ISR-safe). CCID reports
 *  card-removed and stops scanning/relaying while any hold is set. */
void ccid_slot_radio_hold(uint8_t who, bool held);

/** @brief Back-compat: hold/release the CDC radio lock (Q5). */
void ccid_slot_set_cdc_lock(bool locked);

/** @brief True while a card is activated in the CCID slot. */
bool ccid_slot_card_present(void);

/** @brief True if the CCID reader is currently enabled at runtime. */
bool ccid_slot_is_enabled(void);

/** @brief Periodic presence scan (main-loop context). Updates cached presence;
 *  returns true if it changed since the last host notification. @p present gets
 *  the current state. Skips scanning during an active session / CDC lock. */
bool ccid_slot_presence_changed(bool *present);

/** @brief Mark the current presence as delivered to the host (after notify). */
void ccid_slot_mark_notified(void);

#endif /* CCID_DEFS_H */
