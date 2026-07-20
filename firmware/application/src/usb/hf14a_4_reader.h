/**
 * @file hf14a_4_reader.h
 * @brief Minimal ISO14443-4 (T=CL) reader session API for the CCID slot.
 *
 * Wraps the existing proven reader logic (scan_auto + RATS for activation,
 * and the tcl_apdu_ I-block/chaining/WTX helper in app_cmd.c) behind a
 * keep-field session so multiple APDUs run against ONE card activation —
 * required for stateful flows (DESFire auth, GlobalPlatform secure channel)
 * that cmd 6004's re-select-per-APDU would break.
 *
 * v1 implementation is a thin adapter; FU-01 folds this into a shared
 * PICC/PCD T=CL module.
 */
#ifndef HF14A_4_READER_H
#define HF14A_4_READER_H

#include <stdint.h>
#include <stdbool.h>

#define HF14A_4_ATS_MAX     32
#define HF14A_4_CMD_MAX     261   /* command APDU ceiling (matches CCID short) */
#define HF14A_4_FSC_DEFAULT 64    /* fallback frame size if ATS omits it      */
#define HF14A_4_PROBE_TIMEOUT_MS 10 /* idle presence probe: ATQA answers in ~1ms */
#define HF14A_4_FIELD_SETTLE_MS  5  /* ISO14443-3 5.1 minimum after field on     */
#define HF14A_4_SCAN_ATTEMPTS    2  /* tier-2 selects per poll (card settling)   */
#define HF14A_4_SCAN_RETRY_MS    8  /* settle before the retry                   */
#define HF14A_4_PRESENCE_TIMEOUT_MS 30 /* R(NAK) presence check; card answers in ~1ms */
#define HF14A_4_TX_FRAME_MAX 48   /* reliable RC522 TX frame cap (== our FSD); */
                                  /* larger frames near the 64B FIFO are flaky */

/** @brief One kept-alive T=CL session against a single activated card. */
typedef struct {
    bool     active;                   /* card activated, field kept on   */
    uint8_t  blk;                      /* current T=CL block number (0/1) */
    uint8_t  sak;                      /* card SAK                        */
    uint16_t fsc;                      /* card frame size (FSC, from ATS) */
    uint8_t  ats[HF14A_4_ATS_MAX];     /* ATS as returned by RATS         */
    uint8_t  ats_len;                  /* length of ats                   */
    uint8_t  uid[10];
    uint8_t  uid_len;
} hf14a_4_session_t;

/**
 * @brief Activate a 14443-4 card: field cycle, select (anticollision), RATS.
 * Leaves the field ON and the session ready for APDU exchange.
 * @return true if a 14443-4 (SAK 0x20) card was activated, false otherwise.
 */
bool hf14a_4_session_open(hf14a_4_session_t *s);

/**
 * @brief Exchange one APDU over the open session (no re-select).
 * Handles reader->card chaining (command APDUs larger than the card frame
 * size are split into chained I-blocks), card-side response chaining, and
 * S(WTX). Command APDU capped at HF14A_4_CMD_MAX.
 * @param apdu      command APDU bytes (no PCB/CRC)
 * @param apdu_len  length of command APDU
 * @param resp      output buffer for the response APDU (no PCB/CRC)
 * @param resp_len  [out] bytes written to @p resp
 * @param resp_max  capacity of @p resp
 * @return true on a valid response, false on transport error / card mute.
 */
bool hf14a_4_session_apdu(hf14a_4_session_t *s,
                          const uint8_t *apdu, uint16_t apdu_len,
                          uint8_t *resp, uint16_t *resp_len, uint16_t resp_max);

/** @brief Deactivate: drop the RF field and mark the session inactive. */
void hf14a_4_session_close(hf14a_4_session_t *s);

/** @brief Result of a presence poll.
 *
 * The reader reports what it SAW; it deliberately owns no hysteresis. How many
 * misses constitute a removal is a policy question and belongs in exactly one
 * place (ccid_slot.c), rather than being duplicated between the idle probe and
 * the in-session R(NAK) check -- the seam between those two counters is where
 * every presence bug in this work clustered.
 */
typedef enum {
    HF14A_PRES_GONE,    /* nothing answered -- caller applies hysteresis */
    HF14A_PRES_ISO4,    /* ISO14443-4 card, usable by CCID               */
    HF14A_PRES_OTHER,   /* a card, but not ISO14443-4 (e.g. MIFARE)      */
    HF14A_PRES_UNSURE,  /* something is there but would not select yet   */
} hf14a_pres_t;

/**
 * @brief Lightweight presence probe for GetSlotStatus polling.
 *
 * Ensures reader mode + field on, does a quick anticollision scan (no RATS),
 * and reports whether an ISO14443-4-capable (SAK 0x20) card is in the field.
 * Leaves the field ON. Must NOT be called while a full session is active
 * (it would re-select and disrupt an in-progress T=CL exchange).
 *
 * @return what was seen; the caller applies removal hysteresis.
 */
hf14a_pres_t hf14a_4_presence(void);

/** @brief Is the activated card still in the field?
 *
 * Sends an ISO14443-4 R(NAK) and looks for any R-block reply. Transparent to
 * the session: does not reset the card and does not advance the block number,
 * so it is safe to interleave with an open session (WUPA is not). Needed
 * because presence cannot be re-scanned while a session is active.
 *
 * @return true if the card answered.
 */
bool hf14a_4_session_present(hf14a_4_session_t *s);

/** @brief Switch the RF field off. The single antenna-off owner above rc522:
 *  a POWER decision only (disable / hold / suspend / cable pull), never a
 *  presence decision -- session teardown uses S(DESELECT) and keeps the field. */
void hf14a_4_field_off(void);

/** @brief Drop the cached presence classification (call whenever the radio is
 *  handed away or the slot is disabled, so a card swapped out meanwhile is not
 *  reported under the previous card's classification). */
void hf14a_4_presence_reset(void);

#endif /* HF14A_4_READER_H */
