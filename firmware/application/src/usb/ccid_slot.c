/**
 * @file ccid_slot.c
 * @brief CCID slot logic: contactless card presence, pseudo-ATR, APDU relay.
 *
 * SDK-independent (no app_usbd types). The app_usbd_ccid class calls into
 * this layer to service bulk messages; this layer drives the HF reader via
 * the ISO14443-4 session API (see hf14a_4_reader.h). Short-APDU v1: one
 * XfrBlock == one I-block exchange over a kept-alive T=CL session.
 */
#include "ccid_defs.h"
#include "app_timer.h"
#include "hf14a_4_reader.h"
#include <string.h>

/* Runtime enable flag (Q4): CCID interface is always enumerated, but the slot
 * only activates cards / relays APDUs when enabled in settings. When disabled
 * it behaves as an empty reader. Wired to settings.c in IM-10. */
static bool m_ccid_enabled = false;
/* Radio arbitration (Q5 + IM-33): CCID yields the radio to other consumers.
 * Bitmask of who currently owns it — today only the CDC reader/attack commands
 * (buttons pre-empt CCID by disabling it outright, not via a hold). Non-zero =>
 * CCID reports card-removed and won't scan/relay. Set and read only from
 * main-loop context, so no synchronisation is needed. */
static uint8_t m_radio_holders = 0;

static hf14a_4_session_t m_session;
/* Last APDU exchange, so the R(NAK) presence check never lands in the middle of
 * a multi-APDU sequence (e.g. a chained DESFire GetVersion). */
static uint32_t m_last_apdu_tick = 0;

#define CCID_SESSION_IDLE_MS  500
/* Consecutive unanswered R(NAK)s before we believe the card is gone. This
 * hardware misses roughly 1 poll in 10 even on a stationary card, so tearing
 * down on a single miss produced false removals mid-session (measured: presence
 * dropping after ~1.4 s while the card was held on the reader for 3 s). */
/* ONE miss counter for BOTH probes. Previously the idle path had its own
 * hysteresis inside hf14a_4_presence() and the in-session path had another here,
 * so every session open/close silently reset whichever counter was accumulating.
 * That seam is where the presence bugs clustered. Value held at 2 through the
 * restructure so the merge is behaviour-preserving; retune separately. */
#define CCID_MISS_LIMIT  2
static uint8_t m_miss = 0;

static bool session_idle_long_enough(void) {
    uint32_t now = app_timer_cnt_get();
    return app_timer_cnt_diff_compute(now, m_last_apdu_tick) >= APP_TIMER_TICKS(CCID_SESSION_IDLE_MS);
}
/* Current cached presence, and what the HOST is believed to know. The latter is
 * deliberately a tristate: it is a statement about the host's knowledge, not a
 * snapshot of ours. A bool cannot express "the host's view was reset", and the
 * previous invert trick collided with the state the next scan would produce --
 * cable pull zeroed presence while the card was still physically there, so the
 * inverted value matched what the first post-reset scan found and the card was
 * never announced. */
static bool m_card_present = false;
static enum { NOTIFY_UNKNOWN, NOTIFY_ABSENT, NOTIFY_PRESENT } m_notified = NOTIFY_UNKNOWN;
/* True once a poll has actually driven the antenna, which is the only safe
 * signal that the reader is initialised. Owned here because this module is the
 * one that drives RF -- keeping it in usb_main let the flag and reality diverge,
 * which is how the suspend path came to call antenna_off() on an uninitialised
 * SPI instance and hardfault the device. */
static bool m_field_up = false;

static inline bool radio_is_ours(void) {
    return m_ccid_enabled && (m_radio_holders == 0);
}

void ccid_slot_radio_hold(uint8_t who, bool held) {
    if (held) m_radio_holders |= who;
    else      m_radio_holders &= (uint8_t)~who;
    /* Whoever had the radio may have left a different card in the field (and
     * the CDC hooks switch the antenna off), so the cached classification is no
     * longer trustworthy either way. */
    hf14a_4_presence_reset();
}
/* Back-compat wrapper for the CDC reader/attack hooks. */
void ccid_slot_set_cdc_lock(bool locked) { ccid_slot_radio_hold(CCID_HOLD_CDC, locked); }

void ccid_slot_set_enabled(bool en) {
    m_ccid_enabled = en;
    hf14a_4_presence_reset();           /* cards may swap while we are not looking */
    if (!en) {                          /* disable: drop the card (main-loop ctx) */
        hf14a_4_session_close(&m_session);
        hf14a_4_field_off();            /* power decision: stop driving the field */
        m_card_present = false;
    }
}
/* One teardown, so every caller agrees what "the card went away" means. */
static void presence_lost(void) {
    hf14a_4_session_close(&m_session);
    hf14a_4_presence_reset();
    m_card_present = false;
    m_miss = 0;
}

void ccid_slot_radio_shutdown(void) {
    if (!m_field_up) return;            /* nothing was ever driven: nothing to undo */
    presence_lost();
    hf14a_4_field_off();                /* power decision: suspend / cable pull */
    m_field_up = false;
    /* The host's view is now stale or about to be reset, so record that we do
     * not know what it thinks rather than asserting a value that could collide
     * with the next scan result. */
    m_notified = NOTIFY_UNKNOWN;
}

bool ccid_slot_card_present(void) { return m_card_present; }
bool ccid_slot_is_enabled(void)   { return m_ccid_enabled; }

bool ccid_slot_presence_changed(bool *present) {
    if (radio_is_ours()) {
        m_field_up = true;              /* the polls below drive the antenna */
        if (!m_session.active) {
            /* Idle: the reader reports what it saw, the hysteresis policy lives
             * here. UNSURE holds the previous state without touching the counter
             * -- something is in the field, it simply has not selected yet, and
             * a card still settling must not read as a removal. */
            switch (hf14a_4_presence()) {
                case HF14A_PRES_ISO4:
                    m_card_present = true;  m_miss = 0; break;
                case HF14A_PRES_OTHER:      /* a card, but not one CCID can use */
                    m_card_present = false; m_miss = 0; break;
                case HF14A_PRES_UNSURE:
                    break;
                case HF14A_PRES_GONE:
                    if (++m_miss >= CCID_MISS_LIMIT) {
                        hf14a_4_presence_reset();
                        m_card_present = false;
                        m_miss = 0;
                    }
                    break;
            }
        } else if (session_idle_long_enough()) {
            /* A session blocks the normal scan, and the host does not power the
             * card off on SCardDisconnect(SCARD_LEAVE_CARD) -- so without this
             * presence froze at "present" until something else tore the session
             * down (~8-10 s), and cards placed in that window were never
             * reported at all. R(NAK) is transparent to the session. */
            if (hf14a_4_session_present(&m_session)) {
                m_miss = 0;
            } else if (++m_miss >= CCID_MISS_LIMIT) {
                presence_lost();
            }
        }
    } else {
        if (m_session.active) {
            hf14a_4_session_close(&m_session);     /* deferred teardown          */
            hf14a_4_field_off();                   /* the holder owns the radio  */
        }
        m_card_present = false;                    /* not our radio -> absent    */
    }
    *present = m_card_present;
    return (m_card_present ? NOTIFY_PRESENT : NOTIFY_ABSENT) != m_notified;
}

void ccid_slot_mark_notified(void) {
    m_notified = m_card_present ? NOTIFY_PRESENT : NOTIFY_ABSENT;
}

void ccid_slot_invalidate_notify(void) {
    /* After a bus reset the host's slot state is unknown, so forget what we
     * think we told it -- otherwise presence_changed() reports no change and a
     * card already sitting on the reader is never announced across a
     * re-enumeration. Masked on macOS because it also polls GetSlotStatus; a
     * notification-only host would simply never see the card. */
    m_notified = NOTIFY_UNKNOWN;
}

/**
 * @brief Build the PC/SC Part 3 contactless pseudo-ATR from a card ATS.
 *
 * Mirrors the validated host-side calculation (tools/apdu_probe.py):
 * for ATS 06 75 77 81 02 80 the historical byte is 0x80 and the result is
 * 3B 81 80 01 80 80.
 */
uint8_t ccid_pseudo_atr_from_ats(const uint8_t *ats, uint8_t ats_len, uint8_t *out) {
    const uint8_t *hist = 0;
    uint8_t hist_len = 0;

    if (ats != 0 && ats_len >= 1) {
        uint8_t tl = ats[0];               /* length byte (includes itself) */
        uint8_t i = 1;
        if (tl >= 2 && ats_len >= 2) {
            uint8_t t0 = ats[1];
            i = 2;
            if (t0 & 0x10) i++;            /* TA present */
            if (t0 & 0x20) i++;            /* TB present */
            if (t0 & 0x40) i++;            /* TC present */
        }
        uint8_t end = (tl <= ats_len) ? tl : ats_len;
        if (end > i) {
            hist = &ats[i];
            hist_len = (uint8_t)(end - i);
        }
    }
    if (hist_len > 15) hist_len = 15;      /* nibble field in T0 */

    uint8_t n = 0;
    out[n++] = 0x3B;                       /* TS: direct convention */
    out[n++] = (uint8_t)(0x80 | hist_len); /* T0: Y1=1000 (TD1), K=hist_len */
    out[n++] = 0x80;                       /* TD1: Y2=1000 (TD2), T=0 marker */
    out[n++] = 0x01;                       /* TD2: protocol T=1                */
    for (uint8_t k = 0; k < hist_len; k++) {
        out[n++] = hist[k];                /* historical bytes from ATS        */
    }
    uint8_t tck = 0;                       /* TCK = XOR of everything after TS  */
    for (uint8_t k = 1; k < n; k++) tck ^= out[k];
    out[n++] = tck;
    return n;
}

/* ---- little-endian helpers ------------------------------------------- */
static uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* bStatus byte from icc + command status. active=powered, present=in field. */
static uint8_t slot_status_byte(uint8_t cmd_status) {
    uint8_t icc;
    if (!radio_is_ours())      icc = CCID_ICC_STATUS_ABSENT;   /* disabled/pre-empted */
    else if (m_session.active) icc = CCID_ICC_STATUS_ACTIVE;   /* present + powered   */
    else if (m_card_present)   icc = CCID_ICC_STATUS_INACTIVE; /* present, unpowered  */
    else                       icc = CCID_ICC_STATUS_ABSENT;   /* no card             */
    return (uint8_t)(icc | cmd_status);
}

/* Build a bulk-IN header. data payload (dwLength bytes) must already be at
 * resp + CCID_OFF_DATA. Returns total message length. */
static uint16_t build_in_header(uint8_t *resp, uint8_t type, uint32_t data_len,
                                uint8_t slot, uint8_t seq,
                                uint8_t cmd_status, uint8_t error, uint8_t chain_param) {
    resp[CCID_OFF_MSGTYPE] = type;
    wr_u32le(&resp[CCID_OFF_LENGTH], data_len);
    resp[CCID_OFF_SLOT] = slot;
    resp[CCID_OFF_SEQ] = seq;
    resp[CCID_OFF_IN_STATUS] = slot_status_byte(cmd_status);
    resp[CCID_OFF_IN_ERROR] = error;
    resp[CCID_OFF_IN_CHAINPARAM] = chain_param;
    return (uint16_t)(CCID_OFF_DATA + data_len);
}

/* DataBlock with no data + failure status (mute/busy/etc). */
static uint16_t fail_datablock(uint8_t *resp, uint8_t slot, uint8_t seq, uint8_t error) {
    return build_in_header(resp, RDR_TO_PC_DATABLOCK, 0, slot, seq,
                           CCID_CMD_STATUS_FAILED, error, 0);
}

/**
 * @brief Process one complete CCID bulk-OUT message; produce the bulk-IN reply.
 *
 * @param msg      full bulk-OUT message (10-byte header + payload)
 * @param msg_len  length of @p msg
 * @param resp     output buffer for the bulk-IN message (>= CCID_MAX_MESSAGE_LEN)
 * @return length of the bulk-IN message to send, or 0 to send nothing.
 */
uint16_t ccid_slot_process(const uint8_t *msg, uint16_t msg_len,
                           uint8_t *resp, uint16_t resp_max) {
    (void)resp_max;
    if (msg_len < CCID_BULK_HEADER_LEN) return 0;

    uint8_t  type = msg[CCID_OFF_MSGTYPE];
    uint32_t dlen = rd_u32le(&msg[CCID_OFF_LENGTH]);
    uint8_t  slot = msg[CCID_OFF_SLOT];
    uint8_t  seq  = msg[CCID_OFF_SEQ];

    /* Single slot only. */
    if (slot != 0) {
        return build_in_header(resp, RDR_TO_PC_SLOTSTATUS, 0, slot, seq,
                               CCID_CMD_STATUS_FAILED, 5 /* bSlot err */, 0);
    }
    /* Guard against truncated/oversized payloads. */
    if (dlen > CCID_MAX_APDU_LEN || (uint32_t)(msg_len - CCID_BULK_HEADER_LEN) < dlen) {
        return fail_datablock(resp, slot, seq, CCID_ERROR_XFR_OVERRUN);
    }

    /* When disabled or the radio is held by another consumer, act as empty. */
    bool radio_available = radio_is_ours();

    switch (type) {
    case PC_TO_RDR_ICCPOWERON: {
        if (!radio_available || !hf14a_4_session_open(&m_session)) {
            m_session.active = false;
            m_card_present = false;
            /* We announced a card the host could not activate: re-classify next
             * poll rather than re-announcing the same card from a stale cache
             * (which would flap present/absent every poll). */
            hf14a_4_presence_reset();
            return fail_datablock(resp, slot, seq, CCID_ERROR_ICC_MUTE);
        }
        m_card_present = true;
        m_miss = 0;
        /* Start the idle clock here, or the tick still holds a value from a
         * previous session (or 0 at boot), session_idle_long_enough() is true on
         * the very next poll, and R(NAK) fires at a card that has been RATS'd but
         * has never sent an I-block -- a state where "retransmit the last
         * I-block" is undefined and many PICCs simply mute. */
        m_last_apdu_tick = app_timer_cnt_get();
        uint8_t atr_len = ccid_pseudo_atr_from_ats(m_session.ats, m_session.ats_len,
                                                   &resp[CCID_OFF_DATA]);
        return build_in_header(resp, RDR_TO_PC_DATABLOCK, atr_len, slot, seq,
                               CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0);
    }

    case PC_TO_RDR_ICCPOWEROFF:
        hf14a_4_session_close(&m_session);
        return build_in_header(resp, RDR_TO_PC_SLOTSTATUS, 0, slot, seq,
                               CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0);

    case PC_TO_RDR_GETSLOTSTATUS:
        /* Report the cached presence (maintained by the periodic scan in
         * ccid_slot_presence_changed). No inline scan here — avoids double
         * scanning and keeps this handler fast. Serves pollers (e.g. Linux)
         * as a fallback alongside the interrupt-IN NotifySlotChange path. */
        return build_in_header(resp, RDR_TO_PC_SLOTSTATUS, 0, slot, seq,
                               CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0);

    case PC_TO_RDR_XFRBLOCK: {
        if (!radio_available || !m_session.active) {
            return fail_datablock(resp, slot, seq, CCID_ERROR_ICC_MUTE);
        }
        uint16_t rlen = 0;
        bool ok = hf14a_4_session_apdu(&m_session, &msg[CCID_OFF_DATA], (uint16_t)dlen,
                                       &resp[CCID_OFF_DATA], &rlen,
                                       (uint16_t)(CCID_MAX_APDU_LEN));
        /* Stamp AFTER the exchange: a chained transfer with S(WTX) extensions
         * can run for hundreds of ms, and stamping first left the idle guard
         * short by that duration. */
        m_last_apdu_tick = app_timer_cnt_get();
        if (!ok) {
            /* A failed exchange is evidence of ABSENCE, not a reason to defer
             * the presence check. Stamping unconditionally meant a host retrying
             * a lifted card every ~300 ms re-armed the 500 ms idle guard forever,
             * so R(NAK) never ran and presence stayed pinned at "present". */
            if (++m_miss >= CCID_MISS_LIMIT) presence_lost();
            return fail_datablock(resp, slot, seq, CCID_ERROR_ICC_MUTE);
        }
        m_miss = 0;
        return build_in_header(resp, RDR_TO_PC_DATABLOCK, rlen, slot, seq,
                               CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0);
    }

    case PC_TO_RDR_GETPARAMETERS:
    case PC_TO_RDR_SETPARAMETERS:
    case PC_TO_RDR_RESETPARAMETERS: {
        /* Minimal T=1 parameter block (7 bytes) so pcscd is satisfied. */
        uint8_t *pd = &resp[CCID_OFF_DATA];
        pd[0] = 0x11; /* bmFindexDindex (Fi=1,Di=1)            */
        pd[1] = 0x10; /* bmTCCKST1: T=1, direct convention     */
        pd[2] = 0x00; /* bGuardTimeT1                          */
        pd[3] = 0x4D; /* bWaitingIntegersT1 (BWI=4,CWI=13)     */
        pd[4] = 0x00; /* bClockStop                            */
        pd[5] = 0xFE; /* bIFSC (254)                           */
        pd[6] = 0x00; /* bNadValue                             */
        uint16_t n = build_in_header(resp, RDR_TO_PC_PARAMETERS, 7, slot, seq,
                                     CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0x01 /* T=1 */);
        return n;
    }

    case PC_TO_RDR_ABORT:
        return build_in_header(resp, RDR_TO_PC_SLOTSTATUS, 0, slot, seq,
                               CCID_CMD_STATUS_OK, CCID_ERROR_NONE, 0);

    default:
        /* Unsupported command: slot status with command-failed. */
        return build_in_header(resp, RDR_TO_PC_SLOTSTATUS, 0, slot, seq,
                               CCID_CMD_STATUS_FAILED, CCID_ERROR_CMD_UNSUPPORTED, 0);
    }
}
