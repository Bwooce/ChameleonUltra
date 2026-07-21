/**
 * @file nfc_14a_4.c
 * @brief ISO14443-4 T=CL emulation for ChameleonUltra
 *
 * Implements a full ISO14443-4 tag emulator with a static APDU response
 * table.  The table is populated by the host before field activation, so
 * the firmware can respond to an EMV reader autonomously without any USB
 * communication while the RF field is active.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include "nfc_14a_4.h"
#include "iso14443_4_pcb.h"
#include "nfc_14a.h"
#include "tag_emulation.h"
#include "tag_persistence.h"
#include "fds_util.h"
#include "nrf_log.h"

/* PCB constants and predicates now live in rfid/iso14443_4_pcb.h, shared with
 * the reader paths, with host-side unit tests in firmware/tests/. */

/* WTXM we request when no response is ready yet. Emulation policy rather than a
 * protocol constant, so it stays local. 0x3B = 59, near the 0x3F maximum. */
#define WTX_VALUE  0x3B

/* ------------------------------------------------------------------ */
/*  Module state                                                        */
/* ------------------------------------------------------------------ */
static nfc_tag_14a_4_information_t *m_tag_information = NULL;

/* Shadow coll-res references into m_tag_information */
static nfc_tag_14a_coll_res_reference_t m_shadow_coll_res;

/* T=CL session state */
static uint8_t  m_block_num      = 0;
static bool     m_cid_supported  = false;
static bool     m_chaining_in    = false;   /* mid reassembly of a chained command */
static uint8_t  m_cid            = 0;
static uint8_t  m_apdu_buf[NFC_14A_4_MAX_APDU];
static uint16_t m_apdu_len       = 0;
static bool     m_apdu_pending   = false;
static uint8_t  m_resp_buf[NFC_14A_4_MAX_APDU];
static uint16_t m_resp_len       = 0;
static bool     m_response_ready = false;

/* TX scratch buffer */
static uint8_t m_tx_buf[NFC_14A_4_MAX_APDU + 4];

/* Debug counters — readable via hf 14a debug */
static uint8_t  m_dbg_iblocks_rx  = 0;  /* I-blocks received */
static uint8_t  m_dbg_iblocks_tx  = 0;  /* I-blocks sent */
static uint8_t  m_dbg_last_rx_pcb = 0;  /* PCB of last received I-block */
static uint8_t  m_dbg_last_match  = 0;  /* last find_static_response result */

/* Static APDU response table (RAM copy, populated from m_tag_information) */
static nfc_tag_14a_4_static_response_t m_static_resp[NFC_14A_4_MAX_STATIC_RESPONSES];
static uint8_t m_static_resp_count = 0;

/* Large response overflow (RAM only, > NFC_14A_4_MAX_STATIC_RESP_LEN bytes).
 * NOT persisted to flash. Must reload via emv load after power cycle. */
typedef struct {
    uint8_t  cmd[NFC_14A_4_MAX_STATIC_CMD_LEN];
    uint8_t  cmd_len;
    uint8_t  resp[NFC_14A_4_MAX_LARGE_RESP_LEN];
    uint16_t resp_len;
} nfc_tag_14a_4_large_response_t;
static nfc_tag_14a_4_large_response_t m_large_resp[NFC_14A_4_MAX_LARGE_RESPONSES];
static uint8_t m_large_resp_count = 0;

/* ------------------------------------------------------------------ */
/*  Static response table                                               */
/* ------------------------------------------------------------------ */

void nfc_tag_14a_4_add_static_response(const uint8_t *cmd,  uint8_t cmd_len,
                                       const uint8_t *resp, uint16_t resp_len) {
    if (cmd_len > NFC_14A_4_MAX_STATIC_CMD_LEN) cmd_len = NFC_14A_4_MAX_STATIC_CMD_LEN;

    if (resp_len > NFC_14A_4_MAX_STATIC_RESP_LEN) {
        /* Large response: RAM-only overflow table */
        if (m_large_resp_count >= NFC_14A_4_MAX_LARGE_RESPONSES) return;
        if (resp_len > NFC_14A_4_MAX_LARGE_RESP_LEN) resp_len = NFC_14A_4_MAX_LARGE_RESP_LEN;
        nfc_tag_14a_4_large_response_t *le = &m_large_resp[m_large_resp_count++];
        le->cmd_len  = cmd_len;
        le->resp_len = resp_len;
        memcpy(le->cmd,  cmd,  cmd_len);
        memcpy(le->resp, resp, resp_len);
        return;
    }

    /* Normal response: flash-backed table */
    if (m_static_resp_count >= NFC_14A_4_MAX_STATIC_RESPONSES) return;
    nfc_tag_14a_4_static_response_t *e = &m_static_resp[m_static_resp_count++];
    e->cmd_len  = cmd_len;
    e->resp_len = (uint8_t)resp_len;
    memcpy(e->cmd,  cmd,  cmd_len);
    memcpy(e->resp, resp, resp_len);
    if (m_tag_information &&
            m_tag_information->static_resp_count < NFC_14A_4_MAX_STATIC_RESPONSES) {
        memcpy(&m_tag_information->static_resp[m_tag_information->static_resp_count++],
               e, sizeof(*e));
    }
}

void nfc_tag_14a_4_clear_static_responses(void) {
    m_static_resp_count = 0;
    m_large_resp_count  = 0;
    if (m_tag_information) {
        m_tag_information->static_resp_count = 0;
    }
}

static bool find_static_response(const uint8_t *apdu, uint16_t apdu_len,
                                 uint8_t **resp_out, uint16_t *resp_len_out) {
    /* Flash-backed table */
    for (uint8_t i = 0; i < m_static_resp_count; i++) {
        nfc_tag_14a_4_static_response_t *e = &m_static_resp[i];
        if (apdu_len >= e->cmd_len &&
                memcmp(apdu, e->cmd, e->cmd_len) == 0) {
            *resp_out     = e->resp;
            *resp_len_out = e->resp_len;
            return true;
        }
    }
    /* RAM-only large response table */
    for (uint8_t i = 0; i < m_large_resp_count; i++) {
        nfc_tag_14a_4_large_response_t *e = &m_large_resp[i];
        if (apdu_len >= e->cmd_len &&
                memcmp(apdu, e->cmd, e->cmd_len) == 0) {
            *resp_out     = e->resp;
            *resp_len_out = e->resp_len;
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/*  TX helpers                                                          */
/* ------------------------------------------------------------------ */

/* KNOWN GAP: card->reader chaining is not implemented.
 *
 * This emits the whole response in a single I-block, capped only by our own
 * buffer, so a reader advertising a small FSD is over-run by any longer
 * response -- a protocol violation, not merely a truncation. Our own reader
 * hardcodes FSDI=4 (FSD=48), so Chameleon-to-Chameleon breaks above ~45 bytes.
 *
 * The reader's FSD is now available -- nfc_tag_14a_get_reader_fsd() latches it
 * from the RATS -- and the chunk size that would be needed is logged below, so
 * the arithmetic can be checked against a real reader before any chunking code
 * is written. What is still missing is the state machine: emit with the
 * chaining bit set, then send the next chunk on each R(ACK), tracking the
 * offset and block number.
 *
 * Deliberately not written blind. It shares the block-number handling that two
 * inspection-only attempts have already got wrong, and there is no second
 * reader here to test against. */
static void send_iblock(const uint8_t *data, uint16_t len) {
    uint8_t pcb = 0x02 | (m_block_num & 0x01);
    if (m_cid_supported) pcb |= ISO14443_4_PCB_CID;
    uint8_t off = 0;
    m_tx_buf[off++] = pcb;
    if (m_cid_supported) m_tx_buf[off++] = m_cid & 0x0F;
    if (len > NFC_14A_4_MAX_APDU) len = NFC_14A_4_MAX_APDU;
    /* Diagnostic only -- see the gap note above. When this fires, the frame we
     * are about to send exceeds what the reader told us it can accept, and the
     * exchange is expected to fail. Logging it makes the failure legible
     * instead of looking like a dead card. */
    {
        uint16_t fsd  = nfc_tag_14a_get_reader_fsd();
        uint16_t cap  = iso14443_4_max_payload(fsd, m_cid_supported, false);
        if (len > cap) {
            NRF_LOG_WARNING("14A4 response %d B exceeds reader FSD %d (cap %d): "
                            "card->reader chaining not implemented",
                            len, fsd, cap);
        }
    }
    memcpy(&m_tx_buf[off], data, len);
    nfc_tag_14a_tx_bytes(m_tx_buf, off + len, true);
    m_block_num ^= 1;
}

static void send_rack(void) {
    /* The R(ACK) carries the SAME block number as the I-block it acknowledges,
     * and m_block_num is toggled AFTER transmit -- mirroring send_iblock().
     *
     * This file uses the pre-toggled convention: m_block_num holds the number
     * to send NEXT. On receiving I(chain) blk=0 it already holds 0, which is
     * the correct R(ACK) number, so the toggle must come after the frame is
     * built, not before.
     *
     * History worth keeping: the original code sent the right number but never
     * toggled, so the reader's next chunk arrived with a number that no longer
     * matched and fell into the retransmission branch. A previous fix moved a
     * toggle to the TOP of this function, which corrected the advance but made
     * every R(ACK) carry a number one off -- a right diagnosis fixed in the
     * wrong direction. Both bugs are avoided by toggling after transmit. */
    uint8_t pcb = 0xA2 | (m_block_num & 0x01);
    if (m_cid_supported) {
        pcb |= ISO14443_4_PCB_CID;
        uint8_t buf[2] = { pcb, m_cid & 0x0F };
        nfc_tag_14a_tx_bytes(buf, 2, true);
    } else {
        nfc_tag_14a_tx_bytes(&pcb, 1, true);
    }
    m_block_num ^= 1;      /* after transmit, exactly as send_iblock() does */
}

static void send_wtx(void) {
    uint8_t buf[3];
    uint8_t off = 0;
    buf[off++] = ISO14443_4_PCB_S_WTX | (m_cid_supported ? ISO14443_4_PCB_CID : 0);
    if (m_cid_supported) buf[off++] = m_cid & 0x0F;
    buf[off++] = WTX_VALUE;
    nfc_tag_14a_tx_bytes(buf, off, true);
}

/* ------------------------------------------------------------------ */
/*  State handler (called from NFCT ISR on each received frame)        */
/* ------------------------------------------------------------------ */

/* The transport hands us a BIT count -- see nfc_tag_14a_state_handler_t in
 * nfc_14a.h, whose parameter is literally named szBits, and nfc_14a.c which
 * calls cb_state(p_data, szDataBits).
 *
 * This function declared the parameter as szBytes and used it as one, so every
 * length here was ~8x too large. Consequences, on every single frame:
 *   - the APDU relayed to the host was over-long and tail-padded with whatever
 *     followed it in the RX buffer, so relay mode cannot ever have worked;
 *   - memcpy over-read m_nfc_rx_buffer;
 *   - the DESELECT echo transmitted ~24 bytes for a 3-byte frame.
 *
 * It went unnoticed because find_static_response() is a PREFIX match: an
 * over-long, garbage-tailed APDU still matches its first cmd_len bytes. The one
 * path anybody exercised is the one path immune to the defect.
 *
 * The received frame INCLUDES the 2-byte CRC and it is counted -- nfc_14a.c
 * validates RATS as nfc_tag_14a_checks_crc(p_data, 4) for the 4-byte frame
 * E0 P1 CRC CRC. So strip 2 bytes, and check the CRC, which this layer never
 * did although MF1 does it on every command. */
static void nfc_tag_14a_4_state_handler(uint8_t *data, uint16_t szBits) {
    uint16_t szBytes = szBits / 8u;
    if (szBytes < 3u) return;                          /* PCB + CRC minimum */
    if (!nfc_tag_14a_checks_crc(data, szBytes)) return;
    uint16_t frame_len = (uint16_t)(szBytes - 2u);     /* CRC stripped */

    uint8_t pcb = data[0];
    if (!iso14443_4_is_valid_pcb(pcb)) return;         /* b8b7 == 01 is RFU */

    /* ---- S-block ---- */
    if (iso14443_4_is_sblock(pcb)) {
        if (iso14443_4_is_deselect(pcb)) {
            /* Build a fresh S(DESELECT) rather than echoing the received frame:
             * the echo re-transmitted the received CRC as payload and, with the
             * length bug above, sent several times the intended byte count. */
            uint8_t resp[2];
            uint8_t off = 0;
            resp[off++] = (uint8_t)(ISO14443_4_PCB_S_DESELECT |
                                    (m_cid_supported ? ISO14443_4_PCB_CID : 0u));
            if (m_cid_supported) resp[off++] = m_cid & 0x0F;
            nfc_tag_14a_tx_bytes(resp, off, true);
            nfc_tag_14a_4_reset_handler();
            return;
        }
        if (iso14443_4_is_wtx(pcb)) {
            /* A received S(WTX) is the reader GRANTING our request, not a
             * request to us -- S(WTX) is always PICC-initiated. Send exactly
             * one frame in response: the pending I-block if we have one.
             *
             * This used to echo the WTX and then immediately call send_iblock,
             * i.e. two nfc_tag_14a_tx_bytes calls in one ISR pass. They share a
             * single TX buffer and neither waits for TXFRAMEEND, so the second
             * clobbered the first mid-transmission. That is the default path
             * (send_wtx fires for any APDU with no configured response), not an
             * edge case. */
            if (m_response_ready) {
                m_response_ready = false;
                send_iblock(m_resp_buf, m_resp_len);
            }
            return;
        }
        return;
    }

    /* ---- R-block ---- */
    if (iso14443_4_is_rblock(pcb)) {
        /* NOTE: R(NAK) is answered with R(ACK) here, which is wrong -- a reader
         * recovering a lost response gets an acknowledgement instead of the
         * retransmission, and the exchange deadlocks. Our own reader's presence
         * check sends exactly this R(NAK) and relies on the retransmit.
         * Deliberately NOT fixed blind: it is the same block-number state
         * machine that two inspection-only attempts already got wrong. */
        send_rack();
        return;
    }

    /* ---- I-block ---- */
    if (iso14443_4_is_iblock(pcb)) {
        uint8_t reader_blknum = iso14443_4_blocknum(pcb);
        bool    more_chain    = iso14443_4_has_chaining(pcb);

        /* Header is PCB plus any CID and NAD bytes; frame_len already excludes
         * the CRC. Guard uses > not >= so a zero-length I-block -- legal, and
         * used as a chaining continuation -- is not treated as malformed. */
        uint8_t offset = iso14443_4_hdr_len(pcb);
        if (offset > frame_len) {
            send_rack();
            return;
        }

        uint16_t apdu_len = (uint16_t)(frame_len - offset);
        if (apdu_len > NFC_14A_4_MAX_APDU) apdu_len = NFC_14A_4_MAX_APDU;

        m_dbg_iblocks_rx++;
        m_dbg_last_rx_pcb = pcb;
        NRF_LOG_INFO("14A4 I-block #%d: reader_blk=%d m_block_num=%d apdu_len=%d",
                     m_dbg_iblocks_rx, reader_blknum, m_block_num, apdu_len);

        /* Block number check per ISO14443-4 §7.5.3.3:
         * If block number matches expected, process new APDU.
         * If block number does NOT match, it is a retransmit —
         * resend the last response without re-processing. */
        if (reader_blknum != (m_block_num & 0x01)) {
            /* Retransmit: resend last response */
            if (m_resp_len > 0) {
                /* Restore block num to what we sent last time and resend */
                m_block_num ^= 1;  /* undo the increment from last send */
                send_iblock(m_resp_buf, m_resp_len);
            } else {
                send_rack();
            }
            return;
        }

        /* Reader->card chaining: APPEND each fragment. This overwrote the buffer
         * with every chunk, so only the final fragment survived and the
         * reassembled command was garbage. Unreachable until 49fcc10 corrected
         * PCB_CHAIN from 0x20 to 0x10, which is why it was never noticed.
         *
         * NOT VERIFIED ON HARDWARE: exercising it needs a reader that chains a
         * command, and there is only one Chameleon here. Correct by inspection
         * only -- treat with suspicion. */
        if (m_chaining_in) {
            if ((uint16_t)m_apdu_len + apdu_len <= sizeof(m_apdu_buf)) {
                memcpy(&m_apdu_buf[m_apdu_len], &data[offset], apdu_len);
                m_apdu_len = (uint16_t)(m_apdu_len + apdu_len);
            } else {
                /* Oversized chained command: drop it rather than truncate
                 * silently, and reset so the next command starts clean. */
                m_chaining_in = false;
                m_apdu_len = 0;
                send_rack();
                return;
            }
        } else {
            memcpy(m_apdu_buf, &data[offset], apdu_len);
            m_apdu_len = apdu_len;
        }
        m_response_ready = false;

        if (more_chain) {
            /* Mid-chain: the command is incomplete, so do NOT flag it pending.
             * m_apdu_pending was set before this check, so a host polling
             * hf14a_4_apdu_recv could collect and act on a partial command. */
            m_chaining_in = true;
            send_rack();
            return;
        }
        m_chaining_in = false;
        m_apdu_pending = true;

        /* APDU complete — check static table first, then WTX */
        {
            uint8_t  *static_resp = NULL;
            uint16_t  static_len  = 0;
            /* Match on the reassembled length, not this fragment's. They are
             * equal for unchained commands, which is why passing apdu_len went
             * unnoticed; for a reassembled chain a short final fragment could
             * fail the length test and miss a valid entry. Introduced alongside
             * the append fix. */
            bool _found = find_static_response(m_apdu_buf, m_apdu_len,
                                               &static_resp, &static_len);
            m_dbg_last_match = _found ? 1 : 0;
            NRF_LOG_INFO("14A4 find_static: found=%d static_len=%d resp_count=%d",
                         _found, static_len, m_static_resp_count);
            if (_found) {
                m_dbg_iblocks_tx++;
                memcpy(m_resp_buf, static_resp, static_len);
                m_resp_len = static_len;
                send_iblock(m_resp_buf, m_resp_len);
            } else if (m_response_ready) {
                m_response_ready = false;
                send_iblock(m_resp_buf, m_resp_len);
            } else {
                /* No response ready — keep reader alive with WTX */
                send_wtx();
            }
        }
        return;
    }

    NRF_LOG_INFO("14A-4: unknown PCB 0x%02x", pcb);
}


/* ------------------------------------------------------------------ */
/*  APDU relay API (for host-driven responses)                         */
/* ------------------------------------------------------------------ */

bool nfc_tag_14a_4_get_pending_apdu(uint8_t *buf, uint16_t *length) {
    if (!m_apdu_pending) return false;
    m_apdu_pending = false;
    *length = m_apdu_len;
    memcpy(buf, m_apdu_buf, m_apdu_len);
    return true;
}

void nfc_tag_14a_4_set_response(const uint8_t *data, uint16_t length) {
    if (length > NFC_14A_4_MAX_APDU) length = NFC_14A_4_MAX_APDU;
    memcpy(m_resp_buf, data, length);
    m_resp_len = length;
    m_response_ready = true;
}

/* ------------------------------------------------------------------ */
/*  Reset handler                                                       */
/* ------------------------------------------------------------------ */

void nfc_tag_14a_4_reset_handler(void) {
    m_block_num      = 0;
    m_cid_supported  = false;
    m_chaining_in    = false;
    m_cid            = 0;
    m_apdu_pending   = false;
    m_response_ready = false;
    m_apdu_len       = 0;
    m_resp_len       = 0;
}

void nfc_tag_14a_4_get_debug_counters(uint8_t *rx, uint8_t *tx,
                                      uint8_t *last_pcb, uint8_t *last_match) {
    *rx = m_dbg_iblocks_rx;
    *tx = m_dbg_iblocks_tx;
    *last_pcb = m_dbg_last_rx_pcb;
    *last_match = m_dbg_last_match;
}

/* ------------------------------------------------------------------ */
/*  Anti-collision resource                                             */
/* ------------------------------------------------------------------ */

nfc_tag_14a_coll_res_reference_t *nfc_tag_14a_4_get_coll_res(void) {
    if (m_tag_information == NULL) return NULL;
    m_shadow_coll_res.sak  = m_tag_information->res_coll.sak;
    m_shadow_coll_res.atqa = m_tag_information->res_coll.atqa;
    m_shadow_coll_res.uid  = m_tag_information->res_coll.uid;
    m_shadow_coll_res.size = &m_tag_information->res_coll.size;
    m_shadow_coll_res.ats  = &m_tag_information->res_coll.ats;
    return &m_shadow_coll_res;
}

/* ------------------------------------------------------------------ */
/*  Data load / save / factory callbacks                               */
/* ------------------------------------------------------------------ */

int nfc_tag_14a_4_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    int info_size = sizeof(nfc_tag_14a_4_information_t);
    if (buffer->length < info_size) {
        NRF_LOG_ERROR("14A-4 loadcb: buffer too small (%d < %d)",
                      buffer->length, info_size);
        return info_size;
    }
    m_tag_information = (nfc_tag_14a_4_information_t *)buffer->buffer;

    /* Populate RAM static table from persisted slot data */
    m_static_resp_count = m_tag_information->static_resp_count;
    if (m_static_resp_count > NFC_14A_4_MAX_STATIC_RESPONSES)
        m_static_resp_count = NFC_14A_4_MAX_STATIC_RESPONSES;
    memcpy(m_static_resp, m_tag_information->static_resp,
           m_static_resp_count * sizeof(nfc_tag_14a_4_static_response_t));

    nfc_tag_14a_handler_t handler = {
        .get_coll_res = nfc_tag_14a_4_get_coll_res,
        .cb_state     = nfc_tag_14a_4_state_handler,
        .cb_reset     = nfc_tag_14a_4_reset_handler,
    };
    nfc_tag_14a_set_handler(&handler);
    NRF_LOG_INFO("14A-4 loadcb OK: SAK=%02x uid_sz=%d static_resp=%d",
                 m_tag_information->res_coll.sak[0],
                 m_tag_information->res_coll.size,
                 m_static_resp_count);
    return info_size;
}

int nfc_tag_14a_4_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    return sizeof(nfc_tag_14a_4_information_t);
}

bool nfc_tag_14a_4_data_factory(uint8_t slot, tag_specific_type_t tag_type) {
    if (tag_type != TAG_TYPE_HF14A_4) return false;

    /* Build factory defaults on stack and write directly to FDS
     * (same pattern as nfc_tag_mf1_data_factory). */
    nfc_tag_14a_4_information_t info;
    memset(&info, 0, sizeof(info));

    /* Placeholder 7-byte NXP-style UID */
    info.res_coll.size    = NFC_TAG_14A_UID_DOUBLE_SIZE;
    info.res_coll.atqa[0] = 0x04;
    info.res_coll.atqa[1] = 0x00;
    info.res_coll.sak[0]  = 0x20;   /* ISO14443-4 */
    info.res_coll.uid[0]  = 0x04;
    info.res_coll.uid[1]  = 0x01;
    info.res_coll.uid[2]  = 0x02;
    info.res_coll.uid[3]  = 0x03;
    info.res_coll.uid[4]  = 0x04;
    info.res_coll.uid[5]  = 0x05;
    info.res_coll.uid[6]  = 0x06;

    static const uint8_t default_ats[] = {
        /* TL T0 TA TB TC ...
         * TC(1) is 0x00, not 0x02: b2 of TC advertises CID support, and this
         * layer cannot provide it. m_cid_supported is never set true anywhere,
         * so every CID emit path is dead -- a reader that assigned a non-zero
         * CID would have its I-blocks parsed correctly but answered with no CID
         * field, and would discard the responses. Our own reader sends CID=0 so
         * this was unreachable in-tree, but a PC/SC reader may assign one.
         * Advertising honestly is the cheap fix; real CID support is a state
         * machine with no way to test it on the hardware here. */
        0x10, 0x78, 0x80, 0x70, 0x00, 0x00,
        0x31, 0xC1, 0x64, 0x09, 0x97, 0x61,
        0x26, 0x00, 0x90, 0x00
    };
    info.res_coll.ats.length = sizeof(default_ats);
    memcpy(info.res_coll.ats.data, default_ats, sizeof(default_ats));
    info.static_resp_count = 0;

    fds_slot_record_map_t map_info;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map_info);
    bool ret = fds_write_sync(map_info.id, map_info.key, sizeof(info), &info);
    NRF_LOG_INFO("14A-4 factory slot %d: %s", slot, ret ? "OK" : "FAIL");
    return ret;
}
