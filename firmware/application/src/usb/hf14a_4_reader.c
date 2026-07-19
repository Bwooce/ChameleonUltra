/**
 * @file hf14a_4_reader.c
 * @brief ISO14443-4 (T=CL) reader session for the CCID slot (v1).
 *
 * Self-contained on the public rc522.h reader API. Activation mirrors
 * cmd_processor_hf14a_4_reader_apdu; per-APDU exchange mirrors the proven
 * tcl_apdu_ helper (I-block send, card-side chaining, S(WTX) echo, ComIrqReg
 * stale-RxIRq clear) but keeps ONE activation across many APDUs. FU-01 folds
 * this and tcl_apdu_ into a shared T=CL module.
 */
#include "hf14a_4_reader.h"
#include <string.h>

/* Ultra only: the Makefile builds this file solely for the Ultra target
 * (Lite has no MFRC522 HF reader). The guard is belt-and-braces. */
#if defined(PROJECT_CHAMELEON_ULTRA)

#include "rfid_main.h"
#include "bsp_delay.h"
#include "rfid/reader/hf/rc522.h"

#define TCL_RESP_TIMEOUT_MS     600

bool hf14a_4_session_open(hf14a_4_session_t *s) {
    memset(s, 0, sizeof(*s));

    reader_mode_enter();               /* switch antenna path + power reader   */

    /* Field cycle to force the card back to IDLE, then select + RATS. */
    pcd_14a_reader_antenna_off();
    bsp_delay_ms(5);
    pcd_14a_reader_reset();
    pcd_14a_reader_antenna_on();
    bsp_delay_ms(8);

    pcd_14a_reader_timeout_set(200);
    picc_14a_tag_t tag;
    uint8_t status = pcd_14a_reader_scan_auto(&tag);
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
    if (status != STATUS_HF_TAG_OK) {
        pcd_14a_reader_antenna_off();
        return false;
    }
    if (!(tag.sak & 0x20) || tag.ats_len == 0) {
        /* Not an ISO14443-4 card — no APDU transport available. */
        pcd_14a_reader_antenna_off();
        return false;
    }

    s->sak = tag.sak;
    s->uid_len = tag.uid_len > sizeof(s->uid) ? sizeof(s->uid) : tag.uid_len;
    memcpy(s->uid, tag.uid, s->uid_len);
    s->ats_len = tag.ats_len > HF14A_4_ATS_MAX ? HF14A_4_ATS_MAX : tag.ats_len;
    memcpy(s->ats, tag.ats, s->ats_len);
    s->blk = 0;

    /* FSC = max bytes the card accepts per frame, from FSCI (low nibble of the
     * ATS format byte T0, which follows the length byte). Table per ISO14443-4. */
    s->fsc = HF14A_4_FSC_DEFAULT;
    if (s->ats_len >= 2) {
        static const uint16_t fsc_tbl[] = {16, 24, 32, 40, 48, 64, 96, 128, 256};
        uint8_t fsci = s->ats[1] & 0x0F;
        if (fsci < (sizeof(fsc_tbl) / sizeof(fsc_tbl[0]))) s->fsc = fsc_tbl[fsci];
    }

    s->active = true;                  /* leave field ON for APDU exchange     */
    return true;
}

bool hf14a_4_session_apdu(hf14a_4_session_t *s,
                          const uint8_t *apdu, uint16_t apdu_len,
                          uint8_t *resp, uint16_t *resp_len, uint16_t resp_max) {
    *resp_len = 0;
    if (!s->active || apdu_len == 0 || apdu_len > HF14A_4_CMD_MAX) return false;

    uint8_t abuf[3 + DEF_FIFO_LENGTH];   /* one frame: PCB + payload + CRC */
    uint8_t rbuf[288];
    uint8_t crc[2];

    /* Per-frame payload = min(card FSC, reliable RC522 TX frame) - PCB - CRC.
     * The 64B FIFO is unreliable near its limit, so cap well below it and let
     * chaining carry the rest. */
    uint16_t frame_cap = (s->fsc && s->fsc < HF14A_4_TX_FRAME_MAX) ? s->fsc : HF14A_4_TX_FRAME_MAX;
    uint16_t chunk_max = frame_cap - 3;
    if (chunk_max < 1) chunk_max = 1;

    /* Outbound (reader->card) chaining: send all but the last chunk as chained
     * I-blocks (PCB bit4 = 0x10 set), each acknowledged by an R(ACK). */
    uint16_t off = 0;
    while ((uint16_t)(apdu_len - off) > chunk_max) {
        abuf[0] = (uint8_t)(0x02 | (s->blk & 0x01) | 0x10);
        memcpy(&abuf[1], &apdu[off], chunk_max);
        crc_14a_append(abuf, chunk_max + 1);
        write_register_single(ComIrqReg, 0x7F);
        pcd_14a_reader_timeout_set(TCL_RESP_TIMEOUT_MS);
        uint16_t abits = 0;
        uint8_t ast = pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, abuf, (uint8_t)(chunk_max + 3),
                                                    rbuf, &abits, U8ARR_BIT_LEN(rbuf));
        pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
        if (ast != STATUS_HF_TAG_OK || abits < 24u) return false;
        uint16_t arb = abits / 8u;
        crc_14a_calculate(rbuf, arb - 2u, crc);
        if (rbuf[arb - 2] != crc[0] || rbuf[arb - 1] != crc[1]) return false;
        if ((rbuf[0] & 0xF6u) != 0xA2u) return false;   /* expect R(ACK) */
        s->blk ^= 1;
        off += chunk_max;
    }

    /* Final (or only) I-block: no chaining bit. */
    uint16_t last = (uint16_t)(apdu_len - off);
    abuf[0] = (uint8_t)(0x02 | (s->blk & 0x01));
    memcpy(&abuf[1], &apdu[off], last);
    crc_14a_append(abuf, last + 1);
    uint8_t frame_len = (uint8_t)(last + 3);

    /* Clear stale RxIRq before transmit (bytes_transfer only clears Set1). */
    write_register_single(ComIrqReg, 0x7F);
    pcd_14a_reader_timeout_set(TCL_RESP_TIMEOUT_MS);
    uint16_t rbits = 0;
    uint8_t st = pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, abuf, frame_len,
                                               rbuf, &rbits, U8ARR_BIT_LEN(rbuf));
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
    if (st != STATUS_HF_TAG_OK || rbits < 24u) return false;

    uint16_t rb = rbits / 8u;
    crc_14a_calculate(rbuf, rb - 2u, crc);
    if (rbuf[rb - 2] != crc[0] || rbuf[rb - 1] != crc[1]) return false;

    /* NOTE on the overflow bail-outs below: erroring beats silently truncating
     * (a short APDU response reported as success is corrupt data), but we
     * cannot simply return -- blk has already been toggled and the card is
     * mid-chain waiting for an R(ACK) it will never get, so every later APDU on
     * this session would fail. Dropping s->active makes ccid_slot_process()
     * answer ICC_MUTE and forces the host to re-power the card, which
     * re-activates it cleanly. */
    s->blk ^= 1;
    uint8_t  resp_pcb = rbuf[0];
    uint16_t out = 0;
    /* uint16_t, not uint8_t: nothing actually clamps rb to the card FSC -- the
     * transfer is bounded only by rbuf[288] -- so a card sending a >258 byte
     * frame would wrap a uint8_t and silently discard most of the payload. */
    uint16_t dlen = (uint16_t)(rb - 3u);
    if (dlen > 0) {
        if (out + dlen > resp_max) { *resp_len = 0; hf14a_4_session_close(s); return false; } /* overflow: kill the session, see note */
        memcpy(&resp[out], &rbuf[1], dlen);
        out += dlen;
    }

    /* Card-side chaining: PCB bit5 (0x20) => more blocks follow. */
    while (resp_pcb & 0x20u) {
        if ((resp_pcb & 0xC0u) != 0x00u) {
            /* S-block: echo S(WTX), stop on others (e.g. DESELECT). */
            if ((resp_pcb & 0xF0u) == 0xF0u) {
                if (dlen > 0 && out >= dlen) out -= dlen; /* undo spurious WTXM */
                uint8_t wtx[4];
                wtx[0] = resp_pcb;
                wtx[1] = rbuf[1];
                crc_14a_append(wtx, 2);
                write_register_single(ComIrqReg, 0x7F);
                pcd_14a_reader_timeout_set(TCL_RESP_TIMEOUT_MS);
                rbits = 0;
                st = pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, wtx, 4,
                                                   rbuf, &rbits, U8ARR_BIT_LEN(rbuf));
                pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
                if (st != STATUS_HF_TAG_OK || rbits < 24u) break;
                rb = rbits / 8u;
                crc_14a_calculate(rbuf, rb - 2u, crc);
                if (rbuf[rb - 2] != crc[0] || rbuf[rb - 1] != crc[1]) break;
                resp_pcb = rbuf[0];
                dlen = (uint16_t)(rb - 3u);
                if (dlen > 0) {
                    if (out + dlen > resp_max) { *resp_len = 0; hf14a_4_session_close(s); return false; } /* overflow: kill the session, see note */
                    memcpy(&resp[out], &rbuf[1], dlen);
                    out += dlen;
                }
                continue;
            }
            break;
        }

        /* R(ACK), block number matching the received I-block. */
        uint8_t rack[3];
        rack[0] = (uint8_t)(0xA2u | (resp_pcb & 0x01u));
        crc_14a_append(rack, 1);
        write_register_single(ComIrqReg, 0x7F);
        pcd_14a_reader_timeout_set(TCL_RESP_TIMEOUT_MS);
        rbits = 0;
        st = pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, rack, 3,
                                           rbuf, &rbits, U8ARR_BIT_LEN(rbuf));
        pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
        if (st != STATUS_HF_TAG_OK || rbits < 24u) break;
        rb = rbits / 8u;
        crc_14a_calculate(rbuf, rb - 2u, crc);
        if (rbuf[rb - 2] != crc[0] || rbuf[rb - 1] != crc[1]) break;
        resp_pcb = rbuf[0];
        dlen = (uint16_t)(rb - 3u);
        if (dlen > 0) {
            if (out + dlen > resp_max) { *resp_len = 0; hf14a_4_session_close(s); return false; } /* overflow: kill the session, see note */
            memcpy(&resp[out], &rbuf[1], dlen);
            out += dlen;
        }
    }

    *resp_len = out;
    return out > 0;
}

/* Cheap "is anything in the field?" probe: a single WUPA, short timeout.
 *
 * Deliberately NOT pcd_14a_reader_atqa_request(): that retries up to 10 times,
 * and pcd_14a_reader_scan_auto() then runs scan_once twice, so an idle (no
 * card) poll costs ~20 timeouts -- roughly 2 s. Polled every 300 ms that keeps
 * the main loop permanently busy, starving the USB event queue (unusable CDC
 * latency, and before the enumeration gate, a device that never enumerated).
 *
 * WUPA (0x52) rather than REQA (0x26) so HALTed/already-selected cards answer
 * too, which is why no antenna field-cycle is needed here. */
static bool hf14a_4_quick_probe(uint8_t atqa_out[2]) {
    uint8_t wupa[] = { PICC_REQALL };
    uint8_t resp[2] = {0};
    uint16_t len = 0;

    /* The field must be up for the card to answer: hf14a_4_session_close() (and
     * the failure paths in session_open) leave the antenna OFF, so probing
     * without this silently never detects a card. Note we only switch it ON --
     * WUPA removes the need for the old off/on field-cycle, but not the need
     * for a field. The settle delay lets the card power up before it answers. */
    pcd_14a_reader_antenna_on();
    bsp_delay_ms(HF14A_4_FIELD_SETTLE_MS);

    /* Clear stale RxIRq before transmit (bytes_transfer only clears Set1), as
     * every other exchange in this file does. Without it the probe can latch a
     * leftover interrupt from an earlier transfer and report an ATQA that never
     * arrived -- i.e. a removed card keeps reading as present, so removal goes
     * unreported and the next card placed is never announced to the host. The
     * old field-cycling probe hid this; a lean single-shot probe must not. */
    write_register_single(ComIrqReg, 0x7F);

    pcd_14a_reader_timeout_set(HF14A_4_PROBE_TIMEOUT_MS);
    uint8_t status = pcd_14a_reader_bits_transfer(wupa, 7, NULL, resp, NULL, &len,
                                                  U8ARR_BIT_LEN(resp));
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
    if (status != STATUS_HF_TAG_OK || len != 16) return false;  /* ATQA is 2 bytes */
    if (atqa_out) memcpy(atqa_out, resp, 2);
    return true;
}

/* Classification cache for whatever is currently in the field, so repeat polls
 * need only the cheap probe. Only ever set from a SUCCESSFUL tier-2 scan --
 * caching a failed scan would latch a transient RF error (off-centre card,
 * anticollision collision, card still settling) into a permanent "not a
 * smartcard" verdict that survives until the card physically leaves the field.
 * s_miss gives one poll of hysteresis so a single missed WUPA does not report
 * a removal; s_unselectable throttles a card that answers WUPA but will not
 * select, so it costs one full scan per second rather than one per poll. */
static bool    s_classified   = false;
static bool    s_is_iso4      = false;
static uint8_t s_atqa[2]      = {0};
static uint8_t s_miss         = 0;
static uint8_t s_scan_fails   = 0;   /* consecutive tier-2 failures */
static uint8_t s_backoff      = 0;   /* polls left to skip          */

#define HF14A_4_MISS_LIMIT        2   /* consecutive probe misses => absent    */
#define HF14A_4_SCAN_FAIL_LIMIT   4   /* failures before we start backing off  */
#define HF14A_4_UNSELECTABLE_SKIP 3   /* polls to skip once backing off        */

void hf14a_4_presence_reset(void) {
    s_classified   = false;
    s_is_iso4      = false;
    s_miss       = 0;
    s_scan_fails = 0;
    s_backoff    = 0;
    memset(s_atqa, 0, sizeof(s_atqa));
}

bool hf14a_4_presence(void) {
    reader_mode_enter();               /* idempotent: no-op if already reader  */

    /* Tier 1: cheap probe. The common idle case (no card) exits here in a few
     * ms instead of ~2 s. */
    uint8_t atqa[2] = {0};
    if (!hf14a_4_quick_probe(atqa)) {
        /* Hysteresis: one miss is not a removal. A single WUPA can be lost to a
         * cold field (the antenna is switched off by session_close and by the
         * CDC reader hooks) or to momentary detuning, and reporting absent
         * would tear down the host's connection for no reason. */
        if (s_classified && ++s_miss < HF14A_4_MISS_LIMIT) return s_is_iso4;
        hf14a_4_presence_reset();
        return false;
    }
    s_miss = 0;

    /* Same card still sitting there -- no need to re-run the costly scan. The
     * ATQA guard catches a swap to a different card type that happens fast
     * enough that no poll saw an empty field (otherwise we would report the
     * previous card's classification). Same ATQA does not prove same card, but
     * a changed one definitely proves a different card, and it is free here. */
    if (s_classified && memcmp(atqa, s_atqa, sizeof(atqa)) == 0) return s_is_iso4;
    /* Different ATQA => definitely a different card. Drop the cache NOW: if the
     * rescan below fails we would otherwise keep reporting the PREVIOUS card's
     * classification, and a repeatedly-unselectable replacement would hold that
     * stale verdict indefinitely. */
    s_classified = false;
    s_is_iso4    = false;

    /* Something is there but unclassified. Only back off once it has failed
     * repeatedly: a card being placed by hand routinely fails its first scans
     * while it settles into the field, and skipping polls then makes arrival
     * detection miss real cards. Stay responsive first, throttle later. */
    if (s_backoff) { s_backoff--; return false; }

    /* Tier 2: newly arrived -- pay for the full scan, since confirming an
     * ISO14443-4 card needs the SAK (anticollision + SELECT). */
    pcd_14a_reader_timeout_set(100);
    picc_14a_tag_t tag;
    uint8_t status = pcd_14a_reader_scan_auto(&tag);
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);

    /* Leave the card HALTed. scan_auto leaves it SELECTED/ACTIVE, and an ACTIVE
     * card ignores WUPA -- so the next probe would report it gone and presence
     * would flap (the host then never sees a stable card to connect to). The
     * old field-cycle hid this by power-cycling the card back to IDLE; halting
     * achieves the same for far less time. */
    pcd_14a_reader_fast_halt_tag();

    if (status != STATUS_HF_TAG_OK) {
        /* Transient RF failure -- do NOT cache it, or a card that merely landed
         * badly stays invisible even after it is nudged into place. Retry every
         * poll for the first few, then throttle a genuinely un-selectable card. */
        if (s_scan_fails < 255) s_scan_fails++;   /* clamp: wrapping would
                                                   * disengage the back-off */
        if (s_scan_fails >= HF14A_4_SCAN_FAIL_LIMIT) {
            s_backoff = HF14A_4_UNSELECTABLE_SKIP;
        }
        return false;
    }

    /* Must match what session_open() will accept, otherwise we announce a card
     * the subsequent IccPowerOn cannot activate: it requires an ATS as well as
     * the ISO14443-4 SAK bit (scan_once tolerates a card that claims 14443-4
     * but NAKs RATS, leaving ats_len = 0). Mismatch => endless present/absent
     * flapping as each power-on fails. */
    s_is_iso4    = (tag.sak & 0x20) && (tag.ats_len != 0);
    s_classified = true;
    s_scan_fails = 0;
    s_backoff    = 0;
    memcpy(s_atqa, atqa, sizeof(s_atqa));
    return s_is_iso4;
}

bool hf14a_4_session_present(hf14a_4_session_t *s) {
    if (!s->active) return false;

    /* ISO14443-4 presence check: R(NAK) with the CURRENT block number. A card
     * still in the field answers with an R-block; a removed one answers nothing.
     * This is the one interrogation that is transparent to a live session --
     * unlike WUPA it does not reset the card, and unlike an I-block it does not
     * advance the block number, so s->blk is deliberately left untouched. */
    uint8_t rnak[3];
    /* Must fit a full I-BLOCK, not a 3-byte R(ACK): per ISO14443-4 rule 12 a
     * PICC receiving R(NAK) carrying its own block number RE-TRANSMITS its last
     * I-block. Sized for an R(ACK), the driver rejected the reply as an overflow
     * (STATUS_HF_ERR_STAT), so a card that answered perfectly read as absent --
     * deterministically, defeating any miss-hysteresis. */
    uint8_t rbuf[288];
    uint8_t crc[2];

    rnak[0] = (uint8_t)(0xB2u | (s->blk & 0x01u));   /* R(NAK), block = s->blk */
    crc_14a_append(rnak, 1);

    write_register_single(ComIrqReg, 0x7F);          /* clear stale RxIRq */
    pcd_14a_reader_timeout_set(HF14A_4_PRESENCE_TIMEOUT_MS);
    uint16_t rbits = 0;
    uint8_t st = pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, rnak, 3,
                                               rbuf, &rbits, U8ARR_BIT_LEN(rbuf));
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
    if (st != STATUS_HF_TAG_OK || rbits < 24u) return false;

    uint16_t rb = rbits / 8u;
    crc_14a_calculate(rbuf, rb - 2u, crc);
    if (rbuf[rb - 2] != crc[0] || rbuf[rb - 1] != crc[1]) return false;

    /* ANY well-formed answer proves the card is still in the field -- do not
     * require an R-block. Per ISO14443-4 rule 12 a PICC that receives R(NAK)
     * carrying its OWN current block number re-transmits its last I-block
     * instead of acknowledging, so demanding an R-block here reports a
     * perfectly healthy card as gone (observed: presence flapping ~1 s on a
     * stationary card, tearing down the session every poll). We deliberately
     * ignore the payload and leave s->blk untouched: the card's block number
     * does not move for either reply, so the session stays consistent. */
    return true;
}

void hf14a_4_field_off(void) {
    /* The ONLY antenna-off entry point above rc522. Switching the field off is a
     * POWER decision (disable, radio hold, USB suspend, cable pull, de-config),
     * never a presence decision -- see hf14a_4_session_close(). */
    pcd_14a_reader_antenna_off();
}

void hf14a_4_session_close(hf14a_4_session_t *s) {
    if (s->active) {
        /* S(DESELECT): move the card from PROTOCOL to HALT, and LEAVE THE FIELD
         * ON. This matters more than it looks. A RATS'd card sits in PROTOCOL
         * state, where it ignores WUPA -- so if we merely deactivated the
         * session and kept the field, the still-present card would never answer
         * the tier-1 probe again and would be invisible until physically
         * removed. The previous code avoided that by dropping the field, which
         * power-cycles the card back to IDLE -- but that is what made every
         * false teardown reset the card mid-chain (the source of the 91 1C
         * responses) and left the next WUPA running on a cold field with only
         * the ISO minimum settle. HALTed cards DO answer WUPA, which is exactly
         * why the probe uses WUPA rather than REQA. */
        uint8_t desel[3] = { 0xC2 };          /* S(DESELECT) request */
        uint8_t rb[8];
        uint16_t bits = 0;
        crc_14a_append(desel, 1);
        write_register_single(ComIrqReg, 0x7F);
        pcd_14a_reader_timeout_set(HF14A_4_PRESENCE_TIMEOUT_MS);
        (void)pcd_14a_reader_bytes_transfer(PCD_TRANSCEIVE, desel, 3,
                                            rb, &bits, U8ARR_BIT_LEN(rb));
        pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
        /* Response ignored: an absent card simply times out, and either way the
         * card is no longer in PROTOCOL. */
    }
    s->active = false;
    s->blk = 0;
}

#endif /* PROJECT_CHAMELEON_ULTRA */

