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

#if defined(PROJECT_CHAMELEON_ULTRA)

#include "rfid_main.h"
#include "bsp_delay.h"
#include "rfid/reader/hf/rc522.h"

#define TCL_RESP_TIMEOUT_MS     600
#define TCL_MAX_CMD_APDU        250   /* one-frame command APDU (FU-02 chains) */

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
    s->active = true;                  /* leave field ON for APDU exchange     */
    return true;
}

bool hf14a_4_session_apdu(hf14a_4_session_t *s,
                          const uint8_t *apdu, uint16_t apdu_len,
                          uint8_t *resp, uint16_t *resp_len, uint16_t resp_max) {
    *resp_len = 0;
    if (!s->active || apdu_len == 0 || apdu_len > TCL_MAX_CMD_APDU) return false;

    uint8_t abuf[3 + TCL_MAX_CMD_APDU];  /* PCB + APDU + CRC */
    uint8_t rbuf[288];
    uint8_t crc[2];

    /* Build I-block: PCB(I, block_num) + APDU + CRC. */
    abuf[0] = (uint8_t)(0x02 | (s->blk & 0x01));
    memcpy(&abuf[1], apdu, apdu_len);
    crc_14a_append(abuf, apdu_len + 1);
    uint8_t frame_len = (uint8_t)(apdu_len + 3);

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

    s->blk ^= 1;
    uint8_t  resp_pcb = rbuf[0];
    uint16_t out = 0;
    uint8_t  dlen = (uint8_t)(rb - 3u);        /* strip PCB + CRC */
    if (dlen > 0 && out + dlen <= resp_max) {
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
                dlen = (uint8_t)(rb - 3u);
                if (dlen > 0 && out + dlen <= resp_max) {
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
        dlen = (uint8_t)(rb - 3u);
        if (dlen > 0 && out + dlen <= resp_max) {
            memcpy(&resp[out], &rbuf[1], dlen);
            out += dlen;
        }
    }

    *resp_len = out;
    return out > 0;
}

bool hf14a_4_presence(void) {
    reader_mode_enter();               /* idempotent: no-op if already reader  */
    /* Field-cycle so a previously-selected/halted card returns to IDLE and
     * answers REQA again (avoids presence flapping between polls). */
    pcd_14a_reader_antenna_off();
    bsp_delay_ms(2);
    pcd_14a_reader_antenna_on();
    bsp_delay_ms(3);
    pcd_14a_reader_timeout_set(100);
    picc_14a_tag_t tag;
    uint8_t status = pcd_14a_reader_scan_auto(&tag);
    pcd_14a_reader_timeout_set(DEF_COM_TIMEOUT);
    return (status == STATUS_HF_TAG_OK) && (tag.sak & 0x20);
}

void hf14a_4_session_close(hf14a_4_session_t *s) {
    if (s->active) {
        pcd_14a_reader_antenna_off();
    }
    s->active = false;
    s->blk = 0;
}

#else  /* !PROJECT_CHAMELEON_ULTRA */

/* Lite has no MFRC522 HF reader: stubs so the CCID slot links but never
 * activates a card. CCID is not exposed on Lite anyway (see usb_main.c). */
bool hf14a_4_session_open(hf14a_4_session_t *s) { (void)s; return false; }
bool hf14a_4_presence(void) { return false; }
bool hf14a_4_session_apdu(hf14a_4_session_t *s, const uint8_t *apdu, uint16_t apdu_len,
                          uint8_t *resp, uint16_t *resp_len, uint16_t resp_max) {
    (void)s; (void)apdu; (void)apdu_len; (void)resp; (void)resp_max;
    if (resp_len) *resp_len = 0;
    return false;
}
void hf14a_4_session_close(hf14a_4_session_t *s) { (void)s; }

#endif /* PROJECT_CHAMELEON_ULTRA */
