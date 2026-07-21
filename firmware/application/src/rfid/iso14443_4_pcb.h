#ifndef ISO14443_4_PCB_H
#define ISO14443_4_PCB_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ISO/IEC 14443-4 block layer: PCB encoding, block predicates and frame sizes.
 *
 * WHY THIS EXISTS
 * ---------------
 * These values were previously open-coded as hex literals in every
 * implementation -- the three reader paths and the card emulation -- with no
 * shared definition to agree on. They drifted, and the drift was expensive:
 *
 *   - The I-block chaining bit was tested as 0x20 in three separate places
 *     while being SET as 0x10 in the same functions, so every card response
 *     longer than one frame was silently truncated and the last two payload
 *     bytes were handed to the host as a status word.
 *   - The emulation layer defined CID, NAD and chaining each one bit high, and
 *     S(WTX) as 0x30, which is not a valid S-block at all.
 *
 * Neither survived because the code was complicated. They survived because
 * nothing in the tree stated what a PCB byte is.
 *
 * DERIVATION (ISO 14443-4 Table 3)
 * --------------------------------
 * Anchored on the values that are not in dispute -- I-block 0x02, R(ACK) 0xA2,
 * R(NAK) 0xB2, S(DESELECT) 0xC2 -- the I-block layout is:
 *
 *      b8 b7 b6 b5 b4 b3 b2 b1
 *       0  0  0  C  D  N  1  B
 *
 *   b8b7 = 00 marks an I-block (10 = R-block, 11 = S-block)
 *   b5 (0x10) = C: chaining, more blocks follow
 *   b4 (0x08) = D: CID field follows the PCB
 *   b3 (0x04) = N: NAD field follows
 *   b2 (0x02) = fixed 1 for I-blocks
 *   b1 (0x01) = B: block number
 *
 * R-blocks are 1010 00xB (ACK) / 1011 00xB (NAK); the CID bit is b4 as above.
 * S-blocks are 11xx xx10, with b6b5 selecting the type: 00 = DESELECT (0xC2),
 * 11 = WTX (0xF2).
 */

/* ---- block type identification ------------------------------------------ */
#define ISO14443_4_PCB_TYPE_MASK    0xC0u
#define ISO14443_4_PCB_TYPE_I       0x00u
#define ISO14443_4_PCB_TYPE_R       0x80u
#define ISO14443_4_PCB_TYPE_S       0xC0u

/* ---- field bits (identical across block types) --------------------------- */
#define ISO14443_4_PCB_BLOCK_NUM    0x01u   /* b1: block number               */
#define ISO14443_4_PCB_NAD          0x04u   /* b3: NAD follows (I-blocks only) */
#define ISO14443_4_PCB_CID          0x08u   /* b4: CID follows                */
#define ISO14443_4_PCB_CHAINING     0x10u   /* b5: more blocks follow          */

/* ---- canonical block values (no CID, no NAD, block number 0) ------------- */
#define ISO14443_4_PCB_I_BLOCK      0x02u
#define ISO14443_4_PCB_R_ACK        0xA2u
#define ISO14443_4_PCB_R_NAK        0xB2u
#define ISO14443_4_PCB_S_DESELECT   0xC2u
#define ISO14443_4_PCB_S_WTX        0xF2u

/* R(NAK) differs from R(ACK) by b5, which in R-blocks means "negative", not
 * chaining -- the bit position is shared, the meaning is not. */
#define ISO14443_4_PCB_R_NAK_BIT    0x10u

/* Mask off CID and block number to compare a received S-block against the
 * canonical value: S(WTX) with CID is 0xFA, and (0xFA & 0xF7) == 0xF2.
 * Masking with 0x3F instead drops the type bits and keeps the CID bit, so
 * S(WTX) with a CID fails to match -- a real defect this mask prevents. */
#define ISO14443_4_PCB_S_TYPE_MASK  0xF7u

/* ---- predicates ---------------------------------------------------------- */
static inline bool iso14443_4_is_iblock(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_TYPE_MASK) == ISO14443_4_PCB_TYPE_I;
}
static inline bool iso14443_4_is_rblock(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_TYPE_MASK) == ISO14443_4_PCB_TYPE_R;
}
static inline bool iso14443_4_is_sblock(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_TYPE_MASK) == ISO14443_4_PCB_TYPE_S;
}
/* b8b7 == 01 is not assigned by ISO 14443-4: only 00 (I), 10 (R) and 11 (S)
 * exist. Such a block is malformed and must be rejected rather than silently
 * falling through every type test, which is what the open-coded predicates did.
 * Surfaced by the host unit test asserting classification was total -- it is
 * not, and that is the point. */
static inline bool iso14443_4_is_valid_pcb(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_TYPE_MASK) != 0x40u;
}

static inline bool iso14443_4_is_rnak(uint8_t pcb) {
    return iso14443_4_is_rblock(pcb) && (pcb & ISO14443_4_PCB_R_NAK_BIT) != 0u;
}
static inline bool iso14443_4_is_rack(uint8_t pcb) {
    return iso14443_4_is_rblock(pcb) && (pcb & ISO14443_4_PCB_R_NAK_BIT) == 0u;
}
static inline bool iso14443_4_is_wtx(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_S_TYPE_MASK) == ISO14443_4_PCB_S_WTX;
}
static inline bool iso14443_4_is_deselect(uint8_t pcb) {
    return (pcb & ISO14443_4_PCB_S_TYPE_MASK) == ISO14443_4_PCB_S_DESELECT;
}
static inline bool iso14443_4_has_chaining(uint8_t pcb) {
    return iso14443_4_is_iblock(pcb) && (pcb & ISO14443_4_PCB_CHAINING) != 0u;
}
static inline uint8_t iso14443_4_blocknum(uint8_t pcb) {
    return (uint8_t)(pcb & ISO14443_4_PCB_BLOCK_NUM);
}

/* Header length implied by a PCB: the PCB itself plus any CID and NAD bytes. */
static inline uint8_t iso14443_4_hdr_len(uint8_t pcb) {
    uint8_t n = 1u;
    if (pcb & ISO14443_4_PCB_CID) n++;
    if (iso14443_4_is_iblock(pcb) && (pcb & ISO14443_4_PCB_NAD)) n++;
    return n;
}

/* ---- frame sizes (ISO 14443-4 Table 4) ----------------------------------- */
/* FSCI/FSDI nibble -> frame size in bytes, INCLUDING PCB and CRC. Indices 9-14
 * are RFU and 15 is reserved; ISO says treat unknown values as 256. */
static const uint16_t ISO14443_4_FRAME_SIZE[16] = {
    16, 24, 32, 40, 48, 64, 96, 128, 256,
    256, 256, 256, 256, 256, 256, 256
};

static inline uint16_t iso14443_4_frame_size(uint8_t fsdi_or_fsci) {
    return ISO14443_4_FRAME_SIZE[fsdi_or_fsci & 0x0Fu];
}

/* Payload capacity of one frame: frame size less PCB and CRC, and less the CID
 * and NAD bytes when those are in use. */
static inline uint16_t iso14443_4_max_payload(uint16_t frame_size, bool cid, bool nad) {
    uint16_t o = (uint16_t)(3u + (cid ? 1u : 0u) + (nad ? 1u : 0u));
    return (frame_size > o) ? (uint16_t)(frame_size - o) : 1u;
}

#endif /* ISO14443_4_PCB_H */
