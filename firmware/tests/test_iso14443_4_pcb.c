/* Host unit test for iso14443_4_pcb.h. No hardware. Encodes the values that
 * were actually got wrong in this codebase, so a regression fails here first. */
#include "iso14443_4_pcb.h"
#include <stdio.h>
static int fail = 0, pass = 0;
#define CHECK(expr, msg) do { if (expr) pass++; else { fail++; \
    printf("  FAIL: %s  (%s)\n", msg, #expr); } } while (0)

int main(void) {
    printf("iso14443_4_pcb.h unit tests\n\n");

    printf("field bits (the ones defined one bit high in emulation):\n");
    CHECK(ISO14443_4_PCB_CHAINING == 0x10, "chaining is 0x10 not 0x20");
    CHECK(ISO14443_4_PCB_CID      == 0x08, "CID is 0x08 not 0x10");
    CHECK(ISO14443_4_PCB_NAD      == 0x04, "NAD is 0x04 not 0x08");
    CHECK(ISO14443_4_PCB_S_WTX    == 0xF2, "S(WTX) is 0xF2 not 0x30");

    printf("block types are disjoint; 01xxxxxx is RFU and must classify as none:\n");
    for (int p = 0; p < 256; p++) {
        int n = iso14443_4_is_iblock((uint8_t)p)
              + iso14443_4_is_rblock((uint8_t)p)
              + iso14443_4_is_sblock((uint8_t)p);
        int rfu = ((p & 0xC0) == 0x40);
        int want = rfu ? 0 : 1;
        if (n != want) {
            printf("  FAIL: PCB %02X classified %d ways, expected %d\n", p, n, want);
            fail++;
        }
        if (iso14443_4_is_valid_pcb((uint8_t)p) == (rfu != 0)) {
            printf("  FAIL: PCB %02X validity wrong\n", p); fail++;
        }
    }
    pass++;
    CHECK(!iso14443_4_is_valid_pcb(0x40), "RFU block rejected");
    CHECK(iso14443_4_is_valid_pcb(0x02),  "I-block accepted");
    CHECK(iso14443_4_is_valid_pcb(0xF2),  "S(WTX) accepted");

    printf("canonical values classify correctly:\n");
    CHECK(iso14443_4_is_iblock(0x02), "0x02 is an I-block");
    CHECK(iso14443_4_is_iblock(0x03), "0x03 is an I-block");
    CHECK(iso14443_4_is_rack(0xA2),   "0xA2 is R(ACK)");
    CHECK(iso14443_4_is_rnak(0xB2),   "0xB2 is R(NAK)");
    CHECK(!iso14443_4_is_rack(0xB2),  "R(NAK) is not R(ACK)");
    CHECK(!iso14443_4_is_rnak(0xA2),  "R(ACK) is not R(NAK)");
    CHECK(iso14443_4_is_deselect(0xC2), "0xC2 is S(DESELECT)");
    CHECK(iso14443_4_is_wtx(0xF2),      "0xF2 is S(WTX)");

    printf("the two bugs that actually shipped:\n");
    /* chaining tested as 0x20 matched S(WTX) but never a chained I-block */
    CHECK(iso14443_4_has_chaining(0x12),  "chained I-block 0x12 detected");
    CHECK(iso14443_4_has_chaining(0x13),  "chained I-block 0x13 detected");
    CHECK(!iso14443_4_has_chaining(0x02), "unchained I-block not flagged");
    CHECK(!iso14443_4_has_chaining(0xA2), "R(ACK) is not a chained I-block");
    CHECK(!iso14443_4_has_chaining(0xF2), "S(WTX) is not a chained I-block");
    /* S(WTX) with CID: the 0x3F mask failed this, 0xF7 handles it */
    CHECK(iso14443_4_is_wtx(0xFA),      "S(WTX) with CID still matches");
    CHECK(iso14443_4_is_deselect(0xCA), "S(DESELECT) with CID still matches");

    printf("header length:\n");
    CHECK(iso14443_4_hdr_len(0x02) == 1, "plain I-block: 1");
    CHECK(iso14443_4_hdr_len(0x0A) == 2, "I-block + CID: 2");
    CHECK(iso14443_4_hdr_len(0x0E) == 3, "I-block + CID + NAD: 3");
    CHECK(iso14443_4_hdr_len(0xAA) == 2, "R(ACK) + CID: 2");

    printf("frame sizes (Table 4):\n");
    CHECK(iso14443_4_frame_size(4) == 48,  "FSDI 4 -> 48 (our reader's RATS)");
    CHECK(iso14443_4_frame_size(8) == 256, "FSCI 8 -> 256");
    CHECK(iso14443_4_frame_size(0) == 16,  "FSDI 0 -> 16");
    CHECK(iso14443_4_frame_size(15) == 256, "RFU -> 256, never 0");
    /* the arithmetic behind "a card chunks at 45 bytes with our FSD of 48" */
    CHECK(iso14443_4_max_payload(48, false, false) == 45, "FSD 48 -> 45 payload");
    CHECK(iso14443_4_max_payload(48, true, false)  == 44, "with CID -> 44");
    CHECK(iso14443_4_max_payload(16, false, false) == 13, "FSD 16 -> 13");

    printf("\n%d passed, %d failed\n", pass, fail);
    if (!fail) printf("87-byte PPSE at FSD 48 = %d + %d = 2 blocks (even) -- "
                      "the case that desynced the session\n",
                      45, 87 - 45);
    return fail ? 1 : 0;
}
