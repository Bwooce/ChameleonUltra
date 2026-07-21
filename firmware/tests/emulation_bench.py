#!/usr/bin/env python3
"""Hardware bench for the ISO 14443-4 card emulation layer.

WHY THIS EXISTS
---------------
Every defect found in nfc_14a_4.c so far was found by reading, not running --
because nothing in the tree could run it. The Chameleon cannot be both reader
and card, so exercising emulation needs a SECOND reader. This drives one over
PC/SC and checks the specific failures that were shipping undetected.

SETUP
  1. A second PC/SC reader (ACR122U or similar) plugged into this machine.
  2. The Chameleon in tag-emulation mode with a 14443-4 tag in the active slot.
  3. The Chameleon placed on that reader's antenna.
  4. pip3 install pyscard

  The Chameleon's own CCID reader also appears in the PC/SC reader list. This
  script skips any reader whose name contains "Chameleon" -- we need the tag on
  the OTHER reader, not the Chameleon reading something.

WHAT EACH CHECK TARGETS
  Every check names the defect it would have caught. A check that passes on the
  first run proves nothing was broken; a check that FAILS is the interesting
  case, and the message says which fix regressed.
"""
import sys

try:
    from smartcard.System import readers
    from smartcard.util import toHexString
    from smartcard.Exceptions import CardConnectionException, NoCardException
except ImportError:
    sys.exit("pyscard not installed:  pip3 install pyscard")

PASS, FAIL, SKIP = [], [], []


def check(name, ok, detail="", targets=""):
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if detail:
        print(f"         {detail}")
    if not ok and targets:
        print(f"         targets: {targets}")


def skip(name, why):
    SKIP.append(name)
    print(f"  [SKIP] {name}\n         {why}")


def pick_reader():
    rs = readers()
    if not rs:
        sys.exit("no PC/SC readers at all -- is the second reader plugged in?")
    print("readers seen:")
    for r in rs:
        print(f"  - {r}")
    ext = [r for r in rs if "chameleon" not in str(r).lower()]
    if not ext:
        sys.exit("\nonly the Chameleon's own reader is present.\n"
                 "This bench needs a SECOND reader to talk TO the Chameleon.")
    if len(ext) > 1:
        print(f"\nmultiple external readers; using {ext[0]}")
    return ext[0]


def main():
    rdr = pick_reader()
    print(f"\nusing: {rdr}\n")
    conn = rdr.createConnection()
    try:
        conn.connect()
    except (NoCardException, CardConnectionException) as e:
        sys.exit(f"no tag on the antenna ({e}).\n"
                 "Put the Chameleon in emulation mode on the reader.")

    atr = conn.getATR()
    print(f"ATR: {toHexString(atr)}\n")

    print("== activation ==")
    check("ATR is well-formed",
          len(atr) >= 2 and atr[0] in (0x3B, 0x3F),
          f"{len(atr)} bytes",
          "RATS/ATS handling in nfc_14a.c")

    # TC(1) should no longer advertise CID support. The ATR is derived from the
    # ATS, so this is indirect, but a CID-advertising tag that cannot answer
    # with a CID field is the S7 defect.
    print("\n== S1: frame length was a bit count used as a byte count ==")
    print("   Before the fix every relayed APDU was ~8x too long and padded with")
    print("   RX-buffer garbage. A sane response here is the primary signal.")
    try:
        r, sw1, sw2 = conn.transmit([0x00, 0xA4, 0x04, 0x00, 0x00])
        check("SELECT returns a plausible response",
              sw1 in (0x90, 0x6A, 0x6D, 0x62, 0x63, 0x69, 0x67),
              f"SW={sw1:02X}{sw2:02X} data={toHexString(r) if r else '(none)'}",
              "S1 frame length, or the static-response table")
        check("response length is sane (not ~8x inflated)",
              len(r) <= 256,
              f"{len(r)} bytes",
              "S1 frame length")
    except CardConnectionException as e:
        check("SELECT completes", False, str(e), "S1, or the double-TX in the WTX branch")

    print("\n== block-number discipline across one session ==")
    print("   The reader-side mirror of this bug desynced the session after an")
    print("   even-length chain and the card then repeated its previous answer.")
    try:
        r1, s11, s12 = conn.transmit([0x00, 0xA4, 0x04, 0x00, 0x00])
        r2, s21, s22 = conn.transmit([0x00, 0xCA, 0x00, 0x00, 0x00])
        r3, s31, s32 = conn.transmit([0x00, 0xA4, 0x04, 0x00, 0x00])
        distinct = not (r1 == r2 and s11 == s21 and s12 == s22)
        check("second APDU is not a repeat of the first", distinct,
              f"1st SW={s11:02X}{s12:02X} 2nd SW={s21:02X}{s22:02X}",
              "send_rack() toggle order / m_block_num handling")
        check("session still usable on a third APDU",
              (s31, s32) == (s11, s12),
              f"3rd SW={s31:02X}{s32:02X}",
              "block-number desync")
    except CardConnectionException as e:
        check("three APDUs on one session", False, str(e),
              "block-number desync, or the WTX double-TX")

    print("\n== S10: unbounded WTX ping-pong ==")
    print("   send_wtx() is the DEFAULT branch for any APDU with no configured")
    print("   static response, and nothing caps consecutive rounds.")
    try:
        conn.transmit([0x00, 0xFF, 0xFF, 0xFF, 0x00])
        check("unconfigured APDU terminates instead of stalling", True,
              "card answered rather than looping WTX")
    except CardConnectionException as e:
        check("unconfigured APDU terminates", False, f"{e}",
              "S10 WTX cap, or the WTX double-TX")

    print("\n== card->reader chaining (KNOWN NOT IMPLEMENTED) ==")
    fsd = 256  # most PC/SC readers
    print(f"   This reader almost certainly advertises FSD {fsd}, so forcing")
    print(f"   chaining needs a configured response over ~{fsd - 3} bytes.")
    skip("large-response chaining",
         "configure a static response >253 bytes on the Chameleon first; "
         "until card->reader chaining is implemented this is EXPECTED to fail, "
         "and the firmware log should carry the 'exceeds reader FSD' warning")

    print("\n== S4: R(NAK) recovery ==")
    skip("R(NAK) retransmission",
         "not reachable from the PC/SC API -- the reader's own stack owns the "
         "block layer. Needs a second Chameleon (its presence check sends "
         "R(NAK) directly) or a reader with raw transceive")

    print("\n" + "=" * 60)
    print(f"pass {len(PASS)}   fail {len(FAIL)}   skip {len(SKIP)}")
    if SKIP:
        print("SKIPPED IS NOT PASSED -- see each note for what it needs.")
    conn.disconnect()
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
