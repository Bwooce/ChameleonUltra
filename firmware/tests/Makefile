# Host-side unit tests. No hardware, no toolchain -- just `make` and `make test`.
#
# These exist because every ISO 14443-4 block-layer defect found so far was a
# constant or a mask that no test could reach: the chaining bit tested as 0x20
# while being set as 0x10, and CID/NAD/chaining each defined one bit high in the
# emulation layer. Those are pure functions of a byte and are cheap to pin down
# here, where a failure costs seconds rather than a flash cycle and a card.

CFLAGS = -std=c99 -Wall -Wextra -I../application/src/rfid
BINS   = test_iso14443_4_pcb

all: $(BINS)

test_iso14443_4_pcb: test_iso14443_4_pcb.c ../application/src/rfid/iso14443_4_pcb.h
	$(CC) $(CFLAGS) -o $@ $<

test: $(BINS)
	@for t in $(BINS); do echo "== $$t"; ./$$t || exit 1; done

clean:
	rm -f $(BINS)

.PHONY: all test clean
