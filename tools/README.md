# SID host: native checks

The TR+ SID player host (`host/`) plays a `.sid` by emulating
the tune on the Teensy and replaying its SID register writes to the real chip.
The emulated machine is plain C++ with no Arduino dependencies, so the same
files build on a PC and can be checked against a whole SID collection before
anything touches a board.

## The harness

`tests/sidhost_harness.cpp` loads a `.sid`, runs it in the
emulated machine and prints one row per play call: the row number and the 25
SID registers as the tune last wrote them.

```
c++ -std=gnu++17 -O2 -I host -o harness tests/sidhost_harness.cpp host/SidMachine.cpp
./harness some.sid -t 60            # 60 s of rows, start song
./harness some.sid -a 2 -n 500      # song 3 (0-based), 500 rows
./harness some.sid --rows frames    # a row per video frame instead of per play call
./harness some.sid --trace          # every SID write as "cycle address value"
./harness some.sid --io-reads       # every read of $D000-$DFFF and $00/$01, with the PC that made it
```

The first two lines are comments with the parsed header (format, load range,
init and play addresses, clock, speed, the page the driver took) and the
credits. A tune whose init installs its own interrupt handler is never called
through the driver; after 50 frames with writes but no calls the harness says
so and switches to a row per frame.

When a tune stops producing rows, the harness prints where the CPU is (PC,
stack, `$01`, the IRQ vector, the VIC and CIA interrupt state and the bytes at
the PC) so the cause can be read off without a debugger.

`tests/sidhost.test.js` builds this harness and drives it with a
PSID assembled in the test (part of `npm test`; skipped when no `c++` is on
PATH).

## The corpus check

`corpus-check.mjs` runs the harness over a SID collection (HVSC's `C64Music`
directory) and reports what did not play:

```
node tools/corpus-check.mjs --hvsc <C64Music> --sample 2000 --seconds 10
node tools/corpus-check.mjs --hvsc <C64Music> --siddump <siddump exe> --filter PSID --out report.json
```

Without `--siddump` it is a smoke pass: `unsupported` (header rejected, or a
C64 BASIC tune), `no-play` (the driver never reached play), `slow` (play spends
frames in delay loops, so fewer rows arrived than asked), `silent` (every row
all zero), `overflow` (more writes in a frame than the capture buffer holds),
`crash`. With `--siddump` each tune's rows are compared field by field with
[siddump](https://github.com/cadaver/siddump)'s table (frequency, waveform,
ADSR and pulse per voice; filter cutoff, routing, type and volume), one row per
play call on both sides; `near` means under 5% of rows differ, `mismatch` more.
A mismatch is then retried in ways that tell a reference limitation from an
emulation defect, and reported under that class instead: `reads-timing` (the
tune matches once VIC, CIA and SID registers read back their written values,
as siddump's flat memory does), `shift-1` (the tune's init enables interrupts,
so its first play call lands before init returns), `own-irq` (play address 0:
siddump calls the vector once per frame whatever the tune's timer does),
`mirror-writes` (the tune writes the SID through `$D420-$D7FF`, which the chip
answers to) and `reads-port` (the tune reads `$00`/`$01`, `$2F`/`$37` on a C64
and zero in a flat memory). What is left under `mismatch` is unexplained.

`--sample N --seed S` picks the same N tunes on every run; `--list` takes a file
of collection-relative paths instead, for rechecking a report's failures;
`--explain <relative path>` prints both tables around the first rows that
differ; `--harness-arg <flag>` passes an option through to the harness (for
instance `--harness-arg --ntsc`).

`HVSC_DIR` stands in for `--hvsc`. The collection itself stays out of the repo.

## Capturing a real C64

Comparing audio with a native run cannot show the order of a tune's register
writes inside a play call. The host's capture mode can: on the SIDHost screen,
**C** plays the current song on the real 6510 and logs every SID write with its
bus cycle; **Shift+C** does the same with the screen left on.

The C64 gets the same RAM image the emulator runs (the tune, the driver page and
its vectors), and a short prologue in the driver page waits for the bottom border
and sets the CIAs and the VIC up as the emulator does. With the screen off the VIC takes no badlines, so the
spacing of the writes can match the emulator's cycle for cycle; with it on, every
write the VIC delays shows how far. The capture runs for 60 seconds or until its
buffer is full (46,940 writes: a minute of most tunes, 16 seconds of a 4x CIA
player), writes `<tune>.s<song>.<pal|ntsc>.cap` (`.on.cap` for a screen-on one)
beside the `.sid` on the SD card, and resets the C64 back to the SIDHost screen,
which shows the file's name. A tune that loads below `$0400` or under the I/O area
cannot be captured: the stub and the I/O chips sit there.

Compare it with the emulation on a PC:

```
./harness some.sid --compare some.s1.pal.cap
```

The harness emulates the captured song on the captured machine's standard and
pairs the two logs play call by play call (the driver marks each play call's
return), then call by call inside each. It reports how many calls match exactly
(the same writes, in the same order, at the same offsets from the call's first),
how many differ in their writes, how many only in their offsets (with a
histogram), how often a call inside a play call starts at another offset, and the
play period on both sides, and prints the first ten differences with both sides.

Differences that are not defects: a tune that reads OSC3 or ENV3 (the emulator
answers fixed values); a tune that times itself off the raster or a CIA timer,
which the prologue starts at raster line $FA on both sides but only to within its
wait loop's 9 cycles; a tune that reads
memory before writing it (the KERNAL's leftovers on the C64, zeros in the
emulator); a tune that reads chip state the prologue leaves alone, such as CIA 1's
port directions and timer B or CIA 2's timers (the KERNAL's settings on the C64, a
reset's in the emulator); a play routine that leaves through `JMP $EA81` (the
emulator unwinds it, the real KERNAL does not); a tune that writes `$D400-$D7FF`
or `$DFFF` with the I/O area banked out, which reaches RAM, but the cartridge
port cannot see the banking, so the capture logs it as a SID write or a play
marker; a second or third SID at `$DFE0`, whose writes look like the driver's
marker.

`--write-cap <file>` writes a capture from the emulator itself (with `-n` frames
and `--blank` for a screen-off one), which is what the tests compare against.
