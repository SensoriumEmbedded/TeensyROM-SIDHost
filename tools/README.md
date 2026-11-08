# SID host: native checks

The TR+ SID player host (`Source/Teensy/SIDHost/`) plays a `.sid` by emulating
the tune on the Teensy and replaying its SID register writes to the real chip.
The emulated machine is plain C++ with no Arduino dependencies, so the same
files build on a PC and can be checked against a whole SID collection before
anything touches a board.

## The harness

`Source/Teensy/tests/sidhost_harness.cpp` loads a `.sid`, runs it in the
emulated machine and prints one row per play call: the row number and the 25
SID registers as the tune last wrote them.

```
c++ -std=gnu++17 -O2 -I Source/Teensy/SIDHost -o harness Source/Teensy/tests/sidhost_harness.cpp Source/Teensy/SIDHost/SidMachine.cpp
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

`Source/Teensy/tests/sidhost.test.js` builds this harness and drives it with a
PSID assembled in the test (part of `npm test`; skipped when no `c++` is on
PATH).

## The corpus check

`corpus-check.mjs` runs the harness over a SID collection (HVSC's `C64Music`
directory) and reports what did not play:

```
node tools/sidhost/corpus-check.mjs --hvsc <C64Music> --sample 2000 --seconds 10
node tools/sidhost/corpus-check.mjs --hvsc <C64Music> --siddump <siddump exe> --filter PSID --out report.json
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
