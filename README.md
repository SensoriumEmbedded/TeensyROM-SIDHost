# TeensyROM SIDHost

A SID player for the TeensyROM+, built as an *extension host*: a third firmware
image that lives in its own flash slot and is installed and removed without
reflashing the board. It plays a `.sid` by emulating the tune on the Teensy
(MOS 6502 and 6526 cores from floooh's chips, zlib) and replaying each SID write
to the real chip by DMA, on the bus cycle the emulated machine made it. The real
C64 runs only a small cartridge stub that passes keys to the host.

Status: Phases 0-2 are done (emulation checked against siddump over the HVSC
collection, cycle-scheduled playback, subsong and pause keys, bus capture
comparison against a real 6510). The plan's later phases (RSID and BASIC tunes,
visualizations) are open.

## Layout

| Path | What |
|---|---|
| `host/` | The host sketch: `SIDHost.ino`, the emulated machine (`SidMachine`), the bus player (`SidBus.h`) and the vendored chip cores. Flat, because the core's `--host-sketch` overlay does not descend into directories. |
| `stub/` | The C64 half: an 8 KB cartridge (ACME). Its header, `host/SIDHostStub.bin.h`, is committed. |
| `tests/` | The native harness and its tests. Plain C++17, no Arduino. |
| `tools/` | `build-host.mjs`, `build-stub.mjs`, the HVSC corpus checker and its README. |

## Depends on the TeensyROM core

The host is overlaid onto the core's minimal sketch and includes its `Common/`
headers (`ISRs.c`, `DMAControl.c`, `VMRegistry.h` and others), so a build needs a
core checkout. `CORE_PIN` names the core commit this was last verified against:
the tip of a core branch, `sid-host-hook`, which is `main` plus one commit.
Launching a tune from the menu (Shift+Return on a `.sid`) needs a core firmware
with that key, which is not in the core's `main`.

```
set TR_CORE=C:\path\to\TeensyROM
npm run build:host -- --out C:/tr-build/sidhost-fw     # writes the .TRH
npm run build:stub                                      # only after changing stub/
npm test                                                # native harness tests
```

Needs Node 24 and a `c++` on PATH for the tests. The `.TRH` installs like any
extension host; see the core's `docs/Architecture/Extension-Hosts.md`.

## License

MIT, see `LICENSE.md`. The vendored `m6502.h` and `m6526.h` are zlib-licensed
and carry their own notices.
