// SPDX-License-Identifier: MIT
//
// The C64 a SID tune plays in, emulated: a cycle-stepped 6510 (m6502.h), two
// CIAs (m6526.h), the VIC-II's raster counter and raster interrupt, the memory
// banking, and a SID that records every register write with its cycle. The
// recorded writes are what the host replays to the real SID chip by DMA, and
// the register image is what the visualizer reads.
//
// Phase 1 is the PSID environment from HVSC's SID_file_format.txt: a small
// driver in a free page of C64 RAM calls init and then play from the raster or
// CIA interrupt, and an 8 KB pseudo-KERNAL supplies only the vectors and the
// interrupt entry/exit the driver and the tunes expect. No ROM image ships.
//
// Plain C++17, no Arduino dependencies; the native harness in Source/Teensy/tests/
// compiles the same files. The caller owns every memory block (on the Teensy they
// live in the RAM2 arena), so this class holds pointers only.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "SidFile.h"
#include "m6502.h"
#include "m6526.h"

struct SidWrite
{
   uint32_t cycle;   // machine cycle of the write, from the SidMachine's running counter
   uint16_t addr;    // the full address written ($D400-$D7FF, or a header-named second/third SID)
   uint8_t value;
};

struct SidRead { uint32_t cycle; uint16_t pc, addr; uint8_t value; };  // an I/O or processor-port read, for the harness

struct SidMachineMem
{
   uint8_t *ram;     // 64 KB
   uint8_t *kernal;  // 8 KB, $E000-$FFFF
   uint8_t *basic;   // 8 KB, $A000-$BFFF
   uint8_t *color;   // 1 KB, $D800-$DBFF
};

class SidMachine
{
public:
   static constexpr uint32_t PAL_CYCLES_PER_LINE = 63, PAL_LINES = 312;
   static constexpr uint32_t NTSC_CYCLES_PER_LINE = 65, NTSC_LINES = 263;
   static constexpr uint16_t PAL_TIMER = 0x4025, NTSC_TIMER = 0x4295;  // 60 Hz CIA periods the format description names
   static constexpr uint16_t DRIVER_MARKER = 0xDFFF;                    // the driver stores here when play returns; a tune's own write there is a SID 2/3 write

   void init(const SidMachineMem &mem, bool ntsc);
   bool load(const SidHeader &h);     // remembers the tune and picks the driver page; false if no page is free
   void start(uint8_t song0);         // cold start: clears RAM, loads the tune, runs the driver for song0 (0-based)
   // The same cold start as a real C64 gets it when the host hands it this RAM image
   // (SIDHost's capture mode): the CIAs and the VIC as a reset leaves them, and the
   // CPU entering through a prologue in the driver page that sets them up the way
   // start() does in C++. blank also turns the screen off, so a real VIC takes no
   // badlines; the emulated one has none either way. Returns the prologue's address.
   uint16_t startNative(uint8_t song0, bool blank);
   static constexpr uint8_t NATIVE_START = 0x60;  // the prologue's offset in the driver page: after the driver's 82 bytes at most, 95 bytes long
   void tick();                       // one machine cycle
   void runFrame();                   // one video frame of cycles, the driver's idle loop fast-forwarded
   uint32_t cyclesPerFrame() const { return cyclesPerLine_ * linesPerFrame_; }
   bool ntsc() const { return ntsc_; }
   uint16_t driverPage() const { return driverPage_; }

   // Register-write capture. The buffer is the caller's; when it fills, later
   // writes are dropped and writeOverflow() says so.
   void setWriteBuffer(SidWrite *buffer, uint32_t capacity) { writes_ = buffer; writeCap_ = capacity; writeCount_ = 0; writeOverflow_ = false; }
   void clearWrites() { writeCount_ = 0; writeOverflow_ = false; }
   uint32_t writeCount() const { return writeCount_; }
   bool writeOverflow() const { return writeOverflow_; }
   const uint8_t *sidRegs() const { return sid_; }   // SID #1: the last value written to each of $D400-$D41F
   // Optional log of reads from $D000-$DFFF and $00/$01, the places where an
   // emulation can answer differently from another; the harness prints it.
   void setReadLog(SidRead *buffer, uint32_t capacity) { reads_ = buffer; readCap_ = capacity; readCount_ = 0; }
   uint32_t readCount() const { return readCount_; }
   void clearReads() { readCount_ = 0; }
   uint32_t cycle() const { return cycle_; }
   uint32_t playCalls() const { return playCalls_; } // driver-counted returns from play; stays 0 when the tune owns its IRQ
   bool driverCallsPlay() const { return playAddr_ != 0; }
   uint16_t rasterLine() const { return line_; }
   uint8_t cpuPort() const { return cpuPort_; }

   // A look inside, for the harness to say where a tune that stopped is sitting.
   struct Probe { uint16_t pc, sp, irqVector; uint8_t p, port, vicMask, vicIrq, cia1Imr, cia1Icr, cia1Cra; uint16_t cia1Latch; };
   Probe probe() const
   {
      return Probe{cpu_.PC, (uint16_t)(0x100 | cpu_.S), (uint16_t)(mem_.ram[0x314] | (mem_.ram[0x315] << 8)), cpu_.P, cpuPort_,
                   vic_[0x1A], vicIrq_, cia1_.intr.imr, cia1_.intr.icr, cia1_.ta.cr, cia1_.ta.latch};
   }
   uint8_t ram(uint16_t a) const { return mem_.ram[a]; }

   // What reads of the SID's read-only registers answer. Fixed values until the
   // synthesis model supplies them; siddump answers 3 and 0.
   uint8_t osc3 = 3, env3 = 0;
   // Reads of VIC, CIA and SID registers answer the last value written to them
   // instead of the live chip, which is what a timing-free player such as siddump does.
   // For telling an emulation difference from a reference's limitation; never on
   // the Teensy.
   bool flatIo = false;
   uint32_t mirrorWrites() const { return mirrorWrites_; }  // SID #1 writes made through $D420-$D7FF
   uint32_t portReads() const { return portReads_; }        // the tune's own reads of $00/$01 (not the driver's)
   bool zeroRegsAtPlay = false;  // experiment: enter play with A=X=Y=0, as siddump does
   // runFrame skips the cycles the 6502 spends in the driver's idle loop up to the next
   // event that can end it. Off, every cycle is ticked: the reference the harness compares with.
   bool fastIdle = true;
   uint32_t idleCyclesSkipped() const { return skipped_; }  // since start(): what fastIdle saved
   // The driver's marker stores go into the write buffer too, as writes to DRIVER_MARKER,
   // so a trace splits into play calls the way a capture of a real C64 does.
   bool recordMarker = false;

private:
   static uint8_t portIn(void *user);
   static void portOut(uint8_t data, void *user);
   uint8_t read(uint16_t a);
   void write(uint16_t a, uint8_t v);
   uint8_t ioRead(uint16_t a);
   void ioWrite(uint16_t a, uint8_t v);
   void ciaWrite(m6526_t &cia, uint8_t reg, uint8_t value);
   void buildRoms();
   void installDriver(uint8_t song0);
   void begin(uint8_t song0);
   bool ciaSpeed(uint8_t song0) const;
   static uint8_t bankFor(uint16_t addr);
   void record(uint16_t a, uint8_t v);
   uint32_t idleSpan(uint32_t limit) const;
   void skipIdle(uint32_t cycles);

   SidMachineMem mem_{};
   bool ntsc_ = false;
   uint32_t cyclesPerLine_ = PAL_CYCLES_PER_LINE, linesPerFrame_ = PAL_LINES;
   SidHeader tune_{};
   bool loaded_ = false;
   uint16_t driverPage_ = 0, playAddr_ = 0, sid2_ = 0, sid3_ = 0;
   uint16_t inPlayAddr_ = 0, afterPlayAddr_ = 0;  // the driver's flag byte and post-play continuation, which the pseudo-KERNAL exit uses
   uint16_t idleAddr_ = 0;                        // the driver's JMP idle, which runFrame fast-forwards

   m6502_t cpu_{};
   m6526_t cia1_{}, cia2_{};
   uint64_t pins_ = 0;
   uint8_t cpuPort_ = 0x37;
   uint8_t ciaSel_ = 0;          // 1 or 2 while the current cycle addresses that CIA

   uint8_t vic_[0x40]{};         // VIC register image; $D011 bit 7 and $D012 hold the raster compare
   uint16_t line_ = 0, lineCycle_ = 0, rasterCmp_ = 0;
   uint8_t vicIrq_ = 0;          // $D019 latch bits

   uint8_t sid_[0x20]{};
   uint8_t ciaImage_[2][16]{};  // last values written to each CIA, for flatIo
   uint8_t portImage_[2] = {0, 0x37};  // $00/$01 as flat memory would hold them, for flatIo
   uint32_t mirrorWrites_ = 0, portReads_ = 0;
   SidRead *reads_ = nullptr;
   uint32_t readCap_ = 0, readCount_ = 0;
   SidWrite *writes_ = nullptr;
   uint32_t writeCap_ = 0, writeCount_ = 0;
   bool writeOverflow_ = false;
   uint32_t cycle_ = 0, playCalls_ = 0, skipped_ = 0;
};
