// SPDX-License-Identifier: MIT
// See SidMachine.h. This file holds the chips implementations too (CHIPS_IMPL),
// so it is compiled exactly once per image.
#include <string.h>
#include <initializer_list>
// m6502.h asserts on unreachable decoder states; compiled out, so the firmware image
// does not carry newlib's assert path for a branch the generated decoder cannot take.
#ifndef NDEBUG
#define NDEBUG
#endif
#include <assert.h>
#define CHIPS_IMPL
#define CHIPS_ASSERT(c) ((void)0)
#include "SidMachine.h"

// ------------------------------------------------------------------ setup

void SidMachine::init(const SidMachineMem &mem, bool ntsc)
{
   mem_ = mem;
   ntsc_ = ntsc;
   cyclesPerLine_ = ntsc ? NTSC_CYCLES_PER_LINE : PAL_CYCLES_PER_LINE;
   linesPerFrame_ = ntsc ? NTSC_LINES : PAL_LINES;
   loaded_ = false;
}

// The bank the format description prescribes for calling an address: BASIC and
// KERNAL in below $A000, KERNAL only below $D000, I/O only from $E000, RAM only
// for a routine under the I/O area.
uint8_t SidMachine::bankFor(uint16_t addr)
{
   if (addr < 0xA000) return 0x37;
   if (addr < 0xD000) return 0x36;
   if (addr >= 0xE000) return 0x35;
   return 0x34;
}

bool SidMachine::load(const SidHeader &h)
{
   tune_ = h;
   loaded_ = false;
   playAddr_ = h.playAddr;
   sid2_ = sidExtraAddr(h.sid2Addr);
   sid3_ = sidExtraAddr(h.sid3Addr);

   // The driver takes one page the tune does not, visible in every bank it can
   // run in ($0400-$9FFF and $C000-$CFFF): the header's relocation range when it
   // names one, else the lowest free page from $0400, the PlaySID convention.
   // The pages next to a tune's load range are the wrong choice: players keep
   // workspace there without declaring it, and a trial that put the driver after
   // or before the tune stopped 28 of 5,000 tunes that play from $0400.
   auto usable = [&](uint16_t page) {
      const uint16_t lo = (uint16_t)(page << 8), hi = (uint16_t)(lo | 0xFF);
      const bool bankable = (page >= 0x04 && page <= 0x9F) || (page >= 0xC0 && page <= 0xCF);
      return bankable && (hi < h.loadAddr || lo > h.loadEnd);
   };
   driverPage_ = 0;
   if (h.startPage != 0 && h.startPage != 0xFF && h.pageLength != 0 && usable(h.startPage)) driverPage_ = h.startPage;
   for (uint16_t page = 0x04; !driverPage_ && page <= 0xCF; page++) if (usable(page)) driverPage_ = page;
   if (!driverPage_) return false;
   loaded_ = true;
   return true;
}

// The pseudo-KERNAL: RTS everywhere a tune might stray into, plus the vectors
// and the interrupt entry and exit sequences at the addresses tunes jump to.
// A play routine ripped from a program's own IRQ handler often leaves through
// JMP $EA31 or $EA81 instead of RTS; while the driver is inside play, that exit
// drops the JSR return address and continues in the driver, so its own frame
// unwinds. Outside play (a tune running its own IRQ) it is the plain KERNAL exit.
void SidMachine::buildRoms()
{
   memset(mem_.kernal, 0x60, 0x2000);
   memset(mem_.basic, 0x60, 0x2000);
   uint8_t *k = mem_.kernal;
   auto put = [&](uint16_t addr, std::initializer_list<uint8_t> bytes) {
      uint8_t *p = k + (addr - 0xE000);
      for (uint8_t b : bytes) *p++ = b;
   };
   put(0xEA31, {0x4C, 0x81, 0xEA});                                     // JMP $EA81: the IRQ body is skipped
   put(0xEA7E, {0xAD, 0x0D, 0xDC});                                     // LDA $DC0D, falls into $EA81, as in the KERNAL
   put(0xFEBC, {0x4C, 0x81, 0xEA});                                     // the NMI exit: same pulls, same handling
   put(0xEA81, {0xAD, (uint8_t)inPlayAddr_, (uint8_t)(inPlayAddr_ >> 8), // LDA inPlay
                0xD0, 0x06,                                              // BNE +6
                0x68, 0xA8, 0x68, 0xAA, 0x68, 0x40,                      // PLA TAY PLA TAX PLA RTI
                0x68, 0x68, 0x4C, (uint8_t)afterPlayAddr_, (uint8_t)(afterPlayAddr_ >> 8)}); // PLA PLA JMP afterPlay
   put(0xFE43, {0x78, 0x6C, 0x18, 0x03});                               // NMI: SEI JMP ($0318)
   put(0xFE47, {0x40});                                                 // default NMI handler: RTI
   put(0xFE66, {0x4C, 0x81, 0xEA});                                     // default BRK handler: back out through $EA81
   put(0xFF48, {0x48, 0x8A, 0x48, 0x98, 0x48, 0xBA, 0xBD, 0x04, 0x01,   // PHA TXA PHA TYA PHA TSX LDA $0104,X
                0x29, 0x10, 0xF0, 0x03, 0x6C, 0x16, 0x03, 0x6C, 0x14, 0x03}); // AND #$10 BEQ +3 JMP ($0316) JMP ($0314)
   const uint16_t entry = (uint16_t)(driverPage_ << 8);
   put(0xFFFA, {0x43, 0xFE, (uint8_t)entry, (uint8_t)(entry >> 8), 0x48, 0xFF});
}

// The driver, assembled by hand into its page:
//
//   entry:   SEI CLD LDX #$FF TXS LDA #$2F STA $00 LDA #bank(init) STA $01
//            LDA #song LDX #0 LDY #0 JSR init CLI
//   idle:    JMP idle
//   nmi:     RTI
//   irqRaw:  PHA TXA PHA TYA PHA              ; entered from $FFFE with the KERNAL banked out
//   irqBody: LDA $01 PHA LDA #bank(play) STA $01
//            LDA $DC0D LDA #$01 STA $D019      ; acknowledge both interrupt sources
//            LDA #$01 STA inPlay JSR play
//   after:   LDA #$00 STA inPlay LDA #bank(play) STA $01 STA $DFFF  ; the marker store counts the call
//            PLA STA $01 PLA TAY PLA TAX PLA RTI
//   inPlay:  one byte, the page's last
//
// $01 is left where init put it: a tune that banks the KERNAL out to use $FFFE
// expects that to stick, and the RAM copies of the vectors cover that case.
void SidMachine::installDriver(uint8_t song0)
{
   const uint16_t base = (uint16_t)(driverPage_ << 8);
   inPlayAddr_ = (uint16_t)(base + 0xFF);
   uint8_t *d = mem_.ram + base;
   uint8_t n = 0;
   auto emit = [&](std::initializer_list<uint8_t> bytes) { for (uint8_t b : bytes) d[n++] = b; };
   auto lo = [](uint16_t a) { return (uint8_t)a; };
   auto hi = [](uint16_t a) { return (uint8_t)(a >> 8); };
   emit({0x78, 0xD8, 0xA2, 0xFF, 0x9A, 0xA9, 0x2F, 0x85, 0x00, 0xA9, bankFor(tune_.initAddr), 0x85, 0x01});
   emit({0xA9, song0, 0xA2, 0x00, 0xA0, 0x00, 0x20, lo(tune_.initAddr), hi(tune_.initAddr), 0x58});
   const uint8_t idle = n;
   emit({0x4C, lo(base + idle), hi(base + idle)});
   const uint8_t nmi = n;
   emit({0x40});
   const uint8_t irqRaw = n;
   emit({0x48, 0x8A, 0x48, 0x98, 0x48});
   const uint8_t irqBody = n;
   emit({0xA5, 0x01, 0x48, 0xA9, bankFor(playAddr_), 0x85, 0x01, 0xAD, 0x0D, 0xDC, 0xA9, 0x01, 0x8D, 0x19, 0xD0});
   emit({0xA9, 0x01, 0x8D, lo(inPlayAddr_), hi(inPlayAddr_)});
   if (zeroRegsAtPlay) emit({0xA9, 0x00, 0xA2, 0x00, 0xA0, 0x00});
   emit({0x20, lo(playAddr_), hi(playAddr_)});
   afterPlayAddr_ = (uint16_t)(base + n);
   // Play may return with I/O banked out (players that keep data under $D000 do), so
   // the marker store puts the play bank back first; the caller's $01 is restored after.
   emit({0xA9, 0x00, 0x8D, lo(inPlayAddr_), hi(inPlayAddr_), 0xA9, bankFor(playAddr_), 0x85, 0x01, 0x8D, lo(DRIVER_MARKER), hi(DRIVER_MARKER)});
   emit({0x68, 0x85, 0x01, 0x68, 0xA8, 0x68, 0xAA, 0x68, 0x40});

   auto vec = [&](uint16_t at, uint16_t to) { mem_.ram[at] = (uint8_t)to; mem_.ram[at + 1] = (uint8_t)(to >> 8); };
   // KERNAL-in vectors in page 3, and RAM copies of the hardware vectors for a tune
   // that banks the KERNAL out. A tune with play = 0 installs its own IRQ handler,
   // so those stay at the KERNAL defaults.
   vec(0x0314, playAddr_ ? (uint16_t)(base + irqBody) : 0xEA31);
   vec(0x0316, 0xFE66);
   vec(0x0318, 0xFE47);
   vec(0xFFFA, (uint16_t)(base + nmi));
   vec(0xFFFC, base);
   vec(0xFFFE, playAddr_ ? (uint16_t)(base + irqRaw) : (uint16_t)(base + nmi));
   mem_.ram[0x02A6] = ntsc_ ? 0 : 1;
}

void SidMachine::ciaWrite(m6526_t &cia, uint8_t reg, uint8_t value)
{
   uint64_t p = M6526_CS | (reg & M6526_RS);
   M6526_SET_DATA(p, value);
   M6526_SET_PAB(p, 0xFF, 0xFF);
   m6526_tick(&cia, p);
}

void SidMachine::start(uint8_t song0)
{
   if (!loaded_) return;
   memset(mem_.ram, 0, 0x10000);
   memset(mem_.color, 0, 0x400);
   memcpy(mem_.ram + tune_.loadAddr, tune_.data, tune_.dataLen);
   installDriver(song0);
   buildRoms();

   memset(vic_, 0, sizeof vic_);
   memset(sid_, 0, sizeof sid_);
   memset(ciaImage_, 0, sizeof ciaImage_);
   portImage_[0] = 0; portImage_[1] = 0x37;
   mirrorWrites_ = 0;
   portReads_ = 0;
   line_ = lineCycle_ = 0;
   rasterCmp_ = 0;
   vicIrq_ = 0;
   cycle_ = 0;
   playCalls_ = 0;
   clearWrites();

   // The format description's default environment: CIA 1 timer A at 60 Hz and
   // running, its interrupt on for a CIA-speed song and for every RSID; the
   // raster interrupt on only for a VBI PSID, set to line $137 but off for an
   // RSID; everything else off.
   const bool cia = sidSpeedIsCia(tune_, song0) || tune_.rsid;
   const uint16_t period = ntsc_ ? NTSC_TIMER : PAL_TIMER;
   m6526_init(&cia1_);
   m6526_init(&cia2_);
   ciaWrite(cia1_, M6526_REG_ICR, 0x7F);
   ciaWrite(cia2_, M6526_REG_ICR, 0x7F);
   ciaWrite(cia1_, M6526_REG_TALO, (uint8_t)period);
   ciaWrite(cia1_, M6526_REG_TAHI, (uint8_t)(period >> 8));
   ciaWrite(cia1_, M6526_REG_CRA, 0x11);
   if (cia) ciaWrite(cia1_, M6526_REG_ICR, 0x81);
   vic_[0x11] = tune_.rsid ? 0x9B : 0x1B;
   vic_[0x12] = tune_.rsid ? 0x37 : 0x00;
   rasterCmp_ = tune_.rsid ? 0x137 : 0;
   vic_[0x1A] = cia ? 0x00 : 0x01;

   m6502_desc_t desc{};
   desc.m6510_in_cb = portIn;
   desc.m6510_out_cb = portOut;
   desc.m6510_user_data = this;
   desc.m6510_io_pullup = 0x17;
   desc.m6510_io_floating = 0xC8;
   pins_ = m6502_init(&cpu_, &desc);
   cpuPort_ = 0x37;
}

// ------------------------------------------------------------------ bus

uint8_t SidMachine::portIn(void *) { return 0x17; }
void SidMachine::portOut(uint8_t data, void *user) { static_cast<SidMachine *>(user)->cpuPort_ = data; }

uint8_t SidMachine::read(uint16_t a)
{
   const uint8_t bank = cpuPort_ & 7;
   if (a >= 0xE000) return (bank & 2) ? mem_.kernal[a - 0xE000] : mem_.ram[a];
   if (a >= 0xD000)
   {
      if ((bank & 3) == 0) return mem_.ram[a];
      if (!(bank & 4)) return 0;  // character ROM: nothing a tune needs
      return ioRead(a);
   }
   if (a >= 0xA000 && a < 0xC000 && (bank & 3) == 3) return mem_.basic[a - 0xA000];
   return mem_.ram[a];
}

void SidMachine::write(uint16_t a, uint8_t v)
{
   const uint8_t bank = cpuPort_ & 7;
   if (a >= 0xD000 && a < 0xE000 && (bank & 3) != 0 && (bank & 4)) ioWrite(a, v);
   else mem_.ram[a] = v;
}

static inline bool inSid(uint16_t base, uint16_t a) { return base && a >= base && a < base + 0x20; }

uint8_t SidMachine::ioRead(uint16_t a)
{
   if (a < 0xD400)
   {
      const uint8_t reg = a & 0x3F;
      if (flatIo) return reg < 0x2F ? vic_[reg] : 0xFF;
      switch (reg)
      {
         case 0x11: return (uint8_t)((vic_[0x11] & 0x7F) | ((line_ >> 1) & 0x80));
         case 0x12: return (uint8_t)line_;
         case 0x19: return (uint8_t)(vicIrq_ | 0x70 | ((vicIrq_ & vic_[0x1A] & 0x0F) ? 0x80 : 0));
         case 0x1A: return (uint8_t)(vic_[0x1A] | 0xF0);
         case 0x1E: case 0x1F: return 0;
         default: return reg < 0x2F ? vic_[reg] : 0xFF;
      }
   }
   if (a < 0xD800)
   {
      const uint8_t reg = a & 0x1F;
      if (inSid(sid2_, a) || inSid(sid3_, a)) return 0;  // no image kept for the extra chips yet
      if (flatIo) return sid_[reg];                     // the read-only registers too: what a flat memory answers
      if (reg == 0x1B) return osc3;
      if (reg == 0x1C) return env3;
      return sid_[reg];
   }
   if (a < 0xDC00) return mem_.color[a & 0x3FF] & 0x0F;
   if (a < 0xDE00)
   {
      if (flatIo) return ciaImage_[(a >> 8) & 1][a & 0x0F];
      ciaSel_ = (uint8_t)(1 + ((a >> 8) & 1));  // the CIA tick supplies the byte
      return 0;
   }
   return 0;
}

void SidMachine::record(uint16_t a, uint8_t v)
{
   if (writes_ && writeCount_ < writeCap_) writes_[writeCount_++] = SidWrite{cycle_, a, v};
   else writeOverflow_ = true;
}

void SidMachine::ioWrite(uint16_t a, uint8_t v)
{
   if (a < 0xD400)
   {
      const uint8_t reg = a & 0x3F;
      if (reg == 0x19) { vicIrq_ &= (uint8_t)~v; return; }
      vic_[reg] = v;
      if (reg == 0x11 || reg == 0x12) rasterCmp_ = (uint16_t)(vic_[0x12] | ((vic_[0x11] & 0x80) << 1));
      return;
   }
   if (a < 0xD800)
   {
      // The SID answers every $20 bytes of $D400-$D7FF; tunes do write through the mirrors.
      if (!inSid(sid2_, a) && !inSid(sid3_, a)) { sid_[a & 0x1F] = v; if (a >= 0xD420) mirrorWrites_++; }
      record(a, v);
      return;
   }
   if (a < 0xDC00) { mem_.color[a & 0x3FF] = v & 0x0F; return; }
   if (a < 0xDE00) { ciaImage_[(a >> 8) & 1][a & 0x0F] = v; ciaSel_ = (uint8_t)(1 + ((a >> 8) & 1)); return; }
   // The driver's marker store is told from a tune's own write to the same address by
   // where it comes from: a second or third SID can sit at $DFE0-$DFFF.
   if (a == DRIVER_MARKER && (cpu_.PC >> 8) == driverPage_) { playCalls_++; return; }
   if (inSid(sid2_, a) || inSid(sid3_, a)) record(a, v);
}

// ------------------------------------------------------------------ clock

void SidMachine::tick()
{
   pins_ = m6502_tick(&cpu_, pins_);
   const uint16_t addr = M6502_GET_ADDR(pins_);
   const bool rw = pins_ & M6502_RW;
   ciaSel_ = 0;
   if (addr < 2)
   {
      // The port registers, through the 6510 model; with flatIo a read answers what
      // was last stored there, as a flat memory does ($00 reads $00, not the DDR's $2F).
      if (!rw) portImage_[addr] = M6502_GET_DATA(pins_);
      else if ((cpu_.PC >> 8) != driverPage_) portReads_++;
      pins_ = m6510_iorq(&cpu_, pins_);
      if (rw && flatIo) { M6502_SET_DATA(pins_, portImage_[addr]); }
   }
   else if (rw) { M6502_SET_DATA(pins_, read(addr)); }
   else write(addr, M6502_GET_DATA(pins_));
   const bool logRead = reads_ && rw && (addr < 2 || (addr >= 0xD000 && addr < 0xE000 && (cpuPort_ & 7) != 0 && (cpuPort_ & 4)));

   // Both CIAs tick every cycle; the one addressed this cycle also sees the bus.
   // Their ports read as all ones: no key held, nothing on the serial bus.
   uint64_t p = pins_ & (0xFFFFFFULL | M6502_RW);
   M6526_SET_PAB(p, 0xFF, 0xFF);
   uint64_t p1 = p, p2 = p;
   if (ciaSel_ == 1) p1 |= M6526_CS;
   if (ciaSel_ == 2) p2 |= M6526_CS;
   p1 = m6526_tick(&cia1_, p1);
   p2 = m6526_tick(&cia2_, p2);
   if (rw && ciaSel_ == 1) { M6502_SET_DATA(pins_, M6526_GET_DATA(p1)); }
   if (rw && ciaSel_ == 2) { M6502_SET_DATA(pins_, M6526_GET_DATA(p2)); }
   if (logRead && readCount_ < readCap_) reads_[readCount_++] = SidRead{cycle_, cpu_.PC, addr, M6502_GET_DATA(pins_)};

   // VIC: the raster counter and its interrupt, nothing else.
   if (++lineCycle_ >= cyclesPerLine_)
   {
      lineCycle_ = 0;
      if (++line_ >= linesPerFrame_) line_ = 0;
      if (line_ == rasterCmp_) vicIrq_ |= 1;
   }

   const bool irq = (p1 & M6526_IRQ) || (vicIrq_ & vic_[0x1A] & 0x0F);
   const bool nmi = p2 & M6526_IRQ;
   pins_ = (pins_ & ~(M6502_IRQ | M6502_NMI)) | (irq ? M6502_IRQ : 0) | (nmi ? M6502_NMI : 0);
   cycle_++;
}

void SidMachine::runFrame()
{
   for (uint32_t n = cyclesPerFrame(); n; n--) tick();
}
