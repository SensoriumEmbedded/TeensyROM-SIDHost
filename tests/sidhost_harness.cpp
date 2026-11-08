// SPDX-License-Identifier: MIT
// Native driver for the SID host's emulated machine (Source/Teensy/SIDHost/):
// loads a .sid, plays it, and prints the SID register image once per row so the
// output can be checked against siddump's, or every register write with its
// cycle. Build: c++ -std=gnu++17 -O2 -I Source/Teensy/SIDHost sidhost_harness.cpp
// Source/Teensy/SIDHost/SidMachine.cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "SidMachine.h"

static void usage()
{
   fprintf(stderr,
      "usage: sidhost_harness <file.sid> [options]\n"
      "  -a <n>        song to play, 0-based (default: the file's start song)\n"
      "  -n <rows>     rows to print (default 3000)\n"
      "  -t <seconds>  rows as seconds at 50 per second\n"
      "  --rows calls|frames  a row per play call (default when the driver calls play) or per video frame\n"
      "  --pal | --ntsc       override the file's clock\n"
      "  --trace       print every SID write as 'cycle addr value' instead of rows\n"
      "  --max-frames <n>     give up after this many frames without the rows arriving (default 4x rows)\n"
      "  --flat-io     VIC and CIA registers read back what was written, like a timing-free reference\n"
      "  --io-reads    print every read of $D000-$DFFF and $00/$01 as 'cycle pc addr value', per frame\n"
      "  --zero-regs   enter play with A=X=Y=0, as siddump does (experiment)\n");
}

int main(int argc, char **argv)
{
   const char *path = nullptr;
   int song = -1, rows = 3000, maxFrames = -1, clock = 0;
   bool trace = false, rowsPerFrame = false, rowsSet = false, flatIo = false, ioReads = false, zeroRegs = false;
   for (int i = 1; i < argc; i++)
   {
      std::string a = argv[i];
      auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
      if (a == "-a") song = atoi(next());
      else if (a == "-n") rows = atoi(next());
      else if (a == "-t") rows = atoi(next()) * 50;
      else if (a == "--rows") { rowsPerFrame = !strcmp(next(), "frames"); rowsSet = true; }
      else if (a == "--pal") clock = 1;
      else if (a == "--ntsc") clock = 2;
      else if (a == "--trace") trace = true;
      else if (a == "--max-frames") maxFrames = atoi(next());
      else if (a == "--flat-io") flatIo = true;
      else if (a == "--io-reads") { ioReads = true; trace = true; }
      else if (a == "--zero-regs") zeroRegs = true;
      else if (a[0] == '-') { usage(); return 2; }
      else path = argv[i];
   }
   if (!path) { usage(); return 2; }

   FILE *f = fopen(path, "rb");
   if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }
   std::vector<uint8_t> file;
   uint8_t buf[4096];
   for (size_t n; (n = fread(buf, 1, sizeof buf, f)) > 0;) file.insert(file.end(), buf, buf + n);
   fclose(f);

   SidHeader h;
   if (!sidParse(file.data(), (uint32_t)file.size(), h)) { printf("# unsupported: header rejected\n"); return 3; }
   if (h.basic) { printf("# unsupported: C64 BASIC tune\n"); return 3; }
   if (song < 0) song = h.startSong - 1;
   if (song >= h.songs) { printf("# no such song\n"); return 2; }
   const bool ntsc = clock ? clock == 2 : h.clock == 2;

   std::vector<uint8_t> ram(0x10000), kernal(0x2000), basic(0x2000), color(0x400);
   std::vector<SidWrite> writes(1 << 16);
   std::vector<SidRead> reads(ioReads ? 1 << 16 : 0);
   SidMachine m;
   m.init(SidMachineMem{ram.data(), kernal.data(), basic.data(), color.data()}, ntsc);
   if (!m.load(h)) { printf("# unsupported: no free page for the driver\n"); return 3; }
   m.setWriteBuffer(writes.data(), (uint32_t)writes.size());
   m.flatIo = flatIo;
   m.zeroRegsAtPlay = zeroRegs;
   if (ioReads) m.setReadLog(reads.data(), (uint32_t)reads.size());
   m.start((uint8_t)song);

   if (!rowsSet) rowsPerFrame = !m.driverCallsPlay();
   if (maxFrames < 0) maxFrames = rows * 4;

   printf("# %s v%u load $%04X-$%04X init $%04X play $%04X songs %u start %u clock %s%s speed %s driver $%04X rows %s\n",
          h.rsid ? "RSID" : "PSID", h.version, h.loadAddr, h.loadEnd, h.initAddr, h.playAddr, h.songs, h.startSong,
          ntsc ? "NTSC" : "PAL", clock ? " (forced)" : "", sidSpeedIsCia(h, (uint16_t)song) ? "CIA" : "VBI",
          m.driverPage() << 8, rowsPerFrame ? "frames" : "calls");
   printf("# name \"%s\" author \"%s\" released \"%s\"\n", h.name, h.author, h.released);

   int printed = 0, frames = 0;
   uint32_t seenCalls = 0;
   bool overflowed = false;   // a frame held more writes than the buffer; the writes are cleared per frame
   auto row = [&]() {
      printf("%d", printed);
      for (int r = 0; r < 25; r++) printf(" %02X", m.sidRegs()[r]);
      printf("\n");
      printed++;
   };
   while (printed < rows && frames < maxFrames)
   {
      if (trace)
      {
         m.clearWrites();
         m.clearReads();
         m.runFrame();
         if (ioReads)
         {
            for (uint32_t i = 0; i < m.readCount(); i++) printf("%u %04X %04X %02X\n", reads[i].cycle, reads[i].pc, reads[i].addr, reads[i].value);
            frames++; printed = frames; continue;
         }
         for (uint32_t i = 0; i < m.writeCount(); i++) printf("%u %04X %02X\n", writes[i].cycle, writes[i].addr, writes[i].value);
         if (m.writeOverflow()) printf("# write buffer overflow in frame %d\n", frames);
         frames++;
         printed = frames;
         continue;
      }
      if (rowsPerFrame) { m.clearWrites(); m.runFrame(); frames++; overflowed |= m.writeOverflow(); row(); continue; }
      for (uint32_t n = m.cyclesPerFrame(); n && printed < rows; n--)
      {
         m.tick();
         if (m.playCalls() != seenCalls) { seenCalls = m.playCalls(); row(); }
      }
      frames++;
      overflowed |= m.writeOverflow();
      // A tune whose init installs its own interrupt handler never comes back
      // through the driver, so its play address is never called by it; its
      // writes still happen, so from here on a row is a frame.
      if (frames == 50 && m.playCalls() == 0 && m.writeCount() > 0)
      {
         printf("# play is driven by the tune's own interrupt; rows are frames from frame %d\n", frames);
         rowsPerFrame = true;
      }
      if (!rowsPerFrame) m.clearWrites();
   }
   if (!trace && overflowed) printf("# write buffer overflow: a frame held more than %u SID writes\n", (unsigned)writes.size());
   if (!trace && m.mirrorWrites()) printf("# %u SID writes went through mirror addresses\n", m.mirrorWrites());
   if (!trace && m.portReads()) printf("# %u reads of $00/$01 by the tune\n", m.portReads());
   if (!trace && printed < rows)
   {
      const SidMachine::Probe q = m.probe();
      // A play routine that spends frames in delay loops is slow, not stuck; a
      // game's "play" that waits on its own interrupt is what shows up as never called.
      if (printed) printf("# slow: %d rows in %d frames\n", printed, frames);
      else printf("# stopped after %d frames with %d rows: play was not called\n", frames, printed);
      printf("# cpu pc=$%04X sp=$%04X p=$%02X $01=$%02X $0314=$%04X | vic mask=$%02X irq=$%02X line=%u | cia1 imr=$%02X icr=$%02X cra=$%02X latch=$%04X | writes=%u\n",
             q.pc, q.sp, q.p, q.port, q.irqVector, q.vicMask, q.vicIrq, m.rasterLine(), q.cia1Imr, q.cia1Icr, q.cia1Cra, q.cia1Latch, m.writeCount());
      printf("# code at pc:");
      for (int i = 0; i < 12; i++) printf(" %02X", m.ram((uint16_t)(q.pc + i)));
      printf("\n");
   }
   return 0;
}
