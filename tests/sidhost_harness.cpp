// SPDX-License-Identifier: MIT
// Native driver for the SID host's emulated machine (host/):
// loads a .sid, plays it, and prints the SID register image once per row so the
// output can be checked against siddump's, or every register write with its
// cycle. Build: c++ -std=gnu++17 -O2 -I host tests/sidhost_harness.cpp
// host/SidMachine.cpp
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "SidMachine.h"
#include "SidCalls.h"
#include "SidCapture.h"

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
      "  --zero-regs   enter play with A=X=Y=0, as siddump does (experiment)\n"
      "  --no-fast-idle  tick every cycle of the driver's idle loop (the reference for the fast-forward)\n"
      "  --calls       print each frame's bus calls, as the host replays them, and a summary\n"
      "  --call-stats  the summary only\n"
      "  --max-gap <n> cycles between two writes of one call, at most (default: the host's)\n"
      "  --max-span <n>  cycles from a call's first write to its last, at most (default: the host's)\n"
      "  --slot-calls <n> | --slot-writes <n>  a frame slot's capacity (default 1024 calls, 32768 writes)\n"
      "  --native      start as a real C64 does in SIDHost's capture mode: chips from reset, the prologue sets them\n"
      "  --blank       with --native or --write-cap: the prologue turns the screen off, as a capture does\n"
      "  --write-cap <file>  write the writes of -n frames, play markers included, as a capture file\n"
      "  --compare <file>    compare a capture file with this emulation, play call by play call\n");
}

// ------------------------------------------------------------------ captures

static bool readCap(const char *path, SidCapHeader &h, std::vector<SidCapRecord> &recs)
{
   FILE *f = fopen(path, "rb");
   if (!f) return false;
   bool ok = fread(&h, sizeof h, 1, f) == 1 && !memcmp(h.magic, SidCapMagic, sizeof h.magic) && h.version == SidCapVersion;
   if (ok)
   {
      recs.resize(h.count);
      ok = !h.count || fread(recs.data(), sizeof(SidCapRecord), h.count, f) == h.count;
   }
   fclose(f);
   return ok;
}

static bool writeCap(const char *path, const SidCapHeader &h, const std::vector<SidCapRecord> &recs)
{
   FILE *f = fopen(path, "wb");
   if (!f) return false;
   bool ok = fwrite(&h, sizeof h, 1, f) == 1 && (recs.empty() || fwrite(recs.data(), sizeof(SidCapRecord), recs.size(), f) == recs.size());
   return fclose(f) == 0 && ok;
}

struct CallRec { uint32_t at; std::vector<BusWrite> w; };
struct Segment { std::vector<CallRec> calls; };   // the calls between two play markers

// Splits a log into segments at the play markers and each segment into calls by the
// host's rule (sidCutCalls). A run of writes with no gap over maxGap never spans two
// calls' cut points differently, so the runs are cut one at a time, which keeps every
// position inside sidCutCalls' 16 bits.
static std::vector<Segment> segmentLog(const std::vector<SidWrite> &log, uint32_t maxGap, uint32_t maxSpan,
                                       std::vector<uint32_t> &markers, bool &tooLong)
{
   std::vector<Segment> segs(1);
   std::vector<SidWrite> run;
   auto flush = [&]() {
      if (run.empty()) return;
      if (run.back().cycle - run.front().cycle > 0xFFFF || run.size() > 0xFFFF) tooLong = true;
      const uint16_t cap = (uint16_t)std::min<size_t>(run.size(), 0xFFFF);
      std::vector<SidCall> calls(cap);
      std::vector<BusWrite> writes(cap);
      SidFrame f{calls.data(), writes.data(), cap, cap, 0, 0, 0, 0, false};
      sidCutCalls(run.data(), (uint32_t)run.size(), run.front().cycle, maxGap, maxSpan, f);
      for (uint32_t c = 0; c < f.callCount; c++)
         segs.back().calls.push_back(CallRec{run.front().cycle + f.calls[c].at,
            std::vector<BusWrite>(f.writes + f.calls[c].first, f.writes + f.calls[c].first + f.calls[c].count)});
      run.clear();
   };
   for (const SidWrite &w : log)
   {
      if (w.addr == SidMachine::DRIVER_MARKER)
      {
         flush();
         markers.push_back(w.cycle);
         segs.emplace_back();
         continue;
      }
      if (!run.empty() && w.cycle - run.back().cycle > maxGap) flush();
      run.push_back(w);
   }
   flush();
   return segs;
}

static void printCall(const char *side, const CallRec *c)
{
   printf("  %s:", side);
   if (!c) { printf(" (none)\n"); return; }
   for (size_t i = 0; i < c->w.size() && i < 12; i++) printf(" +%u %04X=%02X", c->w[i].offset, c->w[i].addr, c->w[i].value);
   printf(c->w.size() > 12 ? " ...\n" : "\n");
}

// Play call by play call: the segments are paired in order, and inside each the calls
// in order. A call matches when its writes have the same addresses, values and order
// and the same offsets from the call's first; where only offsets differ, the largest
// difference goes into the histogram.
static int compareLogs(const std::vector<SidWrite> &cap, const std::vector<SidWrite> &emu, uint32_t maxGap, uint32_t maxSpan)
{
   std::vector<uint32_t> capMarks, emuMarks;
   bool tooLong = false;
   std::vector<Segment> c = segmentLog(cap, maxGap, maxSpan, capMarks, tooLong);
   std::vector<Segment> e = segmentLog(emu, maxGap, maxSpan, emuMarks, tooLong);
   // Only segments that ended in a marker on both sides: each log's last one was cut off
   // by its end. Without markers the whole log is one segment, paired call by call.
   const size_t n = capMarks.empty() ? std::min(c.size(), e.size()) : std::min(capMarks.size(), emuMarks.size());
   if (capMarks.empty()) printf("# no play markers in the capture: the tune drives play itself, calls are paired in order\n");
   if (tooLong) printf("# a run of writes longer than 65,535 cycles: its offsets wrapped\n");

   uint64_t calls = 0, exact = 0, writes = 0, offsets = 0, shape = 0, startsMoved = 0;
   uint32_t hist[5] = {}, worstStart = 0, shown = 0;
   auto bucket = [](uint32_t d) { return d < 2 ? 0 : d < 4 ? 1 : d < 8 ? 2 : d < 16 ? 3 : 4; };
   for (size_t s = 0; s < n; s++)
   {
      const std::vector<CallRec> &cc = c[s].calls, &ec = e[s].calls;
      if (cc.size() != ec.size())
      {
         shape++;
         if (shown++ < 10)
         {
            printf("! play call %zu: %zu calls captured, %zu emulated\n", s, cc.size(), ec.size());
            for (size_t i = 0; i < std::max(cc.size(), ec.size()) && i < 4; i++)
            {
               printCall("cap", i < cc.size() ? &cc[i] : nullptr);
               printCall("emu", i < ec.size() ? &ec[i] : nullptr);
            }
         }
         continue;
      }
      for (size_t i = 0; i < cc.size(); i++)
      {
         calls++;
         const std::vector<BusWrite> &a = cc[i].w, &b = ec[i].w;
         bool same = a.size() == b.size();
         uint32_t worst = 0;
         for (size_t k = 0; same && k < a.size(); k++)
         {
            same = a[k].addr == b[k].addr && a[k].value == b[k].value;
            const uint32_t d = a[k].offset > b[k].offset ? a[k].offset - b[k].offset : b[k].offset - a[k].offset;
            if (d > worst) worst = d;
         }
         // Inside a play call (not before the first marker, where init and the first
         // play are as far apart as the raster happened to be), each call's start
         // from the call's first keeps the tune's own spacing.
         if (s > 0 && i > 0)
         {
            const uint32_t dc = cc[i].at - cc[0].at, de = ec[i].at - ec[0].at;
            const uint32_t d = dc > de ? dc - de : de - dc;
            if (d) { startsMoved++; if (d > worstStart) worstStart = d; }
         }
         if (!same) writes++;
         else if (worst) { offsets++; hist[bucket(worst)]++; }
         else { exact++; continue; }
         if (shown++ < 10)
         {
            printf("! play call %zu, call %zu: %s\n", s, i, same ? "offsets differ" : "writes differ");
            printCall("cap", &cc[i]);
            printCall("emu", &ec[i]);
         }
      }
   }
   printf("# compare: play calls %zu calls %llu exact %llu writes-differ %llu offsets-differ %llu call-count-differs %llu\n",
          n, (unsigned long long)calls, (unsigned long long)exact, (unsigned long long)writes, (unsigned long long)offsets,
          (unsigned long long)shape);
   printf("# offsets differ by: 1 %u, 2-3 %u, 4-7 %u, 8-15 %u, 16+ %u\n", hist[0], hist[1], hist[2], hist[3], hist[4]);
   printf("# call starts inside a play call moved: %llu, by up to %u cycles\n", (unsigned long long)startsMoved, worstStart);
   const size_t periods = std::min(capMarks.size(), emuMarks.size());
   if (periods > 1)
   {
      uint64_t sumC = 0, sumE = 0;
      uint32_t minC = UINT32_MAX, maxC = 0, worst = 0;
      for (size_t k = 1; k < periods; k++)
      {
         const uint32_t pc = capMarks[k] - capMarks[k - 1], pe = emuMarks[k] - emuMarks[k - 1];
         sumC += pc; sumE += pe;
         if (pc < minC) minC = pc;
         if (pc > maxC) maxC = pc;
         const uint32_t d = pc > pe ? pc - pe : pe - pc;
         if (d > worst) worst = d;
      }
      printf("# play period: captured mean %.2f min %u max %u, emulated mean %.2f, largest difference %u cycles over %zu periods\n",
             (double)sumC / (periods - 1), minC, maxC, (double)sumE / (periods - 1), worst, periods - 1);
   }
   return 0;
}

int main(int argc, char **argv)
{
   const char *path = nullptr;
   int song = -1, rows = 3000, maxFrames = -1, clock = 0;
   bool trace = false, rowsPerFrame = false, rowsSet = false, flatIo = false, ioReads = false, zeroRegs = false, fastIdle = true;
   bool calls = false, callList = false, native = false, blank = false;
   const char *writeCapPath = nullptr, *comparePath = nullptr;
   long maxGap = -1, maxSpanArg = -1;
   int slotCalls = 1024, slotWrites = 1 << 15;
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
      else if (a == "--no-fast-idle") fastIdle = false;
      else if (a == "--calls") calls = callList = true;
      else if (a == "--call-stats") calls = true;
      else if (a == "--max-gap") maxGap = atol(next());
      else if (a == "--max-span") maxSpanArg = atol(next());
      else if (a == "--slot-calls") slotCalls = atoi(next());
      else if (a == "--slot-writes") slotWrites = atoi(next());
      else if (a == "--native") native = true;
      else if (a == "--blank") blank = true;
      else if (a == "--write-cap") writeCapPath = next();
      else if (a == "--compare") comparePath = next();
      else if (a[0] == '-') { usage(); return 2; }
      else path = argv[i];
   }
   if (!path) { usage(); return 2; }

   // A comparison emulates what was captured: the song, the machine's standard (not
   // the tune's), and the screen the prologue left.
   SidCapHeader capHead{};
   std::vector<SidCapRecord> capRecs;
   if (comparePath)
   {
      if (!readCap(comparePath, capHead, capRecs)) { fprintf(stderr, "cannot read capture %s\n", comparePath); return 1; }
      song = capHead.song;
      clock = (capHead.flags & SidCapNtsc) ? 2 : 1;
      blank = !(capHead.flags & SidCapScreenOn);
      native = true;
   }
   if (writeCapPath) native = true;

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
   m.fastIdle = fastIdle;
   if (ioReads) m.setReadLog(reads.data(), (uint32_t)reads.size());
   if (native) m.startNative((uint8_t)song, blank);
   else m.start((uint8_t)song);

   if (writeCapPath || comparePath)
   {
      // One log for the whole run, markers included, never cleared.
      const uint32_t frames = comparePath ? capHead.cycles / m.cyclesPerFrame() + 2 : (uint32_t)rows;
      std::vector<SidWrite> log(comparePath ? capRecs.size() * 2 + 65536 : (size_t)frames * 512 + 4096);
      m.recordMarker = true;
      m.setWriteBuffer(log.data(), (uint32_t)log.size());
      for (uint32_t i = 0; i < frames; i++) m.runFrame();
      if (m.writeOverflow()) printf("# write buffer overflow: the emulated log is cut short\n");
      log.resize(m.writeCount());
      if (writeCapPath)
      {
         SidCapHeader hd{};
         memcpy(hd.magic, SidCapMagic, sizeof hd.magic);
         hd.version = SidCapVersion;
         hd.flags = (uint8_t)((ntsc ? SidCapNtsc : 0) | (blank ? 0 : SidCapScreenOn));
         hd.song = (uint8_t)song;
         hd.count = (uint32_t)log.size();
         hd.cycles = m.cycle();
         std::vector<SidCapRecord> recs;
         for (const SidWrite &w : log) recs.push_back(SidCapRecord{w.cycle, w.addr, w.value, 0});
         if (!writeCap(writeCapPath, hd, recs)) { fprintf(stderr, "cannot write %s\n", writeCapPath); return 1; }
         printf("# wrote %u writes over %u cycles to %s\n", hd.count, hd.cycles, writeCapPath);
         return 0;
      }
      std::vector<SidWrite> cap;
      for (const SidCapRecord &r : capRecs) cap.push_back(SidWrite{r.cycle, r.addr, r.value});
      printf("# capture: %s song %u, %u writes over %u cycles, screen %s\n", ntsc ? "NTSC" : "PAL", (unsigned)song + 1,
             capHead.count, capHead.cycles, blank ? "off" : "on");
      return compareLogs(cap, log, maxGap >= 0 ? (uint32_t)maxGap : SidFramePair().maxGap,
                         maxSpanArg >= 0 ? (uint32_t)maxSpanArg : SidFramePair().maxSpan);
   }

   if (!rowsSet) rowsPerFrame = !m.driverCallsPlay();
   if (maxFrames < 0) maxFrames = rows * 4;

   printf("# %s v%u load $%04X-$%04X init $%04X play $%04X songs %u start %u clock %s%s speed %s driver $%04X rows %s\n",
          h.rsid ? "RSID" : "PSID", h.version, h.loadAddr, h.loadEnd, h.initAddr, h.playAddr, h.songs, h.startSong,
          ntsc ? "NTSC" : "PAL", clock ? " (forced)" : "", sidSpeedIsCia(h, (uint16_t)song) ? "CIA" : "VBI",
          m.driverPage() << 8, rowsPerFrame ? "frames" : "calls");
   printf("# name \"%s\" author \"%s\" released \"%s\"\n", h.name, h.author, h.released);

   if (calls)
   {
      // Frames as the host gets them: through the pair, both slots filled before one is taken.
      std::vector<SidCall> callBuf[2] = {std::vector<SidCall>(slotCalls), std::vector<SidCall>(slotCalls)};
      std::vector<BusWrite> busBuf[2] = {std::vector<BusWrite>(slotWrites), std::vector<BusWrite>(slotWrites)};
      SidFramePair pair;
      auto slot = [&](int i) { return SidFrame{callBuf[i].data(), busBuf[i].data(), (uint16_t)callBuf[i].size(), (uint16_t)busBuf[i].size(), 0, 0, 0, 0, false}; };
      pair.init(slot(0), slot(1), writes.data(), (uint32_t)writes.size());
      if (maxGap >= 0) pair.maxGap = (uint32_t)maxGap;
      if (maxSpanArg >= 0) pair.maxSpan = (uint32_t)maxSpanArg;
      uint64_t nCalls = 0, nWrites = 0, held = 0, cycles = 0;
      uint32_t maxSpan = 0, maxCalls = 0, maxWrites = 0, overflows = 0;
      for (int frame = 0; frame < rows; frame++)
      {
         while (pair.produce(m)) ;
         const SidFrame &f = *pair.front();
         if (callList) printf("F %u %u %u %u %u\n", f.number, f.startCycle, f.callCount, f.writeCount, f.overflow ? 1 : 0);
         for (uint32_t c = 0; c < f.callCount; c++)
         {
            const SidCall &call = f.calls[c];
            if (callList)
            {
               printf("C %u %u %u\n", call.at, call.span, call.count);
               for (uint32_t i = call.first; i < (uint32_t)call.first + call.count; i++)
                  printf("W %u %04X %02X\n", f.writes[i].offset, f.writes[i].addr, f.writes[i].value);
            }
            held += call.span + 1u;
            if (call.span > maxSpan) maxSpan = call.span;
         }
         nCalls += f.callCount;
         nWrites += f.writeCount;
         cycles += m.cyclesPerFrame();
         if (f.callCount > maxCalls) maxCalls = f.callCount;
         if (f.writeCount > maxWrites) maxWrites = f.writeCount;
         if (f.overflow) overflows++;
         pair.release();
      }
      printf("# calls: frames %d gap %u span %u calls %llu writes %llu held %llu of %llu cycles, max span %u, max calls/frame %u, max writes/frame %u, overflowed frames %u\n",
             rows, pair.maxGap, pair.maxSpan, (unsigned long long)nCalls, (unsigned long long)nWrites, (unsigned long long)held,
             (unsigned long long)cycles, maxSpan, maxCalls, maxWrites, overflows);
      return 0;
   }

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
   if (trace) printf("# %u idle cycles skipped\n", m.idleCyclesSkipped());
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
