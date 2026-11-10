// SPDX-License-Identifier: MIT
//
// One emulated frame's SID writes, cut into bus calls: the spans the host holds the
// C64 for and replays with the snoop in SidBus.h. A call is a run of writes with no
// gap between two of them longer than maxGap cycles, and no longer than maxSpan cycles
// from its first write to its last: the snoop runs a whole call in one interrupt, so a
// tune that writes all the time would otherwise leave the emulator no time. Inside a
// call each write keeps its cycle offset from the call's first, and the calls keep the
// tune's write order. A call that is still open when the frame ends is cut there.
//
// SidFramePair holds two frames, so the emulator can fill one while the other plays.
// Plain C++, no Arduino; the native harness uses it too.
#pragma once
#include <stdint.h>
#include "SidMachine.h"

struct BusWrite { uint16_t offset; uint16_t addr; uint8_t value; };  // offset: cycles after the call's first write

struct SidCall
{
   uint16_t at;     // the first write's cycle, counted from the frame's first cycle
   uint16_t span;   // cycles from the first write to the last
   uint16_t first;  // index of the first write in the frame's list
   uint16_t count;
};

struct SidFrame
{
   SidCall *calls;    // the caller's arrays
   BusWrite *writes;
   uint16_t callCap, writeCap;
   uint16_t callCount, writeCount;
   uint32_t number;      // frames since the pair was reset
   uint32_t startCycle;  // the machine's cycle counter at the frame's first cycle
   bool overflow;        // writes were dropped: the machine's buffer or this frame's lists filled
};

// Cuts writes[0..n) of a frame that began at machine cycle frameStart into f's lists.
// A frame is shorter than 65,536 cycles, so every position fits 16 bits.
inline void sidCutCalls(const SidWrite *writes, uint32_t n, uint32_t frameStart, uint32_t maxGap, uint32_t maxSpan, SidFrame &f)
{
   f.callCount = f.writeCount = 0;
   SidCall *call = nullptr;
   uint32_t last = 0;
   for (uint32_t i = 0; i < n; i++)
   {
      const uint32_t at = writes[i].cycle - frameStart;
      if (!call || at - last > maxGap || at - call->at > maxSpan)
      {
         if (f.callCount == f.callCap) { f.overflow = true; return; }
         call = &f.calls[f.callCount++];
         *call = SidCall{(uint16_t)at, 0, f.writeCount, 0};
      }
      if (f.writeCount == f.writeCap) { f.overflow = true; return; }
      f.writes[f.writeCount++] = BusWrite{(uint16_t)(at - call->at), writes[i].addr, writes[i].value};
      call->span = (uint16_t)(at - call->at);
      call->count++;
      last = at;
   }
}

class SidFramePair
{
public:
   uint32_t maxGap = 64, maxSpan = 1000;   // see sidCutCalls

   // scratch takes the machine's writes for one frame before they are cut.
   void init(SidFrame a, SidFrame b, SidWrite *scratch, uint32_t scratchCap)
   {
      slot_[0] = a; slot_[1] = b;
      scratch_ = scratch; scratchCap_ = scratchCap;
      reset();
   }
   void reset() { head_ = count_ = 0; number_ = 0; }

   // Runs the machine's next frame into the free slot and returns it; nullptr, with nothing run,
   // when both are full.
   const SidFrame *produce(SidMachine &m)
   {
      if (count_ == 2) return nullptr;
      SidFrame &f = slot_[(head_ + count_) & 1];
      m.setWriteBuffer(scratch_, scratchCap_);
      f.startCycle = m.cycle();
      f.number = number_++;
      m.runFrame();
      f.overflow = m.writeOverflow();
      sidCutCalls(scratch_, m.writeCount(), f.startCycle, maxGap, maxSpan, f);
      count_++;
      return &f;
   }
   const SidFrame *front() const { return count_ ? &slot_[head_] : nullptr; }  // the oldest frame not yet released
   void release() { if (count_) { head_ ^= 1; count_--; } }
   uint8_t ready() const { return count_; }

private:
   SidFrame slot_[2]{};
   SidWrite *scratch_ = nullptr;
   uint32_t scratchCap_ = 0;
   uint8_t head_ = 0, count_ = 0;
   uint32_t number_ = 0;
};
