// SPDX-License-Identifier: MIT
//
// The tune's SID writes on their bus cycles. Thread mode emulates frames into a SidFramePair
// and queues each one here with the time its first cycle is due; everything after that runs
// in this image's bus snoop, inside the PHI2 interrupt, so the emulator never delays a write:
//
//   - LeadCycles before a call is due, the snoop asks for the hold (DMA_S_StartAsynch), and
//     the interrupt's tail asserts /DMA on a safe cycle as it does for any DMA;
//   - once held, one interrupt runs the whole call: it waits for the due time, puts each write
//     on its cycle offset with DMAByte (the same bus cycle a DMA transfer makes), and releases
//     /DMA on phi2 falling, as the tail does for DMA_S_StartDisable.
//
// A write that finds BA low waits for the next cycle, and the writes after it move with it,
// as the 6510 would have stalled. Nothing in the shared DMA engine changes.

#pragma once
#include "SidCalls.h"

namespace SidPlayer
{
   // Cycles before a call's first write that the hold is asked for. The safe assert waits for a
   // read after a write. The stub's idle loop makes one every few cycles, but the KERNAL's
   // keyboard interrupt can delay it: the bench measured up to ~55 cycles, and 40 left calls late.
   static constexpr uint32_t LeadCycles = 80;

   struct Slot { const SidFrame *frame; uint32_t base; };   // base: ARM cycle at which the frame's cycle 0 is due
   static Slot Ring[2];
   static volatile uint8_t Head, Count;     // the ring: the snoop takes from Head, thread mode adds
   static uint16_t NextCall;                // in Ring[Head]
   static uint32_t ArmPerCycleQ16;          // ARM cycles per emulated cycle, 16.16
   static uint32_t LeadArm;
   static volatile bool Holding;            // the snoop asked for the hold the bus is in or going into
   static volatile bool HoldOff;            // ask for no hold: a thread-mode transfer, a pause or a stop
   // The chips the tune writes: SID 1 at $D400 (and its mirrors up to $D7FF), and the header's second
   // and third, 0 when absent. LastVolume is each one's $18 register as last written, which a pause
   // turns down and resume puts back.
   static uint16_t ChipBase[3];
   static volatile uint8_t LastVolume[3];

   static volatile uint32_t FramesDone, Calls, Writes, LateCalls;
   static volatile uint32_t BaWaits;        // cycles writes waited for BA: up to ~40 for each badline a call runs into
   static volatile int32_t MinSlackArm;     // the least time left between the hold and a call's due time

   // The call at Ring[Head], NextCall, and when it is due. The snoop runs on every bus cycle, so
   // this is worked out once per call (aim) and the cycles in between only compare the clock.
   static volatile bool HaveNext;
   static volatile uint32_t NextDue;

   // Points at the next call to play, passing over frames that are done (or empty) and
   // counting them; leaves HaveNext false when the ring is empty. Interrupt context, or
   // thread mode with interrupts off.
   static inline void aim()
   {
      while (Count)
      {
         const Slot &s = Ring[Head];
         if (NextCall < s.frame->callCount)
         {
            NextDue = s.base + (uint32_t)(((uint64_t)s.frame->calls[NextCall].at * ArmPerCycleQ16) >> 16);
            HaveNext = true;
            return;
         }
         Head ^= 1;
         Count = Count - 1;
         NextCall = 0;
         FramesDone = FramesDone + 1;
      }
      HaveNext = false;
   }

   static FASTRUN bool snoop(uint16_t, bool)
   {
      if (!HaveNext) return false;
      const uint32_t due = NextDue;
      if (!Holding)
      {
         if (HoldOff || DMA_State != DMA_S_DisableReady || (int32_t)(ARM_DWT_CYCCNT + LeadArm - due) < 0) return false;
         DMA_State = DMA_S_StartAsynch;   // this interrupt's tail and the next ones assert /DMA
         Holding = true;
         return false;
      }
      if (DMA_State != DMA_S_TransferReady) return false;

      const Slot &s = Ring[Head];
      const SidCall &c = s.frame->calls[NextCall];
      const int32_t slack = (int32_t)(due - ARM_DWT_CYCCNT);
      if (slack < MinSlackArm) MinSlackArm = slack;
      if (slack < 0) LateCalls = LateCalls + 1;
      DMA_RnW = DMA_WRITE;
      DMA_FixC64Addr = DMA_ADDR_FIXED;
      const BusWrite *w = s.frame->writes + c.first;
      bool started = false;
      uint32_t cycle = 0, slip = 0, next = 0;
      while (next < c.count)
      {
         while (GP6_Phi2(ReadGPIO6));   // phi2 falling
         StartCycCnt = ARM_DWT_CYCCNT;
         if (!started) started = (int32_t)(StartCycCnt - due) >= 0;
         if (started)
         {
            if (cycle >= w[next].offset + slip)
            {
               uint8_t value = w[next].value;
               DMA_StartAddr = w[next].addr;
               if (DMAByte(&value))
               {
                  const uint16_t a = DMA_StartAddr;
                  if ((a & 0x1F) == 0x18)
                  {
                     if (ChipBase[1] && (a & 0xFFE0) == ChipBase[1]) LastVolume[1] = value;
                     else if (ChipBase[2] && (a & 0xFFE0) == ChipBase[2]) LastVolume[2] = value;
                     else if ((a & 0xFC00) == 0xD400) LastVolume[0] = value;   // $D418 or a mirror of it
                  }
                  next++;
               }
               else slip++;
            }
            cycle++;
         }
         while (!GP6_Phi2(ReadGPIO6));  // through phi2 rising, so each pass is one cycle
      }
      while (GP6_Phi2(ReadGPIO6));
      SetDMADeassert;
      DMA_State = DMA_S_DisableReady;
      Holding = false;
      Calls = Calls + 1;
      Writes = Writes + c.count;
      BaWaits = BaWaits + slip;
      NextCall++;
      aim();
      return true;
   }

   // clockHz: the emulated machine's clock, so a tune plays at its own rate on either machine.
   // sid2, sid3: the header's extra chips (sidExtraAddr), 0 when absent.
   static FLASHMEM void begin(uint32_t clockHz, uint16_t sid2, uint16_t sid3)
   {
      ChipBase[0] = 0xD400; ChipBase[1] = sid2; ChipBase[2] = sid3;
      LastVolume[0] = LastVolume[1] = LastVolume[2] = 0;
      ArmPerCycleQ16 = (uint32_t)(((uint64_t)F_CPU_ACTUAL << 16) / clockHz);
      LeadArm = (uint32_t)(((uint64_t)LeadCycles * ArmPerCycleQ16) >> 16);
      Head = Count = 0;
      NextCall = 0;
      HaveNext = Holding = HoldOff = false;
      FramesDone = Calls = Writes = BaWaits = LateCalls = 0;
      MinSlackArm = INT32_MAX;
      fBusSnoop = snoop;
      asm volatile("" ::: "memory");   // fBusSnoop is not volatile: keep the store
   }

   // Asks for no more holds and waits for a call that already has one to finish; false if it
   // does not within waitmS. Stays held off until resume or begin.
   static FLASHMEM bool holdOff(uint32_t waitmS)
   {
      HoldOff = true;
      const uint32_t began = millis();
      while (Holding)
         if (millis() - began >= waitmS) return false;
      return true;
   }

   // Takes the snoop out once no call holds the bus, so no hold is left asserted.
   static FLASHMEM void end()
   {
      holdOff(100);
      HaveNext = false;
      Count = 0;
      fBusSnoop = nullptr;
      asm volatile("" ::: "memory");   // fBusSnoop is not volatile: keep both stores
   }

   // Ends a holdOff with the queued calls moved later, so the next one is due leadArm from now
   // and the rest keep their spacing. Returns how far they moved (mod 2^32, as every time here
   // is), for the caller's own clock; 0 with an empty ring.
   static FLASHMEM uint32_t resume(uint32_t leadArm)
   {
      __disable_irq();
      uint32_t moved = 0;
      if (HaveNext)
      {
         moved = ARM_DWT_CYCCNT + leadArm - NextDue;
         NextDue = NextDue + moved;
         for (uint8_t i = 0; i < Count; i++) Ring[(Head + i) & 1].base += moved;
      }
      HoldOff = false;
      __enable_irq();
      return moved;
   }

   // False when the ring is full.
   static FLASHMEM bool queue(const SidFrame *frame, uint32_t base)
   {
      __disable_irq();
      const bool room = Count < 2;
      if (room)
      {
         Ring[(Head + Count) & 1] = Slot{frame, base};
         Count = Count + 1;
         if (!HaveNext) aim();
      }
      __enable_irq();
      return room;
   }

   // Whether a hold is in progress, or the next queued call is due within arm cycles of now and
   // the player is not held off (a paused player's next call is overdue, and waits anyway).
   static FLASHMEM bool busySoon(uint32_t arm)
   {
      __disable_irq();
      const bool soon = Holding || (!HoldOff && HaveNext && (int32_t)(NextDue - ARM_DWT_CYCCNT) < (int32_t)arm);
      __enable_irq();
      return soon;
   }
}
