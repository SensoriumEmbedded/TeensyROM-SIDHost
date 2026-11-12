// SPDX-License-Identifier: MIT
//
// The capture mode's bus snoop: while the real 6510 plays a tune (SIDHost.ino,
// Capture), every PHI2 cycle is counted, and every write to a SID, to the header's
// second or third SID, or to the driver's play marker is stored with its cycle in
// the format of SidCapture.h. Nothing else runs on the bus during a capture, so the
// count is the C64's own cycle count.
//
// The snoop is on the PHI2 path: a cycle costs an increment and a compare; a logged
// write also waits for the data, as a cartridge's IO write handler does.
#pragma once
#include "SidCapture.h"
#include "SidMachine.h"

namespace SidCapBus
{
   SidCapRecord *Buf;
   uint32_t Cap;
   volatile uint32_t Count, Cycle;
   uint16_t Sid2, Sid3;   // 0 or the chip's base, as sidExtraAddr gives it

   FASTRUN bool snoop(uint16_t addr, bool readCycle)
   {
      const uint32_t cycle = Cycle++;
      if (readCycle) return false;
      if ((addr & 0xFC00) == 0xD400 || addr == SidMachine::DRIVER_MARKER ||
          (Sid2 && (addr & 0xFFE0) == Sid2) || (Sid3 && (addr & 0xFFE0) == Sid3))
      {
         const uint8_t value = DataPortWaitRead();
         const uint32_t n = Count;
         if (n < Cap) { Buf[n] = SidCapRecord{cycle, addr, value, 0}; Count = n + 1; }
      }
      return false;
   }

   FLASHMEM void begin(SidCapRecord *buf, uint32_t cap, uint16_t sid2, uint16_t sid3)
   {
      Buf = buf;
      Cap = cap;
      Count = Cycle = 0;
      Sid2 = sid2;
      Sid3 = sid3;
      fBusSnoop = snoop;
      asm volatile("" ::: "memory");   // fBusSnoop is not volatile: keep the store
   }

   FLASHMEM void end()
   {
      fBusSnoop = nullptr;
      asm volatile("" ::: "memory");
   }
}
