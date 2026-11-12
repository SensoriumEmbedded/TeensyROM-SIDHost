// SPDX-License-Identifier: MIT
//
// The capture file: every SID write a real C64 made while it played a tune, with
// its bus cycle, as SIDHost's capture mode records it from the bus and as the
// native harness writes it from the emulator (--write-cap) for the comparison
// (--compare). Little-endian, which both the Teensy and a PC are.
//
// The driver's play marker (SidMachine::DRIVER_MARKER) is recorded as a write
// like the others; it splits the log into play calls.
#pragma once
#include <stdint.h>

struct SidCapHeader
{
   char magic[8];       // "SIDCAP1\0"
   uint16_t version;    // SidCapVersion
   uint8_t flags;       // SidCapNtsc, SidCapScreenOn
   uint8_t song;        // 0-based
   uint32_t count;      // records that follow
   uint32_t cycles;     // bus cycles the capture ran
   uint8_t reserved[12];
};
static_assert(sizeof(SidCapHeader) == 32, "the header is 32 bytes on both sides");

struct SidCapRecord
{
   uint32_t cycle;      // bus cycle, counted from the start of the capture
   uint16_t addr;
   uint8_t value;
   uint8_t pad;
};
static_assert(sizeof(SidCapRecord) == 8, "a record is 8 bytes on both sides");

enum : uint16_t { SidCapVersion = 1 };
enum : uint8_t { SidCapNtsc = 1, SidCapScreenOn = 2 };
static constexpr char SidCapMagic[8] = {'S', 'I', 'D', 'C', 'A', 'P', '1', 0};
