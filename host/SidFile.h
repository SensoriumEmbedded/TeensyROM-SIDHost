// SPDX-License-Identifier: MIT
//
// PSID/RSID header parsing, per HVSC's SID_file_format.txt (v1 through v4).
// Plain C++ with no Arduino dependencies: the native harness in
// tests/ compiles this file too.
#pragma once
#include <stdint.h>
#include <string.h>

struct SidHeader
{
   bool rsid;
   uint16_t version, dataOffset;
   uint16_t loadAddr, initAddr, playAddr;   // loadAddr is the effective one, taken from the data when the header says 0
   uint16_t songs, startSong;               // startSong is 1-based, as in the file
   uint32_t speed;                          // one bit per song: 0 = VBI, 1 = CIA timer
   char name[33], author[33], released[33]; // NUL-terminated copies
   uint16_t flags;                          // v2+: bit 0 MUS, bit 1 PlaySID-specific / C64 BASIC, bits 2-3 clock, 4-9 SID models
   uint8_t startPage, pageLength;           // v2+: free pages for a driver (0 = clean, 0xFF = none)
   uint8_t sid2Addr, sid3Addr;              // v3/v4: middle byte of $Dxx0, 0 = none
   bool musPlayer, basic;
   uint8_t clock;                           // 0 unknown, 1 PAL, 2 NTSC, 3 both
   const uint8_t *data;                     // the C64 image, without any embedded load address
   uint32_t dataLen;
   uint16_t loadEnd;                        // last address the image occupies
};

static inline uint16_t sidBE16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t sidBE32(const uint8_t *p) { return ((uint32_t)sidBE16(p) << 16) | sidBE16(p + 2); }

// False for anything the format description says a player must reject.
static inline bool sidParse(const uint8_t *file, uint32_t len, SidHeader &h)
{
   memset(&h, 0, sizeof h);
   if (len < 0x76) return false;
   if (!memcmp(file, "PSID", 4)) h.rsid = false;
   else if (!memcmp(file, "RSID", 4)) h.rsid = true;
   else return false;
   h.version = sidBE16(file + 4);
   h.dataOffset = sidBE16(file + 6);
   if (h.version < 1 || h.version > 4) return false;
   if (h.dataOffset != (h.version == 1 ? 0x76 : 0x7C) || h.dataOffset > len) return false;
   if (h.rsid && h.version < 2) return false;
   h.loadAddr = sidBE16(file + 8);
   h.initAddr = sidBE16(file + 0x0A);
   h.playAddr = sidBE16(file + 0x0C);
   h.songs = sidBE16(file + 0x0E);
   h.startSong = sidBE16(file + 0x10);
   h.speed = sidBE32(file + 0x12);
   if (h.songs < 1 || h.songs > 256) return false;
   if (h.startSong < 1 || h.startSong > h.songs) h.startSong = 1;
   memcpy(h.name, file + 0x16, 32);
   memcpy(h.author, file + 0x36, 32);
   memcpy(h.released, file + 0x56, 32);
   if (h.version >= 2)
   {
      h.flags = sidBE16(file + 0x76);
      h.startPage = file[0x78];
      h.pageLength = file[0x79];
      h.musPlayer = h.flags & 1;
      h.basic = h.rsid && (h.flags & 2);
      h.clock = (h.flags >> 2) & 3;
      if (h.version >= 3) h.sid2Addr = file[0x7A];
      if (h.version >= 4) h.sid3Addr = file[0x7B];
   }
   if (h.rsid && (h.loadAddr || h.playAddr || h.speed)) return false;
   if (h.rsid && h.basic && h.initAddr) return false;
   if (h.musPlayer) return false;  // Compute!'s Sidplayer data needs a player this code does not carry

   const uint8_t *p = file + h.dataOffset;
   uint32_t n = len - h.dataOffset;
   if (h.loadAddr == 0)
   {
      if (n < 2) return false;
      h.loadAddr = (uint16_t)(p[0] | (p[1] << 8));
      p += 2; n -= 2;
   }
   if (n == 0 || h.loadAddr + n > 0x10000) return false;
   if (h.initAddr == 0 && !h.basic) h.initAddr = h.loadAddr;
   h.data = p;
   h.dataLen = n;
   h.loadEnd = (uint16_t)(h.loadAddr + n - 1);
   return true;
}

// The speed bit for a 0-based song, with the two historical interpretations of
// songs past 32: v1 and PlaySID-specific files wrap, v2NG files reuse bit 31.
static inline bool sidSpeedIsCia(const SidHeader &h, uint16_t song0)
{
   if (h.rsid) return false;
   uint8_t bit = (h.version == 1 || (h.flags & 2)) ? (song0 & 31) : (song0 > 31 ? 31 : song0);
   return (h.speed >> bit) & 1;
}

// Only a valid middle byte names a second or third SID: $D420-$D7F0 and $DE00-$DFE0, even.
static inline uint16_t sidExtraAddr(uint8_t middle)
{
   if (middle & 1) return 0;
   if ((middle >= 0x42 && middle <= 0x7E) || (middle >= 0xE0 && middle <= 0xFE)) return (uint16_t)(0xD000 | (middle << 4));
   return 0;
}
