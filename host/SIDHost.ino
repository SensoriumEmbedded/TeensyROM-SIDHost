// SPDX-License-Identifier: MIT
//
// The SID host extension: plays a .sid on a TeensyROM+ by emulating the C64 the
// tune runs in (SidMachine) and replaying its SID writes to the real chip by DMA,
// while the C64 itself runs only the small cartridge in Source/C64/SIDHostStub/.
//
// It boots from the launch record the menu's Shift+Return leaves, serves the
// cartridge, draws the tune's header on the C64 screen by DMA, times the emulator
// on this board, plays the tune's start song (SidBus.h puts every write on its
// cycle), reads keys from the cartridge's mailbox, and hands the machine back to
// the menu.
//
//   node tools/build-firmware.mjs --target tr-plus --host-sketch Source/Teensy/SIDHost --host-code-kib 96
//   python3 tools/bench/hostinstall.py build/firmware/TeensyROM+_<ver>_SIDHost.TRH
//
// Keys on the C64: X returns to the menu (so does the menu button, or resetting
// the C64).

#include <SD.h>
#include <EEPROM.h>
#include "Min_TeensyROM.h"
#include "Common/Common_Defs.h"
#include "Common/Menu_Regs.h"
#include "Common/DriveDirLoad.h"
#include "Common/C64Spans.h"
#include "Common/DMAControl.c"
#include "Common/IOHandlers.h"
#include "Common/MachineDetect.h"

uint8_t RAM_Image[RAM_ImageSize];
volatile uint8_t BtnPressed = false;
volatile uint8_t EmulateVicCycles = false;
uint8_t CurrentIOHandler = IOH_None;
StructMenuItem DriveDirMenu;
char DriveDirPath[MaxPathLength];
uint16_t LOROM_Mask, HIROM_Mask;
Stream *CmdChannel = &Serial;

#include "Common/ISRs.c"
#include "Common/VMHostABI.h"
#include "Common/VMFail.h"
#include "Common/VMRegistry.h"
#include "SidMachine.h"
#include "SidBus.h"
#include "SIDHostStub.bin.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

__attribute__((used, section(".vmhostid")))
const VmHostId vmHostId = { VM_HOSTID_MAGIC, VM_ABI, 0, 0, "SIDHost", VM_HOST_CODE_FLOOR };

// The two hooks IOH_EasyFlash.c declares under FeatVMHost. This host never selects
// the EasyFlash handler, so they are never reached; they exist to link.
bool VMHostIO2(uint8_t, bool) { return false; }
void VMHostPoll() {}

// The cartridge's mailbox (Source/C64/SIDHostStub/source/SIDHostStub.asm).
enum : uint16_t { MailboxAddr = 0x033C };
enum : uint8_t { MbSeq, MbKey, MbMod, MbReady, MailboxBytes, MailboxReady = 0xA5 };

// The detail VmFail::HostReturned carries back to the menu, which prints it. ExitStubSilent
// adds what the bus showed (StubSilentDetail), since this image has no other way to say it.
enum : uint32_t { ExitKey = 0, ExitButton = 1, ExitStubSilent = 2, ExitDmaFailed = 3 };

// RAM2 belongs to this image whole (it loads no modules), but the extension linker
// script refuses globals there, so the emulator's memory is addressed, not declared.
// Everything stays below VM_FAIL_BASE, where the record the menu reads lives.
enum : uint32_t {
   Ram2Base     = 0x20200000u,
   EmuRamAddr   = Ram2Base,                 // 64 KB
   EmuKernAddr  = EmuRamAddr + 0x10000,     // 8 KB
   EmuBasicAddr = EmuKernAddr + 0x2000,     // 8 KB
   EmuColorAddr = EmuBasicAddr + 0x2000,    // 1 KB
   FileAddr     = EmuColorAddr + 0x400,     // the .sid file: header, load address, 64 KB of data
   FileMax      = 0x7C + 2 + 0x10000,
   WritesAddr   = (FileAddr + FileMax + 31) & ~31u,
   WritesMax    = 4096,                     // one frame's writes, before they are cut into calls
   SlotCallsMax  = 1024,                    // per frame slot; the HVSC corpus peaks at 258 calls and 419 writes a frame
   SlotWritesMax = WritesMax,
   SlotCallsAddr  = WritesAddr + WritesMax * sizeof(SidWrite),
   SlotWritesAddr = SlotCallsAddr + 2 * SlotCallsMax * sizeof(SidCall),
   Ram2End      = SlotWritesAddr + 2 * SlotWritesMax * sizeof(BusWrite),
};
static_assert((uint32_t)Ram2End <= (uint32_t)VM_FAIL_BASE, "emulator buffers overlap the fail record");

enum : uint32_t {
   StubWaitmS      = 500,    // after reset release: no DMA in a C128's first ~80 mS, and the KERNAL init runs
   StubTimeoutmS   = 5000,   // a stub that has not said ready by then never will
   PollmS          = 20,     // mailbox reads, about once a frame
   BenchFrames     = 50,
   DmaFailLimit    = 10,     // consecutive failed mailbox reads before giving up on the bus
   StartLeadmS     = 20,     // the first frame is due this long after both slots are full
   StatsmS         = 1000,
   DmaGuarduS      = 300,    // a screen or mailbox transfer waits for a gap this long before the next call
   DmaGuardWaitmS  = 20,     // but not longer: a tune that writes all the time leaves none
   DmaIdleWaitmS   = 100,    // a call already holding the bus is over long before this
};

// The emulated machine's clock: the tune plays at its own rate on either machine.
enum : uint32_t { PalClockHz = 985248, NtscClockHz = 1022727 };

static SidMachine Machine;
static SidHeader Tune;
static bool TuneLoaded;
static char Status[41];           // why there is no tune, when there is none
static char TunePath[sizeof(VmLaunchRecord::content)];

static SidFramePair Frames;
static bool Playing, Paused;
static uint8_t CurrentSong;       // 0-based
static uint32_t FramesReleased, Underruns, Overflows;
static uint64_t NextBaseQ16;      // ARM cycle at which the next queued frame's cycle 0 is due, 16.16
static uint32_t EmuFrames, EmuMaxArm;
static uint64_t EmuTotalArm;

static uint32_t ResetReleasedmS, LastPollmS, LastStatsmS;
static bool StubReady;
static uint8_t LastSeq, DmaFails;

// ------------------------------------------------------------- the way back

FLASHMEM void ReturnToMenu(uint32_t detail)
{
   SidPlayer::end();
   SetResetAssert;
   VmFail::set(VmFail::HostReturned, detail);
   EEPROM.write(VM_EEP_BOOTIND_ADDR, VM_BOOT_FROM_MIN);
   delay(10);   // let the EEPROM write complete before the reset
   RebootTR();
   while (true) ;
}

// ------------------------------------------------------------ C64 by DMA

static FLASHMEM void ServicePlayback();

// One transfer, then the bus back to the 6510: PerformDMA leaves /DMA asserted, and the C64
// stopped, until CloseDMA. While a tune plays, the transfer goes between its calls: a call due
// during it would start late, and one the player has already started holds the bus. The
// emulator keeps the player fed while the transfer waits for its gap.
static FLASHMEM bool C64Dma(DMA_Trans_RnW rnw, uint16_t addr, uint8_t *buf, uint32_t len, DMA_Addr_Mode mode)
{
   uint32_t began = millis();
   while (SidPlayer::busySoon(DmaGuarduS * (F_CPU_ACTUAL / 1000000)) && millis() - began < DmaGuardWaitmS)
      if (Playing) ServicePlayback();
   const bool heldOff = SidPlayer::HoldOff;   // a pause keeps the player held off after this
   SidPlayer::HoldOff = true;
   began = millis();
   while (SidPlayer::Holding || DMA_State != DMA_S_DisableReady)
      if (millis() - began >= DmaIdleWaitmS) { SidPlayer::HoldOff = heldOff; return false; }
   const bool ok = PerformDMA(rnw, addr, buf, len, mode) && CloseDMA();
   SidPlayer::HoldOff = heldOff;
   return ok;
}

// Bits 8-15: CIA1's control register A, 0 after a reset and running once the KERNAL's IOINIT
// has started the keyboard timer, so it tells a 6510 that never ran from a cartridge that
// crashed after it. Bits 16-23: the mailbox's ready byte. Bit 24: the reset line, 0 if
// something still holds it low.
static FLASHMEM uint32_t StubSilentDetail(uint8_t ready)
{
   uint8_t cra = 0xEE;   // stays this if the read fails
   C64Dma(DMA_READ, 0xDC0E, &cra, 1, DMA_ADDR_FIXED);
   const uint32_t reset = digitalReadFast(BiDir_Reset_PIN) ? 1 : 0;
   return ExitStubSilent | (uint32_t)cra << 8 | (uint32_t)ready << 16 | reset << 24;
}

static FLASHMEM uint8_t ScreenCode(char c)
{
   if (c >= 'a' && c <= 'z') return c - 'a' + 1;
   if (c >= '@' && c <= ']') return c - '@';
   if (c >= ' ' && c <= '?') return c;
   if (c == '_') return 0x64;   // no underscore in the character set; the low line graphic is closest
   return '?';
}

static FLASHMEM bool PutRow(uint8_t row, const char *text, uint8_t color)
{
   uint8_t cells[40];
   uint8_t n = 0;
   for (; n < 40 && text[n]; n++) cells[n] = ScreenCode(text[n]);
   for (; n < 40; n++) cells[n] = ' ';
   if (!C64Dma(DMA_WRITE, 0x0400 + row * 40, cells, 40, DMA_ADDR_INCREMENT)) return false;
   memset(cells, color, 40);
   return C64Dma(DMA_WRITE, 0xD800 + row * 40, cells, 40, DMA_ADDR_INCREMENT);
}

// ------------------------------------------------------------- the tune

static FLASHMEM void LoadTune()
{
   VmLaunchRecord launch{};
   unsigned attempt = 1;
   while (!SD.sdfs.begin(SdioConfig(FIFO_SDIO)))
   {
      if (++attempt > 3) { strcpy(Status, "SD card did not start"); return; }
      delay(50);
   }
   if (!VmRegistry::consume(launch)) { strcpy(Status, "No launch record"); return; }
   strcpy(TunePath, launch.content);
   FsFile f = SD.sdfs.open(launch.content, O_RDONLY);
   if (!f || f.isDirectory()) { f.close(); strcpy(Status, "File not found"); return; }
   const uint32_t size = f.fileSize();
   uint8_t *file = (uint8_t *)FileAddr;
   const bool read = size <= FileMax && f.read(file, size) == (int)size;
   f.close();
   if (!read) { strcpy(Status, "File too large or unreadable"); return; }
   if (!sidParse(file, size, Tune)) { strcpy(Status, "Header rejected"); return; }
   if (Tune.basic) { strcpy(Status, "C64 BASIC tune: not yet"); return; }
   TuneLoaded = true;
}

// The tune's first BenchFrames frames from a fresh start, in microseconds: the average, the
// worst, and the init call. False if the driver found no free page.
static FLASHMEM bool TimeFrames(const SidMachineMem &mem, uint32_t &avg, uint32_t &worst, uint32_t &init)
{
   Machine.init(mem, Tune.clock == 2);
   if (!Machine.load(Tune)) return false;
   Machine.setWriteBuffer((SidWrite *)WritesAddr, WritesMax);
   const uint32_t perUs = F_CPU_ACTUAL / 1000000;
   const uint32_t start = ARM_DWT_CYCCNT;
   Machine.start((uint8_t)(Tune.startSong - 1));
   init = (ARM_DWT_CYCCNT - start) / perUs;
   uint32_t total = 0;
   worst = 0;
   for (uint32_t frame = 0; frame < BenchFrames; frame++)
   {
      Machine.clearWrites();
      const uint32_t t = ARM_DWT_CYCCNT;
      Machine.runFrame();
      const uint32_t spent = ARM_DWT_CYCCNT - t;
      total += spent;
      if (spent > worst) worst = spent;
   }
   avg = total / BenchFrames / perUs;
   worst /= perUs;
   return true;
}

// How long one frame of the tune takes to emulate on this board: with the PHI2 interrupt running
// as it will while the tune plays, and with it detached, which splits the interrupt's share from
// the emulator's own cost. The stub runs from RAM by now, so the C64 does not need the cartridge
// served while the interrupt is off.
static FLASHMEM void BenchEmulator(char *line, size_t cap, char *line2, size_t cap2)
{
   const SidMachineMem mem{(uint8_t *)EmuRamAddr, (uint8_t *)EmuKernAddr, (uint8_t *)EmuBasicAddr, (uint8_t *)EmuColorAddr};
   uint32_t avg, worst, init;
   if (!TimeFrames(mem, avg, worst, init)) { snprintf(line, cap, "No free page for the driver"); line2[0] = 0; return; }
   snprintf(line, cap, "Emu uS/frame avg %lu max %lu init %lu", (unsigned long)avg, (unsigned long)worst, (unsigned long)init);
   detachInterrupt(digitalPinToInterrupt(PHI2_PIN));
   TimeFrames(mem, avg, worst, init);
   attachInterrupt(digitalPinToInterrupt(PHI2_PIN), isrPHI2, RISING);
   NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
   snprintf(line2, cap2, "No PHI2 int: avg %lu max %lu init %lu", (unsigned long)avg, (unsigned long)worst, (unsigned long)init);
}

// ------------------------------------------------------------- playback

static FLASHMEM SidMachineMem EmuMem()
{
   return SidMachineMem{(uint8_t *)EmuRamAddr, (uint8_t *)EmuKernAddr, (uint8_t *)EmuBasicAddr, (uint8_t *)EmuColorAddr};
}

static FLASHMEM SidFrame EmptySlot(uint8_t n)
{
   return SidFrame{(SidCall *)SlotCallsAddr + n * SlotCallsMax, (BusWrite *)SlotWritesAddr + n * SlotWritesMax,
                   SlotCallsMax, SlotWritesMax, 0, 0, 0, 0, false};
}

// Hands a frame to the player with the time it is due. A frame due in the past means the
// emulator fell behind: the clock starts again from now, a tempo glitch instead of a burst
// of calls played back to back.
static FLASHMEM void QueueFrame(const SidFrame *f)
{
   uint32_t base = (uint32_t)(NextBaseQ16 >> 16);
   if ((int32_t)(base - ARM_DWT_CYCCNT) < 0)
   {
      Underruns++;
      base = ARM_DWT_CYCCNT + SidPlayer::LeadArm * 2;
      NextBaseQ16 = (uint64_t)base << 16;
   }
   SidPlayer::queue(f, base);
   NextBaseQ16 += (uint64_t)Machine.cyclesPerFrame() * SidPlayer::ArmPerCycleQ16;
}

static FLASHMEM const SidFrame *EmulateFrame()
{
   const uint32_t t = ARM_DWT_CYCCNT;
   const SidFrame *f = Frames.produce(Machine);
   const uint32_t spent = ARM_DWT_CYCCNT - t;
   EmuFrames++;
   EmuTotalArm += spent;
   if (spent > EmuMaxArm) EmuMaxArm = spent;
   if (f->overflow) Overflows++;
   return f;
}

static FLASHMEM void ShowSong()
{
   char line[41];
   snprintf(line, sizeof line, "Song %u of %u, %s timed%s", CurrentSong + 1, Tune.songs,
            sidSpeedIsCia(Tune, CurrentSong) ? "CIA" : "raster", Paused ? "  - paused" : "");
   PutRow(19, line, Paused ? 10 : 1);
}

static FLASHMEM bool StartPlayback(uint8_t song0)
{
   CurrentSong = song0;
   Paused = false;
   Machine.init(EmuMem(), Tune.clock == 2);
   if (!Machine.load(Tune)) return false;
   Machine.start(song0);
   Frames.init(EmptySlot(0), EmptySlot(1), (SidWrite *)WritesAddr, WritesMax);
   FramesReleased = Underruns = Overflows = 0;
   EmuFrames = EmuTotalArm = EmuMaxArm = 0;
   SidPlayer::begin(Machine.ntsc() ? NtscClockHz : PalClockHz, sidExtraAddr(Tune.sid2Addr), sidExtraAddr(Tune.sid3Addr));
   const SidFrame *first = EmulateFrame(), *second = EmulateFrame();
   NextBaseQ16 = (uint64_t)(ARM_DWT_CYCCNT + StartLeadmS * (F_CPU_ACTUAL / 1000)) << 16;
   QueueFrame(first);
   QueueFrame(second);
   Playing = true;
   return true;
}

// The player out and the chip silent, as a reset leaves it, before another song starts.
static FLASHMEM void StopPlayback()
{
   if (!Playing) return;
   SidPlayer::end();
   Playing = Paused = false;
   uint8_t regs[25]{};
   C64Dma(DMA_WRITE, 0xD400, regs, sizeof regs, DMA_ADDR_INCREMENT);
}

static FLASHMEM void ChangeSong(int step)
{
   if (!TuneLoaded || Tune.songs < 2) return;
   StopPlayback();
   if (!StartPlayback((uint8_t)((CurrentSong + Tune.songs + step) % Tune.songs))) PutRow(14, "No free page for the driver", 10);
   ShowSong();
}

// Each chip the tune writes gets its last volume register back, or with mute that register's
// filter mode bits and a volume of 0.
static FLASHMEM void SetVolumes(bool mute)
{
   for (uint8_t chip = 0; chip < 3; chip++)
   {
      if (!SidPlayer::ChipBase[chip]) continue;
      uint8_t volume = SidPlayer::LastVolume[chip] & (mute ? 0xF0 : 0xFF);
      C64Dma(DMA_WRITE, SidPlayer::ChipBase[chip] + 0x18, &volume, 1, DMA_ADDR_FIXED);
   }
}

// A pause stops the calls and turns the volume down, as a player on the C64 would; the queued
// frames wait, and resume moves them to now so the tune carries on where it stopped.
static FLASHMEM void TogglePause()
{
   if (!Playing) return;
   if (!Paused)
   {
      SidPlayer::holdOff(DmaIdleWaitmS);
      Paused = true;
      SetVolumes(true);
   }
   else
   {
      SetVolumes(false);
      const uint32_t lead = StartLeadmS * (F_CPU_ACTUAL / 1000);
      const uint32_t moved = SidPlayer::resume(lead);
      if (SidPlayer::HaveNext) NextBaseQ16 += (uint64_t)moved << 16;
      else NextBaseQ16 = (uint64_t)(ARM_DWT_CYCCNT + lead) << 16;
      Paused = false;
   }
   ShowSong();
}

// Frees the slots the player is done with and fills them: the emulator stays one frame ahead.
static FLASHMEM void ServicePlayback()
{
   if (Paused) return;
   for (const uint32_t done = SidPlayer::FramesDone; FramesReleased != done; FramesReleased++) Frames.release();
   if (Frames.ready() < 2) QueueFrame(EmulateFrame());
}

static FLASHMEM void ShowPlayStats()
{
   char line[48];
   const uint32_t perCycle = SidPlayer::ArmPerCycleQ16 >> 16, perUs = F_CPU_ACTUAL / 1000000;
   const int32_t slack = SidPlayer::MinSlackArm;
   snprintf(line, sizeof line, "Frames %lu calls %lu late %lu lead %ld",
            (unsigned long)SidPlayer::FramesDone, (unsigned long)SidPlayer::Calls, (unsigned long)SidPlayer::LateCalls,
            (long)(slack == INT32_MAX ? 0 : slack / (int32_t)perCycle));
   PutRow(14, line, 7);
   snprintf(line, sizeof line, "Emu uS avg %lu max %lu  underruns %lu",
            (unsigned long)(EmuFrames ? (uint32_t)(EmuTotalArm / EmuFrames) / perUs : 0), (unsigned long)(EmuMaxArm / perUs),
            (unsigned long)Underruns);
   PutRow(15, line, 7);
   snprintf(line, sizeof line, "Writes %lu BA wait cycles %lu", (unsigned long)SidPlayer::Writes, (unsigned long)SidPlayer::BaWaits);
   PutRow(16, line, 7);
   snprintf(line, sizeof line, "Chained %lu  frames dropping writes %lu", (unsigned long)SidPlayer::Chained, (unsigned long)Overflows);
   PutRow(17, line, Overflows ? 10 : 7);
}

static FLASHMEM void DrawScreen()
{
   char line[48], line2[48];
   for (uint8_t row = 0; row < 25; row++) PutRow(row, "", 1);
   PutRow(0, "SIDHost", 7);
   const char *name = strrchr(TunePath, '/');
   PutRow(2, name ? name + 1 : "(no file)", 15);   // PutRow keeps the first 40 characters
   if (TuneLoaded)
   {
      PutRow(3, Tune.name, 1);
      PutRow(4, Tune.author, 1);
      PutRow(5, Tune.released, 1);
      snprintf(line, sizeof line, "%s v%u $%04X-$%04X %s, start song %u",
               Tune.rsid ? "RSID" : "PSID", Tune.version, Tune.loadAddr, Tune.loadEnd,
               Tune.clock == 2 ? "NTSC" : "PAL", Tune.startSong);
      PutRow(6, line, 12);
      BenchEmulator(line, sizeof line, line2, sizeof line2);
      PutRow(8, line, 13);
      PutRow(9, line2, 13);
   }
   else PutRow(3, Status, 10);
   snprintf(line, sizeof line, "Machine: %s %s", (TimingVidTODClks & rvtcC128) ? "C128" : "C64",
            (TimingVidTODClks & rvtcNTSC) ? "NTSC" : "PAL");
   PutRow(10, line, 12);
   PutRow(23, "+/-: song  Space: pause  X: menu", 15);
}

static FLASHMEM void ShowKey(const uint8_t *mb)
{
   char line[41];
   snprintf(line, sizeof line, "Key $%02X  modifiers $%02X  count %u", mb[MbKey], mb[MbMod], mb[MbSeq]);
   PutRow(12, line, 1);
}

// --------------------------------------------------------------- setup

void setup()
{
   set_arm_clock(816000000);   // as the stock host: slight overclock, no cooling required
   SetLEDOn;

   for (uint8_t PinNum = 0; PinNum < sizeof(OutputPins); PinNum++) pinMode(OutputPins[PinNum], OUTPUT);
   SetAddrPortDirIn;
   SetAddrBufsIn;
   SetDataPortDirIn;
   SetDataBufIn;
   SetDMADeassert;
   SetIRQDeassert;
   SetNMIDeassert;
   pinMode(BiDir_Reset_PIN, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(BiDir_Reset_PIN), isrButton, FALLING);   // a C64 reset ends the host
   SetResetAssert;
   pinMode(Special_Btn_In_PIN, INPUT_PULLUP);
   for (uint8_t PinNum = 0; PinNum < sizeof(InputPins); PinNum++) pinMode(InputPins[PinNum], INPUT);
   pinMode(Menu_Btn_In_PIN, INPUT_PULLUP);
   pinMode(PHI2_PIN, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(Menu_Btn_In_PIN), isrButton, FALLING);
   attachInterrupt(digitalPinToInterrupt(PHI2_PIN), isrPHI2, RISING);
   NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);

   // Entered on purpose, or not at all (docs/Architecture/Extension-Hosts.md, "The marker").
   if (ReadButton == 0) runMainTRApp();
   uint32_t magic = 0;
   EEPROM.get(VM_EEP_MAGIC_ADDR, magic);
   if (magic != VM_EEP_MAGIC) runMainTRApp();
   char marker[5]{};
   EEPreadNBuf(VM_EEP_BOOTNAME_ADDR, (uint8_t *)marker, 4);
   if (strcmp(marker, VM_HOST_MARKER) != 0) runMainTRApp();

   // PAL/NTSC from PHI2, C64/C128 from the main image's pair, as the stock host.
   if (TimeVideoStdAtBoot() != 0xff) SetVideoStdDMATiming(MeasuredVidStd);
   uint8_t machine[2];
   EEPreadNBuf(VM_EEP_MACHINE_ADDR, machine, 2);
   const int Machine64 = vm_launch_machine(machine);
   if (Machine64 >= 0)
      SetVideoStdDMATiming(MeasuredVidStd == 0xff ? Machine64 : (uint8_t)((Machine64 & ~rvtcNTSC) | MeasuredVidStd));

   LoadTune();

   // The cartridge: 8 KB at ROML, served from RAM because the PHI2 ISR may not read flash.
   memset(RAM_Image, 0xFF, 8192);
   memcpy(RAM_Image, SIDHostStub_bin, sizeof SIDHostStub_bin);
   LOROM_Image = RAM_Image;
   HIROM_Image = NULL;
   LOROM_Mask = HIROM_Mask = 0x1fff;
   CurrentIOHandler = IOH_None;
   EmulateVicCycles = false;
   SetGameDeassert;
   SetExROMAssert;
   doReset = true;
   BtnPressed = !ReadButton;
}

// ---------------------------------------------------------------- loop

void loop()
{
   FollowVideoStd();
   if (BtnPressed) ReturnToMenu(ExitButton);

   if (doReset)
   {
      SetResetAssert;
      delay(50);
      SetResetInput;
      delay(50);   // debounce
      doReset = false;
      BtnPressed = false;
      Phi2ResetReleased();
      ResetReleasedmS = millis();
      return;
   }

   if (Playing) ServicePlayback();
   const uint32_t now = millis();
   if (Playing && now - LastStatsmS >= StatsmS) { LastStatsmS = now; ShowPlayStats(); }
   if (now - ResetReleasedmS < StubWaitmS || now - LastPollmS < (StubReady ? PollmS : 100)) return;
   LastPollmS = now;

   uint8_t mb[MailboxBytes];
   if (!C64Dma(DMA_READ, MailboxAddr, mb, sizeof mb, DMA_ADDR_INCREMENT))
   {
      if (++DmaFails >= DmaFailLimit) ReturnToMenu(ExitDmaFailed);
      return;
   }
   DmaFails = 0;

   if (!StubReady)
   {
      if (mb[MbReady] != MailboxReady)
      {
         if (now - ResetReleasedmS > StubTimeoutmS) ReturnToMenu(StubSilentDetail(mb[MbReady]));
         return;
      }
      StubReady = true;
      LastSeq = mb[MbSeq];
      DrawScreen();
      if (TuneLoaded)
      {
         if (StartPlayback((uint8_t)(Tune.startSong - 1))) ShowSong();
         else PutRow(14, "No free page for the driver", 10);
      }
      return;
   }

   if (mb[MbSeq] == LastSeq) return;
   LastSeq = mb[MbSeq];
   ShowKey(mb);
   switch (mb[MbKey])   // PETSCII, as GETIN returns it
   {
      case 'X': ReturnToMenu(ExitKey); break;
      case '+': ChangeSong(1); break;
      case '-': ChangeSong(-1); break;
      case ' ': TogglePause(); break;
   }
}

// The shared minimal sources call these; same bodies as MinimalBoot.ino.
void EEPwriteNBuf(uint16_t addr, const uint8_t* buf, uint8_t len)
{
   while (len--) EEPROM.write(addr+len, buf[len]);
}

void EEPwriteStr(uint16_t addr, const char* buf)
{
   EEPwriteNBuf(addr, (uint8_t*)buf, strlen(buf)+1); //include terminator
}

void EEPreadNBuf(uint16_t addr, uint8_t* buf, uint16_t len)
{
   while (len--) buf[len] = EEPROM.read(addr+len);
}

void EEPreadStr(uint16_t addr, char* buf)
{
   uint16_t CharNum = 0;

   do
   {
      buf[CharNum] = EEPROM.read(addr+CharNum);
   } while (buf[CharNum++] !=0); //end on termination, but include it in buffer
}
