// SPDX-License-Identifier: MIT
//
// Build profile for the SID host extension.
//
// Close to ExampleHost's, for the same reason: this file and the .ino are what a
// host owes the build, and the minimal sketch it is overlaid onto supplies the
// rest. FeatVMHost is defined because it is what lets isrPHI2 run DMA transfers
// in an extension image (ISRs.c); SIDHost.ino supplies the two hooks the
// EasyFlash handler then expects, and loads no modules.
//
//   node tools/build-host.mjs        (runs the core's tools/build-firmware.mjs --target tr-plus
//                                      --host-sketch host --host-code-kib 96)
//
// 96 KiB because the emulator does not fit beside the inherited cartridge handlers
// in 64; the module window that costs is one this host never uses.
#if !defined(VM_HOST_PROFILE) || !defined(USB_DISABLED)
#error "Build this image with tools/build-firmware.mjs --host-sketch"
#endif

#define MinimumBuild         //Must be defined for minimal build to identify in common files
#define FeatVMHost           //isrPHI2 runs DMA transfers

// No Ethernet and no bank swapping. RAM_Image holds the 8 KB cartridge the C64
// runs, copied out of flash because the PHI2 ISR may not read flash.
#define Num8kSwapBuffers  2
#define EthernetDeduction 0
#define MaxRAM_ImageSize  16
