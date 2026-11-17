; SPDX-License-Identifier: MIT
;
; The C64 half of the SID host extension (host/): an 8 KB
; cartridge the host serves at $8000 while the tune plays on the Teensy.
;
; It draws nothing and plays nothing. The host writes the SID and the screen
; by DMA; this program only keeps the machine in a state where that is safe
; and cheap, and passes the keyboard to the host:
;
;   - $01 = $37, so $D000-$DFFF is I/O and a DMA write to $D4xx reaches the SID
;     and one to $0400/$D800 reaches the screen.
;   - The KERNAL's own interrupt scans the keyboard; the main loop takes each
;     key with GETIN and publishes it in the mailbox below.
;   - Between keys the loop alternates a store and a load, so the DMA engine's
;     safe-assert rule (a read right after a write) is met within a few cycles
;     instead of after its read-loop timeout.
;   - The loop runs from RAM, not from the cartridge. A cartridge read is the
;     PHI2 interrupt's longest path (it drives the bus until the data hold
;     time), and every cycle it spends there is taken from the emulator.
;
; The cartridge is never switched off: after the copy nothing reads it, writes
; to $8000-$9FFF still reach RAM, and the VIC sees RAM there, so every VIC bank
; stays usable for the host's visuals.
;
; Mailbox, in the cassette buffer (the host reads $033C-$033F with one 4-byte
; DMA read; SIDHost.ino's Mailbox constants must match):
;   $033C  sequence, incremented after each new key
;   $033D  the key, as GETIN returns it (PETSCII)
;   $033E  the modifier keys at that moment ($028D: 1 Shift, 2 C=, 4 Ctrl)
;   $033F  $A5 once this program has finished initializing
;   $0340  scratch for the idle loop's store/load pair
;   $0341  the main loop, copied here from the cartridge

MBSEQ      = $033C
MBKEY      = $033D
MBMOD      = $033E
MBREADY    = $033F
SCRATCH    = $0340
LOOPADDR   = $0341
READY      = $A5

SHFLAG     = $028D
COLOR      = $0286

IOINIT     = $FDA3
RAMTAS     = $FD50
RESTOR     = $FD15
CINT       = $FF5B
CHROUT     = $FFD2
GETIN      = $FFE4

* = $8000
        !word coldstart          ; cold start
        !word coldstart          ; warm start (RUN/STOP+RESTORE): start over
        !byte $C3, $C2, $CD, $38, $30    ; CBM80

coldstart:
        sei
        ldx #$FF
        txs
        cld
        ; The KERNAL jumps straight here on finding the cartridge, before it has
        ; initialized anything, so the cartridge does it.
        jsr IOINIT
        jsr RAMTAS
        jsr RESTOR
        jsr CINT
        lda #$2F
        sta $00
        lda #$37
        sta $01
        lda #0
        sta $D020
        sta $D021
        sta MBSEQ
        sta MBKEY
        sta MBMOD
        lda #1
        sta COLOR
        ldx #0
-       lda banner,x
        beq +
        jsr CHROUT
        inx
        bne -
+       ldx #looplen-1
-       lda loopsrc,x
        sta LOOPADDR,x
        dex
        bpl -
        lda #READY
        sta MBREADY
        cli
        jmp LOOPADDR

loopsrc:
!pseudopc LOOPADDR {
main:
        jsr GETIN
        beq idle
        sta MBKEY
        lda SHFLAG
        sta MBMOD
        inc MBSEQ
idle:
        sta SCRATCH
        lda SCRATCH
        jmp main
}
looplen = * - loopsrc

; Shown until the host draws its own screen, so a host that never reaches
; the C64 is told from a stub that never started.
banner:
        !pet "sidhost", 13, 0
