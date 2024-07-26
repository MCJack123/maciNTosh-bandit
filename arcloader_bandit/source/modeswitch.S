#define _LANGUAGE_ASSEMBLY
#include "asm.h"

.text

.global ModeSwitchEntry
ModeSwitchEntry: // (ArcFirmEntry Start, PHW_DESCRIPTION HwDesc)

// save our arguments
// r3 (le entrypoint) into srr0
	mr r29, r3
// r4 (argument) into r31
	mr r31, r4
	mr r30, r5
	li r28,-1

	li r19, 0
	bl BlitScreen

// Disable interrupts.
	mfmsr r7
	rlwinm r8, r7, 0, 17, 15
	mtmsr r8
	isync

// r4 = CONFIG_ADDR
	lis r4, 0xF280
// r5 = CONFIG_DATA
	lis r5, 0xF2C0
// r6 = MODE_SELECT addr
	lis r6, 0x0001
	ori r6, r6, 0x5850

// Ensure we can access bandit pci config registers through the MMU:
	lwbrx r0,0,r4
	lwbrx r0,0,r5

	li r19, 1
	bl BlitScreen

// All exceptions lead to infinite loop. No exceptions.
	li r0,0x10
	mtctr r0
	li r7,0
	lis r8, 0x4800 // b .
exception_wipe_loop:
	stw r8, 4(r7)
	addi r7, r7, 0x100
	bdnz+ exception_wipe_loop

	li r19, 2
	bl BlitScreen

// Set up bandit to r/w MODE_SELECT
	stwbrx r6,0,r4
	eieio
	sync
	lwbrx r0,0,r4
	sync
// Unset BE_MODE in MODE_SELECT
	lwbrx r7,0,r5
	sync
	li r7,0x06
	stwbrx r7,0,r5
	eieio
	sync

	li r19, 3
	bl BlitScreen

// Set MSR_ILE now PCI bus is endian swapping and interrupts are disabled
	mfmsr r7
	lis r8, 1
	or r7, r7, r8
	mtmsr r7
	isync

	li r19, 4
	bl BlitScreen

// set up srr1 ready to set MSR_LE, disable MSR_IR|MSR_DR
	ori r7, r7, 1
	rlwinm r7, r7, 0, 28, 25
	mtsrr1 r7
	mtsrr0 r29

	li r19, 5
	bl BlitScreen

// srr0 already set up
// set the hwdesc arg (vaddr):
	oris r3, r31, 0x8000
// and rfi into le entrypoint
	rfi

// debugging: draw some colored pixels on the framebuffer
// r19 = color, r30 = framebuffer ptr
BlitScreen:
	li r0, 0
	mr r16, r30
	addis r17, r16, 0x0005
	subi r17, r17, 0x5000
BlitScreen_loop:
	stb r19, 0(r16)
	addi r16, r16, 1
	cmpw r16, r17
	ble BlitScreen_loop

	mr r16, r30
BlitScreen_loop2:
	dcbst 0, r16
	addi r16, r16, 8
	cmpw r16, r17
	blt BlitScreen_loop2

	sync
	isync
	blr
