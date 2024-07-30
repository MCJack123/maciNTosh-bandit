#define _LANGUAGE_ASSEMBLY
#include "asm.h"

.text

	.extern FwMain
	.extern BugcheckTrampoline
	.extern __executable_start
	.globl _start
_start:
// We are currently in little endian mode,
// in non-translated mode,
// with interrupts disabled.
	// quick debug test. are we getting here?
	// r28=ffff_ffff, r30=physaddr of framebuffer
	stw r28, 0(r30)
	stw r28, 4(r30)
	stw r28, 8(r30)
	stw r28, 0xc(r30)

	li r19, 6
	bl BlitScreen
	
// r3 is the physical address of our hardware description struct.
// Do not bother touching it right now.
// In fact, save it in a higher register:
	mr r31, r3

// Following init code comes from libogc:

	// clear all BATs
	li		r0,0
	mtspr	IBAT0U,r0; mtspr	IBAT1U,r0; mtspr	IBAT2U,r0; mtspr	IBAT3U,r0		// IBAT0...3
	mtspr	DBAT0U,r0; mtspr	DBAT1U,r0; mtspr	DBAT2U,r0; mtspr	DBAT3U,r0		// DBAT0...3
	isync
	
	// Invalidate all TLBs
	// Comes from mario kart wii forum - ppc pagetable tutorial
	// Open Firmware used pagetables so TLBs have been used so invalidate them:
	li r0,64
	li r3,0
	// Wipe SDR1 here:
	sync
	mtspr 25, r3
	isync

	//li r19, 7
	//bl BlitScreen
	
	mtctr r0
invalidate_tlb_loop:
	tlbie r3
	addi r3, r3, 0x1000
	bdnz+ invalidate_tlb_loop
after_invalidate_tlb:
	tlbsync
	
	// clear all SRs
	lis		r0,0x8000
	mtsr	0,r0; mtsr	1,r0; mtsr	2,r0; mtsr	3,r0; mtsr	4,r0; mtsr	5,r0; mtsr	6,r0
	mtsr	7,r0; mtsr	8,r0; mtsr	9,r0; mtsr	10,r0; mtsr	11,r0; mtsr	12,r0; mtsr	13,r0
	mtsr	14,r0; mtsr	15,r0
	isync
	
	// set DBAT0 and IBAT0:
	// 0x8000_0000 + 256MB => physical 0x0000_0000 (cached, r+w)
	li		r3,2
	lis		r4,0x8000
	ori		r4,r4,0x1fff
	mtspr	IBAT0L,r3
	mtspr	IBAT0U,r4
	mtspr	DBAT0L,r3
	mtspr	DBAT0U,r4
	isync
	
	// Grackle can in theory address up to 1GB, but that would use all available BATs.
	// A BAT has been set to map the first 256MB, that should be more than enough for ARC firmware purposes.
	// When the ARC firmware memory map, just set any memory above 256MB if present as firmware temporary,
	// In this case, boot loaders will not use it; but NT kernel indeed can.
	
	// set DBAT1:
	// 0xc000_0000 + 256MB => physical 0x9000_0000 (uncached, r+w)
	lis r3, 0x9000
	ori r3, r3, 0x2a
	lis r4, 0xc000
	ori r4, r4, 0x1fff
	mtspr DBAT1L, r3
	mtspr DBAT1U, r4
	isync
	
	// set DBAT2:
	// 0xf000_0000 + 256MB => physical 0xf000_0000 (uncached, r+w)
	// Bandit PCI configuration space is here, map the entire 256MB.
	lis r3, 0xf000	
	ori r3, r3, 0x2a
	lis r4, 0xf000
	ori r4, r4, 0x1fff
	mtspr DBAT2L, r3
	mtspr DBAT2U, r4
	isync

	// set DBAT3:
	// 0x9000_0000 + 256MB => physical 0x0000_0000 (uncached, r+w)
	li		r3, 0x2a
	lis		r4, 0x9000
	ori		r4, r4, 0x1fff
	mtspr	DBAT3L, r3
	mtspr	DBAT3U, r4
	isync

	// set up bugcheck vectors
	li r3, 0x0000
	lis r4, BugcheckTrampoline@h
	ori r4, r4, BugcheckTrampoline@l
	xoris r4, r4, 0x8000 // virt -> phys
bugcheck_init_loop:
	lwz r5, 0(r4)
	stw r5, 0(r3)
	lwz r5, 4(r4)
	stw r5, 4(r3)
	lwz r5, 8(r4)
	stw r5, 8(r3)
	lwz r5, 12(r4)
	stw r5, 12(r3)
	dcbst 0, r3
	addi r3, r3, 0x0100
	cmpwi r3, 0x2000
	blt bugcheck_init_loop
	sync
	isync

	// clear DBAT3 (allows null pointer dereferences to fault)
	//li r0, 0
	//mtspr DBAT3L, r0
	//mtspr DBAT3U, r0
	//isync

	//stw r28, 0(r30)
	//dcbst 0, r30
	
	// set up a stack:
	// we are at 9MB, use ram before it for stack.
	// this way we can set up 1MB at 8MB as firmware temporary.
	lis r1, __executable_start@h
	ori r1, r1, __executable_start@l
	subi r1, r1, 8
	//stwu r28, 4(r30)
	//dcbst 0, r30

	li r19, 8
	bl BlitScreen
	
	// switch into translated mode and jump to FwMain
	mr r3, r31
	oris r3, r3, 0x8000
	//stwu r28, 4(r30)
	//dcbst 0, r30
	lis r5, FwMain@h
	ori r5, r5, FwMain@l
	//stwu r5, 4(r30)
	//dcbst 0, r30
	mtsrr0 r5
	mfmsr r4
	ori r4, r4, MSR_DR|MSR_IR
	mtsrr1 r4
	stw r5, 0(r30)
	dcbst 0, r30
	sync
	rfi

.globl BlitScreen
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
