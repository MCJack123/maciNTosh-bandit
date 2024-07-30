// This is the ARC loader.
// Its purpose is to load the ARC firmware and jump to its entry point in little endian mode passing it hardware information.
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include "arc.h"
#include "ofapi.h"
#include "elf_abi.h"
#include "hwdesc.h"
#include "fb.h"
#include "macmodes.h"
#include "controlfb.h"

static OFIHANDLE s_stdout = OFINULL;
static ULONG s_FirstFreePage;
static ULONG s_LastFreePage;
static ULONG s_PhysMemLength;
static ULONG s_GrandCentralStart;
static ULONG s_MrpFlags = 0;
extern volatile unsigned char *via;

//FILE* stdout = NULL;

static bool InitStdout(void) {
	OFHANDLE Handle = OfGetChosen();
	return ARC_SUCCESS(OfGetPropInt(Handle, "stdout", &s_stdout));
}

static void StdOutWrite(const char* str) {
	if (s_stdout == OFINULL) return;
	if (str[0] == 0) return;
	ULONG Count;
	OfWrite(s_stdout, str, strlen(str), &Count);
}

#if 0 // using printf uses ~2.5kb, we need every byte we can get to meet self-imposed 16kb limit
static void print_f(const char* fmt, ...) {
	if (s_stdout == OFINULL) return;
	va_list va;
	va_start(va, fmt);
	char Buffer[256];
	vsnprintf(Buffer, sizeof(Buffer), fmt, va);
	va_end(va);
	StdOutWrite(Buffer);
}
#endif

// comes from https://stackoverflow.com/questions/21501815/optimal-base-10-only-itoa-function
static char * _i32toa(char *const rtn, int i)    {
    if (NULL == rtn) return NULL;

    // declare local buffer, and write to it back-to-front
    char buff[12];
    uint32_t  ut, ui;
    char minus_sign=0;
    char *p = buff + sizeof(buff)-1;
    *p-- = 0;    // nul-terminate buffer

    // deal with negative numbers while using an unsigned integer
    if (i < 0)    {
        minus_sign = '-';
        ui = (uint32_t)((int)-1 * (int)i);
    }    else    {
        ui = i;
    }

    // core code here...
    while (ui > 9) {
        ut = ui;
        ui /= 10;
        *p-- = (ut - (ui * 10)) + '0';
    }
    *p = ui + '0';

    if ('-' == minus_sign) *--p = minus_sign;

    // knowing how much storage we needed, copy chars from buff to rtn...
    memcpy(rtn, p, sizeof(buff)-(p - buff));

    return rtn;
}

const char * const hexmap = "0123456789ABCDEF";

static void print_hex(unsigned long num) {
	char buffer[9] = {
		hexmap[(num >> 28) & 0x0F],
		hexmap[(num >> 24) & 0x0F],
		hexmap[(num >> 20) & 0x0F],
		hexmap[(num >> 16) & 0x0F],
		hexmap[(num >> 12) & 0x0F],
		hexmap[(num >> 8) & 0x0F],
		hexmap[(num >> 4) & 0x0F],
		hexmap[num & 0x0F], 0
	};
	StdOutWrite(buffer);
}

static void print_error(int error) {
	char buffer[12];
	_i32toa(buffer, error);
	StdOutWrite(" (");
	StdOutWrite(buffer);
	StdOutWrite(")\r\n");
}

static ULONG MempGetKernelRamSize(OFHANDLE Handle) {

	// Get all memory.
	OF_REGISTER Memory[16];
	ULONG MemoryCount = 16;
	if (ARC_FAIL(OfGetPropRegs(Handle, "reg", Memory, &MemoryCount))) {
		return 0;
	}

	ULONG BasePage = 0;
	ULONG PhysSize = 0;

	for (ULONG i = 0; i < MemoryCount; i++) {
		ULONG ThisBasePage = Memory[i].PhysicalAddress.LowPart / PAGE_SIZE;
		ULONG PageCount = Memory[i].Size / PAGE_SIZE;
		// Ensure RAM is mapped consecutively for loading the NT kernel. NT kernel can use RAM mapped elsewhere, however.
		if (ThisBasePage != BasePage) {
			// Handle empty RAM slots.
			if (ThisBasePage == 0 && PageCount == 0) continue;
			break;
		}
		BasePage = ThisBasePage + PageCount;
		PhysSize += Memory[i].Size;
	}

	s_PhysMemLength = PhysSize;

	return PhysSize;
}

/// <summary>
/// Maps physical memory to virtual addresses that NT relies on.
/// </summary>
/// <returns>true if succeeded</returns>
int ArcMemInitNtMapping(void) {
	// This loader is for grackle.
	// Grackle can properly switch endianness for the PCI bus.
	// NT runs on systems with this memory controller.
	// Thus, we can kick open firmware out of the way and load our own little endian ARC firmware for these systems.

	// Get OF memory.
	OFHANDLE Chosen = OfGetChosen();
	OFHANDLE Memory;
	{
		OFIHANDLE iMemory;
		if (ARC_FAIL(OfGetPropInt(Chosen, "memory", &iMemory))) {
			return -1;
		}
		Memory = OfInstanceToPackage(iMemory);
		if (Memory == OFNULL) {
			return -2;
		}
	}

	// Find the RAM size:
	ULONG PhysMemSize = MempGetKernelRamSize(Memory);
	ULONG NtMapSize = PhysMemSize;
	//ULONG PhysMemMb = PhysMemSize / (1024 * 1024);
	// Assume that physical memory is mapped consecutively. This isn't the Wii.
	// Store the memory size off for later.
	s_PhysMemLength = NtMapSize;
	if (NtMapSize == 0) return -3;

	s_LastFreePage = NtMapSize / PAGE_SIZE;

	// Theoretically, we can check to see what's mapped;
	// Practically, stage2 has a hardcoded load address anyway.
	ULONG FirstFreeAddress = 0x100000;
	s_FirstFreePage = FirstFreeAddress / PAGE_SIZE;

	// Now everything is known, grab the MMU:
	OFIHANDLE Mmu = OFINULL;
	if (ARC_FAIL(OfGetPropInt(Chosen, "mmu", &Mmu))) {
		return -4;
	}
	if (Mmu == OFINULL) return -4;

	ARC_STATUS Status = _ESUCCESS;

	// Also grab the mac-io bridge so its register space can be obtained.
	OFHANDLE GrandCentral = OfFindDevice("/bandit/gc");
	//ULONG GrandCentralStart = 0;
	if (GrandCentral != OFNULL) {
		// Got it
		OF_REGISTER Register;
		if (ARC_FAIL(OfGetPropReg(GrandCentral, "assigned-addresses", 0, &Register))) return -4;
		ULONG Start = Register.PhysicalAddress.LowPart;
		s_GrandCentralStart = Start;
		//ULONG End = Start + Register.Size;
		// Set the free addresses for a hammerhead system.
		NtMapSize = PhysMemSize;
		s_LastFreePage = NtMapSize / PAGE_SIZE;
		s_FirstFreePage = 0;
		FirstFreeAddress = 0;
		// Start is probably at 0xF3000000 for Bandit/Grand Central
		// TODO: Figure out if there are other addresses
		if (Start == 0xF3000000) {
			// This is a grand central system.
		}
		else if (Start == 0x80800000) {
			// this is a grackle system, this loader is for grand central:
			StdOutWrite("This is a grackle system, you are using the wrong files (this is for grand central)\r\n");
			OfExit();
			return -5;
		}
		else if (Start == 0x80000000) {
			// this is a uni-north system, this loader is for grand central:
			StdOutWrite("This is a uni-north system, you are using the wrong files (this is for grand central)\r\n");
			OfExit();
			return -5;
		}
		else {
			//print_f("mac-io at %08x-%08x\r\n", Start, End);
			StdOutWrite("incompatible, unknown system (");
			print_hex(Start);
			StdOutWrite(")\r\n");
			OfExit();
			return -5;
		}
	}
	else return -5; // temp
	
	// Claim the lowest memory (exception handler space), if for some reason not already done
	StdOutWrite("-> claiming exception handlers\r\n");
	OfClaim((PVOID)NULL, 0x4000, 0);

	// Claim up to where we start
	extern UCHAR __text_start[];
	extern UCHAR _end[];
	StdOutWrite("-> claiming memory to start\r\n");
	if (OfClaim((PVOID)0x4000, (ULONG)(__text_start) - 0x4000, 0) == NULL) return -5;
	// Claim after we end until Open Firmware block at 4 MB
	ULONG LastAddress = 0x1000000;
	if (s_LastFreePage < (LastAddress / PAGE_SIZE)) LastAddress = s_LastFreePage * PAGE_SIZE;
	ULONG EndPage = (ULONG)_end;
	if ((EndPage & (PAGE_SIZE - 1)) != 0) {
		EndPage += PAGE_SIZE;
		EndPage &= ~(PAGE_SIZE - 1);
	}
	StdOutWrite("-> claiming memory after end before OF block\r\n");
	if (OfClaim((PVOID)EndPage, 0x400000 - EndPage, 0) == NULL) return -5;
	// Claim after Open Firmware block, up to 16MB or end of RAM
	StdOutWrite("-> claiming memory after OF block\r\n");
	if (OfClaim((PVOID)0x500000, LastAddress - 0x500000, 0) == NULL) return -5;
	// Map first 16MB (or whatever) of RAM
	StdOutWrite("-> mapping first memory block\r\n");
	Status = OfCallMethod(4, 0, NULL, "map", Mmu, 0, 0, LastAddress, 0);
	if (ARC_FAIL(Status)) return -5;
	// Map in the Bandit PCI configuration space
	StdOutWrite("-> mapping PCI space\r\n");
	Status = OfCallMethod(4, 0, NULL, "map", Mmu, 0xF2000000, 0xF2000000, 0x02000000, 0x2a);
	if (ARC_FAIL(Status)) return -5;

	// Map in the Control framebuffer
	OFHANDLE Control = OfFindDevice("/chaos/control");
	if (Control != OFNULL) {
		ULONG AssignedAddress[2 * 5];
		ULONG AddrLength = sizeof(AssignedAddress);
		ARC_STATUS Status = OfGetProperty(Control, "assigned-addresses", AssignedAddress, &AddrLength);
		if (ARC_FAIL(Status)) {
			OfExit();
			return -5;
		}
		
		ULONG BaseAddress = 0, FbSize = 0;
		for (ULONG i = 0; i < AddrLength / 5; i++) {
			PULONG Aperture = &AssignedAddress[i * 5];
			
			if ((Aperture[2] & 0x80000000) && Aperture[4] >= 0x00100000 && (Aperture[4] & 0xF0FFFFFF) == 0) {
				BaseAddress = Aperture[2]; // 1-4MB vram area
				FbSize = Aperture[4];
				break;
			}
		}

		StdOutWrite("-> mapping framebuffer space\r\n");
		Status = OfCallMethod(4, 0, NULL, "map", Mmu, BaseAddress, BaseAddress, FbSize, 0x2a);
		if (ARC_FAIL(Status)) return -5;
	}

	// Everything should be mapped fine now.
	return 0;
}

static int ElfValid(void* addr) {
	Elf32_Ehdr* ehdr; /* Elf header structure pointer */

	ehdr = (Elf32_Ehdr*)addr;

	StdOutWrite("ELF header: ");
	print_hex(((ULONG*)addr)[0]);
	StdOutWrite(" ");
	print_hex(((ULONG*)addr)[1]);
	StdOutWrite(" ");
	print_hex(((ULONG*)addr)[2]);
	StdOutWrite(" ");
	print_hex(((ULONG*)addr)[3]);
	StdOutWrite(" ");
	print_hex(((ULONG*)addr)[4]);
	StdOutWrite("\r\n");

	if (!IS_ELF(*ehdr))
		{StdOutWrite("bad magic\r\n"); return 0;}

	if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
		{StdOutWrite("wrong class\r\n"); return -1;}

	if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
		{StdOutWrite("wrong data type\r\n"); return -1;}

	if (ehdr->e_ident[EI_VERSION] != EV_CURRENT)
		{StdOutWrite("wrong version\r\n"); return -1;}

	if (ehdr->e_type != ET_EXEC)
		{StdOutWrite("wrong type\r\n"); return -1;}

	if (ehdr->e_machine != EM_PPC)
		{StdOutWrite("wrong architecture\r\n"); return -1;}

	return 1;
}

static void sync_after_write(const void* pv, ULONG len)
{
	ULONG a, b;

	const void* p = (const void*)((ULONG)pv & ~0x80000000);

	a = (ULONG)p & ~0x1f;
	b = ((ULONG)p + len + 0x1f) & ~0x1f;

	for (; a < b; a += 32)
		asm("dcbst 0,%0" : : "b"(a));

	asm("sync ; isync");
}

static void sync_before_exec(const void* pv, ULONG len)
{
	ULONG a, b;

	const void* p = (const void*)((ULONG)pv & ~0x80000000);

	a = (ULONG)p & ~0x1f;
	b = ((ULONG)p + len + 0x1f) & ~0x1f;

	for (; a < b; a += 32)
		asm("dcbst 0,%0 ; sync ; icbi 0,%0" : : "b"(a));

	asm("sync ; isync");
}

// TODO: determine if 64-bit byte swaps are necessary (the PCI bus is 32 bits wide?)

static void MsrLeSwap64Single(ULONG* dest32, ULONG* src32) {
	ULONG temp = src32[1];
	dest32[1] = __builtin_bswap32(src32[0]);
	dest32[0] = __builtin_bswap32(temp);
}

static void MsrLeSwap64(void* dest, const void* src, ULONG len, ULONG memlen) {
	uint64_t* dest64 = (uint64_t*)dest;
	uint64_t* src64 = (uint64_t*)src;
	
	// align swap-len to 64 bits.
	if ((len & 7) != 0) len += 8 - (len & 7);
	for (; len != 0; dest64++, src64++, len -= sizeof(*dest64), memlen -= sizeof(*dest64)) {
		ULONG* dest32 = (ULONG*)dest64;
		if (len < sizeof(*dest64)) {
			uint64_t val64 = *src64 & ((1 << (len * 8)) - 1);
			ULONG* val32 = (ULONG*)&val64;
			MsrLeSwap64Single(dest32, val32);
			continue;
		}
		ULONG* src32 = (ULONG*)src64;
		MsrLeSwap64Single(dest32, src32);
	}
	
	if ((memlen & 7) != 0) memlen += 8 - (memlen & 7);
	for (; memlen > 0; dest64++, memlen -= sizeof(*dest64)) {
		*dest64 = 0;
	}
}

static void MsrLeMunge32(void* ptr, ULONG len) {
	ULONG* ptr32 = (ULONG*)ptr;
	
	for (; len > 0; len -= sizeof(uint64_t), ptr32 += 2) {
		ULONG temp = ptr32[0];
		ptr32[0] = ptr32[1];
		ptr32[1] = temp;
	}
}

static ULONG ElfLoad(void* addr) {
	Elf32_Ehdr* ehdr;
	Elf32_Phdr* phdrs;
	UCHAR* image;
	int i;

	ehdr = (Elf32_Ehdr*)addr;

	if (ehdr->e_phoff == 0 || ehdr->e_phnum == 0) {
		//StdOutWrite("ELF has no phdrs\r\n");
		return 0;
	}

	if (ehdr->e_phentsize != sizeof(Elf32_Phdr)) {
		//StdOutWrite("Invalid ELF phdr size\r\n");
		return 0;
	}

	phdrs = (Elf32_Phdr*)(addr + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		if (phdrs[i].p_type != PT_LOAD) {
			//print_f("skip PHDR %d of type %d\r\n", i, phdrs[i].p_type);
			continue;
		}

		// translate paddr to this BAT setup
		phdrs[i].p_paddr &= 0x3FFFFFFF;
		//phdrs[i].p_paddr |= 0x80000000;

#if 0
		print_f("PHDR %d 0x%08x [0x%x] -> 0x%08x [0x%x] <", i,
			phdrs[i].p_offset, phdrs[i].p_filesz,
			phdrs[i].p_paddr, phdrs[i].p_memsz);

		if (phdrs[i].p_flags & PF_R)
			print_f("R");
		if (phdrs[i].p_flags & PF_W)
			print_f("W");
		if (phdrs[i].p_flags & PF_X)
			print_f("X");
		print_f(">\r\n");
#endif

		if (phdrs[i].p_filesz > phdrs[i].p_memsz) {
			//print_f("-> file size > mem size\r\n");
			return 0;
		}

		if (phdrs[i].p_filesz) {
			StdOutWrite("-> load ");
			print_hex(phdrs[i].p_filesz);
			StdOutWrite(" to ");
			print_hex(phdrs[i].p_paddr);
			StdOutWrite("\r\n");
			//print_f("-> load 0x%x\r\n", phdrs[i].p_filesz);
			image = (UCHAR*)(addr + phdrs[i].p_offset);
			MsrLeSwap64(
				(void*)(phdrs[i].p_paddr),
				(const void*)image,
				phdrs[i].p_filesz,
				phdrs[i].p_memsz
			);
			if (phdrs[i].p_memsz > phdrs[i].p_filesz)
				memset((void*)(phdrs[i].p_paddr + phdrs[i].p_filesz), 0, phdrs[i].p_memsz - phdrs[i].p_filesz);
			memset((void*)image, 0, phdrs[i].p_filesz);

			if (phdrs[i].p_flags & PF_X)
				sync_before_exec((void*)phdrs[i].p_paddr, phdrs[i].p_memsz);
			else
				sync_after_write((void*)phdrs[i].p_paddr, phdrs[i].p_memsz);
		}
		else {
			//print_f("-> skip\r\n");
			memset((void*)phdrs[i].p_paddr + phdrs[i].p_filesz, 0, phdrs[i].p_memsz - phdrs[i].p_filesz);
		}
	}

	// fix the ELF entrypoint to physical address
	ULONG EntryPoint = ehdr->e_entry;
	EntryPoint &= 0x3fffffff;
	return EntryPoint;
}

static inline void out_8(volatile unsigned char *addr, int val)
{
	__asm__ __volatile__("stb%U0%X0 %1,%0; sync"
			     : "=m" (*addr) : "r" (val));
}

static int FbSetDepthControl(OFHANDLE Control, PHW_DESCRIPTION Desc) {
	ULONG AssignedAddress[2 * 5];
	ULONG AddrLength = sizeof(AssignedAddress);
	ARC_STATUS Status = OfGetProperty(Control, "assigned-addresses", AssignedAddress, &AddrLength);
	if (ARC_FAIL(Status)) {
		return false;
	}
	
	ULONG BaseAddress = 0, RegisterAddress = 0, RegisterSize = 0, BaseSize = 0;
	for (ULONG i = 0; i < AddrLength / 5; i++) {
		PULONG Aperture = &AssignedAddress[i * 5];
		if (Aperture[2] & 0x80000000 && (Aperture[4] & 0xF0FF0FFF) == 0) {
			if (Aperture[4] < 0x00100000) {
				RegisterAddress = Aperture[2]; // register map
				RegisterSize = Aperture[4];
			} else {
				BaseAddress = Aperture[2]; // 1-4MB vram area
				BaseSize = Aperture[4];
			}
		}
	}

	// I'm lazy, so we're just plugging in the Linux controlfb driver and letting it do all the work
	struct fb_info_control info = {
		{}, // will be filled in by mac_vmode_to_var
		{}, // will be filled in by controlfb_var_to_par
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

		// apparently this isn't in the device tree - hardcode it (waa)
		0xF301B000,
		0xF301B000,

		RegisterAddress,
		RegisterAddress,
		RegisterSize,

		BaseAddress,
		BaseAddress,
		BaseAddress,
		BaseSize,

		// will be filled in by find_vram_size
		0,
		0,
		0
	};
	int err;

	// Set up virtual resolution depending on VRAM
	find_vram_size(&info);
	if (info.total_vram >= 1024 * 768 * 4) err = mac_vmode_to_var(VMODE_1024_768_60, CMODE_32, &info.var);
	else err = mac_vmode_to_var(VMODE_800_600_60, CMODE_32, &info.var);
	if (err != 0) {
		StdOutWrite("Failed to get mode: ");
		print_error(err);
		return false;
	}

	// Set up physical screen config from virtual res (also make it little endian while we're at it)
	info.vram_attr |= 2;
	if ((err = controlfb_set_par(&info)) != 0) {
		StdOutWrite("Failed to set mode: ");
		print_error(err);
		return false;
	}

	// Set up color palette to be 0-255 (gamma?)
	for (int i = 0; i < 256; i++) {
		out_8(&info.cmap_regs->addr, i);	/* tell clut what addr to fill	*/
		out_8(&info.cmap_regs->lut, i);		/* send one color channel at	*/
		out_8(&info.cmap_regs->lut, i);		/* a time...			*/
		out_8(&info.cmap_regs->lut, i);
	}

	Desc->ControlFbCtrlParam = info.par.ctrl;
	Desc->ControlFbCtrlParamAddr = &info.control_regs->ctrl.r;
	Desc->ControlFbClockParams[0] = info.par.regvals.clock_params[0];
	Desc->ControlFbClockParams[1] = info.par.regvals.clock_params[1];
	Desc->ControlFbClockParams[2] = info.par.regvals.clock_params[2];
	
	return true;
}

static void FbGetDetails(OFHANDLE Control, PHW_DESCRIPTION HwDesc) {
	ULONG AssignedAddress[2 * 5];
	ULONG AddrLength = sizeof(AssignedAddress);
	ARC_STATUS Status = OfGetProperty(Control, "assigned-addresses", AssignedAddress, &AddrLength);
	if (ARC_FAIL(Status)) {
		OfExit();
		return;
	}
	
	ULONG BaseAddress = 0, RegisterAddress = 0;
	for (ULONG i = 0; i <= AddrLength / 5 - 5; i++) {
		PULONG Aperture = &AssignedAddress[i * 5];
		if (Aperture[2] & 0x80000000 && (Aperture[4] & 0xF0FF0FFF) == 0) {
			if (Aperture[4] < 0x00100000) RegisterAddress = Aperture[2]; // register map
			else BaseAddress = Aperture[2]; // 1-4MB vram area
		}
	}
	

	StdOutWrite("-> config regs at ");
	print_hex(RegisterAddress);
	StdOutWrite("\r\n");

	struct control_regs *regs = (struct control_regs*)RegisterAddress;
#if 0
	// I *think* this is right, very good chance it's not though
	// NOTE: This is now BE because switching the mode swapped endian
	ULONG Width = (regs->hperiod.r + 2 -
		regs->hssync.r +
		regs->hsblank.r -
		regs->heblank.r -
		regs->heq.r) << 1;
	ULONG Height = (regs->vsblank.r - regs->veblank.r) >> 1;
	ULONG Stride = regs->pitch.r;
#endif
	ULONG Width = 1024, Height = 768, Stride = 4096;
	ULONG FbAddr = BaseAddress + 0x2C; // 16 bytes for cursor, plus ???? offset

	StdOutWrite("-> framebuffer at ");
	print_hex(FbAddr);
	StdOutWrite(" (size: ");
	print_hex(Width);
	StdOutWrite(" x ");
	print_hex(Height);
	StdOutWrite(", stride ");
	print_hex(Stride);
	StdOutWrite(")\r\n");
	
	// Wipe the screen
	memset((PVOID)FbAddr, 0, Height * Stride);
	// Set the framebuffer details
	HwDesc->FrameBufferBase = FbAddr;
	HwDesc->FrameBufferWidth = Width;
	HwDesc->FrameBufferHeight = Height;
	HwDesc->FrameBufferStride = Stride;
}

typedef void (*ArcFirmEntry)(PHW_DESCRIPTION HwDesc);
extern void __attribute__((noreturn)) ModeSwitchEntry(ArcFirmEntry Start, PHW_DESCRIPTION HwDesc, ULONG FbAddr);


int _start(int argc, char** argv, tfpOpenFirmwareCall of) {
	(void)argc;
	(void)argv;
	
	if (!OfInit(of)) {
		// can't do a thing!
		return -1;
	}

	// init stdout
	InitStdout();
	StdOutWrite("\r\n");
	
	// OF doesn't power on the media bay until something wants to access it
	// so try to open the cd drive
	{
		OFIHANDLE hCd = OfOpen("scsi-int/sd@3:0");
		if (hCd != OFINULL) OfClose(hCd);
	}

	// set up virtual address mappings for NT and set OF properties for stage 2 to find it
	int Error = ArcMemInitNtMapping();
	if (Error < 0) {
		StdOutWrite("Could not init memory mapping for NT");
		print_error(-Error);
		OfExit();
		return -2;
	}
	
	// get full OF path of stage 1.
	static char BootPath[1024];
	OFHANDLE Chosen = OfGetChosen();
	ULONG BootPathLength = sizeof(BootPath);
	ARC_STATUS Status = OfGetPropString(Chosen, "bootpath", BootPath, &BootPathLength);
	if (ARC_FAIL(Status)) {
		StdOutWrite("Could not get bootpath");
		print_error(Status);
		OfExit();
		return -3;
	}
	ULONG BootPathIdx = 0;
	while (BootPathIdx < BootPathLength && BootPath[BootPathIdx] != '\\') BootPathIdx++;
	if (BootPathIdx == BootPathLength) {
		StdOutWrite("Could not parse bootpath: ");
		StdOutWrite(BootPath);
		StdOutWrite("\r\n");
		OfExit();
		return -4;
	}
	BootPathIdx++;
	strcpy(&BootPath[BootPathIdx], "stage2.elf");

	// load stage2 from disk
	OFIHANDLE File = OfOpen(BootPath);
	if (File == OFINULL) {
		StdOutWrite("Could not open stage2: ");
		StdOutWrite(BootPath);
		StdOutWrite("\r\n");
		OfExit();
		return -5;
	}

	// load at exactly 3MB
	// use the NT mapping because the 1:1 mapping may not be present, whereas the NT mapping is guaranteed to be present
	PVOID Addr = (PVOID)((s_FirstFreePage * PAGE_SIZE) + 0x300000);
	ULONG ActualLoad = 0;
	// allow loading up to 1MB, ie, before Open Firmware memory
	ULONG LastAddress = 0x400000;
	if (s_LastFreePage < (LastAddress / PAGE_SIZE)) LastAddress = s_LastFreePage * PAGE_SIZE;
	StdOutWrite("Reading stage2.elf...\r\n");
	Status = OfRead(File, Addr, LastAddress - (ULONG)Addr - (s_FirstFreePage * PAGE_SIZE), &ActualLoad);
	OfClose(File);

	if (ARC_FAIL(Status)) {
		StdOutWrite("Could not read stage2: ");
		StdOutWrite(BootPath);
		print_error(Status);
		OfExit();
		return -7;
	}
	
	StdOutWrite("-> read ");
	print_hex(ActualLoad);
	StdOutWrite(" bytes\r\n");

	// check for validity
	if (ActualLoad < sizeof(Elf32_Ehdr) || ElfValid(Addr) <= 0) {
		StdOutWrite("Invalid ELF for stage2: ");
		StdOutWrite(BootPath);
		StdOutWrite("\r\n");
		OfExit();
		return -8;
	}

	// load ELF
	StdOutWrite("Loading stage2...\r\n");
	ULONG EntryPoint = ElfLoad(Addr);
	if (EntryPoint == 0) {
		StdOutWrite("Failed to load ELF for stage2: ");
		StdOutWrite(BootPath);
		StdOutWrite("\r\n");
		OfExit();
		return -9;
	}

	// zero ELF out of memory
	memset(Addr, 0, ActualLoad);
	
	// We now have free memory at exactly 3MB, we can use this to store our descriptor.
	PHW_DESCRIPTION Desc = (PHW_DESCRIPTION) Addr;
	Desc->MemoryLength = s_PhysMemLength;
	Desc->GrandCentralStart = s_GrandCentralStart;

	// load floppy disk into memory because I'm lazy
	File = OfOpen("/bandit/gc/swim3");
	if (File == OFINULL) {
		StdOutWrite("Could not open ramdisk: ");
		StdOutWrite(BootPath);
		StdOutWrite("\r\n");
		Desc->RamDiskAddr = NULL;
	} else {
		// load at exactly 5MB
		// use the NT mapping because the 1:1 mapping may not be present, whereas the NT mapping is guaranteed to be present
		Addr = (PVOID)((s_FirstFreePage * PAGE_SIZE) + 0x180000);
		ULONG ActualLoad = 0;
		// allow loading up to 1.5MB
		ULONG LastAddress = 0x300000;
		if (s_LastFreePage < (LastAddress / PAGE_SIZE)) LastAddress = s_LastFreePage * PAGE_SIZE;
		StdOutWrite("Reading floppy disk...\r\n");
		Status = OfCallMethod(3, 1, &ActualLoad, "read-blocks", File, (ULONG)Addr, (ULONG)0, (ULONG)2880);
		OfClose(File);

		if (ARC_FAIL(Status)) {
			StdOutWrite("Could not read ramdisk: ");
			StdOutWrite(BootPath);
			StdOutWrite("\r\n");
			Desc->RamDiskAddr = NULL;
		} else {
			// munge to LE
			Desc->RamDiskAddr = (PVOID)((s_FirstFreePage * PAGE_SIZE) + 0x500000);
			MsrLeSwap64(Desc->RamDiskAddr, Addr, ActualLoad * 512, 1474560);

			// zero image out of memory
			memset(Addr, 0, ActualLoad);
		}
	}
	
	StdOutWrite("-> reading decrementer freq\r\n");
	{
		ULONG DecrementerFrequency = 0;
		OFHANDLE Cpu = OfFindDevice("/PowerPC,604");
		if (Cpu == OFNULL) {
			Cpu = OfFindDevice("/PowerPC,603");
		}
		if (Cpu == OFNULL) {
			Cpu = OfFindDevice("/PowerPC,601");
		}
		if (Cpu != OFNULL) {
			OfGetPropInt(Cpu, "timebase-frequency", &DecrementerFrequency);
		}
		if (DecrementerFrequency == 0) {
			StdOutWrite("Could not obtain timebase-frequency\r\n");
			OfExit();
			return -10;
		}
		Desc->DecrementerFrequency = DecrementerFrequency;
	}
	
	s_MrpFlags = MRP_VIA_IS_CUDA | MRP_BANDIT;
	Desc->MrpFlags = s_MrpFlags;
	
	StdOutWrite("-> initializing framebuffer\r\n");
	{
		// Get the framebuffer from OF.
		OFHANDLE Screen = OfFindDevice("/chaos/control");
		if (Screen == OFNULL) {
			StdOutWrite("Could not obtain control device, incompatible Open Firmware\r\n");
			OfExit();
			return -11;
		}
		if (!FbSetDepthControl(Screen, Desc)) {
			StdOutWrite("Could not set up 32bpp framebuffer\r\n");
			OfExit();
			return -12;
		}
		FbGetDetails(Screen, Desc);
	}
	
	ULONG FbAddr = Desc->FrameBufferBase;
	// munge descriptor
	MsrLeMunge32(Desc, sizeof(*Desc));

	StdOutWrite("-> entry point: ");
	print_hex(EntryPoint);
	StdOutWrite("\r\n");
	//OfExit(); return 0;
	StdOutWrite("Booting stage2...\r\n");

	// call entrypoint through mode switch
	ArcFirmEntry NextEntry = (ArcFirmEntry)EntryPoint;
	ModeSwitchEntry(NextEntry, Desc, FbAddr);
}

extern const ULONG StartAixCall[2];
const ULONG StartAixCall[2] = { (ULONG) _start, 0xFEEDFACE };