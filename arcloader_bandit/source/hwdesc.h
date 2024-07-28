#pragma once

enum { // MRP = "MacRISC Paddington"
	MRP_IN_EMULATOR = ARC_BIT(0),
	MRP_VIA_IS_CUDA = ARC_BIT(1),
	MRP_BANDIT = ARC_BIT(2),
};

typedef struct _HW_DESCRIPTION {
	ULONG MemoryLength; // Length of physical memory.
	ULONG GrandCentralStart; // Base address of Mac I/O controller (Grand Central).
	ULONG DecrementerFrequency; // Decrementer frequency.
	ULONG MrpFlags; // MRP_* bit flags.
	ULONG UsbOhciStart[2]; // Base address of USB controller(s).

	// Framebuffer details.
	ULONG FrameBufferBase; // Base address of frame buffer.
	//ULONG FrameBufferLength; // Length of frame buffer in video RAM. (unneeded, can be calculated later by height * stride)
	ULONG FrameBufferWidth; // Display width
	ULONG FrameBufferHeight; // Display height
	ULONG FrameBufferStride; // Number of bytes per line.
	ULONG ControlFbClockParams[3]; // Parameters for the display clock, set in stage2 by PXI
	ULONG ControlFbCtrlParam; // Parameter for display control
	ULONG ControlFbCtrlParamAddr; // Address of display control parameter
	ULONG Padding;
	
	// Boot device details.
	ULONG BootDevice;
	ULONG RamDiskAddr; // Address of driver ramdisk if provided by loader
} HW_DESCRIPTION, *PHW_DESCRIPTION;