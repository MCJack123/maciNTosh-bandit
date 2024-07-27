// SWIM3 floppy driver API header.
#pragma once

enum swim_state {
	idle,
	locating,
	seeking,
	settling,
	do_transfer,
	jogging,
	available,
	revalidating,
	ejecting
};

typedef struct _SWIM3_FLOPPY_DEVICE SWIM3_FLOPPY_DEVICE, * PSWIM3_FLOPPY_DEVICE;
struct _SWIM3_FLOPPY_DEVICE {
	PSWIM3_FLOPPY_DEVICE Next;
	ULONG NumberOfSectors;
	ULONG BytesPerSector;
	int cur_sector;
    int cur_cyl;
	int expect_sector;
	int expect_cyl;
	int req_sector;
	int req_cyl;
	int head;
	enum swim_state state;
	int retries;
	PVOID buf;
	ULONG count;
	bool write;
};

int swim3_init(uint32_t addr, uint32_t dmaaddr);
PSWIM3_FLOPPY_DEVICE swim3_open_drive(void);
ULONG swim3_read_blocks(PSWIM3_FLOPPY_DEVICE drive, PVOID buffer, ULONG sector, ULONG count);
ULONG swim3_write_blocks(PSWIM3_FLOPPY_DEVICE drive, PVOID buffer, ULONG sector, ULONG count);