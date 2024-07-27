// SWIM3 floppy driver.
#include <stddef.h>
#include <string.h>
#include "arc.h"
#include "linux/dbdma.h"
#include "processor.h"
#include "timer.h"
#include "floppy_swim.h"

// Register definitions from torvalds/linux/drivers/block/swim3.c (GPLv2+)

#define REG(x) \
    volatile unsigned char x; \
    char x##_pad[15];

/*
 * The names for these registers mostly represent speculation on my part.
 * It will be interesting to see how close they are to the names Apple uses.
 */
struct swim3 {
    REG(data);
    REG(timer); /* counts down at 1MHz */
    REG(error);
    REG(mode);
    REG(select); /* controls CA0, CA1, CA2 and LSTRB signals */
    REG(setup);
    REG(control); /* writing bits clears them */
    REG(status);  /* writing bits sets them in control */
    REG(intr);
    REG(nseek);  /* # tracks to seek */
    REG(ctrack); /* current track number */
    REG(csect);  /* current sector number */
    REG(gap3);   /* size of gap 3 in track format */
    REG(sector); /* sector # to read or write */
    REG(nsect);  /* # sectors to read or write */
    REG(intr_enable);
};

#define control_bic control
#define control_bis status

/* Bits in select register */
#define CA_MASK 7
#define LSTRB 8

/* Bits in control register */
#define DO_SEEK 0x80
#define FORMAT 0x40
#define SELECT 0x20
#define WRITE_SECTORS 0x10
#define DO_ACTION 0x08
#define DRIVE2_ENABLE 0x04
#define DRIVE_ENABLE 0x02
#define INTR_ENABLE 0x01

/* Bits in status register */
#define FIFO_1BYTE 0x80
#define FIFO_2BYTE 0x40
#define ERROR 0x20
#define DATA 0x08
#define RDDATA 0x04
#define INTR_PENDING 0x02
#define MARK_BYTE 0x01

/* Bits in intr and intr_enable registers */
#define ERROR_INTR 0x20
#define DATA_CHANGED 0x10
#define TRANSFER_DONE 0x08
#define SEEN_SECTOR 0x04
#define SEEK_DONE 0x02
#define TIMER_DONE 0x01

/* Bits in error register */
#define ERR_DATA_CRC 0x80
#define ERR_ADDR_CRC 0x40
#define ERR_OVERRUN 0x04
#define ERR_UNDERRUN 0x01

/* Bits in setup register */
#define S_SW_RESET 0x80
#define S_GCR_WRITE 0x40
#define S_IBM_DRIVE 0x20
#define S_TEST_MODE 0x10
#define S_FCLK_DIV2 0x08
#define S_GCR 0x04
#define S_COPY_PROT 0x02
#define S_INV_WDATA 0x01

/* Select values for swim3_action */
#define SEEK_POSITIVE 0
#define SEEK_NEGATIVE 4
#define STEP 1
#define MOTOR_ON 2
#define MOTOR_OFF 6
#define INDEX 3
#define EJECT 7
#define SETMFM 9
#define SETGCR 13

/* Select values for swim3_select and swim3_readbit */
#define STEP_DIR 0
#define STEPPING 1
#define MOTOR_ON 2
#define RELAX 3 /* also eject in progress */
#define READ_DATA_0 4
#define ONEMEG_DRIVE 5
#define SINGLE_SIDED 6 /* drive or diskette is 4MB type? */
#define DRIVE_PRESENT 7
#define DISK_IN 8
#define WRITE_PROT 9
#define TRACK_ZERO 10
#define TACHO 11
#define READ_DATA_1 12
#define GCR_MODE 13
#define SEEK_COMPLETE 14
#define TWOMEG_MEDIA 15

/*
 * Low-level I/O routines.
 *
 * Copied from <file:arch/powerpc/include/asm/io.h> (which has no copyright)
 */
static inline int in_8(const volatile unsigned char *addr)
{
	int ret;

	__asm__ __volatile__("lbz%U1%X1 %0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "m" (*addr));
	return ret;
}

static inline void out_8(volatile unsigned char *addr, int val)
{
	__asm__ __volatile__("stb%U0%X0 %1,%0; sync"
			     : "=m" (*addr) : "r" (val));
}

static inline unsigned in_le16(const volatile u16 *addr)
{
	unsigned ret;

	__asm__ __volatile__("lhbrx %0,0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "r" (addr), "m" (*addr));

	return ret;
}

static inline unsigned in_be16(const volatile u16 *addr)
{
	unsigned ret;

	__asm__ __volatile__("lhz%U1%X1 %0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "m" (*addr));
	return ret;
}

static inline void out_le16(volatile u16 *addr, int val)
{
	__asm__ __volatile__("sthbrx %1,0,%2; sync" : "=m" (*addr)
			     : "r" (val), "r" (addr));
}

static inline void out_be16(volatile u16 *addr, int val)
{
	__asm__ __volatile__("sth%U0%X0 %1,%0; sync"
			     : "=m" (*addr) : "r" (val));
}

static inline unsigned in_le32(const volatile unsigned *addr)
{
	unsigned ret;

	__asm__ __volatile__("lwbrx %0,0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "r" (addr), "m" (*addr));
	return ret;
}

static inline unsigned in_be32(const volatile unsigned *addr)
{
	unsigned ret;

	__asm__ __volatile__("lwz%U1%X1 %0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "m" (*addr));
	return ret;
}

static inline void out_le32(volatile unsigned *addr, int val)
{
	__asm__ __volatile__("stwbrx %1,0,%2; sync" : "=m" (*addr)
			     : "r" (val), "r" (addr));
}

static inline void out_be32(volatile unsigned *addr, int val)
{
	__asm__ __volatile__("stw%U0%X0 %1,%0; sync"
			     : "=m" (*addr) : "r" (val));
}

static void swim3_select(struct swim3 *sw, int sel) {
    out_8(&sw->select, RELAX);
    if (sel & 8) out_8(&sw->control_bis, SELECT);
    else
        out_8(&sw->control_bic, SELECT);
    out_8(&sw->select, sel & CA_MASK);
}

static void swim3_action(struct swim3 *sw, int action) {
    swim3_select(sw, action);
    udelay(1);
    out_8(&sw->select, sw->select | LSTRB);
    udelay(2);
    out_8(&sw->select, sw->select & ~LSTRB);
    udelay(1);
}

static int swim3_readbit(struct swim3 *sw, int bit) {
    int stat;

    swim3_select(sw, bit);
    udelay(1);
    stat = in_8(&sw->status);
    return (stat & DATA) == 0;
}

static struct swim3 *s_FloppyRegs = NULL;
static struct dbdma_regs *s_FloppyDMA = NULL;
static struct dbdma_cmd s_FloppyDMACommand[5] __attribute__((aligned(0x10)));
static SWIM3_FLOPPY_DEVICE s_FloppyDevice;

static unsigned short write_preamble[] = {
	0x4e4e, 0x4e4e, 0x4e4e, 0x4e4e, 0x4e4e,	/* gap field */
	0, 0, 0, 0, 0, 0,			/* sync field */
	0x99a1, 0x99a1, 0x99a1, 0x99fb,		/* data address mark */
	0x990f					/* no escape for 512 bytes */
};

static unsigned short write_postamble[] = {
	0x9904,					/* insert CRC */
	0x4e4e, 0x4e4e,
	0x9908,					/* stop writing */
	0, 0, 0, 0, 0, 0
};

static void swim3_wfinterrupt(PSWIM3_FLOPPY_DEVICE fs);

static inline void scan_track(struct floppy_state *fs)
{
	struct swim3 *sw = s_FloppyRegs;

	swim3_select(sw, READ_DATA_0);
	in_8(&sw->intr);		/* clear SEEN_SECTOR bit */
	in_8(&sw->error);
	out_8(&sw->control_bis, DO_ACTION);
	swim3_wfinterrupt(fs);
}

static inline void seek_track(PSWIM3_FLOPPY_DEVICE fs, int n)
{
	struct swim3 *sw = s_FloppyRegs;
	if (n >= 0) {
		swim3_action(sw, SEEK_POSITIVE);
		sw->nseek = n;
	} else {
		swim3_action(sw, SEEK_NEGATIVE);
		sw->nseek = -n;
	}
	swim3_select(sw, STEP);
	in_8(&sw->error);
	/* enable intr when seek finished */
	//out_8(&sw->intr_enable, SEEK_DONE);
	out_8(&sw->control_bis, DO_SEEK);
	swim3_wfinterrupt(fs);
}

static inline void init_dma(struct dbdma_cmd *cp, int cmd,
			    void* paddr, int count)	
{
	cp->req_count = count;
	cp->command = cmd;
	cp->phy_addr = paddr;
	cp->xfer_status = 0;
}

#define virt_to_phys(ptr) ((PVOID)((ULONG)(ptr) & ~0x80000000))

static inline void setup_transfer(PSWIM3_FLOPPY_DEVICE fs)
{
	int n;
	struct swim3 *sw = s_FloppyRegs;
	struct dbdma_cmd *cp = s_FloppyDMACommand;
	struct dbdma_regs *dr = s_FloppyDMA;

	if (fs->count <= 0) {
		return;
	}
	if (fs->write)
		n = 1;
	else {
		n = 18 - fs->req_sector + 1;
		if (n > fs->count)
			n = fs->count;
	}

	fs->count = n;
	swim3_select(sw, fs->head? READ_DATA_1: READ_DATA_0);
	out_8(&sw->sector, fs->req_sector);
	out_8(&sw->nsect, n);
	out_8(&sw->gap3, 0);
	out_le32(&dr->cmdptr, virt_to_phys(cp));
	if (fs->write) {
		/* Set up 3 dma commands: write preamble, data, postamble */
		init_dma(cp, OUTPUT_MORE, virt_to_phys(write_preamble),
			 sizeof(write_preamble));
		++cp;
		init_dma(cp, OUTPUT_MORE, virt_to_phys(fs->buf), 512);
		++cp;
		init_dma(cp, OUTPUT_LAST, virt_to_phys(write_postamble),
			sizeof(write_postamble));
	} else {
		init_dma(cp, INPUT_LAST, virt_to_phys(fs->buf), n * 512);
	}
	++cp;
	out_le16(&cp->command, DBDMA_STOP);
	out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
	in_8(&sw->error);
	out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
	if (fs->write)
		out_8(&sw->control_bis, WRITE_SECTORS);
	in_8(&sw->intr);
	out_le32(&dr->control, (RUN << 16) | RUN);
	/* enable intr when transfer complete */
	out_8(&sw->intr_enable, TRANSFER_DONE);
	out_8(&sw->control_bis, DO_ACTION);
	swim3_wfinterrupt(fs);
}

static void act(PSWIM3_FLOPPY_DEVICE fs)
{
	for (;;) {
		switch (fs->state) {
		case idle:
			return;		/* XXX shouldn't get here */

		case locating:
			if (swim3_readbit(fs, TRACK_ZERO)) {
				fs->cur_cyl = 0;
				if (fs->req_cyl == 0)
					fs->state = do_transfer;
				else
					fs->state = seeking;
				break;
			}
			scan_track(fs);
			return;

		case seeking:
			if (fs->cur_cyl < 0) {
				fs->expect_cyl = -1;
				fs->state = locating;
				break;
			}
			if (fs->req_cyl == fs->cur_cyl) {
				fs->state = do_transfer;
				break;
			}
			seek_track(fs, fs->req_cyl - fs->cur_cyl);
			return;

		case settling:
			/* check for SEEK_COMPLETE after 30ms */
			return;

		case do_transfer:
			if (fs->cur_cyl != fs->req_cyl) {
				if (fs->retries > 5) {
					fs->state = idle;
					return;
				}
				fs->state = seeking;
				break;
			}
			setup_transfer(fs);
			return;

		case jogging:
			seek_track(fs, -5);
			return;

		default:
			return;
		}
	}
}

static void swim3_wfinterrupt(PSWIM3_FLOPPY_DEVICE fs)
{
	struct swim3 *sw = s_FloppyRegs;
	int intr, err, n;
	int stat, resid;
	struct dbdma_regs *dr;
	struct dbdma_cmd *cp;
	unsigned long flags;

	//spin_lock_irqsave(&swim3_lock, flags);
	while ((intr = in_8(&sw->intr)) == 0) {}
	err = (intr & ERROR_INTR)? in_8(&sw->error): 0;
	switch (fs->state) {
	case locating:
		if (intr & SEEN_SECTOR) {
			out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
			out_8(&sw->select, RELAX);
			out_8(&sw->intr_enable, 0);
			if (sw->ctrack == 0xff) {
				fs->cur_cyl = -1;
				if (fs->retries > 5) {
					fs->state = idle;
				} else {
					fs->state = jogging;
					act(fs);
				}
				break;
			}
			fs->cur_cyl = sw->ctrack;
			fs->cur_sector = sw->csect;
			fs->state = do_transfer;
			act(fs);
		}
		break;
	case seeking:
	case jogging:
		if (sw->nseek == 0) {
			out_8(&sw->control_bic, DO_SEEK);
			out_8(&sw->select, RELAX);
			out_8(&sw->intr_enable, 0);
			if (fs->state == seeking)
				++fs->retries;
			fs->state = settling;
			act(fs);
		}
		break;
	case settling:
		out_8(&sw->intr_enable, 0);
		act(fs);
		break;
	case do_transfer:
		if ((intr & (ERROR_INTR | TRANSFER_DONE)) == 0)
			break;
		out_8(&sw->intr_enable, 0);
		out_8(&sw->control_bic, WRITE_SECTORS | DO_ACTION);
		out_8(&sw->select, RELAX);
		dr = s_FloppyDMA;
		cp = s_FloppyDMACommand;
		if (fs->write)
			++cp;
		/*
		 * Check that the main data transfer has finished.
		 * On writing, the swim3 sometimes doesn't use
		 * up all the bytes of the postamble, so we can still
		 * see DMA active here.  That doesn't matter as long
		 * as all the sector data has been transferred.
		 */
		if ((intr & ERROR_INTR) == 0 && cp->xfer_status == 0) {
			/* wait a little while for DMA to complete */
			for (n = 0; n < 100; ++n) {
				if (cp->xfer_status != 0)
					break;
				udelay(1);
			}
		}
		/* turn off DMA */
		out_le32(&dr->control, (RUN | PAUSE) << 16);
		stat = cp->xfer_status;
		resid = cp->res_count;
		if (intr & ERROR_INTR) {
			n = fs->count - 1 - resid / 512;
			if (n > 0) {
				fs->req_sector += n;
			}
			if (fs->retries < 5) {
				++fs->retries;
				act(fs);
			} else {
				fs->state = idle;
			}
		} else {
			if ((stat & ACTIVE) == 0 || resid != 0) {
				/* musta been an error */
				fs->state = idle;
				break;
			}
			fs->retries = 0;
			fs->state = idle;
		}
		break;
	default:
	}
}

int swim3_init(uint32_t addr, uint32_t dmaaddr) {
    int n, err = 0;
    s_FloppyRegs = (struct swim3 *)addr;
    s_FloppyDMA = (struct dbdma_regs *)dmaaddr;

    // This magic comes from the Linux driver.
    s_FloppyRegs->setup = S_IBM_DRIVE | S_FCLK_DIV2;
    s_FloppyRegs->control = INTR_ENABLE | DRIVE2_ENABLE | DO_ACTION | WRITE_SECTORS | SELECT | FORMAT | DO_SEEK; // sets to 0
    s_FloppyRegs->mode = 0x95;
    udelay(10);
    s_FloppyRegs->intr_enable = 0;
    s_FloppyRegs->status = DRIVE_ENABLE;
    swim3_action(s_FloppyRegs, MOTOR_ON);
    for (n = 0;; ++n) {
        if (n >= 100 && swim3_readbit(s_FloppyRegs, SEEK_COMPLETE)) break;
        swim3_select(s_FloppyRegs, RELAX);
    }
    if ((swim3_readbit(s_FloppyRegs, SEEK_COMPLETE) == 0 || swim3_readbit(s_FloppyRegs, DISK_IN) == 0)) err = _ENXIO;
    swim3_action(s_FloppyRegs, SETMFM);
    swim3_select(s_FloppyRegs, RELAX);

    if (err) {
        swim3_action(s_FloppyRegs, MOTOR_OFF);
        out_8(&s_FloppyRegs->control_bic, DRIVE_ENABLE | INTR_ENABLE);
        swim3_select(s_FloppyRegs, RELAX);
        return err;
    }

    memset(s_FloppyDMACommand, 0, sizeof(s_FloppyDMACommand));
    s_FloppyDMACommand[1].command = __builtin_bswap16(DBDMA_STOP);

    // Allocate a device entry for it
    memset(&s_FloppyDevice, 0, sizeof(&s_FloppyDevice));
    s_FloppyDevice.NumberOfSectors = 2880;
    s_FloppyDevice.BytesPerSector = 512;
    s_FloppyDevice.cur_sector = -1;
    s_FloppyDevice.cur_cyl = -1;
	s_FloppyDevice.state = idle;

    return 0;
}

PSWIM3_FLOPPY_DEVICE swim3_open_drive(void) {
    return &s_FloppyDevice;
}

ULONG swim3_read_blocks(PSWIM3_FLOPPY_DEVICE drive, PVOID buffer, ULONG sector, ULONG count) {
    uint8_t ftrack = sector / 36;
	uint16_t x = sector % 36;
	uint8_t fhead = x / 18;
	uint8_t fsector = x % 18 + 1;
	drive->req_cyl = ftrack;
	drive->head = fhead;
	drive->req_sector = fsector;
	drive->buf = buffer;
	drive->count = count;
	drive->write = false;
	setup_transfer(drive);
	return drive->count;
}

ULONG swim3_write_blocks(PSWIM3_FLOPPY_DEVICE drive, PVOID buffer, ULONG sector, ULONG count) {
    // unimplemented for now
    return 0;
}
