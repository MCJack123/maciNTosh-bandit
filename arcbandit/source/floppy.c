#if 0
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for the SWIM3 (Super Woz Integrated Machine 3)
 * floppy controller found on Power Macintoshes.
 *
 * Copyright (C) 1996 Paul Mackerras.
 */

/*
 * TODO:
 * handle 2 drives
 * handle GCR disks
 */

#include <errno.h>
#include "linux/dbdma.h"
#include "linux/mediabay.h"
#include "linux/fd.h"
#include "linux/io.h"
#include "linux/blkdev.h"

/* generic data direction definitions */
#define READ			0
#define WRITE			1

#define MAX_FLOPPIES	2

static struct gendisk *disks[MAX_FLOPPIES];

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

#define REG(x)	unsigned char x; char x ## _pad[15];

/*
 * The names for these registers mostly represent speculation on my part.
 * It will be interesting to see how close they are to the names Apple uses.
 */
struct swim3 {
	REG(data);
	REG(timer);		/* counts down at 1MHz */
	REG(error);
	REG(mode);
	REG(select);		/* controls CA0, CA1, CA2 and LSTRB signals */
	REG(setup);
	REG(control);		/* writing bits clears them */
	REG(status);		/* writing bits sets them in control */
	REG(intr);
	REG(nseek);		/* # tracks to seek */
	REG(ctrack);		/* current track number */
	REG(csect);		/* current sector number */
	REG(gap3);		/* size of gap 3 in track format */
	REG(sector);		/* sector # to read or write */
	REG(nsect);		/* # sectors to read or write */
	REG(intr_enable);
};

#define control_bic	control
#define control_bis	status

/* Bits in select register */
#define CA_MASK		7
#define LSTRB		8

/* Bits in control register */
#define DO_SEEK		0x80
#define FORMAT		0x40
#define SELECT		0x20
#define WRITE_SECTORS	0x10
#define DO_ACTION	0x08
#define DRIVE2_ENABLE	0x04
#define DRIVE_ENABLE	0x02
#define INTR_ENABLE	0x01

/* Bits in status register */
#define FIFO_1BYTE	0x80
#define FIFO_2BYTE	0x40
#define ERROR		0x20
#define DATA		0x08
#define RDDATA		0x04
#define INTR_PENDING	0x02
#define MARK_BYTE	0x01

/* Bits in intr and intr_enable registers */
#define ERROR_INTR	0x20
#define DATA_CHANGED	0x10
#define TRANSFER_DONE	0x08
#define SEEN_SECTOR	0x04
#define SEEK_DONE	0x02
#define TIMER_DONE	0x01

/* Bits in error register */
#define ERR_DATA_CRC	0x80
#define ERR_ADDR_CRC	0x40
#define ERR_OVERRUN	0x04
#define ERR_UNDERRUN	0x01

/* Bits in setup register */
#define S_SW_RESET	0x80
#define S_GCR_WRITE	0x40
#define S_IBM_DRIVE	0x20
#define S_TEST_MODE	0x10
#define S_FCLK_DIV2	0x08
#define S_GCR		0x04
#define S_COPY_PROT	0x02
#define S_INV_WDATA	0x01

/* Select values for swim3_action */
#define SEEK_POSITIVE	0
#define SEEK_NEGATIVE	4
#define STEP		1
#define MOTOR_ON	2
#define MOTOR_OFF	6
#define INDEX		3
#define EJECT		7
#define SETMFM		9
#define SETGCR		13

/* Select values for swim3_select and swim3_readbit */
#define STEP_DIR	0
#define STEPPING	1
#define MOTOR_ON	2
#define RELAX		3	/* also eject in progress */
#define READ_DATA_0	4
#define ONEMEG_DRIVE	5
#define SINGLE_SIDED	6	/* drive or diskette is 4MB type? */
#define DRIVE_PRESENT	7
#define DISK_IN		8
#define WRITE_PROT	9
#define TRACK_ZERO	10
#define TACHO		11
#define READ_DATA_1	12
#define GCR_MODE	13
#define SEEK_COMPLETE	14
#define TWOMEG_MEDIA	15

/* Definitions of values used in writing and formatting */
#define DATA_ESCAPE	0x99
#define GCR_SYNC_EXC	0x3f
#define GCR_SYNC_CONV	0x80
#define GCR_FIRST_MARK	0xd5
#define GCR_SECOND_MARK	0xaa
#define GCR_ADDR_MARK	"\xd5\xaa\x00"
#define GCR_DATA_MARK	"\xd5\xaa\x0b"
#define GCR_SLIP_BYTE	"\x27\xaa"
#define GCR_SELF_SYNC	"\x3f\xbf\x1e\x34\x3c\x3f"

#define DATA_99		"\x99\x99"
#define MFM_ADDR_MARK	"\x99\xa1\x99\xa1\x99\xa1\x99\xfe"
#define MFM_INDEX_MARK	"\x99\xc2\x99\xc2\x99\xc2\x99\xfc"
#define MFM_GAP_LEN	12

struct floppy_state {
	enum swim_state	state;
	struct swim3 *swim3;	/* hardware registers */
	struct dbdma_regs *dma;	/* DMA controller registers */
	int	swim3_intr;	/* interrupt number for SWIM3 */
	int	dma_intr;	/* interrupt number for DMA channel */
	int	cur_cyl;	/* cylinder head is on, or -1 */
	int	cur_sector;	/* last sector we saw go past */
	int	req_cyl;	/* the cylinder for the current r/w request */
	int	head;		/* head number ditto */
	int	req_sector;	/* sector number ditto */
	int	scount;		/* # sectors we're transferring at present */
	int	retries;
	int	settle_time;
	int	secpercyl;	/* disk geometry information */
	int	secpertrack;
	int	total_secs;
	int	write_prot;	/* 1 if write-protected, 0 if not, -1 dunno */
	struct dbdma_cmd *dma_cmd;
	int	ref_count;
	int	expect_cyl;
	int	timeout_pending;
	int	ejected;
	int	wanted;
	char	dbdma_cmd_space[5 * sizeof(struct dbdma_cmd)];
	int	index;
	struct request *cur_req;
};

#define swim3_err(fmt, arg...)	do { } while(0)//dev_err(&fs->mdev->ofdev.dev, "[fd%d] " fmt, fs->index, arg)
#define swim3_warn(fmt, arg...)	do { } while(0)//dev_warn(&fs->mdev->ofdev.dev, "[fd%d] " fmt, fs->index, arg)
#define swim3_info(fmt, arg...)	do { } while(0)//dev_info(&fs->mdev->ofdev.dev, "[fd%d] " fmt, fs->index, arg)

#ifdef DEBUG
#define swim3_dbg(fmt, arg...)	dev_dbg(&fs->mdev->ofdev.dev, "[fd%d] " fmt, fs->index, arg)
#else
#define swim3_dbg(fmt, arg...)	do { } while(0)
#endif

static struct floppy_state floppy_states[MAX_FLOPPIES];
static int floppy_count = 0;

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

static void seek_track(struct floppy_state *fs, int n);
static void act(struct floppy_state *fs);
static void swim3_interrupt(int irq, void *dev_id);
/*static void fd_dma_interrupt(int irq, void *dev_id);*/
static int grab_drive(struct floppy_state *fs, enum swim_state state,
		      int interruptible);
static void release_drive(struct floppy_state *fs);
static int fd_eject(struct floppy_state *fs);
static int floppy_ioctl(struct block_device *bdev, blk_mode_t mode,
			unsigned int cmd, unsigned long param);
static int floppy_open(struct floppy_state *fs, blk_mode_t mode);
static unsigned int floppy_check_events(struct floppy_state *fs,
					unsigned int clearing);
static int floppy_revalidate(struct floppy_state *fs);

static bool swim3_end_request(struct floppy_state *fs, blk_status_t err, unsigned int nr_bytes)
{
	struct request *req = fs->cur_req;

	swim3_dbg("  end request, err=%d nr_bytes=%d, cur_req=%p\n",
		  err, nr_bytes, req);

	if (err)
		nr_bytes = blk_rq_cur_bytes(req);
	if (blk_update_request(req, err, nr_bytes))
		return true;
	__blk_mq_end_request(req, err);
	fs->cur_req = NULL;
	return false;
}

static void swim3_select(struct floppy_state *fs, int sel)
{
	struct swim3 *sw = fs->swim3;

	out_8(&sw->select, RELAX);
	if (sel & 8)
		out_8(&sw->control_bis, SELECT);
	else
		out_8(&sw->control_bic, SELECT);
	out_8(&sw->select, sel & CA_MASK);
}

static void swim3_action(struct floppy_state *fs, int action)
{
	struct swim3 *sw = fs->swim3;

	swim3_select(fs, action);
	udelay(1);
	out_8(&sw->select, sw->select | LSTRB);
	udelay(2);
	out_8(&sw->select, sw->select & ~LSTRB);
	udelay(1);
}

static int swim3_readbit(struct floppy_state *fs, int bit)
{
	struct swim3 *sw = fs->swim3;
	int stat;

	swim3_select(fs, bit);
	udelay(1);
	stat = in_8(&sw->status);
	return (stat & DATA) == 0;
}

static blk_status_t swim3_queue_rq(struct floppy_state *fs,
				   struct request *req)
{
	unsigned long x;

	//spin_lock_irq(&swim3_lock);
	if (fs->cur_req || fs->state != idle) {
		//spin_unlock_irq(&swim3_lock);
		return BLK_STS_DEV_RESOURCE;
	}
	blk_mq_start_request(req);
	fs->cur_req = req;
	if (fs->ejected) {
		swim3_dbg("%s", "  disk ejected\n");
		swim3_end_request(fs, BLK_STS_IOERR, 0);
		goto out;
	}
	if (rq_data_dir(req) == WRITE) {
		if (fs->write_prot < 0)
			fs->write_prot = swim3_readbit(fs, WRITE_PROT);
		if (fs->write_prot) {
			swim3_dbg("%s", "  try to write, disk write protected\n");
			swim3_end_request(fs, BLK_STS_IOERR, 0);
			goto out;
		}
	}

	/*
	 * Do not remove the cast. blk_rq_pos(req) is now a sector_t and can be
	 * 64 bits, but it will never go past 32 bits for this driver anyway, so
	 * we can safely cast it down and not have to do a 64/32 division
	 */
	fs->req_cyl = ((long)blk_rq_pos(req)) / fs->secpercyl;
	x = ((long)blk_rq_pos(req)) % fs->secpercyl;
	fs->head = x / fs->secpertrack;
	fs->req_sector = x % fs->secpertrack + 1;
	fs->state = do_transfer;
	fs->retries = 0;

	act(fs);

out:
	//spin_unlock_irq(&swim3_lock);
	return BLK_STS_OK;
}

static inline void scan_track(struct floppy_state *fs)
{
	struct swim3 *sw = fs->swim3;

	swim3_select(fs, READ_DATA_0);
	in_8(&sw->intr);		/* clear SEEN_SECTOR bit */
	in_8(&sw->error);
	//out_8(&sw->intr_enable, SEEN_SECTOR);
	out_8(&sw->control_bis, DO_ACTION);
	while ((in_8(&sw->intr) & SEEN_SECTOR) == 0) {}
	swim3_interrupt(SEEN_SECTOR, fs);
}

static inline void seek_track(struct floppy_state *fs, int n)
{
	struct swim3 *sw = fs->swim3;

	if (n >= 0) {
		swim3_action(fs, SEEK_POSITIVE);
		sw->nseek = n;
	} else {
		swim3_action(fs, SEEK_NEGATIVE);
		sw->nseek = -n;
	}
	fs->expect_cyl = (fs->cur_cyl >= 0)? fs->cur_cyl + n: -1;
	swim3_select(fs, STEP);
	in_8(&sw->error);
	/* enable intr when seek finished */
	//out_8(&sw->intr_enable, SEEK_DONE);
	out_8(&sw->control_bis, DO_SEEK);
	while ((in_8(&sw->intr) & SEEN_SECTOR) == 0) {}
	swim3_interrupt(SEEK_DONE, fs);
}

/*
 * XXX: this is a horrible hack, but at least allows ppc32 to get
 * out of defining virt_to_bus, and this driver out of using the
 * deprecated block layer bounce buffering for highmem addresses
 * for no good reason.
 */
static unsigned long swim3_phys_to_bus(void* paddr)
{
	return paddr + PCI_DRAM_OFFSET;
}

static void* swim3_bio_phys(struct bio *bio)
{
	return page_to_phys(bio_page(bio)) + bio_offset(bio);
}

static inline void init_dma(struct dbdma_cmd *cp, int cmd,
			    void* paddr, int count)	
{
	cp->req_count = cpu_to_le16(count);
	cp->command = cpu_to_le16(cmd);
	cp->phy_addr = cpu_to_le32(swim3_phys_to_bus(paddr));
	cp->xfer_status = 0;
}

static inline void setup_transfer(struct floppy_state *fs)
{
	int n;
	struct swim3 *sw = fs->swim3;
	struct dbdma_cmd *cp = fs->dma_cmd;
	struct dbdma_regs *dr = fs->dma;
	struct request *req = fs->cur_req;

	if (blk_rq_cur_sectors(req) <= 0) {
		swim3_warn("%s", "Transfer 0 sectors ?\n");
		return;
	}
	if (rq_data_dir(req) == WRITE)
		n = 1;
	else {
		n = fs->secpertrack - fs->req_sector + 1;
		if (n > blk_rq_cur_sectors(req))
			n = blk_rq_cur_sectors(req);
	}

	swim3_dbg("  setup xfer at sect %d (of %d) head %d for %d\n",
		  fs->req_sector, fs->secpertrack, fs->head, n);

	fs->scount = n;
	swim3_select(fs, fs->head? READ_DATA_1: READ_DATA_0);
	out_8(&sw->sector, fs->req_sector);
	out_8(&sw->nsect, n);
	out_8(&sw->gap3, 0);
	out_le32(&dr->cmdptr, swim3_phys_to_bus(virt_to_phys(cp)));
	if (rq_data_dir(req) == WRITE) {
		/* Set up 3 dma commands: write preamble, data, postamble */
		init_dma(cp, OUTPUT_MORE, virt_to_phys(write_preamble),
			 sizeof(write_preamble));
		++cp;
		init_dma(cp, OUTPUT_MORE, swim3_bio_phys(req->bio), 512);
		++cp;
		init_dma(cp, OUTPUT_LAST, virt_to_phys(write_postamble),
			sizeof(write_postamble));
	} else {
		init_dma(cp, INPUT_LAST, swim3_bio_phys(req->bio), n * 512);
	}
	++cp;
	out_le16(&cp->command, DBDMA_STOP);
	out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
	in_8(&sw->error);
	out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
	if (rq_data_dir(req) == WRITE)
		out_8(&sw->control_bis, WRITE_SECTORS);
	in_8(&sw->intr);
	out_le32(&dr->control, (RUN << 16) | RUN);
	/* enable intr when transfer complete */
	out_8(&sw->intr_enable, TRANSFER_DONE);
	out_8(&sw->control_bis, DO_ACTION);
	set_timeout(fs, 2*HZ, xfer_timeout);	/* enable timeout */
}

static void act(struct floppy_state *fs)
{
	for (;;) {
		swim3_dbg("  act loop, state=%d, req_cyl=%d, cur_cyl=%d\n",
			  fs->state, fs->req_cyl, fs->cur_cyl);

		switch (fs->state) {
		case idle:
			return;		/* XXX shouldn't get here */

		case locating:
			if (swim3_readbit(fs, TRACK_ZERO)) {
				swim3_dbg("%s", "    locate track 0\n");
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
				swim3_warn("%s", "Whoops, seeking 0\n");
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
					swim3_err("Wrong cylinder in transfer, want: %d got %d\n",
						  fs->req_cyl, fs->cur_cyl);
					swim3_end_request(fs, BLK_STS_IOERR, 0);
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
			swim3_err("Unknown state %d\n", fs->state);
			return;
		}
	}
}

static void swim3_interrupt(int irq, void *dev_id)
{
	struct floppy_state *fs = (struct floppy_state *) dev_id;
	struct swim3 *sw = fs->swim3;
	int intr, err, n;
	int stat, resid;
	struct dbdma_regs *dr;
	struct dbdma_cmd *cp;
	unsigned long flags;
	struct request *req = fs->cur_req;

	swim3_dbg("* interrupt, state=%d\n", fs->state);

	//spin_lock_irqsave(&swim3_lock, flags);
	intr = in_8(&sw->intr);
	err = (intr & ERROR_INTR)? in_8(&sw->error): 0;
	if ((intr & ERROR_INTR) && fs->state != do_transfer)
		swim3_err("Non-transfer error interrupt: state=%d, dir=%x, intr=%x, err=%x\n",
			  fs->state, rq_data_dir(req), intr, err);
	switch (fs->state) {
	case locating:
		if (intr & SEEN_SECTOR) {
			out_8(&sw->control_bic, DO_ACTION | WRITE_SECTORS);
			out_8(&sw->select, RELAX);
			out_8(&sw->intr_enable, 0);
			fs->timeout_pending = 0;
			if (sw->ctrack == 0xff) {
				swim3_err("%s", "Seen sector but cyl=ff?\n");
				fs->cur_cyl = -1;
				if (fs->retries > 5) {
					swim3_end_request(fs, BLK_STS_IOERR, 0);
					fs->state = idle;
				} else {
					fs->state = jogging;
					act(fs);
				}
				break;
			}
			fs->cur_cyl = sw->ctrack;
			fs->cur_sector = sw->csect;
			if (fs->expect_cyl != -1 && fs->expect_cyl != fs->cur_cyl)
				swim3_err("Expected cyl %d, got %d\n",
					  fs->expect_cyl, fs->cur_cyl);
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
			fs->timeout_pending = 0;
			if (fs->state == seeking)
				++fs->retries;
			fs->state = settling;
			act(fs);
		}
		break;
	case settling:
		out_8(&sw->intr_enable, 0);
		fs->timeout_pending = 0;
		act(fs);
		break;
	case do_transfer:
		if ((intr & (ERROR_INTR | TRANSFER_DONE)) == 0)
			break;
		out_8(&sw->intr_enable, 0);
		out_8(&sw->control_bic, WRITE_SECTORS | DO_ACTION);
		out_8(&sw->select, RELAX);
		fs->timeout_pending = 0;
		dr = fs->dma;
		cp = fs->dma_cmd;
		if (rq_data_dir(req) == WRITE)
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
				barrier();
			}
		}
		/* turn off DMA */
		out_le32(&dr->control, (RUN | PAUSE) << 16);
		stat = le16_to_cpu(cp->xfer_status);
		resid = le16_to_cpu(cp->res_count);
		if (intr & ERROR_INTR) {
			n = fs->scount - 1 - resid / 512;
			if (n > 0) {
				blk_update_request(req, 0, n << 9);
				fs->req_sector += n;
			}
			if (fs->retries < 5) {
				++fs->retries;
				act(fs);
			} else {
				swim3_err("Error %sing block %ld (err=%x)\n",
				       rq_data_dir(req) == WRITE? "writ": "read",
				       (long)blk_rq_pos(req), err);
				swim3_end_request(fs, BLK_STS_IOERR, 0);
				fs->state = idle;
			}
		} else {
			if ((stat & ACTIVE) == 0 || resid != 0) {
				/* musta been an error */
				swim3_err("fd dma error: stat=%x resid=%d\n", stat, resid);
				swim3_err("  state=%d, dir=%x, intr=%x, err=%x\n",
					  fs->state, rq_data_dir(req), intr, err);
				swim3_end_request(fs, BLK_STS_IOERR, 0);
				fs->state = idle;
				break;
			}
			fs->retries = 0;
			if (swim3_end_request(fs, 0, fs->scount << 9)) {
				fs->req_sector += fs->scount;
				if (fs->req_sector > fs->secpertrack) {
					fs->req_sector -= fs->secpertrack;
					if (++fs->head > 1) {
						fs->head = 0;
						++fs->req_cyl;
					}
				}
				act(fs);
			} else
				fs->state = idle;
		}
		break;
	default:
		swim3_err("Don't know what to do in state %d\n", fs->state);
	}
	//spin_unlock_irqrestore(&swim3_lock, flags);
}

/*
static void fd_dma_interrupt(int irq, void *dev_id)
{
}
*/

/* Called under the mutex to grab exclusive access to a drive */
static int grab_drive(struct floppy_state *fs, enum swim_state state,
		      int interruptible)
{
	unsigned long flags;

	swim3_dbg("%s", "-> grab drive\n");

	//spin_lock_irqsave(&swim3_lock, flags);
	if (fs->state != idle && fs->state != available) {
		++fs->wanted;
		/* this will enable irqs in order to sleep */
		/*if (!interruptible)
			wait_event_lock_irq(fs->wait,
                                        fs->state == available,
                                        swim3_lock);
		else if (wait_event_interruptible_lock_irq(fs->wait,
					fs->state == available,
					swim3_lock)) {
			--fs->wanted;
			spin_unlock_irqrestore(&swim3_lock, flags);
			return -EINTR;
		}*/
		--fs->wanted;
	}
	fs->state = state;
	//spin_unlock_irqrestore(&swim3_lock, flags);

	return 0;
}

static void release_drive(struct floppy_state *fs)
{
	struct request_queue *q = disks[fs->index]->queue;
	unsigned long flags;

	swim3_dbg("%s", "-> release drive\n");

	//spin_lock_irqsave(&swim3_lock, flags);
	fs->state = idle;
	//spin_unlock_irqrestore(&swim3_lock, flags);

	blk_mq_freeze_queue(q);
	blk_mq_quiesce_queue(q);
	blk_mq_unquiesce_queue(q);
	blk_mq_unfreeze_queue(q);
}

static int fd_eject(struct floppy_state *fs)
{
	int err, n;

	err = grab_drive(fs, ejecting, 1);
	if (err)
		return err;
	swim3_action(fs, EJECT);
	for (n = 20; n > 0; --n) {
		swim3_select(fs, RELAX);
		schedule_timeout_interruptible(1);
		if (swim3_readbit(fs, DISK_IN) == 0)
			break;
	}
	swim3_select(fs, RELAX);
	udelay(150);
	fs->ejected = 1;
	release_drive(fs);
	return err;
}

static struct floppy_struct floppy_type =
	{ 2880,18,2,80,0,0x1B,0x00,0xCF,0x6C,NULL };	/*  7 1.44MB 3.5"   */

static int floppy_locked_ioctl(struct floppy_state *fs, blk_mode_t mode,
			unsigned int cmd, unsigned long param)
{
	int err;

	switch (cmd) {
	case FDEJECT:
		if (fs->ref_count != 1)
			return -EBUSY;
		err = fd_eject(fs);
		return err;
	case FDGETPRM:
	        if (copy_to_user((void *) param, &floppy_type,
				 sizeof(struct floppy_struct)))
			return -EFAULT;
		return 0;
	}
	return -ENOTTY;
}

static int floppy_ioctl(struct block_device *bdev, blk_mode_t mode,
				 unsigned int cmd, unsigned long param)
{
	int ret;

	//mutex_lock(&swim3_mutex);
	ret = floppy_locked_ioctl(bdev, mode, cmd, param);
	//mutex_unlock(&swim3_mutex);

	return ret;
}

static int floppy_open(struct floppy_state *fs, blk_mode_t mode)
{
	struct swim3 *sw = fs->swim3;
	int n, err = 0;

	if (fs->ref_count == 0) {
		out_8(&sw->setup, S_IBM_DRIVE | S_FCLK_DIV2);
		out_8(&sw->control_bic, 0xff);
		out_8(&sw->mode, 0x95);
		udelay(10);
		out_8(&sw->intr_enable, 0);
		out_8(&sw->control_bis, DRIVE_ENABLE | INTR_ENABLE);
		swim3_action(fs, MOTOR_ON);
		fs->write_prot = -1;
		fs->cur_cyl = -1;
		for (n = 0;; ++n) {
			if (n >= 10 && swim3_readbit(fs, SEEK_COMPLETE))
				break;
			swim3_select(fs, RELAX);
		}
		if (err == 0 && (swim3_readbit(fs, SEEK_COMPLETE) == 0
				 || swim3_readbit(fs, DISK_IN) == 0))
			err = -ENXIO;
		swim3_action(fs, SETMFM);
		swim3_select(fs, RELAX);

	} else if (fs->ref_count == -1 || mode & BLK_OPEN_EXCL)
		return -EBUSY;

	if (err == 0 && !(mode & BLK_OPEN_NDELAY) &&
	    (mode & (BLK_OPEN_READ | BLK_OPEN_WRITE))) {
		if (fs->ejected)
			err = -ENXIO;
	}

	if (err == 0 && (mode & BLK_OPEN_WRITE)) {
		if (fs->write_prot < 0)
			fs->write_prot = swim3_readbit(fs, WRITE_PROT);
		if (fs->write_prot)
			err = -EROFS;
	}

	if (err) {
		if (fs->ref_count == 0) {
			swim3_action(fs, MOTOR_OFF);
			out_8(&sw->control_bic, DRIVE_ENABLE | INTR_ENABLE);
			swim3_select(fs, RELAX);
		}
		return err;
	}

	if (mode & BLK_OPEN_EXCL)
		fs->ref_count = -1;
	else
		++fs->ref_count;

	return 0;
}

static int floppy_unlocked_open(struct gendisk *disk, blk_mode_t mode)
{
	int ret;

	//mutex_lock(&swim3_mutex);
	ret = floppy_open(disk, mode);
	//mutex_unlock(&swim3_mutex);

	return ret;
}

static void floppy_release(struct floppy_state *fs)
{
	struct swim3 *sw = fs->swim3;

	//mutex_lock(&swim3_mutex);
	if (fs->ref_count > 0)
		--fs->ref_count;
	else if (fs->ref_count == -1)
		fs->ref_count = 0;
	if (fs->ref_count == 0) {
		swim3_action(fs, MOTOR_OFF);
		out_8(&sw->control_bic, 0xff);
		swim3_select(fs, RELAX);
	}
	//mutex_unlock(&swim3_mutex);
}

static unsigned int floppy_check_events(struct floppy_state *fs,
					unsigned int clearing)
{
	return fs->ejected ? 1 : 0;
}

static int floppy_revalidate(struct floppy_state *fs)
{
	struct swim3 *sw;
	int ret, n;

	sw = fs->swim3;
	grab_drive(fs, revalidating, 0);
	out_8(&sw->intr_enable, 0);
	out_8(&sw->control_bis, DRIVE_ENABLE);
	swim3_action(fs, MOTOR_ON);	/* necessary? */
	fs->write_prot = -1;
	fs->cur_cyl = -1;
	mdelay(1);
	for (;;) {
		if (swim3_readbit(fs, SEEK_COMPLETE))
			break;
		swim3_select(fs, RELAX);
	}
	ret = swim3_readbit(fs, SEEK_COMPLETE) == 0
		|| swim3_readbit(fs, DISK_IN) == 0;
	if (ret)
		swim3_action(fs, MOTOR_OFF);
	else {
		fs->ejected = 0;
		swim3_action(fs, SETMFM);
	}
	swim3_select(fs, RELAX);

	release_drive(fs);
	return ret;
}

static void swim3_mb_event(struct floppy_state *fs, int mb_state)
{
	struct swim3 *sw;

	if (!fs)
		return;

	sw = fs->swim3;

	if (mb_state != MB_FD)
		return;

	/* Clear state */
	out_8(&sw->intr_enable, 0);
	in_8(&sw->intr);
	in_8(&sw->error);
}

static int swim3_add_device(unsigned long swim3addr, unsigned long dmaaddr, int index)
{
	struct floppy_state *fs = &floppy_states[index];
	int rc = -EBUSY;

	fs->index = index;
	
	fs->state = idle;
	fs->swim3 = (struct swim3 *)swim3addr;
	if (fs->swim3 == NULL) {
		swim3_err("%s", "Couldn't map mmio registers\n");
		rc = -ENOMEM;
		goto out_release;
	}
	fs->dma = (struct dbdma_regs *)dmaaddr;
	if (fs->dma == NULL) {
		swim3_err("%s", "Couldn't map dma registers\n");
		iounmap(fs->swim3);
		rc = -ENOMEM;
		goto out_release;
	}
	fs->cur_cyl = -1;
	fs->cur_sector = -1;
	fs->secpercyl = 36;
	fs->secpertrack = 18;
	fs->total_secs = 2880;

	fs->dma_cmd = (struct dbdma_cmd *) DBDMA_ALIGN(fs->dbdma_cmd_space);
	memset(fs->dma_cmd, 0, 2 * sizeof(struct dbdma_cmd));
	fs->dma_cmd[1].command = cpu_to_le16(DBDMA_STOP);

	swim3_mb_event(fs, MB_FD);

	swim3_info("SWIM3 floppy controller\n");

	return 0;

 out_unmap:
 out_release:
	return rc;
}
#endif