/*
 *  controlfb.c -- frame buffer device for the PowerMac 'control' display
 *
 *  Created 12 July 1998 by Dan Jacobowitz <dan@debian.org>
 *  Copyright (C) 1998 Dan Jacobowitz
 *  Copyright (C) 2001 Takashi Oe
 *
 *  Mmap code by Michel Lanners <mlan@cpu.lu>
 *
 *  Frame buffer structure from:
 *    drivers/video/chipsfb.c -- frame buffer device for
 *    Chips & Technologies 65550 chip.
 *
 *    Copyright (C) 1998 Paul Mackerras
 *
 *    This file is derived from the Powermac "chips" driver:
 *    Copyright (C) 1997 Fabio Riccardi.
 *    And from the frame buffer device for Open Firmware-initialized devices:
 *    Copyright (C) 1997 Geert Uytterhoeven.
 *
 *  Hardware information from:
 *    control.c: Console support for PowerMac "control" display adaptor.
 *    Copyright (C) 1996 Paul Mackerras
 *
 *  Updated to 2.5 framebuffer API by Ben Herrenschmidt
 *  <benh@kernel.crashing.org>, Paul Mackerras <paulus@samba.org>,
 *  and James Simmons <jsimmons@infradead.org>.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */


#include "fb.h"
#include "macmodes.h"
#include "controlfb.h"
#include <stddef.h>
#include <errno.h>
#include <string.h>
#ifndef EINVAL
#define EINVAL 22
#endif

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

static inline unsigned in_le16(const volatile __u16 *addr)
{
	unsigned ret;

	__asm__ __volatile__("lhbrx %0,0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "r" (addr), "m" (*addr));

	return ret;
}

static inline unsigned in_be16(const volatile __u16 *addr)
{
	unsigned ret;

	__asm__ __volatile__("lhz%U1%X1 %0,%1; twi 0,%0,0; isync"
			     : "=r" (ret) : "m" (*addr));
	return ret;
}

static inline void out_le16(volatile __u16 *addr, int val)
{
	__asm__ __volatile__("sthbrx %1,0,%2; sync" : "=m" (*addr)
			     : "r" (val), "r" (addr));
}

static inline void out_be16(volatile __u16 *addr, int val)
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

static inline void sync(void)
{
	asm volatile("sync" : : : "memory");
}

static inline void eieio(void)
{
	asm volatile("eieio" : : : "memory");
}

static inline void barrier(void)
{
	asm volatile("" : : : "memory");
}

static inline unsigned long mftb(void)
{
	unsigned long low;

	asm volatile("mftb %0" : "=r" (low));

	return low;
}

static void __delay(unsigned long loops)
{
	unsigned long start;
	start = mftb();
	while (mftb() - start < loops) ;
}

static void invalid_vram_cache(void *addr)
{
	asm("eieio; dcbf 0,%0; sync; eieio; dcbf 0,%0; sync" : : "b"(addr));
}

#define DIRTY(z) ((x)->z != (y)->z)
#define DIRTY_CMAP(z) (memcmp(&((x)->z), &((y)->z), sizeof((y)->z)))
static inline int PAR_EQUAL(struct fb_par_control *x, struct fb_par_control *y)
{
	int i, results;

	results = 1;
	for (i = 0; i < 3; i++)
		results &= !DIRTY(regvals.clock_params[i]);
	if (!results)
		return 0;
	for (i = 0; i < 16; i++)
		results &= !DIRTY(regvals.regs[i]);
	if (!results)
		return 0;
	return (!DIRTY(cmode) && !DIRTY(xres) && !DIRTY(yres)
		&& !DIRTY(vxres) && !DIRTY(vyres));
}

/* control register access macro */
#define CNTRL_REG(INFO,REG) (&(((INFO)->control_regs->REG).r))


/************************** Internal variables *******************************/


static int controlfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			     u_int transp, struct fb_info_control *info)
{
	struct fb_info_control *p = info ;
	__u8 r, g, b;

	if (regno > 255)
		return 1;

	r = red >> 8;
	g = green >> 8;
	b = blue >> 8;

	out_8(&p->cmap_regs->addr, regno);	/* tell clut what addr to fill	*/
	out_8(&p->cmap_regs->lut, r);		/* send one color channel at	*/
	out_8(&p->cmap_regs->lut, g);		/* a time...			*/
	out_8(&p->cmap_regs->lut, b);

	if (regno < 16) {
		int i;
		switch (p->par.cmode) {
		case CMODE_16:
			p->pseudo_palette[regno] =
			    (regno << 10) | (regno << 5) | regno;
			break;
		case CMODE_32:
			i = (regno << 8) | regno;
			p->pseudo_palette[regno] = (i << 16) | i;
			break;
		}
	}

	return 0;
}


/********************  End of controlfb_ops implementation  ******************/

/*
 * Set screen start address according to var offset values
 */
static inline void set_screen_start(int xoffset, int yoffset,
	struct fb_info_control *p)
{
	struct fb_par_control *par = &p->par;

	par->xoffset = xoffset;
	par->yoffset = yoffset;
	out_le32(CNTRL_REG(p,start_addr),
		 par->yoffset * par->pitch + (par->xoffset << par->cmode));
}

#define RADACAL_WRITE(a,d) \
	out_8(&p->cmap_regs->addr, (a)); \
	out_8(&p->cmap_regs->dat,   (d))

#define DEBUG_PIXEL(n) *p->frame_buffer = (n); invalid_vram_cache(p->frame_buffer);

/* Now how about actually saying, Make it so! */
/* Some things in here probably don't need to be done each time. */
void control_set_hardware(struct fb_info_control *p, struct fb_par_control *par)
{
	struct control_regvals	*r;
	volatile struct preg	*rp;
	int			i, cmode;

#if 0
	if (PAR_EQUAL(&p->par, par)) {
		/*
		 * check if only xoffset or yoffset differs.
		 * this prevents flickers in typical VT switch case.
		 */
		if (p->par.xoffset != par->xoffset ||
		    p->par.yoffset != par->yoffset)
			set_screen_start(par->xoffset, par->yoffset, p);

		return;
	}
#endif

	p->par = *par;
	cmode = p->par.cmode;
	r = &par->regvals;

	/* Turn off display */
	out_le32(CNTRL_REG(p,ctrl), 0x400 | par->ctrl);

	//set_control_clock(r->clock_params);

	RADACAL_WRITE(0x20, r->radacal_ctrl);
	RADACAL_WRITE(0x21, p->control_use_bank2 ? 0 : 1);
	RADACAL_WRITE(0x10, 0);
	RADACAL_WRITE(0x11, 0);

	rp = &p->control_regs->vswin;
	for (i = 0; i < 16; ++i, ++rp)
		out_le32(&rp->r, r->regs[i]);

	out_le32(CNTRL_REG(p,pitch), par->pitch);
	out_le32(CNTRL_REG(p,mode), r->mode);
	out_le32(CNTRL_REG(p,vram_attr), p->vram_attr);

	// Setting LE makes the registers BE?
	if (p->vram_attr & 2) {
		out_be32(CNTRL_REG(p,start_addr), par->yoffset * par->pitch
			+ (par->xoffset << cmode));
		out_be32(CNTRL_REG(p,rfrcnt), 0x1e5);
		out_be32(CNTRL_REG(p,intr_ena), 0);

		/* Turn on display */
		//out_be32(CNTRL_REG(p,ctrl), par->ctrl);
	} else {
		out_le32(CNTRL_REG(p,start_addr), par->yoffset * par->pitch
			+ (par->xoffset << cmode));
		out_le32(CNTRL_REG(p,rfrcnt), 0x1e5);
		out_le32(CNTRL_REG(p,intr_ena), 0);

		/* Turn on display */
		//out_le32(CNTRL_REG(p,ctrl), par->ctrl);
	}
}

/* Work out which banks of VRAM we have installed. */
/* danj: I guess the card just ignores writes to nonexistant VRAM... */

void find_vram_size(struct fb_info_control *p)
{
	int bank1, bank2;

	/*
	 * Set VRAM in 2MB (bank 1) mode
	 * VRAM Bank 2 will be accessible through offset 0x600000 if present
	 * and VRAM Bank 1 will not respond at that offset even if present
	 */
	out_le32(CNTRL_REG(p,vram_attr), 0x31);

	out_8(&p->frame_buffer[0x600000], 0xb3);
	out_8(&p->frame_buffer[0x600001], 0x71);
	invalid_vram_cache(&p->frame_buffer[0x600000]);

	bank2 = (in_8(&p->frame_buffer[0x600000]) == 0xb3)
		&& (in_8(&p->frame_buffer[0x600001]) == 0x71);

	/*
	 * Set VRAM in 2MB (bank 2) mode
	 * VRAM Bank 1 will be accessible through offset 0x000000 if present
	 * and VRAM Bank 2 will not respond at that offset even if present
	 */
	out_le32(CNTRL_REG(p,vram_attr), 0x39);

	out_8(&p->frame_buffer[0], 0x5a);
	out_8(&p->frame_buffer[1], 0xc7);
	invalid_vram_cache(&p->frame_buffer[0]);

	bank1 = (in_8(&p->frame_buffer[0]) == 0x5a)
		&& (in_8(&p->frame_buffer[1]) == 0xc7);

	if (bank2) {
		if (!bank1) {
			/*
			 * vram bank 2 only
			 */
			p->control_use_bank2 = 1;
			p->vram_attr = 0x39;
			p->frame_buffer += 0x600000;
			p->frame_buffer_phys += 0x600000;
		} else {
			/*
			 * 4 MB vram
			 */
			p->vram_attr = 0x51;
		}
	} else {
		/*
		 * vram bank 1 only
		 */
		p->vram_attr = 0x31;
	}

        p->total_vram = (bank1 + bank2) * 0x200000;
}

/*
 * Get the monitor sense value.
 * Note that this can be called before calibrate_delay,
 * so we can't use udelay.
 */
static int read_control_sense(struct fb_info_control *p)
{
	int sense;

	out_le32(CNTRL_REG(p,mon_sense), 7);	/* drive all lines high */
	__delay(200);
	out_le32(CNTRL_REG(p,mon_sense), 077);	/* turn off drivers */
	__delay(2000);
	sense = (in_le32(CNTRL_REG(p,mon_sense)) & 0x1c0) << 2;

	/* drive each sense line low in turn and collect the other 2 */
	out_le32(CNTRL_REG(p,mon_sense), 033);	/* drive A low */
	__delay(2000);
	sense |= (in_le32(CNTRL_REG(p,mon_sense)) & 0xc0) >> 2;
	out_le32(CNTRL_REG(p,mon_sense), 055);	/* drive B low */
	__delay(2000);
	sense |= ((in_le32(CNTRL_REG(p,mon_sense)) & 0x100) >> 5)
		| ((in_le32(CNTRL_REG(p,mon_sense)) & 0x40) >> 4);
	out_le32(CNTRL_REG(p,mon_sense), 066);	/* drive C low */
	__delay(2000);
	sense |= (in_le32(CNTRL_REG(p,mon_sense)) & 0x180) >> 7;

	out_le32(CNTRL_REG(p,mon_sense), 077);	/* turn off drivers */

	return sense;
}

/**********************  Various translation functions  **********************/

#define CONTROL_PIXCLOCK_BASE	256016
#define CONTROL_PIXCLOCK_MIN	5000	/* ~ 200 MHz dot clock */

/*
 * calculate the clock parameters to be sent to CUDA according to given
 * pixclock in pico second.
 */
static int calc_clock_params(unsigned long clk, unsigned char *param)
{
	unsigned long p0, p1, p2, k, l, m, n, min;

	if (clk > (CONTROL_PIXCLOCK_BASE << 3))
		return 1;

	p2 = ((clk << 4) < CONTROL_PIXCLOCK_BASE)? 3: 2;
	l = clk << p2;
	p0 = 0;
	p1 = 0;
	for (k = 1, min = l; k < 32; k++) {
		unsigned long rem;

		m = CONTROL_PIXCLOCK_BASE * k;
		n = m / l;
		rem = m % l;
		if (n && (n < 128) && rem < min) {
			p0 = k;
			p1 = n;
			min = rem;
		}
	}
	if (!p0 || !p1)
		return 1;

	param[0] = p0;
	param[1] = p1;
	param[2] = p2;

	return 0;
}


/*
 * This routine takes a user-supplied var, and picks the best vmode/cmode
 * from it.
 */

int control_var_to_par(struct fb_var_screeninfo *var,
	struct fb_par_control *par, const struct fb_info_control *fb_info)
{
	int cmode, piped_diff, hstep;
	unsigned hperiod, hssync, hsblank, hesync, heblank, piped, heq, hlfln,
		 hserr, vperiod, vssync, vesync, veblank, vsblank, vswin, vewin;
	unsigned long pixclock;
	const struct fb_info_control *p = fb_info;
	struct control_regvals *r = &par->regvals;

	switch (var->bits_per_pixel) {
	case 8:
		par->cmode = CMODE_8;
		if (p->total_vram > 0x200000) {
			r->mode = 3;
			r->radacal_ctrl = 0x20;
			piped_diff = 13;
		} else {
			r->mode = 2;
			r->radacal_ctrl = 0x10;
			piped_diff = 9;
		}
		break;
	case 15:
	case 16:
		par->cmode = CMODE_16;
		if (p->total_vram > 0x200000) {
			r->mode = 2;
			r->radacal_ctrl = 0x24;
			piped_diff = 5;
		} else {
			r->mode = 1;
			r->radacal_ctrl = 0x14;
			piped_diff = 3;
		}
		break;
	case 32:
		par->cmode = CMODE_32;
		if (p->total_vram > 0x200000) {
			r->mode = 1;
			r->radacal_ctrl = 0x28;
		} else {
			r->mode = 0;
			r->radacal_ctrl = 0x18;
		}
		piped_diff = 1;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * adjust xres and vxres so that the corresponding memory widths are
	 * 32-byte aligned
	 */
	hstep = 31 >> par->cmode;
	par->xres = (var->xres + hstep) & ~hstep;
	par->vxres = (var->xres_virtual + hstep) & ~hstep;
	par->xoffset = (var->xoffset + hstep) & ~hstep;
	if (par->vxres < par->xres)
		par->vxres = par->xres;
	par->pitch = par->vxres << par->cmode;

	par->yres = var->yres;
	par->vyres = var->yres_virtual;
	par->yoffset = var->yoffset;
	if (par->vyres < par->yres)
		par->vyres = par->yres;

	par->sync = var->sync;

	if (par->pitch * par->vyres + CTRLFB_OFF > p->total_vram)
		return -EINVAL;

	if (par->xoffset + par->xres > par->vxres)
		par->xoffset = par->vxres - par->xres;
	if (par->yoffset + par->yres > par->vyres)
		par->yoffset = par->vyres - par->yres;

	pixclock = (var->pixclock < CONTROL_PIXCLOCK_MIN)? CONTROL_PIXCLOCK_MIN:
		   var->pixclock;
	if (calc_clock_params(pixclock, r->clock_params))
		return -EINVAL;

	hperiod = ((var->left_margin + par->xres + var->right_margin
		    + var->hsync_len) >> 1) - 2;
	hssync = hperiod + 1;
	hsblank = hssync - (var->right_margin >> 1);
	hesync = (var->hsync_len >> 1) - 1;
	heblank = (var->left_margin >> 1) + hesync;
	piped = heblank - piped_diff;
	heq = var->hsync_len >> 2;
	hlfln = (hperiod+2) >> 1;
	hserr = hssync-hesync;
	vperiod = (var->vsync_len + var->lower_margin + par->yres
		   + var->upper_margin) << 1;
	vssync = vperiod - 2;
	vesync = (var->vsync_len << 1) - vperiod + vssync;
	veblank = (var->upper_margin << 1) + vesync;
	vsblank = vssync - (var->lower_margin << 1);
	vswin = (vsblank+vssync) >> 1;
	vewin = (vesync+veblank) >> 1;

	r->regs[0] = vswin;
	r->regs[1] = vsblank;
	r->regs[2] = veblank;
	r->regs[3] = vewin;
	r->regs[4] = vesync;
	r->regs[5] = vssync;
	r->regs[6] = vperiod;
	r->regs[7] = piped;
	r->regs[8] = hperiod;
	r->regs[9] = hsblank;
	r->regs[10] = heblank;
	r->regs[11] = hesync;
	r->regs[12] = hssync;
	r->regs[13] = heq;
	r->regs[14] = hlfln;
	r->regs[15] = hserr;

	if (par->xres >= 1280 && par->cmode >= CMODE_16)
		par->ctrl = 0x7f;
	else
		par->ctrl = 0x3b;

	if (mac_var_to_vmode(var, &par->vmode, &cmode))
		par->vmode = 0;

	return 0;
}


/*
 * Convert hardware data in par to an fb_var_screeninfo
 */

void control_par_to_var(struct fb_par_control *par, struct fb_var_screeninfo *var)
{
	struct control_regints *rv;

	rv = (struct control_regints *) par->regvals.regs;

	memset(var, 0, sizeof(*var));
	var->xres = par->xres;
	var->yres = par->yres;
	var->xres_virtual = par->vxres;
	var->yres_virtual = par->vyres;
	var->xoffset = par->xoffset;
	var->yoffset = par->yoffset;

	switch(par->cmode) {
	default:
	case CMODE_8:
		var->bits_per_pixel = 8;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		break;
	case CMODE_16:	/* RGB 555 */
		var->bits_per_pixel = 16;
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.length = 5;
		break;
	case CMODE_32:	/* RGB 888 */
		var->bits_per_pixel = 32;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->height = -1;
	var->width = -1;
	var->vmode = FB_VMODE_NONINTERLACED;

	var->left_margin = (rv->heblank - rv->hesync) << 1;
	var->right_margin = (rv->hssync - rv->hsblank) << 1;
	var->hsync_len = (rv->hperiod + 2 - rv->hssync + rv->hesync) << 1;

	var->upper_margin = (rv->veblank - rv->vesync) >> 1;
	var->lower_margin = (rv->vssync - rv->vsblank) >> 1;
	var->vsync_len = (rv->vperiod - rv->vssync + rv->vesync) >> 1;

	var->sync = par->sync;

	/*
	 * 10^12 * clock_params[0] / (3906400 * clock_params[1]
	 *			      * 2^clock_params[2])
	 * (10^12 * clock_params[0] / (3906400 * clock_params[1]))
	 * >> clock_params[2]
	 */
	/* (255990.17 * clock_params[0] / clock_params[1]) >> clock_params[2] */
	var->pixclock = CONTROL_PIXCLOCK_BASE * par->regvals.clock_params[0];
	var->pixclock /= par->regvals.clock_params[1];
	var->pixclock >>= par->regvals.clock_params[2];
}

/********************  The functions for controlfb_ops ********************/

/*
 * Checks a var structure
 */
int controlfb_check_var (struct fb_var_screeninfo *var, struct fb_info_control *info)
{
	struct fb_par_control par;
	int err;

	err = control_var_to_par(var, &par, info);
	if (err)
		return err;
	control_par_to_var(&par, var);

	return 0;
}

/*
 * Applies current var to display
 */
int controlfb_set_par (struct fb_info_control *info)
{
	struct fb_info_control *p = info;
	struct fb_par_control par;
	int err;

	if((err = control_var_to_par(&info->var, &par, info))) {
		return err;
	}

	control_set_hardware(p, &par);

	return 0;
}

int controlfb_pan_display(struct fb_var_screeninfo *var,
				 struct fb_info_control *info)
{
	unsigned int xoffset, hstep;
	struct fb_info_control *p = info;
	struct fb_par_control *par = &p->par;

	/*
	 * make sure start addr will be 32-byte aligned
	 */
	hstep = 0x1f >> par->cmode;
	xoffset = (var->xoffset + hstep) & ~hstep;

	if (xoffset+par->xres > par->vxres ||
	    var->yoffset+par->yres > par->vyres)
		return -EINVAL;

	set_screen_start(xoffset, var->yoffset, p);

	return 0;
}

int controlfb_blank(int blank_mode, struct fb_info_control *info)
{
	struct fb_info_control *p = info;
	unsigned ctrl;

	ctrl = in_le32(CNTRL_REG(p, ctrl));
	if (blank_mode > 0)
		switch (blank_mode) {
		case FB_BLANK_VSYNC_SUSPEND:
			ctrl &= ~3;
			break;
		case FB_BLANK_HSYNC_SUSPEND:
			ctrl &= ~0x30;
			break;
		case FB_BLANK_POWERDOWN:
			ctrl &= ~0x33;
			// fallthrough;
		case FB_BLANK_NORMAL:
			ctrl |= 0x400;
			break;
		default:
			break;
		}
	else {
		ctrl &= ~0x400;
		ctrl |= 0x33;
	}
	out_le32(CNTRL_REG(p,ctrl), ctrl);

	return 0;
}
