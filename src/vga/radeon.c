
#include <u.h>
#include <libc.h>
#include <bio.h>

#include "pci.h"
#include "vga.h"

#include "radeon.h"
#include "radeon_pciids.h"

#include "atom-types.h"
#include "atom.h"
#include "atombios.h"

static int debug = 1;
#define DBGPRINT	if (debug) print

#define VERSION_STRING "0.1"

/* *********************************************** */
enum {
	Kilo	= 1024,
	Mega	= (Kilo*Kilo),
};

enum {
	DISPLAY_CRT,
	DISPLAY_FP,
	DISPLAY_LCD,
};

/* *********************************************** */
typedef struct Radeon	Radeon;
struct Radeon {
	ulong		mmio;
	Pcidev*		pci;
	uchar*		bios;

	int			is_atombios; /* New BIOS used in >= R420. */

	ulong		fbsize;
	int			display_type;

	ulong		ovr_clr;
	ulong		ovr_wid_top_bottom;
	ulong		ovr_wid_left_right;
	ulong		ov0_scale_cntl;
	ulong		subpic_cntl;
	ulong		viph_control;
	ulong		i2c_cntl_1;
	ulong		rbbm_soft_reset;
	ulong		cap0_trig_cntl;
	ulong		cap1_trig_cntl;
	ulong		gen_int_cntl;
	ulong		bus_cntl;

	ulong		crtc_gen_cntl;
	ulong		crtc_ext_cntl;
	ulong		dac_cntl;

	ulong		crtc_h_total_disp;
	ulong		crtc_h_sync_strt_wid;
	ulong		crtc_v_total_disp;
	ulong		crtc_v_sync_strt_wid;

	ulong		crtc_pitch;

	ulong		crtc_offset;
	ulong		crtc_offset_cntl;

	ulong		htotal_cntl;

	ulong		surface_cntl;

	int			card_type; // e.g., ATI_R300 (from radeon_pciids.h)

	// inited from rom
	ushort		reference_freq, reference_div, xclk;
	ulong		max_pll_freq, min_pll_freq; // pll out frequecies

	ulong		pll_output_freq;
	ulong		feedback_div;
	ulong		dot_clock_freq;

	ulong		post_div;
	ulong		ppll_ref_div;
	ulong		ppll_div_3;

	// atombios
	struct atom_context *atom_context;
};

/* from io.c */
extern char* readbios(long len, long offset);

/* *********************************************** */
static void radeon300_workaround(Radeon* radeon);

static void
OUTREG8(Radeon* radeon, ulong offset, uchar val)
{
	((uchar*) (radeon->mmio + offset))[0] = val;
}

static void
OUTREG(Radeon* radeon, ulong offset, ulong val)
{
	((ulong*) (radeon->mmio + offset))[0] = val;
}

static ulong
INREG(Radeon* radeon, ulong offset)
{
	ulong	data;

	data = ((ulong*) (radeon->mmio + offset))[0];
	return data;
}

static void
OUTREGP(Radeon* radeon, ulong offset, ulong val, ulong mask)
{
	ulong	tmp;

	tmp = INREG(radeon, offset);
	tmp &= mask;
	tmp |= val;
	OUTREG(radeon, offset, tmp);
}

static void
OUTPLL(Radeon* radeon, ulong offset, ulong val)
{
	uchar	tmp;

	tmp = (offset & 0x3f) | RADEON_PLL_WR_EN;
	OUTREG8(radeon, RADEON_CLOCK_CNTL_INDEX, tmp);
	OUTREG(radeon, RADEON_CLOCK_CNTL_DATA, val);
}

static ulong
INPLL(Radeon* radeon, ulong offset)
{
	ulong	data;

	OUTREG8(radeon, RADEON_CLOCK_CNTL_INDEX, offset & 0x3f);
	data = INREG(radeon, RADEON_CLOCK_CNTL_DATA);
	if (radeon->card_type == ATI_R300)
		radeon300_workaround(radeon);

	return data;
}

static void
OUTPLLP(Radeon* radeon, ulong offset, ulong val, ulong mask)
{
	ulong	tmp;
	
	tmp = INPLL(radeon, offset);
	tmp &= mask;
	tmp |= val;
	OUTPLL(radeon, offset, tmp);
}

static void
radeon300_workaround(Radeon* radeon)
{
    ulong	save, tmp;

    save = INREG(radeon, RADEON_CLOCK_CNTL_INDEX);
    tmp = save & ~(0x3f | RADEON_PLL_WR_EN);
    OUTREG(radeon, RADEON_CLOCK_CNTL_INDEX, tmp);
    tmp = INREG(radeon, RADEON_CLOCK_CNTL_DATA);
    OUTREG(radeon, RADEON_CLOCK_CNTL_INDEX, save);

	USED(tmp);
}

/* *********************************************** */
/* Gratuitous register writing functions that have been copied verbatim
 *  to ideally lower the probability of error. */
/* ATOM accessor methods */
static inline u32int readl(void *addr)
{
	u32int data = ((u32int*) (addr))[0];
	return data;
}

static inline void writel(u32int val, void *addr)
{
	*(u32int *)addr = val;
}

static inline u32int r100_mm_rreg(Radeon *radeon, u32int reg)
{
	if (reg < 0x10000)
		return readl(((char *)radeon->mmio) + reg);
	else {
		writel(reg, ((char *)radeon->mmio) + RADEON_MM_INDEX);
		return readl(((char *)radeon->mmio) + RADEON_MM_DATA);
	}
}

static inline void r100_mm_wreg(Radeon *radeon, u32int reg, u32int v)
{
	if (reg < 0x10000)
		writel(v, ((char *)radeon->mmio) + reg);
	else {
		writel(reg, ((char *)radeon->mmio) + RADEON_MM_INDEX);
		writel(v, ((char *)radeon->mmio) + RADEON_MM_DATA);
	}
}

static void cail_reg_write(struct card_info *info, u32int reg, u32int val)
{
	Radeon *radeon = info->dev;
	r100_mm_wreg(radeon, reg*4, val);
}

static u32int cail_reg_read(struct card_info *info, u32int reg)
{
	Radeon *radeon = info->dev;
	u32int r = r100_mm_rreg(radeon, reg*4);
	return r;
}

/* From kernel DRM driver */
static struct card_info atom_card_info = {
	.dev = nil,
	.reg_read = cail_reg_read,
	.reg_write = cail_reg_write,
	.mc_read = nil, //cail_mc_read,
	.mc_write = nil, //cail_mc_write,
	.pll_read = nil, //cail_pll_read,
	.pll_write = nil, //cail_pll_write,
};

void atombios_blank_crtc(Radeon *radeon, int state)
{
	int index = GetIndexIntoCommandTable(BlankCRTC);
	BLANK_CRTC_PS_ALLOCATION args;

	memset(&args, 0, sizeof(args));

	args.ucCRTC = 1;
	args.ucBlanking = state;

	atom_execute_table(radeon->atom_context, index, (u32int *)&args);
}

/* ATOMBIOS nonsense. */
int radeon_atombios_init(Radeon *radeon)
{
	/* src: radeon drm driver
	 * radeon_device.c:radeon_atombios_init() */
	atom_card_info.dev = radeon;
	
	radeon->atom_context = atom_parse(&atom_card_info, radeon->bios);
	//radeon_atom_initialize_bios_scratch_regs(radeon);

	atombios_blank_crtc(radeon, 1);

	return 0;
}


/* *********************************************** */
static void
radeon_getbiosparams(Radeon* radeon)
{
	ushort	offset;
	ulong	addr;
	uchar*	bios;
	ushort	pib; // pll info block
	int     tmp;
	
	radeon->bios = nil;
	addr = 0xC0000;
	bios = (uchar*)readbios(0x10000, addr);
	if (bios[0] != 0x55 || bios[1] != 0xAA) {
		addr = 0xE0000;
		bios = (uchar*)readbios(0x10000, addr);
		if (bios[0] != 0x55 || bios[1] != 0xAA) {
			print("radeon: bios not found\n");
			return;
		}
	}

	radeon->bios = bios;
	offset = RADEON_BIOS16(radeon, RADEON_BIOS_START);

	/* Does this card include ATOMBIOS tables? */
	tmp = offset + 4;
	if (
		(RADEON_BIOS8(radeon, tmp)   == 'A' &&
		 RADEON_BIOS8(radeon, tmp+1) == 'T' &&
		 RADEON_BIOS8(radeon, tmp+2) == 'O' &&
		 RADEON_BIOS8(radeon, tmp+3) == 'M') ||

		(RADEON_BIOS8(radeon, tmp)   == 'M' &&
		 RADEON_BIOS8(radeon, tmp+1) == 'O' &&
		 RADEON_BIOS8(radeon, tmp+2) == 'T' &&
		 RADEON_BIOS8(radeon, tmp+3) == 'A')
	   )
		radeon->is_atombios = 1;
	else
		radeon->is_atombios = 0;


	/* Handle Legacy (R200, R300) BIOSes. */
	if (!radeon->is_atombios)
	{
		pib = RADEON_BIOS16(radeon, offset+0x30);

		DBGPRINT("radeon: bios=0x%08ulx offset=0x%ux\n", addr, offset);
		DBGPRINT("radeon: pll_info_block: 0x%ux\n", pib);
	
		radeon->reference_freq	= RADEON_BIOS16(radeon, pib+0x0e);
		radeon->reference_div	= RADEON_BIOS16(radeon, pib+0x10);
		radeon->min_pll_freq	= RADEON_BIOS32(radeon, pib+0x12);
		radeon->max_pll_freq	= RADEON_BIOS32(radeon, pib+0x16);
		radeon->xclk			= RADEON_BIOS16(radeon, pib+0x08);
	}
	else /* Read from non-legacy BIOS block. */
	{
		/* Reference: xf86-video-ati
		 * radeon_bios.c:RADEONGetClockInfoFromBIOS */

		tmp = RADEON_BIOS16(radeon, offset + 32);
		pib = RADEON_BIOS16(radeon, tmp + 12);

		radeon->reference_freq = RADEON_BIOS16(radeon, pib + 82);
		radeon->reference_div  = 0; /* Calculated below. */
		radeon->min_pll_freq   = RADEON_BIOS16(radeon, pib + 78);
		radeon->max_pll_freq   = RADEON_BIOS32(radeon, pib + 32);
		radeon->xclk           = RADEON_BIOS16(radeon, pib + 72);

		if (radeon->min_pll_freq == 0)
		{
			if (IS_AVIVO_VARIANT(radeon->card_type))
				radeon->min_pll_freq = 64800;
			else
				radeon->min_pll_freq = 20000;
		}
	}

	/* Calculate reference_div. */
	tmp = INPLL(radeon, RADEON_PPLL_REF_DIV);
	if (IS_R300_VARIANT(radeon->card_type) ||
	    radeon->card_type == ATI_RS300 ||
		radeon->card_type == ATI_RS400 ||
		radeon->card_type == ATI_RS480)
	{
		radeon->reference_div =
			(tmp & R300_PPLL_REF_DIV_ACC_MASK) >>
				R300_PPLL_REF_DIV_ACC_SHIFT;
	} else {
		radeon->reference_div = tmp & RADEON_PPLL_REF_DIV_MASK;
	}
	if (radeon->reference_div < 2)
		radeon->reference_div = 12;

	DBGPRINT("radeon: reference_freq: %ud\n", radeon->reference_freq);
	DBGPRINT("radeon: reference_div:  %ud\n", radeon->reference_div);
	DBGPRINT("radeon: min_pll_freq:   %uld\n", radeon->min_pll_freq);
	DBGPRINT("radeon: max_pll_freq:   %uld\n", radeon->max_pll_freq);
	DBGPRINT("radeon: xclk:           %ud\n", radeon->xclk);
}

/* *********************************************** */

/**
 * Find the first known PCI card that this driver can handle.
 *
 * @param card_type
 *  Address into which the card model will be written;
 *  for example, ATI_RV250 (from radeon_pciids.h).
 *  If nil, nothing is written.
 * @return Pcidev of a Radeon card; nil if no card found.
 */
static Pcidev*
radeonpci(int* card_type)
{
	static Pcidev *p = nil;
	struct pciids *ids;

	if (p != nil)
		return p;

	DBGPRINT("radeon: ATI Technologies Inc. "
		"Radeon driver (" VERSION_STRING ")\n");

	while ((p = pcimatch(p, ATI_PCIVID, 0)) != nil) {
		
		for (ids = radeon_pciids; ids->did; ids++) {
			if (ids->did == p->did) {

				DBGPRINT("radeon: Found %s\n", ids->name);
				DBGPRINT("radeon: did:%04ux rid:%02ux\n",
					p->did, p->rid);

				if (card_type)
					*card_type = ids->type;

				return p;
			}			
		}
	}

	DBGPRINT("radeon: not found!\n");

	return nil;
}

/* *********************************************** */
static void
vga_disable(Vga* vga)
{
	Ctlr*		c;

	for(c = vga->link; c; c = c->link)
		if (strncmp(c->name, "vga", 3) == 0) {
			c->load = nil;
		}
}

/* *********************************************** */
static void
snarf(Vga* vga, Ctlr* ctlr)
{
	Radeon 	*radeon;
	Pcidev 	*p;
	ulong	mmio, tmp;
	int		card_type;

	if (vga->private == nil) {
		vga_disable(vga);

		vga->private = alloc(sizeof(Radeon));
		radeon = vga->private;

		p = radeonpci(&card_type);
		if (p == nil)
			error("%s: not found\n", ctlr->name);

		vgactlw("type", ctlr->name);

		mmio = (ulong) segattach(0, "radeonmmio", (void*) 0, p->mem[2].size);
		if (mmio == ~0)
			error("%s: can't attach mmio segment\n", ctlr->name);

		DBGPRINT("radeon: mmio address: 0x%08ulx [size=0x%x]\n",
			mmio, p->mem[2].size);

		radeon->pci             = p;
		radeon->card_type       = card_type;
		radeon->mmio            = mmio;
	}

	radeon = vga->private;

	if (radeon->card_type >= ATI_R600)
		radeon->fbsize = INREG(radeon, R600_CONFIG_MEMSIZE);
	else
		radeon->fbsize = INREG(radeon, RADEON_CONFIG_MEMSIZE);
	vga->vmz = radeon->fbsize;
	DBGPRINT("radeon: frame buffer size=%uld [%uldMB]\n",
	         radeon->fbsize, radeon->fbsize/Mega);

	tmp = INREG(radeon, RADEON_FP_GEN_CNTL);
	if (tmp & RADEON_FP_EN_TMDS)
		radeon->display_type = DISPLAY_FP;
	else
		radeon->display_type = DISPLAY_CRT;

	DBGPRINT("radeon: display type: %s\n",
	         radeon->display_type == DISPLAY_CRT ? "CRT" : "FLAT PANEL");

	if (radeon->display_type != DISPLAY_CRT)
		error("unsupported NON CRT Display\n");

	radeon_getbiosparams(radeon);

 	radeon->bus_cntl = INREG(radeon, RADEON_BUS_CNTL);

	DBGPRINT("radeon: PPLL_CNTL=0x%08ulx\n", INPLL(radeon, RADEON_PPLL_CNTL));

	/* ATOMBIOS testing. */
	radeon_atombios_init(radeon);

	ctlr->flag |= Fsnarf;
}

/* *********************************************** */
static void
options(Vga*, Ctlr* ctlr)
{
	ctlr->flag |= Hlinear|Foptions;
}

/* *********************************************** */
static int
radeondiv(int n, int d)
{
	return (n + (d / 2)) / d;
}

/* *********************************************** */
static void
radeon_init_common_registers(Radeon* radeon)
{
    radeon->ovr_clr				= 0;
    radeon->ovr_wid_left_right	= 0;
    radeon->ovr_wid_top_bottom	= 0;
    radeon->ov0_scale_cntl		= 0;
    radeon->subpic_cntl			= 0;
    radeon->viph_control		= 0;
    radeon->i2c_cntl_1			= 0;
    radeon->rbbm_soft_reset		= 0;
    radeon->cap0_trig_cntl		= 0;
    radeon->cap1_trig_cntl		= 0;

    if (radeon->bus_cntl & RADEON_BUS_READ_BURST)
		radeon->bus_cntl |= RADEON_BUS_RD_DISCARD_EN;
}

/* *********************************************** */
static void
radeon_init_crtc_registers(Radeon* radeon, Mode* mode)
{
	int			format, dac6bit;
	int			hsync_wid, vsync_wid;
	int			hsync_start;
	int			hsync_fudge, bpp;
	int			hsync_fudge_default[] = {
		0x00, 0x12, 0x09, 0x09, 0x06, 0x05
	};

	format = 0; bpp = 0; dac6bit = 0;
	switch (mode->z) {
	case 6:		format = 2; dac6bit = 1; bpp =  8; break;
    case 8:		format = 2; dac6bit = 0; bpp =  8; break;
    case 15:	format = 3; dac6bit = 0; bpp = 16; break;
    case 16:	format = 4; dac6bit = 0; bpp = 16; break;
    case 24:	format = 5; dac6bit = 0; bpp = 24; break;
    case 32:	format = 6; dac6bit = 0; bpp = 32; break;
	default:	error("radeon: unsupported mode depth %d\n", mode->z);
	}

	hsync_fudge = hsync_fudge_default[format-1];

	DBGPRINT("mode->z = %d (format = %d, bpp = %d, dac6bit = %s)\n",
		mode->z, format, bpp, dac6bit ? "true" : "false");

	radeon->crtc_gen_cntl 	=
		RADEON_CRTC_EN | 
		RADEON_CRTC_EXT_DISP_EN |
		(format << 8) |
		(mode->interlace ? RADEON_CRTC_INTERLACE_EN : 0);

	radeon->crtc_ext_cntl	=
		RADEON_VGA_ATI_LINEAR |
		RADEON_XCRT_CNT_EN |
		RADEON_CRTC_CRT_ON;

    radeon->dac_cntl		= 
		RADEON_DAC_MASK_ALL |
		RADEON_DAC_VGA_ADR_EN |
		(dac6bit ? 0 : RADEON_DAC_8BIT_EN);

	// -----------------------------------------------------------

    radeon->crtc_h_total_disp = ((((mode->ht / 8) - 1) & 0x3ff)
			       | ((((mode->x / 8) - 1) & 0x1ff) << 16));

    hsync_wid = (mode->ehb - mode->shb) / 8;
    if (! hsync_wid)
		hsync_wid = 1;

    hsync_start = mode->shb - 8 + hsync_fudge;

	DBGPRINT("hsync_start=%d hsync_wid=%d hsync_fudge=%d\n",
		hsync_start, hsync_wid, hsync_fudge);

    radeon->crtc_h_sync_strt_wid = (
		(hsync_start & 0x1fff)
		| ((hsync_wid & 0x3f) << 16)
		| (mode->hsync ? RADEON_CRTC_H_SYNC_POL : 0));

	// -----------------------------------------------------------

    radeon->crtc_v_total_disp = (((mode->vt - 1) & 0xffff)
			       | ((mode->y - 1) << 16));

    vsync_wid = mode->vre - mode->vrs;
    if (! vsync_wid)
		vsync_wid = 1;

    radeon->crtc_v_sync_strt_wid = (
		((mode->vrs - 1) & 0xfff)
		| ((vsync_wid & 0x1f) << 16)
		| (mode->vsync ? RADEON_CRTC_V_SYNC_POL : 0));

	// -----------------------------------------------------------

	radeon->crtc_offset			= 0;
	radeon->crtc_offset_cntl	= INREG(radeon, RADEON_CRTC_OFFSET_CNTL);
    radeon->crtc_pitch			= ((mode->x * bpp) + ((bpp * 8) - 1)) / (bpp * 8);
    radeon->crtc_pitch 			|= radeon->crtc_pitch << 16;
}

/* *********************************************** */
static void
radeon_init_pll_registers(Radeon* radeon, ulong freq)
{
    struct {
		int 		divider;
		int 		bitvalue;
    } *post_div, post_divs[]   = {
		{  1, 0 },
		{  2, 1 },
		{  4, 2 },
		{  8, 3 },
		{  3, 4 },
		{ 16, 5 },
		{  6, 6 },
		{ 12, 7 },
		{  0, 0 }
    };

	DBGPRINT("radeon: initpll: freq=%uld\n", freq);

    if (freq > radeon->max_pll_freq)
		freq = radeon->max_pll_freq;
    if (freq * 12 < radeon->min_pll_freq)
		freq = radeon->min_pll_freq / 12;

    for (post_div = &post_divs[0]; post_div->divider; ++post_div) {
		radeon->pll_output_freq = post_div->divider * freq;
		if (radeon->pll_output_freq >= radeon->min_pll_freq
		    && radeon->pll_output_freq <= radeon->max_pll_freq)
			break;
    }

    radeon->dot_clock_freq = freq;
    radeon->feedback_div   = 
		radeondiv(radeon->reference_div * radeon->pll_output_freq,
			radeon->reference_freq);
    radeon->post_div       = post_div->divider;

    DBGPRINT("dc=%uld, of=%uld, fd=%uld, pd=%uld\n",
		radeon->dot_clock_freq,
		radeon->pll_output_freq,
		radeon->feedback_div,
		radeon->post_div);

    radeon->ppll_ref_div = radeon->reference_div;
    radeon->ppll_div_3 = (radeon->feedback_div | 
		(post_div->bitvalue << 16));
    radeon->htotal_cntl = 0;

    radeon->surface_cntl = 0;
}

/* *********************************************** */
static void
init(Vga* vga, Ctlr* ctlr)
{
	Radeon*		radeon;
	Mode*		mode;

	radeon	= vga->private;
	mode	= vga->mode;

	DBGPRINT("radeon: monitor type = '%s'\n", mode->type);
	DBGPRINT("radeon: size = '%s'\n", mode->size);
	DBGPRINT("radeon: chan = '%s'\n", mode->chan);
	DBGPRINT("radeon: freq=%d deffreq=%d x=%d y=%d z=%d\n",
		mode->frequency, mode->deffrequency, mode->x, mode->y, mode->z);
	DBGPRINT("radeon: ht=%d shb=%d ehb=%d shs=%d ehs=%d hsync='%c'\n",
		mode->ht, mode->shb, mode->ehb, mode->shs, mode->ehs, 
		mode->hsync ? mode->hsync : ' ');
	DBGPRINT("radeon: vt=%d vrs=%d vre=%d vsync='%c'\n",
		mode->vt, mode->vrs, mode->vre, mode->vsync ? mode->vsync : ' ');

	radeon_init_common_registers(radeon);
	radeon_init_crtc_registers(radeon, mode);
 	radeon_init_pll_registers(radeon, mode->frequency / 10000);

	ctlr->flag |= (Finit|Ulinear);
}

/* *********************************************** */
static void
radeon_blank(Radeon* radeon)
{
	OUTREGP(radeon, RADEON_CRTC_EXT_CNTL,
		RADEON_CRTC_DISPLAY_DIS |
		RADEON_CRTC_VSYNC_DIS |
		RADEON_CRTC_HSYNC_DIS,
		~(RADEON_CRTC_DISPLAY_DIS |
		  RADEON_CRTC_VSYNC_DIS |
		  RADEON_CRTC_HSYNC_DIS));
}

static void
radeon_unblank(Radeon* radeon)
{
	OUTREGP(radeon, RADEON_CRTC_EXT_CNTL,
		RADEON_CRTC_CRT_ON,
		~(RADEON_CRTC_DISPLAY_DIS |
		  RADEON_CRTC_VSYNC_DIS |
		  RADEON_CRTC_HSYNC_DIS));
}

/* *********************************************** */
static void
radeon_load_common_registers(Radeon* radeon)
{
    OUTREG(radeon, RADEON_OVR_CLR,				radeon->ovr_clr);
    OUTREG(radeon, RADEON_OVR_WID_LEFT_RIGHT,	radeon->ovr_wid_left_right);
    OUTREG(radeon, RADEON_OVR_WID_TOP_BOTTOM,	radeon->ovr_wid_top_bottom);
    OUTREG(radeon, RADEON_OV0_SCALE_CNTL,		radeon->ov0_scale_cntl);
    OUTREG(radeon, RADEON_SUBPIC_CNTL,			radeon->subpic_cntl);
    OUTREG(radeon, RADEON_VIPH_CONTROL,			radeon->viph_control);
    OUTREG(radeon, RADEON_I2C_CNTL_1,			radeon->i2c_cntl_1);
    OUTREG(radeon, RADEON_GEN_INT_CNTL,			radeon->gen_int_cntl);
    OUTREG(radeon, RADEON_CAP0_TRIG_CNTL,		radeon->cap0_trig_cntl);
    OUTREG(radeon, RADEON_CAP1_TRIG_CNTL,		radeon->cap1_trig_cntl);
    OUTREG(radeon, RADEON_BUS_CNTL,				radeon->bus_cntl);
    OUTREG(radeon, RADEON_SURFACE_CNTL,			radeon->surface_cntl);
}

/* *********************************************** */
static void
radeon_load_crtc_registers(Radeon* radeon)
{
    OUTREG(radeon, RADEON_CRTC_GEN_CNTL,		radeon->crtc_gen_cntl);

    OUTREGP(radeon, RADEON_CRTC_EXT_CNTL,		radeon->crtc_ext_cntl,
		RADEON_CRTC_VSYNC_DIS | RADEON_CRTC_HSYNC_DIS | 
		RADEON_CRTC_DISPLAY_DIS);

    OUTREGP(radeon, RADEON_DAC_CNTL,			radeon->dac_cntl,
		RADEON_DAC_RANGE_CNTL | RADEON_DAC_BLANKING);

    OUTREG(radeon, RADEON_CRTC_H_TOTAL_DISP,	radeon->crtc_h_total_disp);
    OUTREG(radeon, RADEON_CRTC_H_SYNC_STRT_WID,	radeon->crtc_h_sync_strt_wid);
    OUTREG(radeon, RADEON_CRTC_V_TOTAL_DISP,	radeon->crtc_v_total_disp);
    OUTREG(radeon, RADEON_CRTC_V_SYNC_STRT_WID,	radeon->crtc_v_sync_strt_wid);
    OUTREG(radeon, RADEON_CRTC_OFFSET,			radeon->crtc_offset);
    OUTREG(radeon, RADEON_CRTC_OFFSET_CNTL,		radeon->crtc_offset_cntl);
    OUTREG(radeon, RADEON_CRTC_PITCH,			radeon->crtc_pitch);
}

/* *********************************************** */
static void
radeon_pllwaitupd(Radeon* radeon)
{
	int i;

	for (i = 0; i < 10000; i++) {
		ulong	pllreg;

		pllreg = INPLL(radeon, RADEON_PPLL_REF_DIV);
		if ((pllreg & RADEON_PPLL_ATOMIC_UPDATE_R) == 0)
			break;
	}
}

/* *********************************************** */
static void
radeon_pllwriteupd(Radeon* radeon)
{
	while (1) {
		ulong	pllreg;

		pllreg = INPLL(radeon, RADEON_PPLL_REF_DIV);
		if ((pllreg & RADEON_PPLL_ATOMIC_UPDATE_R) == 0)
			break;
	}

	OUTPLLP(radeon, RADEON_PPLL_REF_DIV,
		RADEON_PPLL_ATOMIC_UPDATE_W, ~RADEON_PPLL_ATOMIC_UPDATE_W);
}

/* *********************************************** */
static void
radeon_load_pll_registers(Radeon* radeon)
{
    OUTPLLP(radeon, RADEON_VCLK_ECP_CNTL,
	    RADEON_VCLK_SRC_SEL_CPUCLK, ~RADEON_VCLK_SRC_SEL_MASK);

    OUTPLLP(radeon, RADEON_PPLL_CNTL,
		RADEON_PPLL_RESET
		| RADEON_PPLL_ATOMIC_UPDATE_EN
		| RADEON_PPLL_VGA_ATOMIC_UPDATE_EN,
		~(RADEON_PPLL_RESET
		  | RADEON_PPLL_ATOMIC_UPDATE_EN
		  | RADEON_PPLL_VGA_ATOMIC_UPDATE_EN));

    OUTREGP(radeon, RADEON_CLOCK_CNTL_INDEX,
	    RADEON_PLL_DIV_SEL, ~RADEON_PLL_DIV_SEL);

    if (radeon->card_type == ATI_R300) {
		DBGPRINT("r300_workaround\n");

		if (radeon->ppll_ref_div & R300_PPLL_REF_DIV_ACC_MASK) {
		    /* When restoring console mode, use saved PPLL_REF_DIV
		     * setting.
		     */
		    OUTPLLP(radeon, RADEON_PPLL_REF_DIV,
			    radeon->ppll_ref_div,
			    0);
		} else {
		    /* R300 uses ref_div_acc field as real ref divider */
		    OUTPLLP(radeon, RADEON_PPLL_REF_DIV,
			    (radeon->ppll_ref_div << R300_PPLL_REF_DIV_ACC_SHIFT), 
			    ~R300_PPLL_REF_DIV_ACC_MASK);
		}
    } else {
		OUTPLLP(radeon, RADEON_PPLL_REF_DIV,
			radeon->ppll_ref_div,
			~RADEON_PPLL_REF_DIV_MASK);
    }

    OUTPLLP(radeon, RADEON_PPLL_DIV_3,
	    radeon->ppll_div_3,
	    ~RADEON_PPLL_FB3_DIV_MASK);

    OUTPLLP(radeon, RADEON_PPLL_DIV_3,
	    radeon->ppll_div_3,
	    ~RADEON_PPLL_POST3_DIV_MASK);

    radeon_pllwriteupd(radeon);
    radeon_pllwaitupd(radeon);

    OUTPLL(radeon, RADEON_HTOTAL_CNTL, radeon->htotal_cntl);

    OUTPLLP(radeon, RADEON_PPLL_CNTL,
	    0,
	    ~(RADEON_PPLL_RESET
	      | RADEON_PPLL_SLEEP
	      | RADEON_PPLL_ATOMIC_UPDATE_EN
	      | RADEON_PPLL_VGA_ATOMIC_UPDATE_EN));

	if (debug) {
	    Bprint(&stdout, "Wrote: 0x%08ulx 0x%08ulx 0x%08ulx (0x%08ulx)\n",
		       radeon->ppll_ref_div,
		       radeon->ppll_div_3,
		       radeon->htotal_cntl,
		       INPLL(radeon, RADEON_PPLL_CNTL));
	    Bprint(&stdout, "Wrote: rd=%uld, fd=%uld, pd=%uld\n",
		       radeon->ppll_ref_div & RADEON_PPLL_REF_DIV_MASK,
		       radeon->ppll_div_3 & RADEON_PPLL_FB3_DIV_MASK,
		       (radeon->ppll_div_3 & RADEON_PPLL_POST3_DIV_MASK) >> 16);
	}

	/* Let the clock to lock */
    sleep(5);

    OUTPLLP(radeon, RADEON_VCLK_ECP_CNTL,
	    RADEON_VCLK_SRC_SEL_PPLLCLK, ~RADEON_VCLK_SRC_SEL_MASK);
}

/* *********************************************** */
static void
load(Vga* vga, Ctlr* ctlr)
{
	Radeon*	radeon;

	radeon = (Radeon*) vga->private;

	radeon_blank(radeon);
	radeon_load_common_registers(radeon);
	radeon_load_crtc_registers(radeon);
	radeon_load_pll_registers(radeon);
	radeon_unblank(radeon);

	// init palette [gamma]
	if (vga->mode->z > 8) {
		int	i;

		OUTREG(radeon, RADEON_PALETTE_INDEX, 0);
		for (i = 0; i < 256; i++)
			OUTREG(radeon, RADEON_PALETTE_DATA, (i<<16)|(i<<8)|i);
	}

	ctlr->flag |= Fload;
}

/* *********************************************** */
static void
dump(Vga* vga, Ctlr* ctlr)
{
	Radeon *	radeon;

	USED(ctlr);

	radeon = (Radeon*) vga->private;

	USED(radeon);
}

/* *********************************************** */
Ctlr radeon = {
	"radeon",			/* name */
	snarf,				/* snarf */
	options,			/* options */
	init,				/* init */
	load,				/* load */
	dump,				/* dump */
};

/* *********************************************** */
Ctlr radeonhwgc = {
	"radeonhwgc",		/* name */
	0,					/* snarf */
	0,					/* options */
	0,					/* init */
	0,					/* load */
	0,					/* dump */
};

