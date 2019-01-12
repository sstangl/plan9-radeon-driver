
// ATI Technologies Inc
#define ATI_PCIVID		0x1002

#define IS_R300_VARIANT(x) (((x) == ATI_R300) || \
	((x) == ATI_RV350) || \
	((x) == ATI_R350)  || \
	((x) == ATI_RV380) || \
	((x) == ATI_R420)  || \
	((x) == ATI_RV410) || \
	((x) == ATI_RS400) || \
	((x) == ATI_RS480))

#define IS_AVIVO_VARIANT(x) ((x) >= ATI_RV515)

struct pciids {
	ushort			did;
	int				type;
	char*			name;
};

/* Must be kept sorted by card, with the most recent
 * varities at the bottom. */
enum {
	ATI_M9,
	ATI_M7,
	ATI_M6,
	ATI_R100,
	ATI_RV100,

	ATI_R200,
	ATI_RV200,
	ATI_R250,
	ATI_RV250,

	ATI_R300,
	ATI_R350,
	ATI_RV350,
	ATI_RV380,
	ATI_RS300,

	ATI_RS400,
	ATI_RV410,
	ATI_R420,
	ATI_RS480,

	ATI_RV515,
	ATI_RV530,

	ATI_R600,
	ATI_RV670,
};

struct pciids radeon_pciids[] =
{
	// ATI_M6, LY
	// ATI_M6, LZ

	0x4c57, ATI_M7,		"Radeon Mobility M7 LW [Radeon Mobility 7500]",
	0x4c58, ATI_M7,		"Radeon RV200 LX [Mobility FireGL 7800 M7]",

	0x4c64, ATI_M9,		"Radeon R250 Ld [Radeon Mobility 9000 M9]",
	0x4c65, ATI_M9,		"Radeon R250 Le [Radeon Mobility 9000 M9]",
	0x4c66, ATI_M9,		"Radeon R250 Lf [Radeon Mobility 9000 M9]",
	0x4c67, ATI_M9,		"Radeon R250 Lg [Radeon Mobility 9000 M9]",

	0x5159, ATI_RV100,	"Radeon RV100 QY [Radeon 7000/VE]",
	0x515a, ATI_RV100,	"Radeon RV100 QZ [Radeon 7000/VE]",

	0x5144, ATI_R100,	"Radeon R100 QD [Radeon 7200]",
	0x5145, ATI_R100,	"Radeon R100 QE",
	0x5146, ATI_R100,	"Radeon R100 QF",
	0x5147, ATI_R100,	"Radeon R100 QG",

	0x5148, ATI_R200,	"Radeon R200 QH [Radeon 8500]",
	0x5149, ATI_R200,	"Radeon R200 QI",
	0x514a, ATI_R200,	"Radeon R200 QJ",
	0x514b, ATI_R200,	"Radeon R200 QK",
	0x514c, ATI_R200,	"Radeon R200 QL [Radeon 8500 LE]",
	0x514d, ATI_R200,	"Radeon R200 QM [Radeon 9100]",
	0x514e, ATI_R200,	"Radeon R200 QN [Radeon 8500LE]",
	0x514f, ATI_R200,	"Radeon R200 QO [Radeon 8500LE]",
	0x5168, ATI_R200,	"Radeon R200 Qh",
	0x5169, ATI_R200,	"Radeon R200 Qi",
	0x516a, ATI_R200,	"Radeon R200 Qj",
	0x516b, ATI_R200,	"Radeon R200 Qk",
	0x516c, ATI_R200,	"Radeon R200 Ql",

	0x5157, ATI_RV200,	"Radeon RV200 QW [Radeon 7500]",
	0x5158, ATI_RV200,	"Radeon RV200 QX [Radeon 7500]",

	0x4964, ATI_RV250, 	"Radeon R250 Id [Radeon 9000]",
	0x4965, ATI_RV250, 	"Radeon R250 Ie [Radeon 9000]",
	0x4966, ATI_RV250, 	"Radeon R250 If [Radeon 9000]",
	0x4967, ATI_RV250,	"Radeon R250 Ig [Radeon 9000]",

	0x4144, ATI_R300, 	"Radeon R300 AD [Radeon 9500 Pro]",
	0x4145, ATI_R300, 	"Radeon R300 AE [Radeon 9500 Pro]",
	0x4146, ATI_R300, 	"Radeon R300 AF [Radeon 9500 Pro]",
	0x4147, ATI_R300, 	"Radeon R300 AG [FireGL Z1/X1]",
	0x4e44, ATI_R300,	"Radeon R300 ND [Radeon 9700]",
	0x4e45, ATI_R300,	"Radeon R300 NE [Radeon 9700]",
	0x4e46, ATI_R300,	"Radeon R300 NF [Radeon 9700]",
	0x4e47, ATI_R300,	"Radeon R300 NG [FireGL X1]",

	0x4e64, ATI_R300,	"Radeon R300 [Radeon 9700 Pro] (Secondary)",
	0x4e65, ATI_R300,	"Radeon R300 [Radeon 9700] (Secondary)",
	0x4e66, ATI_R300,	"Radeon R300 [Radeon 9700] (Secondary)",
	0x4e67, ATI_R300,	"Radeon R300 [FireGL X1] (Secondary)",

	0x71c0, ATI_RV530,	"Radeon RV530 [Radeon X1600]",
	0x71e0, ATI_RV530,	"Radeon RV530 [Radeon X1600] (Secondary)",

	0x9505, ATI_RV670, "Radeon RV670 [Radeon HD 3850]",

	0
};
