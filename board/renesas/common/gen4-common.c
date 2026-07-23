// SPDX-License-Identifier: GPL-2.0+
/*
 * board/renesas/common/gen4-common.c
 *
 * Copyright (C) 2021-2024 Renesas Electronics Corp.
 */

#include <asm/arch/renesas.h>
#include <asm/arch/sys_proto.h>
#include <asm/armv8/mmu.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <command.h>
#include <image.h>
#include <linux/errno.h>

DECLARE_GLOBAL_DATA_PTR;

void __weak reset_cpu(void)
{
	writel(RST_SPRES, RST_SRESCR0);
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

/* R-Car Gen4 TFA BL31 handoff structure and handling. */
struct param_header {
	u8			type;
	u8			version;
	u16			size;
	u32			attr;
};

struct tfa_image_info {
	struct param_header	h;
	uintptr_t		image_base;
	u32			image_size;
	u32			image_max_size;
};

struct aapcs64_params {
	u64			arg0;
	u64			arg1;
	u64			arg2;
	u64			arg3;
	u64			arg4;
	u64			arg5;
	u64			arg6;
	u64			arg7;
};

struct entry_point_info {
	struct param_header	h;
	uintptr_t		pc;
	u32			spsr;
	struct aapcs64_params	args;
};

struct bl2_to_bl31_params_mem {
	struct tfa_image_info	bl32_image_info;
	struct tfa_image_info	bl33_image_info;
	struct entry_point_info	bl33_ep_info;
	struct entry_point_info	bl32_ep_info;
};

/* Default jump address, return to U-Boot */
#define BL33_BASE	0x44100000
/* Custom parameters address passed to TFA by ICUMXA loader */
#define PARAMS_BASE	0x46422200
#define BL31_BASE	0x46400000
#define BL31_MAX_SIZE	(PARAMS_BASE - BL31_BASE)
#define TEE_BASE	0x44100000
#define TEE_MAX_SIZE	(BL31_BASE - TEE_BASE)

/* Usually such a structure is produced by ICUMXA and passed in at 0x46422200 */
static const struct bl2_to_bl31_params_mem blinfo_template = {
	.bl33_ep_info.h.type = 1,	/* PARAM_EP */
	.bl33_ep_info.h.version = 2,	/* Version 2 */
	.bl33_ep_info.h.size = sizeof(struct entry_point_info),
	.bl33_ep_info.h.attr = 0x81,	/* Executable | Non-Secure */
	.bl33_ep_info.spsr = 0x2c9,	/* Mode=EL2, SP=ELX, Exceptions=OFF */
	.bl33_ep_info.pc = BL33_BASE,

	.bl33_image_info.h.type = 1,	/* PARAM_EP */
	.bl33_image_info.h.version = 2,	/* Version 2 */
	.bl33_image_info.h.size = sizeof(struct image_info),
	.bl33_image_info.h.attr = 0,
	.bl33_image_info.image_base = BL33_BASE,
};

static bool tfa_bl31_image_loaded;
static ulong tfa_bl31_image_addr;
static u32 tfa_bl31_image_size;
static bool tee_image_loaded;
static ulong tee_image_addr;
static u32 tee_image_size;

static void fill_bl32_info(struct bl2_to_bl31_params_mem *blinfo)
{
	if (!tee_image_loaded)
		return;

	blinfo->bl32_ep_info.h.type = 1;
	blinfo->bl32_ep_info.h.version = 2;
	blinfo->bl32_ep_info.h.size = sizeof(struct entry_point_info);
	blinfo->bl32_ep_info.h.attr = 0;
	blinfo->bl32_ep_info.pc = tee_image_addr;
	blinfo->bl32_ep_info.spsr = 0x3c5;

	blinfo->bl32_image_info.h.type = 1;
	blinfo->bl32_image_info.h.version = 2;
	blinfo->bl32_image_info.h.size = sizeof(struct tfa_image_info);
	blinfo->bl32_image_info.h.attr = 0;
	blinfo->bl32_image_info.image_base = tee_image_addr;
	blinfo->bl32_image_info.image_size = tee_image_size;
	blinfo->bl32_image_info.image_max_size = tee_image_size;
}

static void tfa_bl31_image_process(ulong image, size_t size)
{
	/* Custom parameters address passed to TFA by ICUMXA loader */
	struct bl2_to_bl31_params_mem *blinfo = (struct bl2_to_bl31_params_mem *)PARAMS_BASE;

	/* Not in EL3, do nothing. */
	if (current_el() != 3)
		return;

	/* Clear a page and copy template */
	memset((void *)PARAMS_BASE, 0, PAGE_SIZE);
	memcpy(blinfo, &blinfo_template, sizeof(*blinfo));
	tfa_bl31_image_addr = image;
	tfa_bl31_image_size = size;
	tfa_bl31_image_loaded = true;
}

U_BOOT_FIT_LOADABLE_HANDLER(IH_TYPE_TFA_BL31, tfa_bl31_image_process);

static int do_tfa_prepare(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct bl2_to_bl31_params_mem *blinfo =
		(struct bl2_to_bl31_params_mem *)PARAMS_BASE;
	ulong bl31_addr, bl31_size, tee_addr, tee_size;

	if (current_el() != 3) {
		printf("tfa_prepare: U-Boot is not running at EL3\n");
		return CMD_RET_FAILURE;
	}

	if (argc != 5)
		return CMD_RET_USAGE;

	bl31_addr = hextoul(argv[1], NULL);
	bl31_size = hextoul(argv[2], NULL);
	tee_addr = hextoul(argv[3], NULL);
	tee_size = hextoul(argv[4], NULL);

	if (bl31_addr != BL31_BASE || !bl31_size || bl31_size > BL31_MAX_SIZE ||
	    tee_addr != TEE_BASE || !tee_size || tee_size > TEE_MAX_SIZE) {
		printf("tfa_prepare: invalid BL31/TEE address or size\n");
		return CMD_RET_FAILURE;
	}

	memset((void *)PARAMS_BASE, 0, PAGE_SIZE);
	memcpy(blinfo, &blinfo_template, sizeof(*blinfo));
	tfa_bl31_image_addr = bl31_addr;
	tfa_bl31_image_size = bl31_size;
	tfa_bl31_image_loaded = true;
	tee_image_addr = tee_addr;
	tee_image_size = tee_size;
	tee_image_loaded = true;
	fill_bl32_info(blinfo);

	flush_dcache_range(bl31_addr, bl31_addr + bl31_size);
	flush_dcache_range(tee_addr, tee_addr + tee_size);
	flush_dcache_range(PARAMS_BASE, PARAMS_BASE + PAGE_SIZE);
	printf("Prepared BL31 @ %08lx (%08lx), TEE @ %08lx (%08lx)\n",
	       bl31_addr, bl31_size, tee_addr, tee_size);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	tfa_prepare, 5, 0, do_tfa_prepare,
	"prepare R-Car Gen4 BL31/BL32 handoff for a subsequent booti",
	"<bl31_addr> <bl31_size> <tee_addr> <tee_size>"
);

void armv8_switch_to_el2_prep(u64 args, u64 mach_nr, u64 fdt_addr,
			      u64 arg4, u64 entry_point, u64 es_flag)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);
	image_entry_noargs_t image_entry =
		(image_entry_noargs_t)(void *)tfa_bl31_image_addr;
	struct bl2_to_bl31_params_mem *blinfo =
		(struct bl2_to_bl31_params_mem *)PARAMS_BASE;

	/* Not in EL3, do nothing. */
	if (current_el() != 3)
		return;

	/*
	 * Destination address in arch/arm/cpu/armv8/transition.S
	 * right past the first bl in armv8_switch_to_el2() to let
	 * the rest of U-Boot pre-Linux code run. The code does run
	 * without stack pointer!
	 */
	const u64 ep = ((u64)(uintptr_t)&armv8_switch_to_el2) + 4;

	/* If TFA BL31 was not part of the fitImage, do regular boot. */
	if (!tfa_bl31_image_loaded)
		return;

	/*
	 * Set up kernel entry point and parameters:
	 * x0 is FDT address, x1..x3 must be 0
	 */
	blinfo->bl33_ep_info.pc = ep;
	blinfo->bl33_ep_info.args.arg0 = args;
	blinfo->bl33_ep_info.args.arg1 = mach_nr;
	blinfo->bl33_ep_info.args.arg2 = fdt_addr;
	blinfo->bl33_ep_info.args.arg3 = arg4;
	blinfo->bl33_ep_info.args.arg4 = entry_point;
	blinfo->bl33_ep_info.args.arg5 = es_flag;
	blinfo->bl33_image_info.image_base = ep;
	fill_bl32_info(blinfo);
	flush_dcache_range(PARAMS_BASE, PARAMS_BASE + PAGE_SIZE);
	flush_dcache_range(tfa_bl31_image_addr,
			   tfa_bl31_image_addr + tfa_bl31_image_size);
	if (tee_image_loaded)
		flush_dcache_range(tee_image_addr, tee_image_addr + tee_image_size);

	/* Jump to TFA BL31 */
	image_entry();
}
