// SPDX-License-Identifier: GPL-2.0+
/*
 * Unified SPL for RZ/CMN and Sparrowhawk (R-Car V4H).
 * All functions dispatch on soc_id at runtime.
 *
 * Copyright (C) 2025 Marek Vasut <marek.vasut+renesas@mailbox.org>
 */

#include <asm/arch/renesas.h>
#include <configs/rz-cmn.h>
#include <asm/arch/rcar-gen4-base.h>
#include <asm/io.h>
#include <compiler.h>
#include <cpu_func.h>
#include <dbsc5.h>
#include <dm/uclass.h>
#include <dm/util.h>
#include <fdt_support.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <linux/bitops.h>
#include <log.h>
#include <mapmem.h>
#include <spi.h>
#include <spi_flash.h>
#include <spl.h>
#include <string.h>

#include "../../../drivers/mtd/spi/sf_internal.h"

extern u64 board_id;
extern u64 soc_id;

#define BOARD_ID_RCAR_V4H_SPARROWHAWK	0x40
#define RZ_SOC_RCAR_V4H			0x04

typedef struct __attribute__((packed)) {
	u32 model_id;
	u32 revision_minor : 16;
	u32 revision_major : 16;
	char model_string[256];
	char mfg_name[256];
} platform_desc_t;

#if defined(CONFIG_XPL_BUILD)

#define CNTCR_EN	BIT(0)

void board_debug_uart_init(void)
{
}

static void init_generic_timer(void)
{
	const u32 freq = CONFIG_SYS_CLK_FREQ;

	/* Update memory mapped and register based freqency */
	if (IS_ENABLED(CONFIG_ARM64))
		asm volatile("msr cntfrq_el0, %0" :: "r" (freq));
	else
		asm volatile("mcr p15, 0, %0, c14, c0, 0" :: "r" (freq));

	writel(freq, CNTFID0_RCAR_GEN4);

	/* Enable counter */
	setbits_le32(CNTCR_BASE_RCAR_GEN4, CNTCR_EN);
}

/* ---------- Sparrowhawk: platform settings ---------- */

void spl_board_id_setup(void)
{
	if (soc_id == RZ_SOC_RCAR_V4H)
		board_id = 0;
}

#ifndef CFG_SPL_PLATFORM_SETTINGS_OFFSET
#define CFG_SPL_PLATFORM_SETTINGS_OFFSET	0
#endif

static const u8 sf_ids_evta1[6] = { 0x01, 0x02, 0x20, 0x4d, 0x00, 0x81 };

static void read_platform_settings(struct spi_flash *flash)
{
	platform_desc_t pdesc;
	int ret;

	if (soc_id != RZ_SOC_RCAR_V4H)
		return;

	ret = spi_flash_read(flash, CFG_SPL_PLATFORM_SETTINGS_OFFSET,
			     sizeof(pdesc), &pdesc);
	if (ret) {
		printf("SPL: failed to read platform-settings: %d\n", ret);
		return;
	}

	printf("SPL: platform: model_id=0x%x, model_string=%s\n",
	       pdesc.model_id, pdesc.model_string);
	board_id = pdesc.model_id;
}

/* ---------- Sparrowhawk: DRAM configs ---------- */

static const struct renesas_dbsc5_board_config
renesas_v4h_sparrowhawk_8g_6400_dbsc5_board_config = {
	.bdcfg_phyvalid	= 0xF,
	.bdcfg_vref_r	= 0x0,
	.bdcfg_vref_w	= 0x0,
	.bdcfg_vref_ca	= 0x0,
	.bdcfg_rfm_chk	= true,
	.ch = {
		[0] = {
			.bdcfg_ddr_density =	{ 0x06, 0xFF },
			.bdcfg_ca_swap =	0x04506132,
			.bdcfg_dqs_swap =	0x01,
			.bdcfg_dq_swap =	{ 0x26157084, 0x12306854 },
			.bdcfg_dm_swap =	{ 0x03, 0x07 },
			.bdcfg_cs_swap =	0x10
		},
		[1] = {
			.bdcfg_ddr_density =	{ 0x06, 0xFF },
			.bdcfg_ca_swap =	0x02431065,
			.bdcfg_dqs_swap =	0x10,
			.bdcfg_dq_swap =	{ 0x56782314, 0x70423856 },
			.bdcfg_dm_swap =	{ 0x00, 0x01 },
			.bdcfg_cs_swap =	0x10
		},
		[2] = {
			.bdcfg_ddr_density =	{ 0x06, 0xFF },
			.bdcfg_ca_swap =	0x02150643,
			.bdcfg_dqs_swap =	0x10,
			.bdcfg_dq_swap =	{ 0x58264031, 0x40587236 },
			.bdcfg_dm_swap =	{ 0x07, 0x01 },
			.bdcfg_cs_swap =	0x10
		},
		[3] = {
			.bdcfg_ddr_density =	{ 0x06, 0xFF },
			.bdcfg_ca_swap =	0x01546230,
			.bdcfg_dqs_swap =	0x01,
			.bdcfg_dq_swap =	{ 0x45761328, 0x68023745 },
			.bdcfg_dm_swap =	{ 0x00, 0x01 },
			.bdcfg_cs_swap =	0x10
		}
	}
};

static const struct renesas_dbsc5_board_config
renesas_v4h_sparrowhawk_16g_5500_dbsc5_board_config = {
	.bdcfg_phyvalid	= 0xF,
	.bdcfg_vref_r	= 0x0,
	.bdcfg_vref_w	= 0x0,
	.bdcfg_vref_ca	= 0x0,
	.bdcfg_rfm_chk	= true,
	.ch = {
		[0] = {
			.bdcfg_ddr_density =	{ 0x06, 0x06 },
			.bdcfg_ca_swap =	0x04506132,
			.bdcfg_dqs_swap =	0x01,
			.bdcfg_dq_swap =	{ 0x26157084, 0x12306854 },
			.bdcfg_dm_swap =	{ 0x03, 0x07 },
			.bdcfg_cs_swap =	0x10
		},
		[1] = {
			.bdcfg_ddr_density =	{ 0x06, 0x06 },
			.bdcfg_ca_swap =	0x02431065,
			.bdcfg_dqs_swap =	0x10,
			.bdcfg_dq_swap =	{ 0x56782314, 0x70423856 },
			.bdcfg_dm_swap =	{ 0x00, 0x01 },
			.bdcfg_cs_swap =	0x10
		},
		[2] = {
			.bdcfg_ddr_density =	{ 0x06, 0x06 },
			.bdcfg_ca_swap =	0x02150643,
			.bdcfg_dqs_swap =	0x10,
			.bdcfg_dq_swap =	{ 0x58264031, 0x40587236 },
			.bdcfg_dm_swap =	{ 0x07, 0x01 },
			.bdcfg_cs_swap =	0x10
		},
		[3] = {
			.bdcfg_ddr_density =	{ 0x06, 0x06 },
			.bdcfg_ca_swap =	0x01546230,
			.bdcfg_dqs_swap =	0x01,
			.bdcfg_dq_swap =	{ 0x45761328, 0x68023745 },
			.bdcfg_dm_swap =	{ 0x00, 0x01 },
			.bdcfg_cs_swap =	0x10
		}
	}
};

const struct renesas_dbsc5_board_config *
dbsc5_get_board_data(struct udevice *dev, const u32 modemr0)
{
	if (soc_id == RZ_SOC_RCAR_V4H) {
		if ((renesas_get_cpu_rev_integer() >= 3) && (modemr0 & BIT(19)))
			return &renesas_v4h_sparrowhawk_16g_5500_dbsc5_board_config;
		else
			return &renesas_v4h_sparrowhawk_8g_6400_dbsc5_board_config;
	}
	return NULL;
}

/* ---------- Sparrowhawk: SPI offset & FDT fixups ---------- */

static bool renesas_v4h_sparrowhawk_is_evta1 = false;

unsigned int spl_spi_get_uboot_offs(struct spi_flash *flash)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return 0;

	read_platform_settings(flash);

	renesas_v4h_sparrowhawk_is_evta1 = !memcmp(sf_ids_evta1, flash->info->id,
						   sizeof(sf_ids_evta1));

	return CONFIG_SYS_SPI_U_BOOT_OFFS;
}

void spl_perform_board_fixups(struct spl_image_info *spl_image)
{
	void *blob = spl_image_fdt_addr(spl_image);
	int err, offs;
	u32 size;

	if (soc_id != RZ_SOC_RCAR_V4H)
		return;

	// if (!renesas_v4h_sparrowhawk_is_evta1) {
	// 	if (board_id == BOARD_ID_RCAR_V4H_SPARROWHAWK)
	// 		printf("EVTB1 board detected (board_id=0x%lx)\n",
	// 		       (unsigned long)board_id);
	// 	return;
	// }

	if (!blob)
		return;

	err = fdt_check_header(blob);
	if (err < 0) {
		printf("Invalid FDT header: %s\n", fdt_strerror(err));
		return;
	}

	size = fdt_totalsize(blob);
	err = fdt_open_into(blob, blob, size + 64);
	if (err < 0) {
		printf("Failed to expand DT\n");
		return;
	}

	offs = fdt_path_offset(blob, "/regulator-vcc-sdhi");
	if (offs < 0) {
		printf("Failed to locate MicroSD regulator node: %d\n", offs);
		return;
	}

	err = fdt_setprop_string(blob, offs, "compatible", "regulator-fixed");
	if (err < 0) {
		printf("Failed to set fixed MicroSD regulator: %d\n", err);
		return;
	}

	err = fdt_setprop_u32(blob, offs, "regulator-min-microvolt", 3300000);
	if (err < 0) {
		printf("Failed to set MicroSD regulator minimum voltage: %d\n", err);
		return;
	}

	err = fdt_nop_property(blob, offs, "gpios");
	if (err < 0) {
		printf("Failed to remove MicroSD regulator gpios: %d\n", err);
		return;
	}

	err = fdt_nop_property(blob, offs, "gpios-states");
	if (err < 0) {
		printf("Failed to remove MicroSD regulator gpio states: %d\n", err);
		return;
	}

	err = fdt_nop_property(blob, offs, "states");
	if (err < 0) {
		printf("Failed to remove MicroSD regulator states: %d\n", err);
		return;
	}

	offs = fdt_path_offset(blob, "/soc/mmc@ee140000");
	if (offs < 0) {
		printf("Failed to locate MicroSD device node: %d\n", offs);
		return;
	}

	err = fdt_nop_property(blob, offs, "sd-uhs-sdr50");
	if (err < 0) {
		printf("Failed to disable SDR50 mode in MicroSD node: %d\n", err);
		return;
	}

	err = fdt_nop_property(blob, offs, "sd-uhs-sdr104");
	if (err < 0) {
		printf("Failed to disable SDR104 mode in MicroSD node: %d\n", err);
		return;
	}

	err = fdt_setprop_string(blob, offs, "pinctrl-names", "default");
	if (err < 0) {
		printf("Failed to set fixed MicroSD pin names: %d\n", err);
		return;
	}

	err = fdt_nop_property(blob, offs, "pinctrl-1");
	if (err < 0) {
		printf("Failed to disable UHS pins in MicroSD node: %d\n", err);
		return;
	}

	offs = fdt_path_offset(blob, "/soc/pinctrl@e6050000/avb0/pins-vddq18-25-avb");
	if (offs < 0) {
		printf("Failed to locate AVB pinctrl node: %d\n", offs);
		return;
	}

	err = fdt_setprop_u32(blob, offs, "power-source", 2500);
	if (err < 0) {
		printf("Failed to set AVB IO voltage: %d\n", err);
		return;
	}
}

/* ---------- RZ/CMN stubs ----------
 * Marked __weak so they only apply when no strong override exists
 */

int board_fit_config_name_match(const char *name)
{
	if (soc_id == RZ_SOC_RCAR_V4H)
		return !strstr(name, "sparrow-hawk") ? -1 : 0;

	if (soc_id == RZ_SOC_RZV2H)
		return !strstr(name, "rzv2h") ? -1 : 0;

	if (soc_id == RZ_SOC_RZG2L || soc_id == RZ_SOC_RZV2L)
		return !strstr(name, "rzg2l") && !strstr(name, "rzv2l") &&
		       !strstr(name, "rs-g2l") ? -1 : 0;

	return -1;
}

bool spl_board_needs_dbsc5_init(void)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return false;
	/* DBSC5 init is needed to make low DDR accessible for SPL stack
	 * relocation (CONFIG_SPL_STACK_R_ADDR=0x44000000 is below
	 * CFG_SYS_SDRAM_BASE). Without this, memcpy in
	 * spl_relocate_stack_gd() data aborts.
	 * ATF only initializes the upper DDR; DBSC5 completes the rest.
	 */
	return true;
}

void board_init_f(ulong dummy)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return;
	struct udevice *dev;
	int ret;

	if (CONFIG_IS_ENABLED(OF_CONTROL)) {
		ret = spl_early_init();
		if (ret) {
			debug("spl_early_init() failed: %d\n", ret);
			hang();
		}
	}

	preloader_console_init();

	spl_board_id_setup();

	if (spl_board_needs_dbsc5_init()) {
		ret = uclass_get_device_by_name(UCLASS_NOP, "ram@e6780000", &dev);
		if (ret)
			printf("DBSC5 init failed: %d\n", ret);

		ret = uclass_get_device_by_name(UCLASS_RAM, "ram@ffec0000", &dev);
		if (ret)
			printf("RTVRAM init failed: %d\n", ret);
	}
};

void __weak spl_board_init(void)
{
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_SPI;
}

struct legacy_img_hdr *spl_get_load_buffer(ssize_t offset, size_t size)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return NULL;
	return map_sysmem(CONFIG_SYS_LOAD_ADDR + offset, 0);
}

#define APMU_BASE 0xe6170000U
#define CL0GRP3_BIT			BIT(3)
#define CL1GRP3_BIT			BIT(7)
#define RTGRP3_BIT			BIT(19)
#define APMU_ACC_ENB_FOR_ARM_CPU	(CL0GRP3_BIT | CL1GRP3_BIT | RTGRP3_BIT)

void s_init(void)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return;
	/* Unlock CPG access */
	writel(0x5A5AFFFF, CPGWPR_RCAR_GEN4);
	writel(0xA5A50000, CPGWPCR_RCAR_GEN4);
	init_generic_timer();

	/* Define for Work Around of APMU */
	writel(0x00ff00ff, APMU_BASE + 0x10);
	writel(0x00ff00ff, APMU_BASE + 0x14);
	writel(0x00ff00ff, APMU_BASE + 0x18);
	writel(0x00ff00ff, APMU_BASE + 0x1c);
	clrbits_le32(APMU_BASE + 0x68, BIT(29));
}

void reset_cpu(void)
{
}

void __weak __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	if (soc_id != RZ_SOC_RCAR_V4H)
		return;
	typedef void __noreturn (*image_entry_noargs_t)(void);
	image_entry_noargs_t image_entry =
		(image_entry_noargs_t)spl_image->entry_point;
	image_entry();
}

#endif /* CONFIG_XPL_BUILD */
