// SPDX-License-Identifier: GPL-2.0+
/*
 * Unified SPL for RZ/CMN and Sparrowhawk (R-Car V4H).
 * All functions dispatch on soc_id at runtime.
 *
 * Copyright (C) 2025 Marek Vasut <marek.vasut+renesas@mailbox.org>
 */

#include <asm/io.h>
#include <compiler.h>
#include <dbsc5.h>
#include <fdt_support.h>
#include <init.h>
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

	printf("SPL: platform: model_id=0x%x, model_string=%s, mfg=%s\n",
	       pdesc.model_id, pdesc.model_string, pdesc.mfg_name);
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

	if (!renesas_v4h_sparrowhawk_is_evta1) {
		if (board_id == BOARD_ID_RCAR_V4H_SPARROWHAWK)
			printf("EVTB1 board detected (board_id=0x%lx)\n",
			       (unsigned long)board_id);
		return;
	}

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
 * (e.g. gen4-spl.c for CONFIG_RCAR_GEN4 builds).
 */
void __weak board_init_f(ulong dummy)
{
}

void __weak spl_board_init(void)
{
}

u32 __weak spl_boot_device(void)
{
	return BOOT_DEVICE_MMC1;
}

void __weak s_init(void)
{
}

void __weak reset_cpu(void)
{
}

void __weak __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);
	image_entry_noargs_t image_entry =
		(image_entry_noargs_t)spl_image->entry_point;
	image_entry();
}

#endif /* CONFIG_XPL_BUILD */
