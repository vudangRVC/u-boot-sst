// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Marek Vasut <marek.vasut+renesas@mailbox.org>
 */

#include <asm/io.h>
#include <compiler.h>
#include <dbsc5.h>
#include <spi.h>
#include <spi_flash.h>
#include <spl.h>

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

void spl_board_id_setup(void)
{
	board_id = BOARD_ID_RCAR_V4H_SPARROWHAWK;
	soc_id = RZ_SOC_RCAR_V4H;
}

static void read_platform_settings(struct spi_flash *flash)
{
	platform_desc_t pdesc;
	int ret;

	ret = spi_flash_read(flash, CFG_SPL_PLATFORM_SETTINGS_OFFSET,
			     sizeof(pdesc), &pdesc);
	if (ret) {
		printf("SPL: failed to read platform-settings: %d\n", ret);
		return;
	}

	printf("SPL: platform: model_id=%u, model_string=%s, mfg=%s\n",
	       pdesc.model_id, pdesc.model_string, pdesc.mfg_name);
}

static const struct renesas_dbsc5_board_config
renesas_v4h_sparrowhawk_8g_6400_dbsc5_board_config = {
	/* RENESAS V4H Sparrow Hawk (64Gbit 1rank) */
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
	/* RENESAS V4H Sparrow Hawk (64Gbit 2rank) */
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
	if ((renesas_get_cpu_rev_integer() >= 3) && (modemr0 & BIT(19)))
		return &renesas_v4h_sparrowhawk_16g_5500_dbsc5_board_config;
	else
		return &renesas_v4h_sparrowhawk_8g_6400_dbsc5_board_config;
}

static bool renesas_v4h_sparrowhawk_is_evta1 = false;

unsigned int spl_spi_get_uboot_offs(struct spi_flash *flash)
{
	const u8 sf_ids_evta1[6] = { 0x01, 0x02, 0x20, 0x4d, 0x00, 0x81 };

	printf("SPL: board_id=0x%llx, soc_id=0x%llx",
	       board_id, soc_id);

	if (board_id == BOARD_ID_RCAR_V4H_SPARROWHAWK &&
	    soc_id == RZ_SOC_RCAR_V4H)
		printf(" (Sparrowhawk R-Car V4H)");
	printf("\n");

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

	if (!renesas_v4h_sparrowhawk_is_evta1) {
		if (board_id == BOARD_ID_RCAR_V4H_SPARROWHAWK)
			printf("EVTB1 board detected (board_id=0x%llx)\n",
			       board_id);
		return;
	}

	printf("EVTA1 board detected (board_id=0x%llx, soc_id=0x%llx)\n",
	       board_id, soc_id);

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
#endif
