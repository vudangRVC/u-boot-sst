// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Marek Vasut <marek.vasut+renesas@mailbox.org>
 */

#include <asm/io.h>
#include <asm/global_data.h>
#include <linux/bitops.h>
#include <linux/kconfig.h>
#include <mach/renesas.h>

#define RST_MODEMR0			0xe6160000

DECLARE_GLOBAL_DATA_PTR;

void renesas_dram_init_banksize(void)
{
	const u32 modemr0 = readl(RST_MODEMR0);
	int bank;

	/* 8 GiB device, do nothing. */
	if (!((renesas_get_cpu_rev_integer() >= 3) && (modemr0 & BIT(19))))
		return;

	/* 16 GiB device, adjust memory map. */
	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		if (gd->bd->bi_dram[bank].start == 0x480000000ULL)
			gd->bd->bi_dram[bank].size = 0x180000000ULL;
		else if (gd->bd->bi_dram[bank].start == 0x600000000ULL)
			gd->bd->bi_dram[bank].size = 0x200000000ULL;
	}
}

#define SRCR6			0xe6152c18
#define SRCR11			0xe6152c2c
#define SRSTCLR6		0xe6152c98
#define SRSTCLR11		0xe6152cac
#define SRCR_PCIEC0_PWR_RESET	BIT(24)
#define SRCR_PCIEC1_PWR_RESET	BIT(25)
#define SRCR_PCIEC0_APP_RESET	BIT(21)
#define SRCR_PCIEC1_APP_RESET	BIT(22)

void board_cleanup_before_linux(void)
{
	if (!IS_ENABLED(CONFIG_PCI_RCAR_GEN4))
		return;

	/* Set cold and application reset for both PCIe cores */
	writel(SRCR_PCIEC0_PWR_RESET | SRCR_PCIEC1_PWR_RESET, SRCR6);
	readl(SRCR6);
	writel(SRCR_PCIEC0_APP_RESET | SRCR_PCIEC1_APP_RESET, SRCR11);
	readl(SRCR11);

	/* Clear cold and application reset for both PCIe cores */
	writel(SRCR_PCIEC0_PWR_RESET | SRCR_PCIEC1_PWR_RESET, SRSTCLR6);
	readl(SRSTCLR6);
	writel(SRCR_PCIEC0_APP_RESET | SRCR_PCIEC1_APP_RESET, SRSTCLR11);
	readl(SRSTCLR11);
}

int board_late_init(void)
{
	return 0;
}
