#include <common.h>
#include <cpu_func.h>
#include <image.h>
#include <init.h>
#include <malloc.h>
#include <netdev.h>
#include <dm.h>
#include <dm/platform_data/serial_sh.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rmobile.h>
#include <asm/arch/rcar-mstp.h>
#include <asm/arch/sh_sdhi.h>
#include <miiphy.h>
#include <i2c.h>
#include <mmc.h>
#include <command.h>

#if !defined(CONFIG_TARGET_RZPI)
#  error This platform support file is for the NovTech RZ/pi.
#endif

DECLARE_GLOBAL_DATA_PTR;

#define MAC_ADDR_EEPROM_LOC	0xa0

#define PFC_BASE		0x11030000

#define ETH_CH0		(PFC_BASE + 0x300c)
#define ETH_CH1		(PFC_BASE + 0x3010)
#define I2C_CH1		(PFC_BASE + 0x1870)
#define ETH_PVDD_3300	0x00
#define ETH_PVDD_1800	0x01
#define ETH_PVDD_2500	0x02
#define ETH_MII_RGMII	(PFC_BASE + 0x3018)

/* CPG */
#define CPG_BASE		0x11010000
#define CPG_CLKON_BASE		(CPG_BASE + 0x500)
#define CPG_RESET_BASE		(CPG_BASE + 0x800)
#define CPG_RESET_ETH			(CPG_RESET_BASE + 0x7C)
#define CPG_RESET_I2C			(CPG_RESET_BASE + 0x80)
#define CPG_PL2_SDHI_DSEL	(CPG_BASE + 0x218)
#define CPG_CLK_STATUS		(CPG_BASE + 0x280)

void s_init(void)
{
	/* can go in board_eht_init() once enabled */
	*(volatile u32 *)(ETH_CH0) = (*(volatile u32 *)(ETH_CH0) & 0xFFFFFFFC) | ETH_PVDD_1800;
	*(volatile u32 *)(ETH_CH1) = (*(volatile u32 *)(ETH_CH1) & 0xFFFFFFFC) | ETH_PVDD_1800;
	/* Enable RGMII for both ETH{0,1} */
	*(volatile u32 *)(ETH_MII_RGMII) = (*(volatile u32 *)(ETH_MII_RGMII) & 0xFFFFFFFC);
	/* ETH CLK */
	*(volatile u32 *)(CPG_RESET_ETH) = 0x30003;
	/* I2C CLK */
	*(volatile u32 *)(CPG_RESET_I2C) = 0xF000F;
	/* I2C pin non GPIO enable */
	*(volatile u32 *)(I2C_CH1) = 0x01010101;
	/* SD CLK */
	*(volatile u32 *)(CPG_PL2_SDHI_DSEL) = 0x00110011;
	while (*(volatile u32 *)(CPG_CLK_STATUS) != 0)
		;
}

int board_early_init_f(void)
{

	return 0;
}

#define CONFIG_SYS_SH_SDHI0_BASE  0x11C00000
#define CONFIG_SYS_SH_SDHI1_BASE  0x11C10000

int board_mmc_init(struct bd_info *bis)
{
	return sh_sdhi_init(CONFIG_SYS_SH_SDHI0_BASE, 0, SH_SDHI_QUIRK_64BIT_BUF);
}

int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_TEXT_BASE + 0x50000;

	return 0;
}

void reset_cpu(void)
{
}

static void configure_gpy111_phys(void)
{
	static const unsigned char addrs[] = { 1, 4 };
	struct list_head *entry;
	struct mii_dev *dev;
	unsigned int i;
	unsigned short data;

	list_for_each(entry, mdio_get_list_head()) {
		dev = list_entry(entry, struct mii_dev, link);

		for(i = 0; i < ARRAY_SIZE(addrs); i++) {
			const char *dn = dev->name;
			unsigned char addr = addrs[i];

			if ((miiphy_write(dn, addr, 0x17, 0xb400) != 0) ||
			    (miiphy_read(dn, addr, MII_BMCR, &data) != 0) ||
			    (miiphy_write(dn, addr,  MII_BMCR, data | 1) != 0))
			{
				printf("Can't configure delay for PHY %u", (unsigned int )addr);
			}
		}
	}
}

#define PFC_P10          0x0010
#define PFC_PM10         0x0120
#define PFC_IOLH16       0x10B0

/* Enable the 32KHz clock generator which the Bluetooth/Wi-Fi module needs */
static void enable_32khz_clock(void)
{
	static const uchar enable = 0x40;
	struct udevice *bus, *chip;

	if (!uclass_get_device_by_seq(UCLASS_I2C, 0, &bus) &&
			!i2c_get_chip(bus, 0x12, 1, &chip) &&
			!i2c_set_chip_offset_len(chip, 1))
	{
		dm_i2c_write(chip, 0x6c, &enable, sizeof(enable));
	}
}

static void setup_pins(void)
{
	volatile u8  *prt = (volatile u8  *)PFC_BASE;

	/* Set port 6_0 drive ability to maximum, 12mA. */
	*(volatile u32 *)(PFC_BASE + PFC_IOLH16) |= 0x03;

	/* Turn the blue LED on (P0_0).  */
	prt[PFC_PM10] = (prt[PFC_PM10] & 0xF0) | 0x02; /* Output only. */
	prt[PFC_P10]  = (prt[PFC_P10]  & 0xFE) | 0x01; /* Set high. */
}

int board_late_init(void)
{
	uchar enetaddrs[ETH_ALEN * 2];
	struct udevice *bus, *chip;

	if (!uclass_get_device_by_seq(UCLASS_I2C, 0, &bus) &&
	    !i2c_get_chip(bus, 0x54, 1, &chip) &&
	    !i2c_set_chip_offset_len(chip, 1))
	{
		if (dm_i2c_read(chip, MAC_ADDR_EEPROM_LOC, enetaddrs, sizeof(enetaddrs)) == 0)
		{
			if (is_valid_ethaddr(enetaddrs))
				eth_env_set_enetaddr("ethaddr", enetaddrs);
			if (is_valid_ethaddr(enetaddrs + ETH_ALEN))
				eth_env_set_enetaddr("eth1addr", enetaddrs + ETH_ALEN);
		}
	}

	setup_pins();
	return 0;
}

int last_stage_init(void)
{
	configure_gpy111_phys();
	enable_32khz_clock();

	return 0;
}

#ifndef CONFIG_SPL_BUILD

static int do_set_mac_addresses
(
 struct cmd_tbl *table,
 int             flag,
 int             argc,
 char *const     argv[]
)
{
	uchar           enetaddrs[ETH_ALEN * 2];
	struct udevice *bus, *chip;

	if (argc != 3)
		return (CMD_RET_USAGE);

	string_to_enetaddr(argv[1], enetaddrs);
	if (!is_valid_ethaddr(enetaddrs))
	{
		printf("Invalid MAC address 0: %s\n", argv[1]);
		return CMD_RET_FAILURE;
	}

	string_to_enetaddr(argv[2], enetaddrs + ETH_ALEN);
	if (!is_valid_ethaddr(enetaddrs + ETH_ALEN))
	{
		printf("Invalid MAC address 1: %s\n", argv[2]);
		return CMD_RET_FAILURE;
	}

	if (!uclass_get_device_by_seq(UCLASS_I2C, 0, &bus)
	&&  !i2c_get_chip(bus, 0x54, 1, &chip)
	&&  !i2c_set_chip_offset_len(chip, 1))
	{
		if (dm_i2c_write(chip, MAC_ADDR_EEPROM_LOC, enetaddrs, sizeof(enetaddrs)))
		{
			printf("Error writing to EEPROM\n");
			return CMD_RET_FAILURE;
		}
	}
	else
	{
		printf("Can't find EEPROM\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	set_ether_hwaddr, 3, 1, do_set_mac_addresses,
	NULL, NULL
);

#endif
