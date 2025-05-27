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
#include <asm/sections.h>
#include <linux/delay.h>
#ifdef CONFIG_RENESAS_RZG2LWDT
#include <wdt.h>
#include <rzg2l_wdt.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* H SERIES definition */
#define PFC_BASE			0x10410000
#define PWPR				(PFC_BASE + 0x3C04)
#define P_2A				(PFC_BASE + 0x002A)
#define PM_2A				(PFC_BASE + 0x0154)
#define PMC_2A				(PFC_BASE + 0x022A)
#define PFC_PMC26			(PFC_BASE + 0x0226)
#define PFC_PFC26			(PFC_BASE + 0x0498)
#define PFC_PMC29			(PFC_BASE + 0x0229)
#define PFC_PFC29			(PFC_BASE + 0x04A4)

#define PMC_20				(PFC_BASE + 0x0220)
#define PFC_20				(PFC_BASE + 0x0480)
#define PMC_23				(PFC_BASE + 0x0223)
#define PFC_23				(PFC_BASE + 0x048C)
#define PMC_24				(PFC_BASE + 0x0224)
#define PFC_24				(PFC_BASE + 0x0490)
#define PFC_OSCBYPS			(PFC_BASE + 0x3C00)

#define PWPR_REGWE_A		BIT(6)
#define PWPR_REGWE_B		BIT(5)

/* CPG */
#define CPG_BASE			0x10420000
#define CPG_SSEL0			(CPG_BASE + 0x0300)
#define CPG_SSEL1			(CPG_BASE + 0x0304)
#define CPG_CLKON_11		(CPG_BASE + 0x062C)
#define CPG_CLKON_12		(CPG_BASE + 0x0630)
#define CPG_CLKMON_5		(CPG_BASE + 0x0814)
#define CPG_CLKMON_6		(CPG_BASE + 0x0818)
#define CPG_RST_11			(CPG_BASE + 0x092C)
#define CPG_RSTMON_5		(CPG_BASE + 0x0A14)
#define CPG_RST_USB_V2H		(CPG_BASE + 0x0928)
#define CPG_RSTMON4_USB		(CPG_BASE + 0x0A10)
#define CPG_RSTMON5_USB		(CPG_BASE + 0x0A14)
#define CPG_CLKON_USB_V2H	(CPG_BASE + 0x062C)
#define CPG_CLKMON_USB		(CPG_BASE + 0x0814)
#define CPG_CLKON_9			(CPG_BASE + 0x0624)
#define CPG_RST_9			(CPG_BASE + 0x0924)
#define CPG_RST_10			(CPG_BASE + 0x0928)

#define PFC_OEN				(PFC_BASE + 0x3C40)
#define PFC_OEN_OEN0		BIT(0)
#define PFC_OEN_OEN1		BIT(1)
#define PFC_PWPR			(PFC_BASE + 0x3C04)

#define ICU_IPTSR_REG		0x10400060

/* USB */
#define USBPHY20_BASE		(0x15830000)
#define USBPHY21_BASE		(0x15840000)
#define USBPHY20_RESET		(USBPHY20_BASE + 0x000u)
#define USBPHY21_RESET		(USBPHY21_BASE + 0x000u)

#define USB20_BASE			(0x15800000)
#define USB21_BASE			(0x15810000)
#define USBF_BASE			(0x15820000)

#define USB2_PHY_UTMICTRL2	0xb04
#define USB2_PHY_RESET		0x000
#define USB2_PHY_OTGR		0x600

#define SYS_ADC_CFG			0x10431600

/* L SERIES definition */

#define MAC_ADDR_EEPROM_LOC	0xa0

/* CPG */			
#define CPG_BASE_L_SERIES				0x11010000
#define CPG_CLKON_BASE					(CPG_BASE_L_SERIES + 0x500)
#define CPG_RESET_BASE					(CPG_BASE_L_SERIES + 0x800)
#define CPG_RESET_ETH					(CPG_RESET_BASE + 0x7C)
#define CPG_RESET_I2C					(CPG_RESET_BASE + 0x80)
#define CPG_PL2_SDHI_DSEL				(CPG_BASE_L_SERIES + 0x218)
#define CPG_CLK_STATUS					(CPG_BASE_L_SERIES + 0x280)
#define CPG_RST_USB_L_SERIES			(CPG_BASE_L_SERIES + 0x878)
#define CPG_CLKON_USB_L_SERIES			(CPG_BASE_L_SERIES + 0x578)

/* PFC */
#define PFC_BASE_L_SERIES				0x11030000			
#define PFC_P37							(PFC_BASE_L_SERIES + 0x037)
#define PFC_PM37						(PFC_BASE_L_SERIES + 0x16E)
#define PFC_PMC37						(PFC_BASE_L_SERIES + 0x237)
#define PFC_PWPR_L_SERIES 				(PFC_BASE_L_SERIES + 0x3014)
#define PFC_PMC14						(PFC_BASE_L_SERIES + 0x214)
#define PFC_PFC14						(PFC_BASE_L_SERIES + 0x450)
#define PFC_PMC15						(PFC_BASE_L_SERIES + 0x215)
#define PFC_PFC15						(PFC_BASE_L_SERIES + 0x454)
#define PFC_PMC3A						(PFC_BASE_L_SERIES + 0x23a)
#define PFC_PFC3A						(PFC_BASE_L_SERIES + 0x4e8)

#define ETH_CH0							(PFC_BASE_L_SERIES + 0x300c)
#define ETH_CH1							(PFC_BASE_L_SERIES + 0x3010)
#define I2C_CH1							(PFC_BASE_L_SERIES + 0x1870)
#define ETH_PVDD_3300					0x00
#define ETH_PVDD_1800					0x01
#define ETH_PVDD_2500					0x02
#define ETH_MII_RGMII					(PFC_BASE_L_SERIES + 0x3018)

#define USBPHY_BASE						0x11c40000
#define USB0_BASE						0x11c50000
#define USB1_BASE						0x11c70000
#define USBF_BASE_L_SERIES				0x11c60000
#define USBPHY_RESET					(USBPHY_BASE + 0x000u)


#define RPC_CMNCR						0x10060000

/* WDT */
#define WDT_INDEX						0

/* Common defination */
#define COMMCTRL						0x800
#define HcRhDescriptorA					0x048
#define LPSTS							0x102

extern u64 rcar_atf_boot_args[];
extern u64 board_id;

int board_fit_config_name_match(const char *name)
{
	if ((board_id == BOARD_ID_RZV2H_EVK) &&
		!strcmp(name, "rzv2h-evk-ver1"))
		return 0;

	if ((board_id == BOARD_ID_RZV2L_EVK) &&
		!strcmp(name, "smarc-rzv2l"))
		return 0;

	if ((board_id == BOARD_ID_RZG2L_EVK) &&
		!strcmp(name, "smarc-rzg2l"))
		return 0;

	if ((board_id == BOARD_ID_RZG2L_SBC) &&
		!strcmp(name, "rzpi"))
		return 0;

	return -EINVAL;
}

void s_init_v2h(void)
{
	*(volatile u32 *)PWPR |= (PWPR_REGWE_A | PWPR_REGWE_B);

	/* Enable ADC */
	*(volatile u32 *)(SYS_ADC_CFG) = 0;

	/* SD1  */
	*(volatile u8 *)PMC_2A   &= ~(0x03 << 2);/* PA3,PA2 port */
	*(volatile u8 *)P_2A      = (*(volatile u32 *)P_2A  & ~(0x03<<2)) | (0x01 <<3); /* PA3=1,PA2=0		*/
	*(volatile u16 *)PM_2A    = (*(volatile u32 *)PM_2A & ~(0x0f<<4)) | (0x0a <<4); /* PA3,PA2 output	*/

	/* I2C3	*/
	*(volatile u32 *)PFC_23  = (*(volatile u32 *)PFC_23 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)PMC_23   |= (0x03) << 6;	/* P37,P36 multiplexed function	*/

	/* I2C6	*/
	*(volatile u32 *)PFC_24  = (*(volatile u32 *)PFC_24 & 0xFF00FFFF) | (0x01 << 20) | (0x01 << 16);
	*(volatile u8 *)PMC_24   |= (0x03) << 4;	/* P45,P44 multiplexed function	*/

	/* I2C7	*/
	*(volatile u32 *)PFC_24  = (*(volatile u32 *)PFC_24 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)PMC_24   |= (0x03) << 6;	/* P45,P44 multiplexed function	*/

	/* I2C3	*/
	*(volatile u32 *)CPG_CLKON_9 = 0x00800080;
	*(volatile u32 *)CPG_RST_9   = 0x08000800;
	/* I2C6	*/
	*(volatile u32 *)CPG_CLKON_9 = 0x04000400;
	*(volatile u32 *)CPG_RST_9   = 0x40004000;
	/* I2C7	*/
	*(volatile u32 *)CPG_CLKON_9 = 0x08000800;
	*(volatile u32 *)CPG_RST_9   = 0x80008000;
	/* I2C8 */
	*(volatile u32 *)PFC_20  = (*(volatile u32 *)PFC_20 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)PMC_20   |= (0x03) << 6;	/* P07,P06 multiplexed function	*/

	*(volatile u32 *)CPG_CLKON_9 = 0x00080008;
	*(volatile u32 *)CPG_RST_10  = 0x00010001;

	/* Enale OE of IO block for xSPI */
	*(volatile u32 *)(PFC_OEN) &= ~GENMASK(5,2);

	// Use PLL clock for clk_tx_i only for RGMII mode
	// Wite OEN reg. OEN0 bit "0" for output direction
	*(volatile u32 *)(PFC_OEN) &= ~(PFC_OEN_OEN1 | PFC_OEN_OEN0);
	while((*(volatile u32 *)(PFC_OEN) & (PFC_OEN_OEN1 | PFC_OEN_OEN0)) != 0x0)
		;
	
	*(volatile u32 *)PWPR &= ~(PWPR_REGWE_A | PWPR_REGWE_B);

	/* Set Bypass and Powerdown mode for Audio OSC */
	*(volatile u32 *)(PFC_OSCBYPS) = 0x001C0406;

	*(volatile u32 *)(ICU_IPTSR_REG) = 0;
	
	/* Reset ETH 0,1 */
	*(volatile u32 *)(CPG_RST_11) = 0x00030000;
	while((*(volatile u32 *)(CPG_RSTMON_5) & 0x00000006) == 0x0)
		;

	/* Release reset ETH0,1 */
	*(volatile u32 *)(CPG_RST_11) = 0x00030003;
	while((*(volatile u32 *)(CPG_RSTMON_5) & 0x00000006) != 0x0)
		;

	/* Disable SMUX2_GBE0_RXCLK and SMUX2_GBE1_RXCLK */
	*(volatile u32 *) (CPG_SSEL0) = 0x10000000;
	*(volatile u32 *) (CPG_SSEL1) = 0x00100000;

	/* Enable SMUX2_GBE0_RXCLK and SMUX2_GBE1_RXCLK */
	*(volatile u32 *) (CPG_SSEL0) = 0x10001000;
	*(volatile u32 *) (CPG_SSEL1) = 0x00100010;

	/* Enable aclk_csr, aclk, tx, rx, tx_180, rx_180 for ETH0 */
	/* Enable tx, rx for ETH1 */
	*(volatile u32 *)(CPG_CLKON_11) = 0xFF00FF00;
	while((*(volatile u32 *)(CPG_CLKMON_5) & 0xFF000000) != 0xFF000000)
		;

	/* Enable aclk_csr, aclk, tx_180, rx_180 for ETH1 */
	*(volatile u32 *)(CPG_CLKON_12) = 0x000F000F;
	while((*(volatile u32 *)(CPG_CLKMON_6) & 0x0000000F) != 0x0000000F)
		;
}

void s_init_rzv2l()
{
	/* SD1
	*(volatile u32 *)(PFC_PMC37) &= 0xFFFFFFF9; /* Port func mode 0b00 */
	*(volatile u32 *)(PFC_PM37) = (*(volatile u32 *)(PFC_PM37) & 0xFFFFFFC3) | 0x28; /* Port output mode 0b1010 */
	*(volatile u32 *)(PFC_P37) = (*(volatile u32 *)(PFC_P37) & 0xFFFFFFF9) | 0x6;	/* Port 39[2:1] output value 0b11*/
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
	*(volatile u32 *)(RPC_CMNCR) = 0x01FFF300;
}

void s_init_rzpi()
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

void s_init_rzg2l()
{
	return;
}

void s_init(void)
{
	if (board_id == BOARD_ID_RZV2H_EVK) {
		s_init_v2h();
	} else if (board_id == BOARD_ID_RZG2L_SBC) {
		s_init_rzpi();
	} else if (board_id == BOARD_ID_RZV2L_EVK || board_id == BOARD_ID_RZG2L_EVK) {
		s_init_rzv2l();
	} else {
		return;
	}
}

static void _usbphy_init(void)
{
	/* Overwrite SLEEPM/SUSPENDM signals by USB2PHY Control */
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_UTMICTRL2)) = 0x00000303;
	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_UTMICTRL2)) = 0x00000303;

	/* Assert USB2PHY reset */
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_RESET)) = 0x00000206;
	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_RESET)) = 0x00000206;

	/* Delay 10us */
	udelay(10);

	/* De-Assert USB2PHY reset */
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_RESET)) = 0x00000200;
	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_RESET)) = 0x00000200;

	/* Release overwrites of SLEEPM/SUSMENDM signals, and RESET signal */
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_UTMICTRL2)) = 0x00000003;
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_RESET)) = 0;

	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_UTMICTRL2)) = 0x00000003;
	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_RESET)) = 0;

	/* Activate VBUS Valid comparator */
	(*(volatile u32 *)(USBPHY20_BASE + USB2_PHY_OTGR)) = 0x00000909;
	(*(volatile u32 *)(USBPHY21_BASE + USB2_PHY_OTGR)) = 0x00000909;
}

static void _reset_usb2(void)
{
	/* Reset for USBTEST */
	(*(volatile u32 *)CPG_RST_USB_V2H) |= 0x80008000;
	while((*(volatile u32 *)(CPG_RSTMON5_USB) & 0x00000001) != 0x0);

	/* Reset for USB2 host 0 and 1 */
	(*(volatile u32 *)CPG_RST_USB_V2H) |= 0x30003000;
	while((*(volatile u32 *)(CPG_RSTMON4_USB) & 0x60000000) != 0x0);
}

static void board_usb_init_rzv2h(void)
{
	/* Reset USB*/
	_reset_usb2();

	/* Enable clock for USB */
	(*(volatile u32 *)CPG_CLKON_USB_V2H) = 0x00F800F8;
	while((*(volatile u32 *)(CPG_CLKMON_USB) & 0x00F80000) != 0x00F80000);

	/* Setup  */
	/* Disable GPIO Write Protect */
	(*(volatile u32 *)PFC_PWPR) |= (0x1u << 6);

		/* Set P9_5 as Func.14 for VBUSEN */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)PFC_PMC29) |= (0x1u << 5);
		(*(volatile u32 *)PFC_PFC29) &= ~(0xF << 20);
		/* Function mode 15 */
		(*(volatile u32 *)PFC_PFC29) |= (0x0E << 20);

		/* Set P9_6 as Func.14 for OVRCUR */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)PFC_PMC29) |= (0x1u << 6);
		(*(volatile u32 *)PFC_PFC29) &= ~(0xF << 24);
		/* Function mode 14 */
		(*(volatile u32 *)PFC_PFC29) |= (0x0E << 24);

		/* Set P6_6 as Func.14 for VBUSEN */
		/* Control mode (multiplexed function) */
		 (*(volatile u32 *)PFC_PMC26) |= (0x1u << 6);
		 (*(volatile u32 *)PFC_PFC26) &= ~(0xF << 24);
		/* Function mode 14 */
		 (*(volatile u32 *)PFC_PFC26) |= (0x0E << 24);

		/* Set P6_7 as Func.14 for OVRCUR */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)PFC_PMC26) |= (0x1u << 7);
		(*(volatile u32 *)PFC_PFC26) &= ~(0xF << 28);
		/* Function mode 15 */
		(*(volatile u32 *)PFC_PFC26) |= (0x0E << 28);

	/* Enable Write protect */
	(*(volatile u32 *)PFC_PWPR) &= ~(0x1u << 6);

	/* Initialize phy */
	_usbphy_init();

	/*USB0 is HOST*/
	(*(volatile u32 *)(USB20_BASE + COMMCTRL)) = 0;

	/*USB1 is HOST*/
	(*(volatile u32 *)(USB21_BASE + COMMCTRL)) = 0;

	/* Set USBPHY normal operation (Function only) */
	(*(volatile u16 *)(USBF_BASE + LPSTS)) |= (0x1u << 14);

	/* Overcurrent is not supported */
	(*(volatile u32 *)(USB20_BASE + HcRhDescriptorA)) |= (0x1u << 12);
	(*(volatile u32 *)(USB21_BASE + HcRhDescriptorA)) |= (0x1u << 12);
}

static void board_usb_init_rzv2l(void)
{
	/*Enable USB*/
	(*(volatile u32 *)CPG_RST_USB_L_SERIES) = 0x000f000f;
	(*(volatile u32 *)CPG_CLKON_USB_L_SERIES) = 0x000f000f;

	/* Setup  */
	/* Disable GPIO Write Protect */
	(*(volatile u32 *)PFC_PWPR_L_SERIES) &= ~(0x1u << 7);    /* PWPR.BOWI = 0 */
	(*(volatile u32 *)PFC_PWPR_L_SERIES) |= (0x1u << 6);     /* PWPR.PFCWE = 1 */

	/* set P4_0 as Func.1 for VBUSEN */
	(*(volatile u8 *)PFC_PMC14) |= (0x1u << 0);     /* PMC14.b0 = 1 */
	(*(volatile u8 *)PFC_PFC14) &= ~(0x7u << 0);    /* PFC14.PFC0 = 0 */
	(*(volatile u8 *)PFC_PFC14) |= (0x1u << 0);

	/* set P5_0 as Func.1 for OVERCUR */
	(*(volatile u8 *)PFC_PMC15) |= (0x1u << 0);     /* PMC15.b0 = 1 */
	(*(volatile u8 *)PFC_PFC15) &= ~(0x7u << 0);    /* PFC15.PFC0 = 0 */
	(*(volatile u8 *)PFC_PFC15) |= (0x1u << 0);

	/* set P42_0 as Func.1 for VBUSEN */
	(*(volatile u8 *)PFC_PMC3A) |= (0x1u << 0);     /* PMC14.b0 = 1 */
	(*(volatile u8 *)PFC_PFC3A) &= ~(0xfu << 0);    /* PFC15.PFC0 = 0 */
	(*(volatile u8 *)PFC_PFC3A) |= (0x1u << 0);

	/* set P42_1 as Func.1 for OVERCUR */
	(*(volatile u8 *)PFC_PMC3A) |= (0x1u << 0);     /* PMC14.b1 = 1 */
	(*(volatile u8 *)PFC_PFC3A) &= ~(0xfu << 4);    /* PFC15.PFC1 = 0 */
	(*(volatile u8 *)PFC_PFC3A) |= (0x1u << 4);

	/* Enable write protect */
	(*(volatile u32 *)PFC_PWPR_L_SERIES) &= ~(0x1u << 6);    /* PWPR.PFCWE = 0 */
	(*(volatile u32 *)PFC_PWPR_L_SERIES) |= (0x1u << 7);     /* PWPR.BOWI = 1 */

	/*Enable 2 USB ports*/
	(*(volatile u32 *)USBPHY_RESET) = 0x00001000u;
	/*USB0 is HOST*/
	(*(volatile u32 *)(USB0_BASE + COMMCTRL)) = 0;
	/*USB1 is HOST*/
	(*(volatile u32 *)(USB1_BASE + COMMCTRL)) = 0;
	/* Set USBPHY normal operation (Function only) */
	(*(volatile u16 *)(USBF_BASE_L_SERIES + LPSTS)) |= (0x1u << 14);		/* USBPHY.SUSPM = 1 (func only) */
	/* Overcurrent is not supported */
	(*(volatile u32 *)(USB0_BASE + HcRhDescriptorA)) |= (0x1u << 12);       /* NOCP = 1 */
	(*(volatile u32 *)(USB1_BASE + HcRhDescriptorA)) |= (0x1u << 12);       /* NOCP = 1 */
}

static void board_pmic_i2c_init(void)
{
	struct udevice *bus, *dev;
	int ret;
	u8 reg_addr, reg_val, read_val;

	/* Get the I2C bus */
	ret = uclass_get_device_by_seq(UCLASS_I2C, 8, &bus);
	if (ret)
		goto pmic_failed;

	/* Initialize I2C device at address 0x6a */
	ret = dm_i2c_probe(bus, 0x6a, 0, &dev);
	if (ret)
		goto pmic_failed;

	/* Write 0x00 to register 0x24 of device 0x6a */
	reg_addr = 0x24;
	reg_val = 0x00;
	ret = dm_i2c_write(dev, reg_addr, &reg_val, 1);
	if (ret)
		goto pmic_failed;

	/* Read the value of register 0x24 of device address 0x6a */
	ret = dm_i2c_read(dev, reg_addr, &read_val, 1);
	if (ret) {
		printf("Failed to get value of register 0x%x\n", reg_addr);
		goto pmic_failed;
	}

	/* Check if DCDC installation was successful */
	if (read_val != reg_val) {
		printf("Written value was not correctly at register 0x%x\n",
			   reg_addr);
		goto pmic_failed;
	}

	return;

pmic_failed:
	printf("Can not initialize PMIC settings via I2C8\n");
	return;
}

int board_early_init_f(void)
{
	return 0;
}

#define CONFIG_SYS_SH_SDHI0_BASE  0x11C00000
#define CONFIG_SYS_SH_SDHI1_BASE  0x11C10000

int board_mmc_init(struct bd_info *bis)
{
	if (board_id == BOARD_ID_RZG2L_SBC)
		return sh_sdhi_init(CONFIG_SYS_SH_SDHI0_BASE, 0, SH_SDHI_QUIRK_64BIT_BUF);
	else
		return -1;
}

int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_TEXT_BASE + 0x50000;

	if(board_id == BOARD_ID_RZV2L_EVK || board_id == BOARD_ID_RZG2L_EVK)
	{
		board_usb_init_rzv2l();
	} else if (board_id == BOARD_ID_RZV2H_EVK)
	{
		board_usb_init_rzv2h();
		/* Initialize PMIC I2C devices */
		board_pmic_i2c_init();
	}

	return 0;
}

void reset_cpu(void)
{
#ifdef CONFIG_RENESAS_RZG2LWDT
	struct udevice *wdt_dev;
	if (uclass_get_device(UCLASS_WDT, WDT_INDEX, &wdt_dev) < 0) {
		printf("failed to get wdt device. cannot reset\n");
		return;
	}
	if (wdt_expire_now(wdt_dev, 0) < 0) {
		printf("failed to expire_now wdt\n");
	}
#endif
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

			if (miiphy_write(dn, addr, 0x17, 0xb400) != 0)
			{
				printf("Can't configure delay for PHY %u", (unsigned int )addr);
			}

			if (miiphy_read(dn, addr, MII_BMCR, &data) != 0)
			{
				printf("Can't read control register status for PHY %u", (unsigned int )addr);
			}

			if (miiphy_write(dn, addr,  MII_BMCR, data | 1) != 0)
			{
				printf("Can't reset for PHY %u", (unsigned int )addr);
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
	volatile u8  *prt = (volatile u8  *)PFC_BASE_L_SERIES;

	/* Set port 6_0 drive ability to maximum, 12mA. */
	*(volatile u32 *)(PFC_BASE_L_SERIES + PFC_IOLH16) |= 0x03;

	/* Turn the blue LED on (P0_0).  */
	prt[PFC_PM10] = (prt[PFC_PM10] & 0xF0) | 0x02; /* Output only. */
	prt[PFC_P10]  = (prt[PFC_P10]  & 0xFE) | 0x01; /* Set high. */
}


int board_late_init(void)
{
	if(board_id == BOARD_ID_RZG2L_SBC)
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
	}
#ifdef CONFIG_RENESAS_RZG2LWDT
	rzg2l_reinitr_wdt();
#endif
	return 0;
}

int last_stage_init(void)
{	
	if(board_id == BOARD_ID_RZG2L_SBC)
	{
		configure_gpy111_phys();
		enable_32khz_clock();
	}

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
	if(board_id == BOARD_ID_RZG2L_SBC)
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
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	set_ether_hwaddr, 3, 1, do_set_mac_addresses,
	NULL, NULL
);
#endif
