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
#include <asm/arch/renesas.h>
#include <asm/arch/rcar-mstp.h>
#include <asm/arch/sh_sdhi.h>
#include <miiphy.h>
#include <i2c.h>
#include <mmc.h>
#include <env.h>
#include <command.h>
#include <asm/sections.h>
#include <linux/delay.h>
#include <linux/unaligned/be_byteshift.h>
#include <spi_flash.h>
#include <blk.h>
#ifdef CONFIG_RENESAS_RZG2LWDT
#include <wdt.h>
#include <rzg2l_wdt.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* SYS */
#define	RZV2H_SYS_BASE					(0x10430000)
#define	RZV2H_SYS_LSI_MODE				(RZV2H_SYS_BASE + 0x00000300)

#define RZV2H_MASK_BOOTM_DEVICE			(0x7)

/* PFC */
#define RZV2H_PFC_BASE			0x10410000
#define RZV2H_PWPR			(RZV2H_PFC_BASE + 0x3C04)
#define RZV2H_P_2A			(RZV2H_PFC_BASE + 0x002A)
#define RZV2H_PM_2A			(RZV2H_PFC_BASE + 0x0154)
#define RZV2H_PMC_2A		(RZV2H_PFC_BASE + 0x022A)
#define RZV2H_PFC_PMC26		(RZV2H_PFC_BASE + 0x0226)
#define RZV2H_PFC_PFC26		(RZV2H_PFC_BASE + 0x0498)
#define RZV2H_PFC_PMC29		(RZV2H_PFC_BASE + 0x0229)
#define RZV2H_PFC_PFC29		(RZV2H_PFC_BASE + 0x04A4)

#define RZV2H_PMC_20		(RZV2H_PFC_BASE + 0x0220)
#define RZV2H_PFC_20		(RZV2H_PFC_BASE + 0x0480)
#define RZV2H_PMC_23		(RZV2H_PFC_BASE + 0x0223)
#define RZV2H_PFC_23		(RZV2H_PFC_BASE + 0x048C)
#define RZV2H_PMC_24		(RZV2H_PFC_BASE + 0x0224)
#define RZV2H_PFC_24		(RZV2H_PFC_BASE + 0x0490)
#define RZV2H_PFC_OSCBYPS	(RZV2H_PFC_BASE + 0x3C00)

#define RZV2H_PWPR_REGWE_A		BIT(6)
#define RZV2H_PWPR_REGWE_B		BIT(5)

/* CPG */
#define RZV2H_CPG_BASE				0x10420000
#define RZV2H_CPG_SSEL0				(RZV2H_CPG_BASE + 0x0300)
#define RZV2H_CPG_SSEL1				(RZV2H_CPG_BASE + 0x0304)
#define RZV2H_CPG_CLKON_11			(RZV2H_CPG_BASE + 0x062C)
#define RZV2H_CPG_CLKON_12			(RZV2H_CPG_BASE + 0x0630)
#define RZV2H_CPG_CLKMON_5			(RZV2H_CPG_BASE + 0x0814)
#define RZV2H_CPG_CLKMON_6			(RZV2H_CPG_BASE + 0x0818)
#define RZV2H_CPG_RST_11			(RZV2H_CPG_BASE + 0x092C)
#define RZV2H_CPG_RSTMON_5			(RZV2H_CPG_BASE + 0x0A14)
#define RZV2H_CPG_RST_USB			(RZV2H_CPG_BASE + 0x0928)
#define RZV2H_CPG_RSTMON4_USB		(RZV2H_CPG_BASE + 0x0A10)
#define RZV2H_CPG_RSTMON5_USB		(RZV2H_CPG_BASE + 0x0A14)
#define RZV2H_CPG_CLKON_USB			(RZV2H_CPG_BASE + 0x062C)
#define RZV2H_CPG_CLKMON_USB		(RZV2H_CPG_BASE + 0x0814)
#define RZV2H_CPG_CLKON_9			(RZV2H_CPG_BASE + 0x0624)
#define RZV2H_CPG_RST_9				(RZV2H_CPG_BASE + 0x0924)
#define RZV2H_CPG_RST_10			(RZV2H_CPG_BASE + 0x0928)

#define RZV2H_PFC_OEN				(RZV2H_PFC_BASE + 0x3C40)
#define RZV2H_PFC_OEN_OEN0			BIT(0)
#define RZV2H_PFC_OEN_OEN1			BIT(1)
#define RZV2H_PFC_PWPR				(RZV2H_PFC_BASE + 0x3C04)

#define RZV2H_ICU_IPTSR_REG			0x10400060

/* USB */
#define RZV2H_USBPHY20_BASE			(0x15830000)
#define RZV2H_USBPHY21_BASE			(0x15840000)

#define RZV2H_USB20_BASE				(0x15800000)
#define RZV2H_USB21_BASE				(0x15810000)
#define RZV2H_USBF_BASE				(0x15820000)

#define RZV2H_USB2_PHY_UTMICTRL2		0xb04
#define RZV2H_USB2_PHY_RESET			0x000
#define RZV2H_USB2_PHY_OTGR				0x600

#define RZV2H_SYS_ADC_CFG				0x10431600

/* SYS*/
#define	RZG2L_SYS_BASE					(0x11020000)
#define	RZG2L_SYS_LSI_MODE				(RZG2L_SYS_BASE + 0x00000A00)

#define RZG2L_MASK_BOOTM_DEVICE			(0x0F)

/* CPG */			
#define RZG2L_CPG_BASE					0x11010000
#define RZG2L_CPG_RESET_BASE			(RZG2L_CPG_BASE + 0x800)
#define RZG2L_CPG_RESET_ETH				(RZG2L_CPG_RESET_BASE + 0x7C)
#define RZG2L_CPG_RESET_I2C				(RZG2L_CPG_RESET_BASE + 0x80)
#define RZG2L_CPG_PL2_SDHI_DSEL			(RZG2L_CPG_BASE + 0x218)
#define RZG2L_CPG_CLK_STATUS			(RZG2L_CPG_BASE + 0x280)
#define RZG2L_CPG_RST_USB				(RZG2L_CPG_BASE + 0x878)
#define RZG2L_CPG_CLKON_USB				(RZG2L_CPG_BASE + 0x578)

/* PFC */
#define RZG2L_PFC_BASE					0x11030000
#define RZG2L_PFC_P37					(RZG2L_PFC_BASE + 0x037)
#define RZG2L_PFC_PM37					(RZG2L_PFC_BASE + 0x16E)
#define RZG2L_PFC_PMC37					(RZG2L_PFC_BASE + 0x237)
#define RZG2L_PFC_PWPR	 				(RZG2L_PFC_BASE + 0x3014)
#define RZG2L_PFC_PMC14					(RZG2L_PFC_BASE + 0x214)
#define RZG2L_PFC_PFC14					(RZG2L_PFC_BASE + 0x450)
#define RZG2L_PFC_PMC15					(RZG2L_PFC_BASE + 0x215)
#define RZG2L_PFC_PFC15					(RZG2L_PFC_BASE + 0x454)
#define RZG2L_PFC_PMC3A					(RZG2L_PFC_BASE + 0x23a)
#define RZG2L_PFC_PFC3A					(RZG2L_PFC_BASE + 0x4e8)

#define RZG2L_ETH_CH0					(RZG2L_PFC_BASE + 0x300c)
#define RZG2L_ETH_CH1					(RZG2L_PFC_BASE + 0x3010)
#define RZG2L_I2C_CH1					(RZG2L_PFC_BASE + 0x1870)
#define RZG2L_ETH_PVDD_3300				0x00
#define RZG2L_ETH_PVDD_1800				0x01
#define RZG2L_ETH_PVDD_2500				0x02
#define RZG2L_ETH_MII_RGMII				(RZG2L_PFC_BASE + 0x3018)

#define RZG2L_USBPHY_BASE				0x11c40000
#define RZG2L_USB0_BASE					0x11c50000
#define RZG2L_USB1_BASE					0x11c70000
#define RZG2L_USBF_BASE					0x11c60000
#define RZG2L_USBPHY_RESET				(RZG2L_USBPHY_BASE + 0x000u)

#define RZG2L_MAC_ADDR_EEPROM_LOC		0xa0

#define RZG2L_RPC_CMNCR					0x10060000

/* WDT */
#define RZG2L_WDT_INDEX					0

/* Common defination */
#define COMMCTRL						0x800
#define HcRhDescriptorA					0x048
#define LPSTS							0x102
#define MAX_SIZE_LEN					256
#define BOARD_INFO_LOAD_ADDR			0x48000000
#define BOARD_INFO_SIZE_BYTES			0x810

/* xSPI */
#define RZG2L_XSPI_BOARD_INFO_OFFSET 	0x1C700
#define RZV2H_XSPI_BOARD_INFO_OFFSET	0x5F300

/* eMMC */
#define EMMC_BOARD_INFO_DEV				0
#define EMMC_BOARD_INFO_PART			1
#define RZG2L_EMMC_BOARD_INFO_OFFSET	0xFA
#define RZV2H_EMMC_BOARD_INFO_OFFSET	0x2FA

extern u64 rcar_atf_boot_args[];
extern u64 board_id;
extern u64 soc_id;

/* Platform descriptor */
typedef struct __attribute__((packed)) platform_desc {
	uint32_t model_id;
	uint32_t revision_minor : 16;
	uint32_t revision_major : 16;
	char model_string[MAX_SIZE_LEN];
	char mfg_name[MAX_SIZE_LEN];

	uint32_t bl2_loc        : 4;
	uint32_t bl2_dtb_loc    : 4;
	uint32_t u_boot_loc     : 4;
	uint32_t u_boot_dtb_loc : 4;
	uint32_t kernel_loc     : 4;
	uint32_t kernel_dtb_loc : 4;
	uint32_t res_loc        : 4;    // reserved
	uint32_t res1_loc       : 4;    // reserved

	uint32_t bl2_id         : 4;
	uint32_t bl2_dtb_id     : 4;
	uint32_t u_boot_id      : 4;
	uint32_t u_boot_dtb_id  : 4;
	uint32_t kernel_id      : 4;
	uint32_t kernel_dtb_id  : 4;
	uint32_t res_id         : 4;    // reserved
	uint32_t res1_id        : 4;    // reserved

	uint8_t bl2_desc[MAX_SIZE_LEN];
	uint8_t bl2_dtb_desc[MAX_SIZE_LEN];
	uint8_t u_boot_desc[MAX_SIZE_LEN];
	uint8_t u_boot_dtb_desc[MAX_SIZE_LEN];
	uint8_t kernel_desc[MAX_SIZE_LEN];
	uint8_t kernel_dtb_desc[MAX_SIZE_LEN];
} platform_desc_t;

typedef enum {
	SYS_BOOT_MODE_ESD = 0,
	SYS_BOOT_MODE_EMMC_1_8,
	SYS_BOOT_MODE_EMMC_3_3,
	SYS_BOOT_MODE_SPI_1_8,
	SYS_BOOT_MODE_SPI_3_3,
} boot_mode_t;

boot_mode_t boot_modes[4][7] = {
	/* 
	 * RZG2L boot modes order (See "Overview" description in the Boot Mode section of the RZG2L hardware manual).
	 *
	 * | STAT_MD_BOOT[2:0] | Boot Mode      |
	 * |-------------------|----------------|
	 * | 000               | eSD            |
	 * | 001               | eMMC 1.8V      |
	 * | 010               | eMMC 3.3V      |
	 * | 011               | SPI 1.8V       |
	 * | 100               | SPI 3.3V       |
	 */
	[RZ_SOC_RZG2L] = {
		[0] = SYS_BOOT_MODE_ESD,
		[1] = SYS_BOOT_MODE_EMMC_1_8,
		[2] = SYS_BOOT_MODE_EMMC_3_3,
		[3] = SYS_BOOT_MODE_SPI_1_8,
		[4] = SYS_BOOT_MODE_SPI_3_3,
	},

	/* 
	 * RZV2L boot modes order (See "Overview" description in the Boot Mode section of the RZV2L hardware manual).
	 *
	 * | STAT_MD_BOOT[2:0] | Boot Mode      |
	 * |-------------------|----------------|
	 * | 000               | eSD            |
	 * | 001               | eMMC 1.8V      |
	 * | 010               | eMMC 3.3V      |
	 * | 011               | SPI 1.8V       |
	 * | 100               | SPI 3.3V       |
	 */
	[RZ_SOC_RZV2L] = {
		[0] = SYS_BOOT_MODE_ESD,
		[1] = SYS_BOOT_MODE_EMMC_1_8,
		[2] = SYS_BOOT_MODE_EMMC_3_3,
		[3] = SYS_BOOT_MODE_SPI_1_8,
		[4] = SYS_BOOT_MODE_SPI_3_3,
	},

	/* 
	 * RZV2H boot modes order (See "Boot Operation" description in the Overview section of the RZV2H hardware manual).
	 *
	 * | STAT_MD_BOOT[2:0] | Boot Mode      |
	 * |-------------------|----------------|
	 * | 000               | eSD            |
	 * | 001               | eMMC 3.3V      |
	 * | 101               | eMMC 1.8V      |
	 * | 010               | SPI 3.3V       |
	 * | 110               | SPI 1.8V       |
	 */
	[RZ_SOC_RZV2H] = {
		[0] = SYS_BOOT_MODE_ESD,
		[5] = SYS_BOOT_MODE_EMMC_1_8,
		[1] = SYS_BOOT_MODE_EMMC_3_3,
		[6] = SYS_BOOT_MODE_SPI_1_8,
		[2] = SYS_BOOT_MODE_SPI_3_3,
	},
};

/**
 * sys_get_boot_mode - Resolve the boot mode for the active SoC instance
 *
 * Determine the boot medium by reading the SoC specific STAT_MD_BOOT bits
 * and mapping them to the generic boot_mode_t enumeration.
 *
 * @return boot_mode_t value describing the detected boot medium.
 */
boot_mode_t sys_get_boot_mode(void)
{
	boot_mode_t boot_mode;
	uint32_t stat_md_boot;

	if (soc_id == RZ_SOC_RZG2L || soc_id == RZ_SOC_RZV2L) {
		stat_md_boot = *(volatile uint32_t*)(RZG2L_SYS_LSI_MODE) &
				RZG2L_MASK_BOOTM_DEVICE;
	} else if (soc_id == RZ_SOC_RZV2H) {
		stat_md_boot = *(volatile uint32_t*)(RZV2H_SYS_LSI_MODE) &
				RZV2H_MASK_BOOTM_DEVICE;
	} else {
		printf("Runtime: unknown or unsupported soc_id = %llu\n", soc_id);
		return boot_mode;
	}

	boot_mode = boot_modes[soc_id][stat_md_boot];

	return boot_mode;
}

/**
 * populate_env_from_board_info - Common populate U-Boot environment from board info
 *
 * @param[in] board_info Pointer to platform descriptor structure
 * 
 * Populate common U-Boot environment variables from the provided
 * platform descriptor structure. This includes variables such as
 * model_string, revision_minor, revision_major, mmcdev, mmcpart,
 * mmc_args, image_addr, env_addr, dtb_addr, and dtbo_addr (if applicable).
 *
 */
static void populate_env_from_board_info(const platform_desc_t *board_info)
{
	char tmp_buf[256];
	uint32_t tmp_val;

	/*
	 * Common u-boot env variables section.
	 */
	snprintf(tmp_buf, sizeof(tmp_buf), "%s", board_info->model_string);
	env_set("model_string", tmp_buf);

	snprintf(tmp_buf, sizeof(tmp_buf), "%u", board_info->revision_minor);
	env_set("revision_minor", tmp_buf);

	snprintf(tmp_buf, sizeof(tmp_buf), "%u", board_info->revision_major);
	env_set("revision_major", tmp_buf);

	snprintf(tmp_buf, sizeof(tmp_buf), "%u", board_info->u_boot_desc[0]);
	env_set("mmcdev", tmp_buf);

	snprintf(tmp_buf, sizeof(tmp_buf), "%u", board_info->u_boot_desc[1]);
	env_set("mmcpart", tmp_buf);

	snprintf(tmp_buf, sizeof(tmp_buf),
		 "setenv bootargs rw rootwait earlycon root=/dev/mmcblk%up%u",
		 board_info->u_boot_desc[2], board_info->u_boot_desc[3]);
	env_set("mmc_args", tmp_buf);

	/* Extract BE32 from u_boot_desc[4..7] for image_addr */
	tmp_val = get_unaligned_be32(&board_info->u_boot_desc[4]);
	snprintf(tmp_buf, sizeof(tmp_buf), "0x%08X", tmp_val);
	env_set("image_addr", tmp_buf);

	/* Extract BE32 from u_boot_desc[8..11] for env_addr */
	tmp_val = get_unaligned_be32(&board_info->u_boot_desc[8]);
	snprintf(tmp_buf, sizeof(tmp_buf), "0x%08X", tmp_val);
	env_set("env_addr", tmp_buf);

	/*
	 * Extract BE32 from u_boot_dtb_desc for device tree section.
	 */
	tmp_val = get_unaligned_be32(&board_info->u_boot_dtb_desc[0]);
	snprintf(tmp_buf, sizeof(tmp_buf), "0x%08X", tmp_val);
	env_set("dtb_addr", tmp_buf);

	tmp_val = get_unaligned_be32(&board_info->u_boot_dtb_desc[4]);
	snprintf(tmp_buf, sizeof(tmp_buf), "0x%08X", tmp_val);
	env_set("dtbo_addr", tmp_buf);
}

/**
 * setup_uboot_info_from_emmc - Load board-specific U-Boot environment from eMMC
 *
 * This function probes the eMMC, reads the platform descriptor structure
 * from a board-specific offset, and populates common U-Boot environment.
 *
 */
int setup_uboot_info_from_emmc(void)
{
	struct mmc *mmc;
	struct blk_desc *desc;
	platform_desc_t *board_info;
	uchar *raw;
	unsigned int block_len;
	lbaint_t start_sector;
	lbaint_t sector_count;
	int ret;

	unsigned int original_part;
	int switched_part = 0;

	mmc = find_mmc_device(EMMC_BOARD_INFO_DEV);
	if (!mmc) {
		printf("Failed to find eMMC device %d\n", EMMC_BOARD_INFO_DEV);
		return -ENODEV;
	}

	ret = mmc_init(mmc);
	if (ret) {
		printf("Failed to init eMMC device %d (%d)\n",
				EMMC_BOARD_INFO_DEV, ret);
		return ret;
	}

	desc = mmc_get_blk_desc(mmc);
	if (!desc) {
		printf("Failed to get block descriptor for eMMC device %d\n",
				EMMC_BOARD_INFO_DEV);
		return -ENODEV;
	}

	original_part = desc->hwpart;
	if (EMMC_BOARD_INFO_PART != desc->hwpart) {
		ret = mmc_switch_part(mmc, EMMC_BOARD_INFO_PART);
		if (ret) {
			printf("Failed to switch eMMC device %d to part %u (%d)\n",
					EMMC_BOARD_INFO_DEV, EMMC_BOARD_INFO_PART, ret);
			return ret;
		}
		switched_part = 1;
	}

	block_len = desc->blksz;
	start_sector = (soc_id == RZ_SOC_RZV2H) ?
		RZV2H_EMMC_BOARD_INFO_OFFSET : RZG2L_EMMC_BOARD_INFO_OFFSET;
	sector_count = (BOARD_INFO_SIZE_BYTES + block_len - 1) / block_len;

	raw = (uchar *)(uintptr_t)BOARD_INFO_LOAD_ADDR;
	if (blk_dread(desc, start_sector, sector_count, raw) != sector_count) {
		printf("Failed to read board info from eMMC\n");
		ret = -EIO;
		goto cleanup;
	}

	board_info = (platform_desc_t *)raw;
	populate_env_from_board_info(board_info);
	ret = 0;

cleanup:
	if (switched_part)
		mmc_switch_part(mmc, original_part);

	return ret;
}

/**
 * setup_uboot_info_from_qspi - Load board-specific U-Boot environment from QSPI
 *
 * This function probes the QSPI SPI flash, reads the platform descriptor
 * structure from a board-specific offset, and populates common U-Boot
 * environment variables such as board_id, mmcdev, mmcpart, boot arguments,
 * image address, and device tree addresses.
 *
 * Data in the flash is stored in big-endian format; manual byte assembly is
 * currently used to extract 32-bit values.
 *
 */
int setup_uboot_info_from_qspi(void)
{
	int ret = 0;
	struct spi_flash *flash;
	platform_desc_t *board_info =
		(platform_desc_t *)(uintptr_t)BOARD_INFO_LOAD_ADDR;

	flash = spi_flash_probe(CONFIG_ENV_SPI_BUS, CONFIG_ENV_SPI_CS,
				     CONFIG_ENV_SPI_MAX_HZ, CONFIG_ENV_SPI_MODE);
	if (!flash) {
		printf("Failed to probe SPI flash\n");
		ret = -ENODEV;
		goto cleanup;
	}

	switch (soc_id) {
		case RZ_SOC_RZV2H:
			ret = spi_flash_read(flash, RZV2H_XSPI_BOARD_INFO_OFFSET, CONFIG_ENV_SIZE, board_info);
			break;
		case RZ_SOC_RZG2L:
		case RZ_SOC_RZV2L:
			ret = spi_flash_read(flash, RZG2L_XSPI_BOARD_INFO_OFFSET, CONFIG_ENV_SIZE, board_info);
			break;
		default:
			printf("Runtime: unknown or unsupported soc_id = %llu\n", soc_id);
			ret = -EINVAL;
			goto cleanup;
	}

	if (ret) {
		printf("Failed to read SPI flash: %d\n", ret);
		ret = -EIO;
		goto cleanup;
	}

	populate_env_from_board_info(board_info);

cleanup:
	if (flash)
		spi_flash_free(flash);
	return ret;
}

void s_init_rzv2h(void)
{
	*(volatile u32 *)RZV2H_PWPR |= (RZV2H_PWPR_REGWE_A | RZV2H_PWPR_REGWE_B);

	/* Enable ADC */
	*(volatile u32 *)(RZV2H_SYS_ADC_CFG) = 0;

	/* SD1  */
	*(volatile u8 *)RZV2H_PMC_2A   &= ~(0x03 << 2);/* PA3,PA2 port */
	*(volatile u8 *)RZV2H_P_2A      = (*(volatile u32 *)RZV2H_P_2A  & ~(0x03<<2)) | (0x01 <<3); /* PA3=1,PA2=0		*/
	*(volatile u16 *)RZV2H_PM_2A    = (*(volatile u32 *)RZV2H_PM_2A & ~(0x0f<<4)) | (0x0a <<4); /* PA3,PA2 output	*/

	/* I2C3	*/
	*(volatile u32 *)RZV2H_PFC_23  = (*(volatile u32 *)RZV2H_PFC_23 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)RZV2H_PMC_23   |= (0x03) << 6;	/* P37,P36 multiplexed function	*/

	/* I2C6	*/
	*(volatile u32 *)RZV2H_PFC_24  = (*(volatile u32 *)RZV2H_PFC_24 & 0xFF00FFFF) | (0x01 << 20) | (0x01 << 16);
	*(volatile u8 *)RZV2H_PMC_24   |= (0x03) << 4;	/* P45,P44 multiplexed function	*/

	/* I2C7	*/
	*(volatile u32 *)RZV2H_PFC_24  = (*(volatile u32 *)RZV2H_PFC_24 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)RZV2H_PMC_24   |= (0x03) << 6;	/* P45,P44 multiplexed function	*/

	/* I2C3	*/
	*(volatile u32 *)RZV2H_CPG_CLKON_9 = 0x00800080;
	*(volatile u32 *)RZV2H_CPG_RST_9   = 0x08000800;
	/* I2C6	*/
	*(volatile u32 *)RZV2H_CPG_CLKON_9 = 0x04000400;
	*(volatile u32 *)RZV2H_CPG_RST_9   = 0x40004000;
	/* I2C7	*/
	*(volatile u32 *)RZV2H_CPG_CLKON_9 = 0x08000800;
	*(volatile u32 *)RZV2H_CPG_RST_9   = 0x80008000;
	/* I2C8 */
	*(volatile u32 *)RZV2H_PFC_20  = (*(volatile u32 *)RZV2H_PFC_20 & 0x00FFFFFF) | (0x01 << 28) | (0x01 << 24);
	*(volatile u8 *)RZV2H_PMC_20   |= (0x03) << 6;	/* P07,P06 multiplexed function	*/

	*(volatile u32 *)RZV2H_CPG_CLKON_9 = 0x00080008;
	*(volatile u32 *)RZV2H_CPG_RST_10  = 0x00010001;

	/* Enale OE of IO block for xSPI */
	*(volatile u32 *)(RZV2H_PFC_OEN) &= ~GENMASK(5,2);

	// Use PLL clock for clk_tx_i only for RGMII mode
	// Wite OEN reg. OEN0 bit "0" for output direction
	*(volatile u32 *)(RZV2H_PFC_OEN) &= ~(RZV2H_PFC_OEN_OEN1 | RZV2H_PFC_OEN_OEN0);
	while((*(volatile u32 *)(RZV2H_PFC_OEN) & (RZV2H_PFC_OEN_OEN1 | RZV2H_PFC_OEN_OEN0)) != 0x0)
		;
	
	*(volatile u32 *)RZV2H_PWPR &= ~(RZV2H_PWPR_REGWE_A | RZV2H_PWPR_REGWE_B);

	/* Set Bypass and Powerdown mode for Audio OSC */
	*(volatile u32 *)(RZV2H_PFC_OSCBYPS) = 0x001C0406;

	*(volatile u32 *)(RZV2H_ICU_IPTSR_REG) = 0;
	
	/* Reset ETH 0,1 */
	*(volatile u32 *)(RZV2H_CPG_RST_11) = 0x00030000;
	while((*(volatile u32 *)(RZV2H_CPG_RSTMON_5) & 0x00000006) == 0x0)
		;

	/* Release reset ETH0,1 */
	*(volatile u32 *)(RZV2H_CPG_RST_11) = 0x00030003;
	while((*(volatile u32 *)(RZV2H_CPG_RSTMON_5) & 0x00000006) != 0x0)
		;

	/* Disable SMUX2_GBE0_RXCLK and SMUX2_GBE1_RXCLK */
	*(volatile u32 *) (RZV2H_CPG_SSEL0) = 0x10000000;
	*(volatile u32 *) (RZV2H_CPG_SSEL1) = 0x00100000;

	/* Enable SMUX2_GBE0_RXCLK and SMUX2_GBE1_RXCLK */
	*(volatile u32 *) (RZV2H_CPG_SSEL0) = 0x10001000;
	*(volatile u32 *) (RZV2H_CPG_SSEL1) = 0x00100010;

	/* Enable aclk_csr, aclk, tx, rx, tx_180, rx_180 for ETH0 */
	/* Enable tx, rx for ETH1 */
	*(volatile u32 *)(RZV2H_CPG_CLKON_11) = 0xFF00FF00;
	while((*(volatile u32 *)(RZV2H_CPG_CLKMON_5) & 0xFF000000) != 0xFF000000)
		;

	/* Enable aclk_csr, aclk, tx_180, rx_180 for ETH1 */
	*(volatile u32 *)(RZV2H_CPG_CLKON_12) = 0x000F000F;
	while((*(volatile u32 *)(RZV2H_CPG_CLKMON_6) & 0x0000000F) != 0x0000000F)
		;
}

static void s_init_rzg2l(void)
{
	/* SD1 */
	*(volatile u32 *)(RZG2L_PFC_PMC37) &= 0xFFFFFFF9; /* Port func mode 0b00 */
	*(volatile u32 *)(RZG2L_PFC_PM37) = (*(volatile u32 *)(RZG2L_PFC_PM37) & 0xFFFFFFC3) | 0x28; /* Port output mode 0b1010 */
	*(volatile u32 *)(RZG2L_PFC_P37) = (*(volatile u32 *)(RZG2L_PFC_P37) & 0xFFFFFFF9) | 0x6;	/* Port 39[2:1] output value 0b11*/
	/* can go in board_eht_init() once enabled */
	*(volatile u32 *)(RZG2L_ETH_CH0) = (*(volatile u32 *)(RZG2L_ETH_CH0) & 0xFFFFFFFC) | RZG2L_ETH_PVDD_1800;
	*(volatile u32 *)(RZG2L_ETH_CH1) = (*(volatile u32 *)(RZG2L_ETH_CH1) & 0xFFFFFFFC) | RZG2L_ETH_PVDD_1800;
	/* Enable RGMII for both ETH{0,1} */
	*(volatile u32 *)(RZG2L_ETH_MII_RGMII) = (*(volatile u32 *)(RZG2L_ETH_MII_RGMII) & 0xFFFFFFFC);
	/* ETH CLK */
	*(volatile u32 *)(RZG2L_CPG_RESET_ETH) = 0x30003;
	/* I2C CLK */
	*(volatile u32 *)(RZG2L_CPG_RESET_I2C) = 0xF000F;
	/* I2C pin non GPIO enable */
	*(volatile u32 *)(RZG2L_I2C_CH1) = 0x01010101;
	*(volatile u32 *)(RZG2L_RPC_CMNCR) = 0x01FFF300;
}

static void s_init_rzg2l_sbc(void)
{
	/* can go in board_eth_init() once enabled */
	*(volatile u32 *)(RZG2L_ETH_CH0) = (*(volatile u32 *)(RZG2L_ETH_CH0) & 0xFFFFFFFC) | RZG2L_ETH_PVDD_1800;
	*(volatile u32 *)(RZG2L_ETH_CH1) = (*(volatile u32 *)(RZG2L_ETH_CH1) & 0xFFFFFFFC) | RZG2L_ETH_PVDD_1800;
	/* Enable RGMII for both ETH{0,1} */
	*(volatile u32 *)(RZG2L_ETH_MII_RGMII) = (*(volatile u32 *)(RZG2L_ETH_MII_RGMII) & 0xFFFFFFFC);
	/* ETH CLK */
	*(volatile u32 *)(RZG2L_CPG_RESET_ETH) = 0x30003;
	/* I2C CLK */
	*(volatile u32 *)(RZG2L_CPG_RESET_I2C) = 0xF000F;
	/* I2C pin non GPIO enable */
	*(volatile u32 *)(RZG2L_I2C_CH1) = 0x01010101;
	/* SD CLK */
	*(volatile u32 *)(RZG2L_CPG_PL2_SDHI_DSEL) = 0x00110011;
	while (*(volatile u32 *)(RZG2L_CPG_CLK_STATUS) != 0)
		;
}

void s_init(void)
{
	if (board_id == BOARD_ID_RZV2H_EVK || board_id == BOARD_ID_RZV2H_RDK) {
		s_init_rzv2h();
	} else if (board_id == BOARD_ID_RZG2L_SBC) {
		s_init_rzg2l_sbc();
	} else if (board_id == BOARD_ID_RZV2L_EVK || board_id == BOARD_ID_RZG2L_EVK) {
		s_init_rzg2l();
	} else {
		return;
	}
}

static void rzv2h_usbphy_init(void)
{
	/* Overwrite SLEEPM/SUSPENDM signals by USB2PHY Control */
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_UTMICTRL2)) = 0x00000303;
	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_UTMICTRL2)) = 0x00000303;

	/* Assert USB2PHY reset */
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_RESET)) = 0x00000206;
	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_RESET)) = 0x00000206;

	/* Delay 10us */
	udelay(10);

	/* De-Assert USB2PHY reset */
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_RESET)) = 0x00000200;
	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_RESET)) = 0x00000200;

	/* Release overwrites of SLEEPM/SUSMENDM signals, and RESET signal */
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_UTMICTRL2)) = 0x00000003;
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_RESET)) = 0;

	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_UTMICTRL2)) = 0x00000003;
	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_RESET)) = 0;

	/* Activate VBUS Valid comparator */
	(*(volatile u32 *)(RZV2H_USBPHY20_BASE + RZV2H_USB2_PHY_OTGR)) = 0x00000909;
	(*(volatile u32 *)(RZV2H_USBPHY21_BASE + RZV2H_USB2_PHY_OTGR)) = 0x00000909;
}

static void rzv2h_reset_usb2(void)
{
	/* Reset for USBTEST */
	(*(volatile u32 *)RZV2H_CPG_RST_USB) |= 0x80008000;
	while((*(volatile u32 *)(RZV2H_CPG_RSTMON5_USB) & 0x00000001) != 0x0);

	/* Reset for USB2 host 0 and 1 */
	(*(volatile u32 *)RZV2H_CPG_RST_USB) |= 0x30003000;
	while((*(volatile u32 *)(RZV2H_CPG_RSTMON4_USB) & 0x60000000) != 0x0);
}

static void board_usb_init_rzv2h(void)
{
	/* Reset USB*/
	rzv2h_reset_usb2();

	/* Enable clock for USB */
	(*(volatile u32 *)RZV2H_CPG_CLKON_USB) = 0x00F800F8;
	while((*(volatile u32 *)(RZV2H_CPG_CLKMON_USB) & 0x00F80000) != 0x00F80000);

	/* Setup  */
	/* Disable GPIO Write Protect */
	(*(volatile u32 *)RZV2H_PFC_PWPR) |= (0x1u << 6);

		/* Set P9_5 as Func.14 for VBUSEN */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)RZV2H_PFC_PMC29) |= (0x1u << 5);
		(*(volatile u32 *)RZV2H_PFC_PFC29) &= ~(0xF << 20);
		/* Function mode 15 */
		(*(volatile u32 *)RZV2H_PFC_PFC29) |= (0x0E << 20);

		/* Set P9_6 as Func.14 for OVRCUR */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)RZV2H_PFC_PMC29) |= (0x1u << 6);
		(*(volatile u32 *)RZV2H_PFC_PFC29) &= ~(0xF << 24);
		/* Function mode 14 */
		(*(volatile u32 *)RZV2H_PFC_PFC29) |= (0x0E << 24);

		/* Set P6_6 as Func.14 for VBUSEN */
		/* Control mode (multiplexed function) */
		 (*(volatile u32 *)RZV2H_PFC_PMC26) |= (0x1u << 6);
		 (*(volatile u32 *)RZV2H_PFC_PFC26) &= ~(0xF << 24);
		/* Function mode 14 */
		 (*(volatile u32 *)RZV2H_PFC_PFC26) |= (0x0E << 24);

		/* Set P6_7 as Func.14 for OVRCUR */
		/* Control mode (multiplexed function) */
		(*(volatile u32 *)RZV2H_PFC_PMC26) |= (0x1u << 7);
		(*(volatile u32 *)RZV2H_PFC_PFC26) &= ~(0xF << 28);
		/* Function mode 15 */
		(*(volatile u32 *)RZV2H_PFC_PFC26) |= (0x0E << 28);

	/* Enable Write protect */
	(*(volatile u32 *)RZV2H_PFC_PWPR) &= ~(0x1u << 6);

	/* Initialize phy */
	rzv2h_usbphy_init();

	/*USB0 is HOST*/
	(*(volatile u32 *)(RZV2H_USB20_BASE + COMMCTRL)) = 0;

	/*USB1 is HOST*/
	(*(volatile u32 *)(RZV2H_USB21_BASE + COMMCTRL)) = 0;

	/* Set USBPHY normal operation (Function only) */
	(*(volatile u16 *)(RZV2H_USBF_BASE + LPSTS)) |= (0x1u << 14);

	/* Overcurrent is not supported */
	(*(volatile u32 *)(RZV2H_USB20_BASE + HcRhDescriptorA)) |= (0x1u << 12);
	(*(volatile u32 *)(RZV2H_USB21_BASE + HcRhDescriptorA)) |= (0x1u << 12);
}

static void board_usb_init_rzg2l(void)
{
	/*Enable USB*/
	(*(volatile u32 *)RZG2L_CPG_RST_USB) = 0x000f000f;
	(*(volatile u32 *)RZG2L_CPG_CLKON_USB) = 0x000f000f;

	/* Setup  */
	/* Disable GPIO Write Protect */
	(*(volatile u32 *)RZG2L_PFC_PWPR) &= ~(0x1u << 7);    /* PWPR.BOWI = 0 */
	(*(volatile u32 *)RZG2L_PFC_PWPR) |= (0x1u << 6);     /* PWPR.PFCWE = 1 */

	/* set P4_0 as Func.1 for VBUSEN */
	(*(volatile u8 *)RZG2L_PFC_PMC14) |= (0x1u << 0);     /* PMC14.b0 = 1 */
	(*(volatile u8 *)RZG2L_PFC_PFC14) &= ~(0x7u << 0);    /* PFC14.PFC0 = 0 */
	(*(volatile u8 *)RZG2L_PFC_PFC14) |= (0x1u << 0);

	/* set P5_0 as Func.1 for OVERCUR */
	(*(volatile u8 *)RZG2L_PFC_PMC15) |= (0x1u << 0);     /* PMC15.b0 = 1 */
	(*(volatile u8 *)RZG2L_PFC_PFC15) &= ~(0x7u << 0);    /* PFC15.PFC0 = 0 */
	(*(volatile u8 *)RZG2L_PFC_PFC15) |= (0x1u << 0);

	/* set P42_0 as Func.1 for VBUSEN */
	(*(volatile u8 *)RZG2L_PFC_PMC3A) |= (0x1u << 0);     /* PMC14.b0 = 1 */
	(*(volatile u8 *)RZG2L_PFC_PFC3A) &= ~(0xfu << 0);    /* PFC15.PFC0 = 0 */
	(*(volatile u8 *)RZG2L_PFC_PFC3A) |= (0x1u << 0);

	/* set P42_1 as Func.1 for OVERCUR */
	(*(volatile u8 *)RZG2L_PFC_PMC3A) |= (0x1u << 0);     /* PMC14.b1 = 1 */
	(*(volatile u8 *)RZG2L_PFC_PFC3A) &= ~(0xfu << 4);    /* PFC15.PFC1 = 0 */
	(*(volatile u8 *)RZG2L_PFC_PFC3A) |= (0x1u << 4);

	/* Enable write protect */
	(*(volatile u32 *)RZG2L_PFC_PWPR) &= ~(0x1u << 6);    /* PWPR.PFCWE = 0 */
	(*(volatile u32 *)RZG2L_PFC_PWPR) |= (0x1u << 7);     /* PWPR.BOWI = 1 */

	/*Enable 2 USB ports*/
	(*(volatile u32 *)RZG2L_USBPHY_RESET) = 0x00001000u;
	/*USB0 is HOST*/
	(*(volatile u32 *)(RZG2L_USB0_BASE + COMMCTRL)) = 0;
	/*USB1 is HOST*/
	(*(volatile u32 *)(RZG2L_USB1_BASE + COMMCTRL)) = 0;
	/* Set USBPHY normal operation (Function only) */
	(*(volatile u16 *)(RZG2L_USBF_BASE + LPSTS)) |= (0x1u << 14);		/* USBPHY.SUSPM = 1 (func only) */
	/* Overcurrent is not supported */
	(*(volatile u32 *)(RZG2L_USB0_BASE + HcRhDescriptorA)) |= (0x1u << 12);       /* NOCP = 1 */
	(*(volatile u32 *)(RZG2L_USB1_BASE + HcRhDescriptorA)) |= (0x1u << 12);       /* NOCP = 1 */
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
	s_init();
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
	gd->bd->bi_boot_params = CONFIG_TEXT_BASE + 0x50000;

	if(board_id == BOARD_ID_RZV2L_EVK || board_id == BOARD_ID_RZG2L_EVK)
	{
		board_usb_init_rzg2l();
	} else if (board_id == BOARD_ID_RZV2H_EVK)
	{
		board_usb_init_rzv2h();
		/* Initialize PMIC I2C devices */
		board_pmic_i2c_init();
	}

	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

void reset_cpu(void)
{
#ifdef CONFIG_RENESAS_RZG2LWDT
	struct udevice *wdt_dev;
	if (uclass_get_device(UCLASS_WDT, RZG2L_WDT_INDEX, &wdt_dev) < 0) {
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

	printf("Configuring GPHY111 PHYs for RGMII delay...\n");
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
	volatile u8  *prt = (volatile u8  *)RZG2L_PFC_BASE;

	/* Set port 6_0 drive ability to maximum, 12mA. */
	*(volatile u32 *)(RZG2L_PFC_BASE + PFC_IOLH16) |= 0x03;

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
			if (dm_i2c_read(chip, RZG2L_MAC_ADDR_EEPROM_LOC, enetaddrs, sizeof(enetaddrs)) == 0)
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
	boot_mode_t boot_mode = sys_get_boot_mode();

	switch (boot_mode) {
		case SYS_BOOT_MODE_SPI_1_8:
		case SYS_BOOT_MODE_SPI_3_3:
			setup_uboot_info_from_qspi();
			break;
		case SYS_BOOT_MODE_EMMC_1_8:
		case SYS_BOOT_MODE_EMMC_3_3:
			setup_uboot_info_from_emmc();
			break;
		case SYS_BOOT_MODE_ESD:
			/* ESD is not supported */
			break;
	default:
		break;
	}

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
			if (dm_i2c_write(chip, RZG2L_MAC_ADDR_EEPROM_LOC, enetaddrs, sizeof(enetaddrs)))
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
