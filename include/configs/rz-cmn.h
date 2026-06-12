/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Renesas Electronics Corporation
 */

#ifndef __RZ_CMN_H
#define __RZ_CMN_H

#include <asm/arch/renesas.h>
#include <configs/rz-cmn_env.h>

#ifndef CONFIG_REMAKE_ELF
#define CONFIG_REMAKE_ELF
#endif

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.scif"
#endif

#ifdef CONFIG_SPL_BUILD
/* SPL stub - serial base for build only, SPL never runs on this board */
#define SCIF0_BASE	0x1004b800
#endif

/* RZ board id defines, it will be used to compare with the parameter
 * passed by ATF to decide how to configure U-Boot
 */
#define BOARD_ID_RZG2L_EVK				0x10
#define BOARD_ID_RZG2L_SBC				0x11
#define BOARD_ID_RS_G2L100				0x12
#define BOARD_ID_RZV2L_EVK				0x20
#define BOARD_ID_RZV2H_EVK				0x30
#define BOARD_ID_RZV2H_RDK				0x31
#define BOARD_ID_IMDT_V2H_SBC			0x32
#define BOARD_ID_RCAR_V4H_SPARROWHAWK	0x40

/*
 * RZ Board SoC Identifiers
 * These macros define unique IDs for supported SoC variants.
 * Use them for conditional compilation or SoC-specific configurations.
 */
#define RZ_SOC_RZG2L					0x01
#define RZ_SOC_RZV2L					0x02
#define RZ_SOC_RZV2H					0x03
#define RZ_SOC_RCAR_V4H					0x04

/* boot option */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Generic Interrupt Controller Definitions */
/* RZ/V2H, V2L, G2L and RZG2L-SBC use GIC-v3 */
#ifndef CONFIG_GICV3
#define CONFIG_GICV3
#endif

#define GICD_BASE_RZV2H		0x14900000
#define GICR_BASE_RZV2H		0x14940000
#define GICD_BASE_RZG2L		0x11900000
#define GICR_BASE_RZG2L		0x11940000

/* R-Car Gen4 (Sparrowhawk) base addresses */
#define RWDT_BASE_RCAR_GEN4		0xE6020000
#define SWDT_BASE_RCAR_GEN4		0xE6030000
#define TMU_BASE_RCAR_GEN4		0xE61E0000
#define SCIF0_BASE_RCAR_GEN4		0xE6E60000
#define SCIF1_BASE_RCAR_GEN4		0xE6E68000
#define SCIF2_BASE_RCAR_GEN4		0xE6E88000
#define SCIF3_BASE_RCAR_GEN4		0xE6C50000
#define SCIF4_BASE_RCAR_GEN4		0xE6C40000
#define SCIF5_BASE_RCAR_GEN4		0xE6F30000
#define CPGWPR_RCAR_GEN4		0xE6150000
#define CPGWPCR_RCAR_GEN4		0xE6150004
#define RST_BASE_RCAR_GEN4		0xE6160000
#define CNTCR_BASE_RCAR_GEN4		0xE6080000
#define GICD_BASE_RCAR_GEN4		0xF1000000
#define GICR_LPI_BASE_RCAR_GEN4	0xF1060000
#define GICR_BASE_RCAR_GEN4		(GICR_LPI_BASE_RCAR_GEN4)
#define GICR_SGI_BASE_RCAR_GEN4	0xF1070000
#define GICR_WAKER_RCAR_GEN4		0x0014
#define GICR_PWRR_RCAR_GEN4		0x0024
#define GICR_LPI_WAKER_RCAR_GEN4	(GICR_LPI_BASE_RCAR_GEN4 + GICR_WAKER_RCAR_GEN4)
#define GICR_LPI_PWRR_RCAR_GEN4	(GICR_LPI_BASE_RCAR_GEN4 + GICR_PWRR_RCAR_GEN4)
#define GICR_IGROUPR0_RCAR_GEN4	0x0080
#define CNTFID0_RCAR_GEN4		(CNTCR_BASE_RCAR_GEN4 + 0x020)
#define RST_WDTRSTCR_RCAR_GEN4		(RST_BASE_RCAR_GEN4 + 0x10)
#define RST_RWDT_RCAR_GEN4		0xA55A8002

/* PHY needs a longer autoneg timeout */
#define PHY_ANEG_TIMEOUT		20000

/* MEMORY */
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

/* SDHI clock freq */
#define CONFIG_SH_SDHI_FREQ		133000000

#define DRAM_RSV_SIZE			0x08000000
#ifndef CFG_SYS_SDRAM_BASE
#define CFG_SYS_SDRAM_BASE		0x48000000
#endif
#ifndef CFG_SYS_SDRAM_SIZE
#define CFG_SYS_SDRAM_SIZE		(0x200000000u - DRAM_RSV_SIZE) //total 8GB
#endif
#define CONFIG_SYS_LOAD_ADDR		0x58000000
#define CONFIG_LOADADDR			CONFIG_SYS_LOAD_ADDR // Default load address for tfpt,bootp...
#define CONFIG_VERY_BIG_RAM
#define CFG_MAX_MEM_MAPPED		(0x80000000u - DRAM_RSV_SIZE)

/* ENV setting */
#ifndef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	"bootenvfile=uEnv.txt\0" \
	"image_flavor=normal\0" \
	"importbootenv=echo Importing environment from mmc${mmcdev} ...; " \
		"env import -t ${env_addr} ${filesize}\0" \
	"loadbootenv=fatload mmc ${mmcdev}:${mmcpart} ${env_addr} ${bootenvfile}\0" \
	"envboot=mmc dev ${mmcdev}; " \
		"if mmc rescan; then " \
			"echo SD/MMC found on device ${mmcdev};" \
			"if run loadbootenv; then " \
				"echo Loaded env from ${bootenvfile};" \
				"run importbootenv;" \
			"fi;" \
		"fi;\0" \
	"bootimage=booti ${image_addr} - ${dtb_addr} \0" \
	RZ_ENV_DEFAULTS
#endif

#ifndef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND	"run envboot;run prodsdboot"
#endif

/* For board */
/* Ethernet RAVB */
#define CONFIG_BITBANGMII_MULTI

#endif /* __RZ_CMN_H */
