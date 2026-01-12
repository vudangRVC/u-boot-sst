/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2022 Renesas Electronics Corporation
 */

#ifndef __RZ_CMN_H
#define __RZ_CMN_H

#include <asm/arch/renesas.h>
#include <configs/rz-cmn_env.h>

#define CONFIG_REMAKE_ELF

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.scif"
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

/*
 * RZ Board SoC Identifiers
 * These macros define unique IDs for supported SoC variants.
 * Use them for conditional compilation or SoC-specific configurations.
 */
#define RZ_SOC_RZG2L					0x01
#define RZ_SOC_RZV2L					0x02
#define RZ_SOC_RZV2H					0x03
/* boot option */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Generic Interrupt Controller Definitions */
/* RZ/V2H, V2L, G2L and RZG2L-SBC use GIC-v3 */
// #define CONFIG_GICV3

#define GICD_BASE_RZV2H		0x14900000
#define GICR_BASE_RZV2H		0x14940000
#define GICD_BASE_RZG2L		0x11900000
#define GICR_BASE_RZG2L		0x11940000

/* console */
// #define CONFIG_SYS_CBSIZE		2048
// #define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
// #define CONFIG_SYS_MAXARGS		64
// #define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 38400 }

/* PHY needs a longer autoneg timeout */
#define PHY_ANEG_TIMEOUT		20000

/* MEMORY */
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

/* SDHI clock freq */
#define CONFIG_SH_SDHI_FREQ		133000000

#define DRAM_RSV_SIZE			0x08000000
#define CONFIG_SYS_SDRAM_BASE		(0x40000000 + DRAM_RSV_SIZE)
#define CONFIG_SYS_SDRAM_SIZE		(0x200000000u - DRAM_RSV_SIZE) //total 8GB
#define CONFIG_SYS_LOAD_ADDR		0x58000000
#define CONFIG_LOADADDR			CONFIG_SYS_LOAD_ADDR // Default load address for tfpt,bootp...
#define CONFIG_VERY_BIG_RAM
#define CONFIG_MAX_MEM_MAPPED		(0x80000000u - DRAM_RSV_SIZE)

// #define CONFIG_SYS_MONITOR_BASE		0x00000000
// #define CONFIG_SYS_MONITOR_LEN		(1 * 1024 * 1024)
// #define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
// #define CONFIG_SYS_BOOTM_LEN		(64 << 20)

/* The HF/QSPI layout permits up to 1 MiB large bootloader blob */
#define CONFIG_BOARD_SIZE_LIMIT		1048576

/* ENV setting */
#define CFG_EXTRA_ENV_SETTINGS \
	"bootenvfile=uEnv.txt\0" \
	"image=Image\0" \
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

#define CONFIG_BOOTCOMMAND	"run envboot;run prodsdboot"

/* For board */
/* Ethernet RAVB */
#define CONFIG_BITBANGMII_MULTI

#endif /* __RZ_CMN_H */