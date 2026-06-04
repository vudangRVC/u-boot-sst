/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/configs/sparrowhawk.h
 *     This file is Sparrow Hawk board configuration.
 *
 * Copyright (C) 2025 Marek Vasut <marek.vasut+renesas@mailbox.org>
 */

#ifndef __SPARROWHAWK_H
#define __SPARROWHAWK_H

#include "rcar-gen4-common.h"

/* Platform settings stored in SPI flash at offset 0x1F00000 */
#define CFG_SPL_PLATFORM_SETTINGS_OFFSET	0x400000

/* SDHI clock freq */
#define CONFIG_SH_SDHI_FREQ		133000000

/*
 * RZ Board SoC Identifiers
 * These macros define unique IDs for supported SoC variants.
 * Use them for conditional compilation or SoC-specific configurations.
 */
#define RZ_SOC_RZG2L					0x01
#define RZ_SOC_RZV2L					0x02
#define RZ_SOC_RZV2H					0x03
#define RZ_SOC_RCAR_V4H					0x04

#endif /* __SPARROWHAWK_H */
