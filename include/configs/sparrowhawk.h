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

#endif /* __SPARROWHAWK_H */
