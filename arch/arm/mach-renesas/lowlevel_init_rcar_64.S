/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * arch/arm/cpu/armv8/rcar_gen3/lowlevel_init.S
 *	This file is lowlevel initialize routine.
 *
 * (C) Copyright 2015 Renesas Electronics Corporation
 *
 * This file is based on the arch/arm/cpu/armv8/start.S
 *
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>
#include <asm/macro.h>

	.align  8
	.globl  rcar_atf_boot_args
rcar_atf_boot_args:
	.dword  0      /* x0 */
	.dword  0      /* x1 */
	.dword  0      /* x2 (legacy: board_id/model) */
	.dword  0      /* x3 (new: soc_id) */

	.data
	.align  3
	.globl  board_id
board_id:
	.dword  0

	.align  3
	.globl  soc_id
soc_id:
	.dword  0

	.text
ENTRY(save_boot_params)
	adr     x8, rcar_atf_boot_args
	stp     x0, x1, [x8], #16
	/* If soc_id (x3) == 0, default to R-Car V4H (0x04) */
	cbnz    x3, 1f
	mov     x3, #0x04
1:
	stp     x2, x3, [x8], #16
	b       save_boot_params_ret
ENDPROC(save_boot_params)
