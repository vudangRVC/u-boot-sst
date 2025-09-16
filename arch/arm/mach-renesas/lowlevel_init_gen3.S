/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * arch/arm/cpu/armv8/rcar_gen3/lowlevel_init.S
 *	This file is lowlevel initialize routine.
 *
 * (C) Copyright 2015-2026 Renesas Electronics Corporation
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

ENTRY(save_boot_params)
	adr     x8, rcar_atf_boot_args
	stp     x0, x1, [x8], #16
	stp     x2, x3, [x8], #16
	b       save_boot_params_ret
ENDPROC(save_boot_params)

.pushsection .text.s_init, "ax"
WEAK(s_init)
	ret
ENDPROC(s_init)
.popsection

.text
ENTRY(lowlevel_init)
	mov	x29, lr			/* Save LR */

	/* board_id <= arg2 (x2) for legacy users */
	ldr x8, =rcar_atf_boot_args
	add x8, x8, #16                     /* 2 * 8 = offset to x2 */
	ldr x0, [x8]
	ldr x9, =board_id
	str x0, [x9]
	/* soc_id <= arg3 (x3) */
	ldr x8, =rcar_atf_boot_args
	add x8, x8, #24                     /* 3 * 8 = offset to x3 */
	ldr x0, [x8]
	ldr x9, =soc_id
	str x0, [x9]
	ldr w11, [x9]                       /* w11 = soc_id (32-bit) */
	
	/* ---------- Select GICD base explicitly by SoC ---------- */
#if !DEBUG_RZG2L_FPGA
	mov		w10, #RZ_SOC_RZG2L
	cmp		w11, w10
	b.eq	set_gicd_g2l

	mov		w10, #RZ_SOC_RZV2L
	cmp		w11, w10
	b.eq	set_gicd_g2l

	mov		w10, #RZ_SOC_RZV2H
	cmp		w11, w10
	b.eq	set_gicd_v2h

set_gicd_default:
	ldr		x0, =GICD_BASE_RZG2L
	b		continue_gic_init

set_gicd_g2l:
	ldr		x0, =GICD_BASE_RZG2L
	b		continue_gic_init

set_gicd_v2h:
	ldr		x0, =GICD_BASE_RZV2H

continue_gic_init:

#ifndef CONFIG_ARMV8_MULTIENTRY
	bl		gic_init_secure
#else
	branch_if_slave x0, 1f
	bl	gic_init_secure
1:
	/* Set GICR_BASE based on SoC ID for secure_percpu init */
	mov		w10, #RZ_SOC_RZG2L
	cmp		w11, w10
	b.eq	set_gicr_g2l

	mov		w10, #RZ_SOC_RZV2L
	cmp		w11, w10
	b.eq	set_gicr_g2l

	mov		w10, #RZ_SOC_RZV2H
	cmp		w11, w10
	b.eq	set_gicr_v2h

set_gicr_default:
	ldr		x0, =GICR_BASE_RZG2L
	b		do_gicr

set_gicr_g2l:
	ldr		x0, =GICR_BASE_RZG2L
	b		do_gicr

set_gicr_v2h:
	ldr		x0, =GICR_BASE_RZV2H

do_gicr:
	bl		gic_init_secure_percpu

	branch_if_master x0, 2f

	/*
	 * Slave should wait for master clearing spin table.
	 * This sync prevent salves observing incorrect
	 * value of spin table and jumping to wrong place.
	 */
	bl	gic_wait_for_interrupt

	/* Enter EL2 (and optionally EL1) */
	adr		x4, lowlevel_in_el2
	ldr		x5, =ES_TO_AARCH64
	bl		armv8_switch_to_el2

lowlevel_in_el2:
#ifdef CONFIG_ARMV8_SWITCH_TO_EL1
	adr	x4, lowlevel_in_el1
	ldr	x5, =ES_TO_AARCH64
	bl	armv8_switch_to_el1

lowlevel_in_el1:
#endif
#endif /* CONFIG_ARMV8_MULTIENTRY */
#endif /* !DEBUG_RZG2L_FPGA */

	bl      s_init

2:
	mov	lr, x29			/* Restore LR */
	ret
ENDPROC(lowlevel_init)

ENTRY(smp_kick_all_cpus)
	/* Kick secondary cpus up by SGI 0 interrupt */
#if defined(CONFIG_GICV2) || defined(CONFIG_GICV3)
	ldr		x8, =rcar_atf_boot_args
	add		x8, x8, #24
	ldr		x0, [x8]
	ldr		x9, =soc_id
	str		x0, [x9]
	ldr		w11, [x9]

	mov		w10, #RZ_SOC_RZG2L
	cmp		w11, w10
	b.eq	use_g2l_gicd

	mov		w10, #RZ_SOC_RZV2L
	cmp		w11, w10
	b.eq	use_g2l_gicd

	mov		w10, #RZ_SOC_RZV2H
	cmp		w11, w10
	b.eq	use_v2h_gicd

	/* default */
	ldr		x0, =GICD_BASE_RZG2L
	b		2f

use_g2l_gicd:
	ldr		x0, =GICD_BASE_RZG2L
	b		2f

use_v2h_gicd:
	ldr		x0, =GICD_BASE_RZV2H

2:
	b	gic_kick_secondary_cpus
#endif
	ret
ENDPROC(smp_kick_all_cpus)
