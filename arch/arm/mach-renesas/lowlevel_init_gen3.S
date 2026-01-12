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

.align 8
.globl	rcar_atf_boot_args
rcar_atf_boot_args:
	.dword 0
	.dword 0
	.dword 0
	.dword 0

.data
.align 3
.globl board_id
board_id:
	.dword 0

ENTRY(save_boot_params)
	adr	x8, rcar_atf_boot_args
	stp	x0, x1, [x8], #16
	stp	x2, x3, [x8], #16
	b	save_boot_params_ret
ENDPROC(save_boot_params)

.pushsection .text.s_init, "ax"
WEAK(s_init)
	ret
ENDPROC(s_init)
.popsection

.text
ENTRY(lowlevel_init)
	mov	x29, lr			/* Save LR */

	/* Load board_id from rcar_atf_boot_args[2] */
	ldr x8, =rcar_atf_boot_args
	add x8, x8, #16
	ldr x0, [x8]
	ldr x9, =board_id
	str x0, [x9]
	ldr w11, [x9]

	cmp w11, #BOARD_ID_RZG2L_SBC
	b.eq set_gic_base_v2l

	cmp w11, #BOARD_ID_RZG2L_EVK
	b.eq set_gic_base_v2l

	cmp w11, #BOARD_ID_RZV2L_EVK
	b.eq set_gic_base_v2l

	cmp w11, #BOARD_ID_RZV2H_EVK
	ldr x0, =GICD_BASE_RZV2H
	b continue_gic_init

set_gic_base_v2l:
	ldr x0, =GICD_BASE_RZV2L

continue_gic_init:

#ifndef CONFIG_ARMV8_MULTIENTRY
	/*
	 * For single-entry systems the lowlevel init is very simple.
	 */
	bl	gic_init_secure

#else /* CONFIG_ARMV8_MULTIENTRY is set */

	branch_if_slave x0, 1f
	bl	gic_init_secure
1:
	/* Set GICR_BASE based on board ID for secure_percpu init */
	cmp w11, #BOARD_ID_RZG2L_SBC
	b.eq 2f

	cmp w11, #BOARD_ID_RZG2L_EVK
	b.eq 2f

	cmp w11, #BOARD_ID_RZV2L_EVK
	b.eq 2f

	ldr x0, =GICR_BASE_RZV2H
	b 3f

2:
	ldr x0, =GICR_BASE_RZV2L

3:
	bl 	gic_init_secure_percpu

	branch_if_master x0, 2f

	/*
	 * Slave should wait for master clearing spin table.
	 * This sync prevent salves observing incorrect
	 * value of spin table and jumping to wrong place.
	 */
	bl	gic_wait_for_interrupt

	/*
	 * All slaves will enter EL2 and optionally EL1.
	 */
	adr	x4, lowlevel_in_el2
	ldr	x5, =ES_TO_AARCH64
	bl	armv8_switch_to_el2

lowlevel_in_el2:
#ifdef CONFIG_ARMV8_SWITCH_TO_EL1
	adr	x4, lowlevel_in_el1
	ldr	x5, =ES_TO_AARCH64
	bl	armv8_switch_to_el1

lowlevel_in_el1:
#endif
#endif /* CONFIG_ARMV8_MULTIENTRY */

	bl      s_init

2:
	mov	lr, x29			/* Restore LR */
	ret
ENDPROC(lowlevel_init)

ENTRY(smp_kick_all_cpus)
	/* Kick secondary cpus up by SGI 0 interrupt */
#if defined(CONFIG_GICV2) || defined(CONFIG_GICV3)
	ldr x8, =rcar_atf_boot_args
	add x8, x8, #16
	ldr x0, [x8]
	ldr x9, =board_id
	str x0, [x9]
	ldr w11, [x9]

	cmp w11, #BOARD_ID_RZG2L_SBC
	b.eq 1f

	cmp w11, #BOARD_ID_RZG2L_EVK
	b.eq 1f

	cmp w11, #BOARD_ID_RZV2L_EVK
	b.eq 1f

	ldr x0, =GICD_BASE_RZV2H
	b 2f

1:
	ldr x0, =GICD_BASE_RZV2L

2:
	b	gic_kick_secondary_cpus
#endif
	ret
ENDPROC(smp_kick_all_cpus)