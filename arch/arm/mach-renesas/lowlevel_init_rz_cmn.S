/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Merged lowlevel init for Renesas R-Car Gen4 and RZ common SoCs.
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

/*
 * GIC base addresses per SoC family.
 * Fallback defines in case headers are not included for a given family.
 */
#ifndef GICD_BASE_RZG2L
#define GICD_BASE_RZG2L		0x11900000
#endif
#ifndef GICR_BASE_RZG2L
#define GICR_BASE_RZG2L		0x11940000
#endif
#ifndef GICD_BASE_RZV2H
#define GICD_BASE_RZV2H		0x14900000
#endif
#ifndef GICR_BASE_RZV2H
#define GICR_BASE_RZV2H		0x14940000
#endif
/*
 * R-Car Gen4 GIC base (not provided in shared headers, define locally).
 * Gen4: GICD=0xF1000000, GICR=GICR_LPI_BASE=0xF1060000
 */
#define GICD_BASE_RCAR_V4H	0xF1000000
#define GICR_BASE_RCAR_V4H	0xF1060000

.align 8

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
	mov		w10, #0x01		/* RZ_SOC_RZG2L */
	cmp		w11, w10
	b.eq	set_gicd_g2l

	mov		w10, #0x02		/* RZ_SOC_RZV2L */
	cmp		w11, w10
	b.eq	set_gicd_g2l

	mov		w10, #0x03		/* RZ_SOC_RZV2H */
	cmp		w11, w10
	b.eq	set_gicd_v2h

	mov		w10, #0x04		/* RZ_SOC_RCAR_V4H */
	cmp		w11, w10
	b.eq	set_gicd_v4h

	/* Unknown SoC: default to RZG2L */
set_gicd_default:
	ldr		x0, =GICD_BASE_RZG2L
	b		continue_gic_init

set_gicd_g2l:
	ldr		x0, =GICD_BASE_RZG2L
	b		continue_gic_init

set_gicd_v2h:
	ldr		x0, =GICD_BASE_RZV2H
	b		continue_gic_init

set_gicd_v4h:
	ldr		x0, =GICD_BASE_RCAR_V4H

continue_gic_init:

#ifndef CONFIG_ARMV8_MULTIENTRY
	bl		gic_init_secure
#else
	branch_if_slave x0, 1f
	bl	gic_init_secure
1:
	/* Set GICR_BASE based on SoC ID for secure_percpu init */
	mov		w10, #0x01		/* RZ_SOC_RZG2L */
	cmp		w11, w10
	b.eq	set_gicr_g2l

	mov		w10, #0x02		/* RZ_SOC_RZV2L */
	cmp		w11, w10
	b.eq	set_gicr_g2l

	mov		w10, #0x03		/* RZ_SOC_RZV2H */
	cmp		w11, w10
	b.eq	set_gicr_v2h

	mov		w10, #0x04		/* RZ_SOC_RCAR_V4H */
	cmp		w11, w10
	b.eq	set_gicr_v4h

set_gicr_default:
	ldr		x0, =GICR_BASE_RZG2L
	b		do_gicr

set_gicr_g2l:
	ldr		x0, =GICR_BASE_RZG2L
	b		do_gicr

set_gicr_v2h:
	ldr		x0, =GICR_BASE_RZV2H
	b		do_gicr

set_gicr_v4h:
	ldr		x0, =GICR_BASE_RCAR_V4H

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

	mov		w10, #0x01		/* RZ_SOC_RZG2L */
	cmp		w11, w10
	b.eq	use_g2l_gicd

	mov		w10, #0x02		/* RZ_SOC_RZV2L */
	cmp		w11, w10
	b.eq	use_g2l_gicd

	mov		w10, #0x03		/* RZ_SOC_RZV2H */
	cmp		w11, w10
	b.eq	use_v2h_gicd

	mov		w10, #0x04		/* RZ_SOC_RCAR_V4H */
	cmp		w11, w10
	b.eq	use_v4h_gicd

	/* default */
	ldr		x0, =GICD_BASE_RZG2L
	b		2f

use_g2l_gicd:
	ldr		x0, =GICD_BASE_RZG2L
	b		2f

use_v2h_gicd:
	ldr		x0, =GICD_BASE_RZV2H
	b		2f

use_v4h_gicd:
	ldr		x0, =GICD_BASE_RCAR_V4H

2:
	b	gic_kick_secondary_cpus
#endif
	ret
ENDPROC(smp_kick_all_cpus)
