/*
 * Common env helpers for Renesas RZ boards:
 * - Board DTB selection (table-driven)
 * - Device Tree overlay application (flag-driven + list)
 *
 * Copyright (c) 2025, Renesas Electronics Corporation. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef __RZ_CMN_ENV_H__
#define __RZ_CMN_ENV_H__

/* Apply a single overlay by filename (relative to ${overlaydir}) */
#define RZ_OVERLAY_APPLY_ONE(dtbo_name) \
	"echo Applying DT overlay: " dtbo_name "; " \
	"if fatload mmc ${mmcdev}:${mmcpart} ${dtbo_addr} ${overlaydir}/" dtbo_name "; then " \
		"fdt addr ${dtb_addr}; fdt apply ${dtbo_addr}; " \
	"else " \
		"echo WARN: missing overlay: ${overlaydir}/" dtbo_name "; " \
	"fi; "

/* Apply overlay if env flag exists and is 1/yes */
#define RZ_OVERLAY_IF_FLAG(flag, dtbo_name) \
	"if env exists " flag " && test ${" flag "} = 1 -o ${" flag "} = yes; then " \
		RZ_OVERLAY_APPLY_ONE(dtbo_name) \
	"fi; "

/* Loop through optional list in $fdt_extra_overlays */
#define RZ_OVERLAY_APPLY_LIST \
	"if env exists fdt_extra_overlays && test -n ${fdt_extra_overlays}; then " \
		"for dtbo_file in ${fdt_extra_overlays}; do " \
			"echo Applying DT overlay: ${dtbo_file}; " \
			"if fatload mmc ${mmcdev}:${mmcpart} ${dtbo_addr} ${overlaydir}/${dtbo_file}; then " \
				"fdt addr ${dtb_addr}; fdt apply ${dtbo_addr}; " \
			"else " \
				"echo WARN: missing overlay: ${overlaydir}/${dtbo_file}; " \
			"fi; " \
		"done; " \
	"fi; "

/* Board selection cases */
#define RZ_FDT_CASE(model, maj, min, dtb) \
	"elif test \"${model_string}\" = \"" model "\" && " \
		"test \"${revision_major}\" = \"" maj "\" && " \
		"test \"${revision_minor}\" = \"" min "\"; then " \
		"setenv fdtfile " dtb "; "

#define RZ_FDT_SELECT_BEGIN "if false; then :; "
#define RZ_FDT_SELECT_END   "else echo WARN: unknown board ${model_string}-${revision_major}.${revision_minor}; fi; "

/* Built-in DTB table — add more boards here */
#ifndef RZ_FDT_SELECT_TABLE
#define RZ_FDT_SELECT_TABLE \
	RZ_FDT_CASE("rzg2l-sbc", "1", "0", "rzg2l-sbc.dtb") \
	RZ_FDT_CASE("rzg2l-evk", "1", "0", "r9a07g044l2-smarc.dtb") \
	RZ_FDT_CASE("rzv2l-evk", "1", "0", "r9a07g054l2-smarc.dtb") \
	RZ_FDT_CASE("rzv2h-evk", "1", "0", "r9a09g057h4-evk-ver1.dtb") \
	RZ_FDT_CASE("rzv2h-evk", "2", "0", "r9a09g057h4-evk-ver1.dtb")
#endif

/* Built-in overlay flags — add more overlays here */
#ifndef RZ_OVERLAY_FLAGS_TABLE
#define RZ_OVERLAY_FLAGS_TABLE \
	RZ_OVERLAY_IF_FLAG("enable_overlay_i2c",        "rzg2l-sbc-ext-i2c.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_spi",        "rzg2l-sbc-ext-spi.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_can",        "rzg2l-sbc-can.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_dsi",        "rzg2l-sbc-dsi.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_csi_ov5640", "rzg2l-sbc-ov5640.dtbo")
#endif

#define RZ_ENV_DEFAULTS \
	"overlaydir=dtb/renesas/overlays\0" \
	"fdt_select=" \
		"if env exists fdtfile && test -n ${fdtfile}; then " \
			":; " \
		"else " \
			RZ_FDT_SELECT_BEGIN \
			RZ_FDT_SELECT_TABLE \
			RZ_FDT_SELECT_END \
			"if env exists fdt_user_cases; then run fdt_user_cases; fi; " \
		"fi; \0" \
	"fdt_load=run fdt_select; " \
		"fatload mmc ${mmcdev}:${mmcpart} ${dtb_addr} dtb/renesas/${fdtfile}; \0" \
	"fdt_ovrun=if run fdt_load; then " \
			"fdt addr ${dtb_addr}; fdt resize 0x10000; " \
			RZ_OVERLAY_FLAGS_TABLE \
			RZ_OVERLAY_APPLY_LIST \
			"if env exists overlay_user_cases; then run overlay_user_cases; fi; " \
		"else echo WARN: Cannot load base DT; fi; \0" \
	"mmc_do_boot=run mmc_args; " \
		"fatload mmc ${mmcdev}:${mmcpart} ${image_addr} Image; " \
		"run fdt_ovrun; " \
		"booti ${image_addr} - ${dtb_addr}\0"

#endif /* __RZ_CMN_ENV_H__ */
