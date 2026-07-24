/*
 * Common env helpers for Renesas RZ boards:
 * - Board DTB selection (table-driven)
 * - Device Tree overlay application (flag-driven + list)
 *
 * Copyright (c) 2026, Renesas Electronics Corporation. All rights reserved.
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

#define RZ_FDT_SELECT_BEGIN "if false; then ; "
#define RZ_FDT_SELECT_END   "else echo WARN: unknown board ${model_string}-${revision_major}.${revision_minor}; fi; "

/* Built-in DTB table — add more boards here */
#ifndef RZ_FDT_SELECT_TABLE
#define RZ_FDT_SELECT_TABLE \
	RZ_FDT_CASE("rs-g2l100", "1", "0", "rs-g2l100.dtb") \
	RZ_FDT_CASE("rzg2l-sbc", "1", "0", "rzg2l-sbc.dtb") \
	RZ_FDT_CASE("rzg2l-evk", "1", "0", "rzg2l-evk.dtb") \
	RZ_FDT_CASE("rzv2l-evk", "1", "0", "rzv2l-evk.dtb") \
	RZ_FDT_CASE("rzv2h-evk", "1", "0", "rzv2h-evk-ver1.dtb") \
	RZ_FDT_CASE("rzv2h-evk", "2", "0", "rzv2h-evk-ver1.dtb") \
	RZ_FDT_CASE("rzv2h-rdk", "1", "0", "rzv2h-rdk-ver1.dtb") \
	RZ_FDT_CASE("imdt-v2h-sbc", "1", "0", "imdt-v2h-sbc.dtb") \
	RZ_FDT_CASE("sparrow-hawk", "1", "0", "r8a779g3-sparrow-hawk.dtb")
#endif

/* Built-in overlay flags — add more overlays here */
#ifndef RZ_OVERLAY_FLAGS_TABLE
#define RZ_OVERLAY_FLAGS_TABLE \
	RZ_OVERLAY_IF_FLAG("enable_overlay_i2c",          "${model_string}-${revision_major}.${revision_minor}-ext-i2c.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_spi",          "${model_string}-${revision_major}.${revision_minor}-ext-spi.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_can",          "${model_string}-${revision_major}.${revision_minor}-can.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_dsi",          "${model_string}-${revision_major}.${revision_minor}-dsi.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_audio_codec",  "${model_string}-${revision_major}.${revision_minor}-audio-codec.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_csi_ov5640",   "${model_string}-${revision_major}.${revision_minor}-ov5640.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_csi_ov5645",   "${model_string}-${revision_major}.${revision_minor}-cru-csi-ov5645.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_csi_j1_imx219",   "${model_string}-${revision_major}.${revision_minor}-cru-csi-j1-imx219.dtbo") \
	RZ_OVERLAY_IF_FLAG("enable_overlay_csi_j2_imx219",   "${model_string}-${revision_major}.${revision_minor}-cru-csi-j2-imx219.dtbo")
#endif

/* Image selection cases */
#define RZ_IMAGE_SELECT_BEGIN \
        "if test -z \"${image_flavor}\" || test \"${image_flavor}\" = \"normal\"; then " \
                "setenv kernel_image Image; "

#define RZ_IMAGE_SELECT_END \
        "else " \
                "echo WARN: unknown image_flavor=${image_flavor}, fallback to normal; " \
                "setenv image_flavor normal; " \
                "setenv kernel_image Image; " \
        "fi; \0"

#define RZ_IMAGE_CASE(image_flavor, kernel_image) \
	"elif test \"${image_flavor}\" = \"" image_flavor "\"; then " \
		"setenv kernel_image " kernel_image "; "

#define RZ_IMAGE_SELECT_TABLE \
	RZ_IMAGE_CASE("preempt_rt", "Image-preempt_rt") \
	RZ_IMAGE_CASE("nonpreempt", "Image-nonpreempt")

/* The direct BL31/OP-TEE handoff is only valid for the V4H Sparrow-Hawk DTB. */
#define RZ_V4H_DIRECT_OPTEE_IF_V4H \
	"if test \"${fdtfile}\" = \"r8a779g3-sparrow-hawk.dtb\"; then " \
		"run tfa_boot; " \
	"else "

#define RZ_V4H_DIRECT_OPTEE_END "fi; "

#define RZ_ENV_DEFAULTS \
	"overlaydir=dtb/renesas/overlays\0" \
	"image_select=" \
		RZ_IMAGE_SELECT_BEGIN \
		RZ_IMAGE_SELECT_TABLE \
		RZ_IMAGE_SELECT_END \
	"fdt_select=" \
		"if env exists fdtfile && test -n ${fdtfile}; then " \
			"; " \
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
	"tfa_boot=run mmc_args; run image_select; run fdt_select; " \
		"if test \"${fdtfile}\" != \"r8a779g3-sparrow-hawk.dtb\"; then " \
			"echo ERROR: tfa_boot is Sparrow-Hawk only; " \
		"elif ext4load mmc ${rootfs_mmcdev}:${rootfs_mmcpart} 0x58000000 /boot/v4h-direct-optee.env; then " \
			"setenv manifest_bytes ${filesize}; env import -t 0x58000000 ${manifest_bytes}; " \
			"if test \"${bl31_addr}\" = \"0x46400000\" && test \"${tee_addr}\" = \"0x44100000\" && " \
				"test ${bl31_size} -gt 0 && test ${bl31_size} -le 0x22200 && " \
				"test ${tee_size} -gt 0 && test ${tee_size} -le 0x300000; then " \
				"if ext4load mmc ${rootfs_mmcdev}:${rootfs_mmcpart} ${bl31_addr} ${bl31_file}; then " \
					"setenv bl31_loaded_size ${filesize}; " \
					"if test ${bl31_loaded_size} = ${bl31_size} && crc32 -v ${bl31_addr} ${bl31_loaded_size} bl31_crc32; then " \
						"if ext4load mmc ${rootfs_mmcdev}:${rootfs_mmcpart} ${tee_addr} ${tee_file}; then " \
							"setenv tee_loaded_size ${filesize}; " \
							"if test ${tee_loaded_size} = ${tee_size} && crc32 -v ${tee_addr} ${tee_loaded_size} tee_crc32; then " \
								"if ext4load mmc ${rootfs_mmcdev}:${rootfs_mmcpart} ${image_addr} /boot/${kernel_image} && " \
									"ext4load mmc ${rootfs_mmcdev}:${rootfs_mmcpart} ${dtb_addr} /boot/dtb/renesas/r8a779g3-sparrow-hawk.dtb; then " \
									"tfa_prepare ${bl31_addr} ${bl31_loaded_size} ${tee_addr} ${tee_loaded_size}; " \
									"booti ${image_addr} - ${dtb_addr}; " \
								"fi; " \
							"else echo ERROR: TEE size or CRC mismatch; fi; " \
						"else echo ERROR: TEE load failed; fi; " \
					"else echo ERROR: BL31 size or CRC mismatch; fi; " \
				"else echo ERROR: BL31 load failed; fi; " \
			"else echo ERROR: invalid direct OP-TEE manifest; fi; " \
		"else echo ERROR: cannot load direct OP-TEE manifest; fi\0" \
	"mmc_do_boot=run mmc_args; run image_select; run fdt_select; " \
		RZ_V4H_DIRECT_OPTEE_IF_V4H \
			"fatload mmc ${mmcdev}:${mmcpart} ${image_addr} ${kernel_image}; " \
			"run fdt_ovrun; " \
			"booti ${image_addr} - ${dtb_addr}; " \
		RZ_V4H_DIRECT_OPTEE_END \
		"\0"

#endif /* __RZ_CMN_ENV_H__ */
