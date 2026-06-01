// SPDX-License-Identifier: GPL-2.0+
/*
 * RZ/CMN SPL stub - SPL is not used on this platform.
 * U-Boot proper is loaded directly by ATF (BL31).
 * This file exists only to satisfy the build system when CONFIG_SPL is enabled.
 */

#include <spl.h>
#include <init.h>

void board_init_f(ulong dummy)
{
}

void spl_board_init(void)
{
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_MMC1;
}

void s_init(void)
{
}

void reset_cpu(void)
{
}

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);
	image_entry_noargs_t image_entry =
		(image_entry_noargs_t)spl_image->entry_point;
	image_entry();
}
