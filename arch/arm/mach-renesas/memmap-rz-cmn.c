// SPDX-License-Identifier: GPL-2.0+
/*
 * Memory map tables for Renesas RZ common SoCs
 *
 * Copyright (C) 2017 Marek Vasut <marek.vasut@gmail.com>
 * Copyright (C) 2026 Renesas Electronics Corp.
 */

#include <asm/armv8/mmu.h>
#include <asm/global_data.h>
#include <asm/system.h>
#include <cpu_func.h>

extern u64 soc_id;

#define RZ_CMN_NR_REGIONS 16

/*
 * RZ/G2L family: up to 4 GiB RAM starting at 0x40000000, of
 * which the first 128 MiB is reserved by TF-A.
 */
static struct mm_region rzg2l_mem_map[RZ_CMN_NR_REGIONS] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = 0x03F00000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0x47E00000UL,
		.phys = 0x47E00000UL,
		.size = 0x78200000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xc0000000UL,
		.phys = 0xc0000000UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

/* R-Car V4H: DRAM may extend beyond the 4 GiB boundary */
static struct mm_region gen3_mem_map[RZ_CMN_NR_REGIONS] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = 0x03F00000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0x47E00000UL,
		.phys = 0x47E00000UL,
		.size = 0x78200000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xc0000000UL,
		.phys = 0xc0000000UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		.virt = 0x100000000UL,
		.phys = 0x100000000UL,
		.size = 0xf00000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = rzg2l_mem_map;

DECLARE_GLOBAL_DATA_PTR;

#define debug_memmap(i, map) \
	debug("memmap %d: virt 0x%llx -> phys 0x%llx, size=0x%llx, attrs=0x%llx\n", \
	      i, map[i].virt, map[i].phys, map[i].size, map[i].attrs)

__weak void renesas_dram_init_banksize(void) { }

static void enable_caches_rzg2l(void)
{
	unsigned int bank, i = 0;
	u64 start, size;

	mem_map = rzg2l_mem_map;

	/* Create map for register access */
	rzg2l_mem_map[i].virt = 0x0ULL;
	rzg2l_mem_map[i].phys = 0x0ULL;
	rzg2l_mem_map[i].size = 0x40000000ULL;
	rzg2l_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
				 PTE_BLOCK_NON_SHARE |
				 PTE_BLOCK_PXN | PTE_BLOCK_UXN;
	debug_memmap(i, rzg2l_mem_map);
	i++;

	/* Generate entries for DRAM in 32bit address space */
	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start = gd->bd->bi_dram[bank].start;
		size = gd->bd->bi_dram[bank].size;

		/* Skip empty DRAM banks */
		if (!size)
			continue;

		/* Mark memory reserved by ATF as cacheable too. */
		if (start == 0x48000000) {
			/* Unmark protection area (0x43F00000 to 0x47DFFFFF) */
			rzg2l_mem_map[i].virt = 0x40000000ULL;
			rzg2l_mem_map[i].phys = 0x40000000ULL;
			rzg2l_mem_map[i].size = 0x03F00000ULL;
			rzg2l_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
						 PTE_BLOCK_INNER_SHARE;
			debug_memmap(i, rzg2l_mem_map);
			i++;

			start = 0x47E00000ULL;
			size += 0x00200000ULL;
		}

		rzg2l_mem_map[i].virt = start;
		rzg2l_mem_map[i].phys = start;
		rzg2l_mem_map[i].size = size;
		rzg2l_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
					 PTE_BLOCK_INNER_SHARE;
		debug_memmap(i, rzg2l_mem_map);
		i++;
	}

	/* Create map for register access (e.g. PRR at 0xfff00044) */
	rzg2l_mem_map[i].virt = 0xc0000000ULL;
	rzg2l_mem_map[i].phys = 0xc0000000ULL;
	rzg2l_mem_map[i].size = 0x40000000ULL;
	rzg2l_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
				 PTE_BLOCK_NON_SHARE |
				 PTE_BLOCK_PXN | PTE_BLOCK_UXN;
	debug_memmap(i, rzg2l_mem_map);
	i++;

	/* Zero out the remaining regions. */
	for (; i < RZ_CMN_NR_REGIONS; i++) {
		rzg2l_mem_map[i].virt = 0;
		rzg2l_mem_map[i].phys = 0;
		rzg2l_mem_map[i].size = 0;
		rzg2l_mem_map[i].attrs = 0;
		debug_memmap(i, rzg2l_mem_map);
	}

	if (!icache_status())
		icache_enable();

	dcache_enable();
}

static void enable_caches_gen3(void)
{
	u64 start, size;
	int bank, i = 0;

	mem_map = gen3_mem_map;

	/* Create map for RPC access */
	gen3_mem_map[i].virt = 0x0ULL;
	gen3_mem_map[i].phys = 0x0ULL;
	gen3_mem_map[i].size = 0x40000000ULL;
	gen3_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
				PTE_BLOCK_NON_SHARE |
				PTE_BLOCK_PXN | PTE_BLOCK_UXN;
	i++;

	/* Generate entries for DRAM in 32bit address space */
	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start = gd->bd->bi_dram[bank].start;
		size = gd->bd->bi_dram[bank].size;

		/* Skip empty DRAM banks */
		if (!size)
			continue;

		/* Skip DRAM above 4 GiB */
		if (start >> 32ULL)
			continue;

		/* Mark memory reserved by ATF as cacheable too. */
		if (start == 0x48000000) {
			/* Unmark protection area (0x43F00000 to 0x47DFFFFF) */
			gen3_mem_map[i].virt = 0x40000000ULL;
			gen3_mem_map[i].phys = 0x40000000ULL;
			gen3_mem_map[i].size = 0x03F00000ULL;
			gen3_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
						PTE_BLOCK_INNER_SHARE;
			i++;

			start = 0x47E00000ULL;
			size += 0x00200000ULL;
		}

		gen3_mem_map[i].virt = start;
		gen3_mem_map[i].phys = start;
		gen3_mem_map[i].size = size;
		gen3_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
					PTE_BLOCK_INNER_SHARE;
		i++;
	}

	/* Create map for register access */
	gen3_mem_map[i].virt = 0xc0000000ULL;
	gen3_mem_map[i].phys = 0xc0000000ULL;
	gen3_mem_map[i].size = 0x40000000ULL;
	gen3_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
				PTE_BLOCK_NON_SHARE |
				PTE_BLOCK_PXN | PTE_BLOCK_UXN;
	i++;

	/* Generate entries for DRAM in 64bit address space */
	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start = gd->bd->bi_dram[bank].start;
		size = gd->bd->bi_dram[bank].size;

		/* Skip empty DRAM banks */
		if (!size)
			continue;

		/* Skip DRAM below 4 GiB */
		if (!(start >> 32ULL))
			continue;

		gen3_mem_map[i].virt = start;
		gen3_mem_map[i].phys = start;
		gen3_mem_map[i].size = size;
		gen3_mem_map[i].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
					PTE_BLOCK_INNER_SHARE;
		i++;
	}

	/* Zero out the remaining regions. */
	for (; i < RZ_CMN_NR_REGIONS; i++) {
		gen3_mem_map[i].virt = 0;
		gen3_mem_map[i].phys = 0;
		gen3_mem_map[i].size = 0;
		gen3_mem_map[i].attrs = 0;
	}

	if (!icache_status())
		icache_enable();

	dcache_enable();
}

void enable_caches(void)
{
	if (soc_id == RZ_SOC_RCAR_V4H)
		enable_caches_gen3();
	else
		enable_caches_rzg2l();
}

int dram_init(void)
{
	int ret = fdtdec_setup_mem_size_base();

	if (current_el() == 3 && gd->ram_base == 0x48000000) {
		/*
		 * If this U-Boot runs in EL3, make the bottom 128 MiB
		 * available for loading of follow up firmware blobs.
		 */
		gd->ram_base -= 0x8000000;
		gd->ram_size += 0x8000000;
	}

	return ret;
}

int dram_init_banksize(void)
{
	int bank;

	fdtdec_setup_memory_banksize();

	if (current_el() == 3) {
		for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
			if (gd->bd->bi_dram[bank].start != 0x48000000)
				continue;

			/*
			 * If this U-Boot runs in EL3, make the bottom 128 MiB
			 * available for loading of follow up firmware blobs.
			 */
			gd->bd->bi_dram[bank].start -= 0x8000000;
			gd->bd->bi_dram[bank].size += 0x8000000;
			break;
		}
	}

	renesas_dram_init_banksize();

	return 0;
}
