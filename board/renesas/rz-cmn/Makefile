#
# board/renesas/rz-cmn/Makefile
#
# Copyright (C) 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: GPL-2.0+
#

ifdef CONFIG_SPL_BUILD
ifdef CONFIG_RCAR_GEN4
obj-y	:= spl_sparrow.o
else
obj-y	:= spl_rz_cmn.o
endif
else
obj-$(CONFIG_RZ_CMN)	+= rz-cmn.o
obj-$(CONFIG_RCAR_GEN4)	+= sparrowhawk.o
endif
