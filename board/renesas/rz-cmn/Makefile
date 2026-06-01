#
# board/renesas/rz-cmn/Makefile
#
# Copyright (C) 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: GPL-2.0+
#

ifdef CONFIG_SPL_BUILD
obj-y	:= spl_rz_cmn.o
obj-$(CONFIG_XPL_BUILD) += spl_sparrow.o
else
obj-y	:= rz-cmn.o
endif
