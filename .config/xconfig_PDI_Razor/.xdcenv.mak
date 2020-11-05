#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = G:/ti/win/uia_2_30_01_02/packages;G:/ti/win/bios_6_75_02_00/packages;G:/ti/win/customPkg/packages;G:/ti/win/pdk_omapl138_1_0_8/packages;G:/workspace/PDI_Razor/.config
override XDCROOT = G:/ti/win/ccs910/xdctools_3_55_02_22_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = G:/ti/win/uia_2_30_01_02/packages;G:/ti/win/bios_6_75_02_00/packages;G:/ti/win/customPkg/packages;G:/ti/win/pdk_omapl138_1_0_8/packages;G:/workspace/PDI_Razor/.config;G:/ti/win/ccs910/xdctools_3_55_02_22_core/packages;..
HOSTOS = Windows
endif
