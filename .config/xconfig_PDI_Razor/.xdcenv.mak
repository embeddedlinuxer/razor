#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = G:/myRepository/packages;G:/workspace/PDI_Razor;C:/ti/ipc_1_25_03_15/packages;C:/ti/bios_6_42_03_35/packages;C:/ti/uia_2_00_03_43/packages;C:/ti/dsplib_c674x_3_1_1_1/packages;C:/ti/edma3_lld_02_11_09_08/packages;C:/ti/imglib_c64Px_3_1_1_0/packages;C:/ti/mathlib_c674x_3_0_2_0/packages;C:/ti/xdais_7_24_00_04/packages;C:/ti/xdais_7_24_00_04/examples;C:/ti/ccsv6/ccs_base;G:/workspace/PDI_Razor/.config
override XDCROOT = c:/ti/xdctools_3_30_06_67_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = G:/myRepository/packages;G:/workspace/PDI_Razor;C:/ti/ipc_1_25_03_15/packages;C:/ti/bios_6_42_03_35/packages;C:/ti/uia_2_00_03_43/packages;C:/ti/dsplib_c674x_3_1_1_1/packages;C:/ti/edma3_lld_02_11_09_08/packages;C:/ti/imglib_c64Px_3_1_1_0/packages;C:/ti/mathlib_c674x_3_0_2_0/packages;C:/ti/xdais_7_24_00_04/packages;C:/ti/xdais_7_24_00_04/examples;C:/ti/ccsv6/ccs_base;G:/workspace/PDI_Razor/.config;c:/ti/xdctools_3_30_06_67_core/packages;..
HOSTOS = Windows
endif
