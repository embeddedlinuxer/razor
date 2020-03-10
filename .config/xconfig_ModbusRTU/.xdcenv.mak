#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/edma3_lld_02_11_09_08/packages;C:/ti/ipc_1_25_03_15/packages;C:/ti/bios_6_42_03_35/packages;C:/ti/uia_2_00_03_43/packages;C:/ti/ccsv6/ccs_base;C:/Users/daskew/CCS_Workspace/PDI_Modbus/.config
override XDCROOT = c:/ti/xdctools_3_30_06_67_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/edma3_lld_02_11_09_08/packages;C:/ti/ipc_1_25_03_15/packages;C:/ti/bios_6_42_03_35/packages;C:/ti/uia_2_00_03_43/packages;C:/ti/ccsv6/ccs_base;C:/Users/daskew/CCS_Workspace/PDI_Modbus/.config;c:/ti/xdctools_3_30_06_67_core/packages;..
HOSTOS = Windows
endif
