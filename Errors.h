/*------------------------------------------------------------------------
* This Information is proprietary to Phase Dynamics Inc, Richardson, Texas
* and MAY NOT be copied by any method or incorporated into another program
* without the express written consent of Phase Dynamics Inc. This information
* or any portion thereof remains the property of Phase Dynamics Inc.
* The information contained herein is believed to be accurate and Phase
* Dynamics Inc assumes no responsibility or liability for its use in any way
* and conveys no license or title under any patent or copyright and makes
* no representation or warranty that this Information is free from patent
* or copyright infringement.
*
* Copyright (c) 2018 Phase Dynamics Inc. ALL RIGHTS RESERVED.
*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------
* Errors.h
*-------------------------------------------------------------------------
* Error handling
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#ifndef _ERRORS
#define _ERRORS

#ifdef  ERRORS_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

_EXTERN Uint32 DIAGNOSTICS;
_EXTERN Uint32 DIAGNOSTICS_MASK;
_EXTERN void checkError(double val, double BOUND_LOW, double BOUND_HIGH, int ERR_LOW, int ERR_HIGH);

/////////////////////////////////////////////
////////////// ERROR HANDLER ////////////////
/////////////////////////////////////////////

#define MAX_ERRORS 12

enum ERRORS {
ERR_WAC_HI	= 1 << 0,
ERR_WAC_LO	= 1 << 1, 
ERR_TMP_HI	= 1 << 2,
ERR_TMP_LO	= 1 << 3,
ERR_FRQ_HI	= 1 << 4,
ERR_FRQ_LO	= 1 << 5,
ERR_VAR_HI	= 1 << 6,
ERR_VAR_LO	= 1 << 7,
ERR_DNS_HI 	= 1 << 8,
ERR_DNS_LO 	= 1 << 9,
ERR_DNS_ADJ_HI 	= 1 << 10,
ERR_DNS_ADJ_LO 	= 1 << 11,

};

static char *errorType[MAX_ERRORS] = 
{
"           Hi WC",
"           Lo WC",
"         Hi Temp",
"         Lo Temp",
"         Hi Freq",
"         Lo Freq",
"    Var Hi Bound",
"    Var Lo Bound",
"      Hi Density",
"      Lo Density",
"     Hi Dens Adj",
"     Lo Dens Adj",
};

/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

/// Relay Modes
#define RELAY_MODE_HI_ONLY		1
#define RELAY_MODE_LO_ONLY		2
#define RELAY_MODE_HI_AND_LO	3
#define RELAY_MODE_MANUAL		4

/// Errors & Malfunctions
#define	ERROR_I2C_EXPANDER		0x10000
#define ERROR_MODBUS_CMD		0x20000
#define ERROR_MODBUS_PKT		0x40000
#define ERROR_MODBUS_ADDR		0x80000
#define ERROR_FLASH_WRITE		0x100000
#define ERROR_FLASH_READ		0x200000

#define reg_direct_i	 		0x00010000
#define reg_direct_f	 		0x00020000
#define reg_indirect	 		0x00030000	// check bounds, alarm points of this var, uses VAR update, scale int
#define reg_indirect_unit		0x00040000
#define reg_direct_bmax 		0x00050000
#define reg_direct_bmin 		0x00060000
#define reg_direct_alarm_hi		0x00070000
#define reg_direct_alarm_lo		0x00080000
#define reg_indirect_calc_unit	0x00090000
#define reg_direct_regvect 		0x000B0000
#define reg_direct_raw_int 		0x000C0000
#define reg_indirect_m	 		0x000D0000	// locates the indicated register within the HART_DV_TABLE and maps it
#define reg_indirect_class		0x000E0000

#undef _EXTERN
#undef  ERRORS_H
#endif /* ERRORS_H_ */
