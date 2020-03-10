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
* API.h
*-------------------------------------------------------------------------
* Also adapted from the EEA firmware. This code allows us to convert 
* between API gravity and oil density of various units of measurement. 
* It also has functions that calculate density at standard conditions 
* given the conditions at process.
* (This file was lifted wholesale from the EEA code.)
*-------------------------------------------------------------------------
* HISTORY:
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#ifndef _API
#define _API

#ifdef API_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

_EXTERN void API_VCF(float* VCFw, float* VCFo);
_EXTERN double API_15C_PT(const double r, int* k_set);
_EXTERN double API_60F_PT(const double r, int* k_set);
_EXTERN double API_PT_ST(const double r, const BOOL F60, const BOOL RET_VCF);
_EXTERN double API_to_kgm3(const double r);
_EXTERN double kgm3_to_API(const double r);
_EXTERN void API_STATUS(const BOOL type);

#define API_error_num	-99.0

#undef _EXTERN
#undef API_H
#endif
