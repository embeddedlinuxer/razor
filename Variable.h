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
* Variable.h
*-------------------------------------------------------------------------
* This code implements the locking and unlocking of various levels of write
* protection. It uses the SHA256 hash function to store and verify passwords.
* The Basic write-protection lock is disabled (unlocked) by default, and
* when unlocked it allows the user to change a handful of innocuous
* variables/registers that won't get your typical operator into trouble.
* "Tech"-level write protection, when unlocked, allows for the modification
* of most variables we expect the customer may want to change.
* "Factory" unlock allows overwriting many variables that we typically do not
* want customers to ever change, such as the Model Code of their analyzer.
* These write protections are unlocked by writing the appropriate password to
* REG_UNLOCK_PASS[0-2] as an ASCII-coded character string and then forcing the
* appropriate coil ("COIL_UNLOCK_XXXXX") to logic high. The password fields
* will then be cleared regardless of whether the password was accepted or denied.
* This file was in large part lifted from the EEA code
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#ifndef _VARIABLE
#define _VARIABLE

#ifdef VARIABLE_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

#define NULL_INT ((void*)0)
#define NULL_DBL ((double*)0)
#define NULL_VAR ((VAR*)0)
#define TOLERANCE 			0.0001
#define DEF_SCALE_LONG		1000.0
#define DEF_SCALE			100.0
#define var_dampen			0x00000001
#define var_no_bound		0x00000002
#define var_no_alarm		0x00000004
#define var_NaNum			0x00000008
#define var_bound_hi		0x00000010
#define var_bound_lo		0x00000020
#define var_alarm_hi		0x00000040
#define var_alarm_lo		0x00000080
#define var_round			0x00000100
#define var_roll			0x00000200
#define var_aux				0x00000400
#define var_NaNproof		0x00000800
#define CALC_UNIT			0
#define USER_UNIT			1

typedef struct 	{
					double		base_val;		// original value prior to calculations
					double		calc_val;		// value used in calculations
					float		val;			// value that is read by user & clipped
					int			STAT;
					int			aux;
					int			unit;			// unit that is read by user
					int			calc_unit;		// unit that is used in calculations
					int			class;			// variable classification
					float		scale;			// amount to scale the integer values
					float		scale_long;		// amount to scale the integer values
					float		bound_hi_set;
					float		bound_lo_set;
					float		alarm_hi_set;
					float		alarm_lo_set;
					Swi_Handle 	swi;			// triggered inside ModbusRTU.c 
					Swi_Handle 	swi2;			// triggered before VAR_Update()
	    		} VAR;

typedef struct 	{
					double   	val;
					Swi_Handle 	swi;
				} REGSWI; 						// a REGSWI is Modbus register (represented as a double) with a SWI associated with it

typedef struct 	{
					Uint8   	val;
					Swi_Handle 	swi;
	    		} COIL;

_EXTERN void VAR_Copy(VAR *source, VAR* dest);
_EXTERN void VAR_Initialize(VAR *v, int class, int calc_unit, float scale, float scale_long, int STAT);
_EXTERN void COIL_Initialize(COIL *c, Uint8 val, Swi_Handle swi);
_EXTERN BOOL VAR_Setup_Unit (VAR *v, int unit, float upper, float lower, float alarm_hi, float alarm_lo);
_EXTERN BOOL VAR_CheckSet_Bounds(VAR* v, double* t);
_EXTERN BOOL VAR_Check_Bounds(VAR* v, double* t);
_EXTERN BOOL VAR_Set_Unit(VAR *v, int unit, BOOL check_bounds_only);
_EXTERN BOOL VAR_Set_Unit_Param(VAR *v, unsigned int p, float val, int type, BOOL user_unit);
_EXTERN float VAR_Get_Unit_Param(VAR *v, unsigned int p, int type, BOOL user_unit);
_EXTERN void VAR_NaN(VAR *v);
_EXTERN BOOL VAR_Update(VAR *v, double valin, BOOL user_unit);
_EXTERN BOOL Get_Unit_Coeff(VAR* v, int unit, int class, double* m, double* b);
_EXTERN double Time_Scale_Flow(double in, int class, int unit, int flow_unit);
_EXTERN void Breakout_Flow_Units(int class, int flow_unit, int* units, float* r);
_EXTERN double Convert(int class, int from_unit, int to_unit, double val, BOOL scale_only, int aux);
_EXTERN BOOL Get_Unit(int class, int unit, char* str);
_EXTERN BOOL Get_Unit_Clipped(int class, int unit, char* str, Uint8 total_length);
_EXTERN BOOL Get_Class(int class, char* str);
_EXTERN int Get_Next_Unit(int class, int unit);
_EXTERN int Get_Prev_Unit(int class, int unit);
_EXTERN int Join_MassTime_Units(int m, int t);

#undef _EXTERN
#undef VARIABLE_H
#endif
