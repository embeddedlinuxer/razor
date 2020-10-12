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
* Variable.c
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

#define VARIABLE_H
#define UNITS_H

#include "Globals.h"
#include "Utils.h"
#include "Variable.h"

/****************************************************************************/
/* VAR UPDATE																*/
/*                                                                          */
/* Description: Updates a variable, does the "VAR = x;" operation.          */
/*                                                                          */
/* Arguments:   VAR POINTER v			- variable location					*/
/*				double valin			- input value						*/
/*				BOOL user_unit         - indicates value in user_unit      */
/*                                                                          */
/* Returns:     BOOL r - TRUE for normal operation.                         */
/*                                                                          */
/****************************************************************************/
BOOL VAR_Update(VAR *v, double valin, BOOL user_unit)
{
    // SWI 2 - triggered BEFORE doing anything 
    if (v->swi2 != (Swi_Handle)NULL) Swi_post(v->swi2);

	BOOL   r = TRUE;	/* return TRUE OR FALSE */
	double t;			/* user unit value 		*/
	double A;			/* dampened value 		*/
	double bv; 			/* base value 			*/
	static unsigned int timer = 0;

	v->base_val = valin;

    // CONVER TO CALC_UNIT
	if (user_unit) v->base_val = Convert(v->class, v->unit, v->calc_unit, valin, 0, v->aux);

	bv = v->base_val;

	// process incoming variable as calc_unit
	r = VAR_CheckSet_Bounds(v, &bv);

	if (((v->STAT & var_NaNum)==0) || (v->STAT & var_dampen) == var_dampen) 		// if it is a number (not a NaN)
	{
		if ((v->STAT & var_dampen) == var_dampen)
		{
			if (timer < REG_AO_DAMPEN*2) timer++;			// dampened variable scans every 0.5 sec so needs to multiply by 2
			else
            {
                timer = 0;
                A = 1 - exp(-1 / (REG_AO_DAMPEN*10.0));
                t = (A * v->calc_val) + ((1-A) * v->val);
                v->calc_val = t;
            }
		}
		/* convert back to user units */
		t = Convert(v->class, v->calc_unit, v->unit, v->calc_val, 0, v->aux); ///changed
		((v->STAT & var_round)) ? (v->val = Round_N(t,0)) : (v->val = t);
    
        if (v->swi != (Swi_Handle)NULL) Swi_post(v->swi);
	}

	return r;
}


/****************************************************************************/
/* CONVERT																	*/
/*                                                                          */
/* Description: Converts a value from one unit to the other                 */
/*                                                                          */
/* Arguments:   INT  class		- unit class code							*/
/*				INT  from_unit	- unit code to convert from					*/
/*				INT  to_unit	- unit code to convert to					*/
/*				double val - value that needs converting					*/
/*				BOOL scale_only - offset is ignored							*/
/*				INT  aux        -                                           */
/*                                                                          */
/* Returns:		double r - converted value                            		*/
/*                                                                          */
/* Notes:       none                                                        */
/*                                                                          */
/****************************************************************************/
double Convert(int class, int from_unit, int to_unit, double val, BOOL scale_only, int aux)
{
	double m;		/* multiplier 	*/
	double b;		/* offset 		*/
	double r;		/* return		*/
	double kgm3;	/* kgm3		    */
	double kgm3_15;	/* kgm3@15c		*/
	double kgm3_60;	/* kgm3@60f		*/
	BOOL found;		/* found flag	*/

	/// input value, and return value later
	r = val;

	/// same unit, no need to convert
	if ((from_unit&0xFF)==(to_unit&0xFF)) return r;

	found = FALSE;

    /// temperature conversion
	if (class==c_temperature) 
	{
		found = TRUE;
		switch (from_unit&0xFF)
		{
			case u_temp_F	:
				r = (val - 32) / 1.8;
				break;
			case u_temp_C	:
				r = (val * 1.8) + 32;
				break;
			default			:
				break;
		}
	}
	else
	{   /// density conversion
		if (class==c_mass_per_volume) 
		{
			found = TRUE;
			switch (from_unit&0xFF)
			{
				case u_mpv_kg_cm:
					kgm3 	= r;
					kgm3_15 = API2KGM3_15(kgm3,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_deg_API:
					kgm3  	= (141.5*999.012)/(131.5+r);
					kgm3_15 = API2KGM3_15(kgm3,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_lbs_cf:
					kgm3	= r*16.0105;
					kgm3_15 = API2KGM3_15(kgm3,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_sg:
					kgm3	= r*998.23;
					kgm3_15 = API2KGM3_15(kgm3,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_kg_cm_15C:
					kgm3_15	= r; 
					kgm3  	= API2KGM3(kgm3_15,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_deg_API_60F:
					kgm3_15	= API2KGM3_15(((141.5*999.012)/(131.5+r)),15.5556);
					kgm3  	= API2KGM3(kgm3_15,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_lbs_cf_60F:
					kgm3_15	= API2KGM3_15((r*16.0185),15.5556);
					kgm3  	= API2KGM3(kgm3_15,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				case u_mpv_sg_15C:
					kgm3_15	= r*998.23;
					kgm3  	= API2KGM3(kgm3_15,REG_TEMP_USER.calc_val);
					kgm3_60	= API2KGM3(kgm3_15,15.5556);
					break;
				default:
				{
					found = FALSE;
					break;
				}
			}
		}

		if (!found)
		{
			Get_Unit_Coeff((VAR*)0, from_unit&0xFF, class, &m, &b);
			if (scale_only) b = 0;
			r = (r-b)/m;
		}

		found = FALSE;

		if (class==c_mass_per_volume)
		{
			found = TRUE;
			switch (to_unit&0xFF)
			{
				case u_mpv_kg_cm:
					r = kgm3;
					break;
				case u_mpv_deg_API:
					r = (141.5*999.012)/kgm3 - 131.5;
					break;
				case u_mpv_lbs_cf:
					r = kgm3/16.0185;
					break;
				case u_mpv_sg:
					r = kgm3/998.23;
					break;
				case u_mpv_kg_cm_15C:
					r = kgm3_15;
					break;
				case u_mpv_deg_API_60F:
					r = (141.5*999.012)/kgm3_60 - 131.5;
					break;
				case u_mpv_lbs_cf_60F:
					r = kgm3_60/16.0185;
					break;
				case u_mpv_sg_15C:
					r = kgm3_15/998.23;
					break;
				default:
				{
					found = FALSE;
					break;
				}
			}
		}

		if (!found)
		{
			Get_Unit_Coeff((VAR*)0, to_unit&0xFF, class, &m, &b);
			if (scale_only) b = 0;
			r = (r*m)+b;
		}
	}

	return r;
}


/****************************************************************************/
/* VAR INITIALIZE															*/
/*                                                                          */
/* Description: Initializes a variable                                      */
/*                                                                          */
/* Arguments:   VAR POINTER v		- variable location						*/
/*				INT class			- class									*/
/*				FLOAT scale			- scale									*/
/*				FLOAT scale_long	- scale_long							*/
/*				INT STAT			- STATUS register                       */
/*                                                                          */
/* Returns:     void                                                        */
/*                                                                          */
/* Notes:       calculations are done with the base_unit and upper/lower 	*/
/*				limits are compared to the actual calculated unit           */
/*				STAT var_dampen will make it a dampened variable			*/
/*                                             	                             */
/****************************************************************************/
void VAR_Initialize(VAR *v, int class, int calc_unit, float scale, float scale_long, int STAT)
{
	v->STAT  	  	= STAT;
	v->STAT 	   &= var_NaNum	   ^ 0xFFFFFFFF;
	v->STAT 	   &= var_alarm_lo ^ 0xFFFFFFFF;
	v->STAT 	   &= var_alarm_hi ^ 0xFFFFFFFF;
	v->STAT 	   &= var_bound_lo ^ 0xFFFFFFFF;
	v->STAT 	   &= var_bound_hi ^ 0xFFFFFFFF;
	v->STAT 	   &= var_aux	   ^ 0xFFFFFFFF;
	v->calc_unit  	= calc_unit;
	v->unit 	  	= v->calc_unit;
	v->class 	  	= class;
	v->swi			= (Swi_Handle)NULL; // triggered inside ModbusRTU.c
	v->swi2			= (Swi_Handle)NULL; // swi triggered before VAR_Update
	v->scale 	  	= scale;
	v->scale_long 	= scale_long;
	v->val 		  	= 0.0;
	v->calc_val   	= 0.0;
	v->base_val   	= 0.0;
	v->aux 		 	= 0;
	v->bound_hi_set =  1000000000.0;
	v->bound_lo_set = -1000000000.0;
	v->alarm_hi_set =  1000000000.0;
	v->alarm_lo_set = -1000000000.0;

	if ((v->STAT & var_roll)==var_roll) /* check if a roll variable */
		v->bound_lo_set = 0;
}

void COIL_Initialize(COIL *c, Uint8 val, Swi_Handle swi)
{
	c->val = val;

	if ((swi == NULL) || (swi == 0)) c->swi = (Swi_Handle)NULL;
	else c->swi = swi;
}

/****************************************************************************/
/* VAR SETUP UNIT															*/
/*                                                                          */
/* Description: sets up the variable unit bounds and alarms                 */
/*                                                                          */
/* Arguments:   VAR POINTER v 	- variable address							*/
/*				INT unit		- input unit code							*/
/*				FLOAT upper		- upper limit								*/
/*				FLOAT lower		- lower limit								*/
/*				FLOAT alarm_hi	- upper alarm								*/
/*				FLOAT alarm_lo  - lower alarm                               */
/*                                                                          */
/* Returns:     BOOL always TRUE                                            */
/*                                                                          */
/* Notes:       if input unit is same as var->calc unit, set bounds/alarms  */
/*                                                                          */
/****************************************************************************/
BOOL VAR_Setup_Unit (VAR *v, int unit, float upper, float lower, float alarm_hi, float alarm_lo)
{
	v->unit = unit;

	if (v->calc_unit==v->unit)
	{
		v->bound_hi_set = upper;
		v->bound_lo_set = lower;
		v->alarm_hi_set = alarm_hi;
		v->alarm_lo_set = alarm_lo;
	}

	if ((v->STAT & var_roll)==var_roll)
		v->bound_lo_set = 0;

	return TRUE;
}

/****************************************************************************/
/* VAR SET UNIT																*/
/*                                                                          */
/* Description: Sets variable unit with check_bounds_only & user_unit FALSE*/
/*                                                                          */
/* Arguments:   VAR POINTER v			- variable address					*/
/*				INT unit				- input unit code					*/
/*				BOOL check_bounds_only  - check bounds only?                */
/*                                                                          */
/* Returns:     BOOL always TRUE                                            */
/*                                                                          */
/* Notes:       if check_bounds_only, this effectively does nothing.        */
/*                                                                          */
/****************************************************************************/
BOOL VAR_Set_Unit(VAR *v, int unit, BOOL check_bounds_only)
{
	if (check_bounds_only)
		return TRUE;

	v->unit = unit;

	VAR_Update(v, v->base_val, 0);

	return TRUE;
}

/****************************************************************************/
/* VAR SET UNIT PARAM														*/
/*                                                                          */
/* Description: Sets unit parameters                                        */
/*                                                                          */
/* Arguments:   VAR POINTER v 	- variable address                          */
/* 				UNSIGNED INT p 	- parameter									*/
/*				FLOAT val 		- value										*/
/*				INT type 		- type (see notes)							*/
/*				BOOL user_unit - user units?								*/
/*                                                                          */
/* Returns:     BOOL always TRUE                                            */
/*                                                                          */
/* Notes:       type -  0 = no scale										*/
/*						1 = int scale										*/
/*						x = long int scale                                  */
/*                                                                          */
/****************************************************************************/
BOOL VAR_Set_Unit_Param(VAR *v, unsigned int p, float val, int type, BOOL user_unit)
{
	double val1;	/* scaled value 	*/

//	_GIEP;

	if (type==0)
		val1 = val;
	else if (type==1)
		val1 = val / v->scale;
	else
		val1 = val / v->scale_long;

	val1 = Convert(v->class, v->unit, v->calc_unit, val1, 0, v->aux);

    switch (p)
    {/* select variable to update */
		case reg_direct_bmax:
		{
			v->bound_hi_set = val1;

			if ((v->STAT & var_round))
				v->bound_hi_set = Round_N(v->bound_hi_set,0);

			break;
		}
		case reg_direct_bmin:
		{
			v->bound_lo_set = val1;

			if ((v->STAT & var_round))
		 		v->bound_lo_set = Round_N(v->bound_lo_set,0);

			break;
		}
		case reg_direct_alarm_hi:
		{
		 	v->alarm_hi_set = val1;

			if ((v->STAT & var_round))
				v->alarm_hi_set = Round_N(v->alarm_hi_set,0);

			break;
		}
		case reg_direct_alarm_lo:
		{
		 	v->alarm_lo_set = val1;

			if ((v->STAT & var_round))
		 		v->alarm_lo_set = Round_N(v->alarm_lo_set,0);

			break;
		}
    }

//	GIEP;
	return TRUE;
}

/****************************************************************************/
/* VAR GET UNIT PARAM														*/
/*                                                                          */
/* Description: Gets converted unit parameter                               */
/*                                                                          */
/* Arguments:   VAR POINTER v	- variable									*/
/*				UNSIGNED INT p	- unit parameter							*/
/*				INT type		- unit scale								*/
/*				BOOL user_unit - user units ? v->unit : v->calc_unit       */
/*                                                                          */
/* Returns:     FLOAT a - converted unit value                              */
/*                                                                          */
/* Notes:       type -  0 = no scale										*/
/*						1 = int scale										*/
/*						x = long int scale                                  */
/*                                                                          */
/****************************************************************************/
float VAR_Get_Unit_Param(VAR *v, unsigned int p, int type, BOOL user_unit)
{
	double a;	/* converted unit value */
//	int	uu;			/* user unit value 		*/
//
//	/* choose unit or calc_unit */
//	if (user_unit)
//		uu = v->unit;
//	else
//		uu = v->calc_unit;

    switch (p)
    {/* choose which value to convert */
		case reg_direct_bmax:
		{/* bound_hi_set */
			a = Convert(v->class, v->calc_unit, v->unit, v->bound_hi_set, 0, v->aux);

			break;
		}
		case reg_direct_bmin:
		{/* bound_lo_set */
			a = Convert(v->class, v->calc_unit, v->unit, v->bound_lo_set, 0, v->aux);

			break;
		}
		case reg_direct_alarm_hi:
		{/* alarm_hi_set */
			a = Convert(v->class, v->calc_unit, v->unit, v->alarm_hi_set, 0, v->aux);

			break;
		}
		case reg_direct_alarm_lo:
		{/* alarm_lo_set */
			a = Convert(v->class, v->calc_unit, v->unit, v->alarm_lo_set, 0, v->aux);

			break;
		}
    }

	/* return based on type */
	if (type==0)
		return (float)(a);
	else if (type==1)
		return (float)(a*v->scale);
	else
		return (float)(a*v->scale_long);
}

/****************************************************************************/
/* VAR NAN																	*/
/*                                                                          */
/* Description: Sets variable to NAN state.                                 */
/*                                                                          */
/* Arguments:   VARIABLE POINTER v - variable                               */
/*                                                                          */
/* Returns:     void                                                        */
/*                                                                          */
/* Notes:       NAN - Not A Number                                          */
/*                                                                          */
/****************************************************************************/
void VAR_NaN(VAR *v)
{
//	_GIEP;

	if (!(v->STAT&var_NaNproof))
		v->STAT |= var_NaNum;
	else
		v->STAT &= var_NaNum ^ 0xFFFFFFFF;

	v->STAT |= var_alarm_lo;
	v->STAT |= var_alarm_hi;
	v->STAT |= var_bound_lo;
	v->STAT |= var_bound_hi;
	v->STAT &= var_aux ^ 0xFFFFFFFF;

	//if (INITIALIZING || (!(v->STAT & var_NaNproof)))
	if  (!(v->STAT & var_NaNproof))
	{/* if initializing OR variable can be NAN */
//		v->val1     = 0.0; // NaN_val;
//		v->val2     = 0.0; // NaN_val;
		v->val      = 0.0; // NaN_val;
		v->calc_val = 0.0; // NaN_val;
		v->base_val = 0.0; // NaN_val;
	}

//	GIEP;
}


/****************************************************************************/
/* F COMPARE																*/
/*                                                                          */
/* Description: compares float s to r +/- TOLERANCE                               */
/*                                                                          */
/* Arguments:	FLOAT POINTER s - compared value							*/
/*			    FLOAT r			- comparison value							*/
/*				BOOL reset      - compared = comparison						*/
/*                                                                          */
/* Returns:     INT return value - < -1										*/
/*								   >  1										*/
/*								   =  0										*/
/*                                                                          */
/* Notes:		Don't know what TOLERANCE is.                                     */
/*                                                                          */
/****************************************************************************/
int f_compare(float* s, float r, BOOL reset)
{
	if (s[0]<(r-TOLERANCE))
		return -1;
	else if (s[0]>(r+TOLERANCE))
		return 1;
	else
	{
		if (reset)
			s[0] = r;	/* force to compared value */

		return 0;
	}
}

/****************************************************************************/
/* L COMPARE																*/
/*                                                                          */
/* Description: compares double s to r +/- TOLERANCE                        */
/*                                                                          */
/* Arguments:   double POINTER s	- compared value						*/
/*				double r 			- comparison value						*/
/*				BOOL reset				- compared = comparison             */
/*                                                                          */
/* Returns:     INT return value - 	< -1									*/
/*									>  1									*/
/*									=  0                                    */
/*                                                                          */
/* Notes:       Don't know what TOLERANCE is. //TOLERANCE is tolerance      */
/*                                                                          */
/****************************************************************************/
int l_compare(double* s, double r, BOOL reset)
{
	if (s[0]<(r-TOLERANCE))
		return -1;
	else if (s[0]>(r+TOLERANCE))
		return 1;
	else
	{
		if (reset)
			s[0] = r;	/* force to compared value */

		return 0;
	}
}

/****************************************************************************/
/* VAR CHECKSET BOUNDS														*/
/*                                                                          */
/* Description: Checks the value to see if it's within bounds               */
/*                                                                          */
/* Arguments:   VAR POINTER v 		- variable								*/
/*				double POINTER t 	- value to check                    	*/
/*                                                                          */
/* Returns:     BOOL r - T no error, F bounds error                    		*/
/*                                                                          */
/* Notes:       Errors are set here if there is an out of bounds or alarm   */
/*                                                                          */
/****************************************************************************/
BOOL VAR_CheckSet_Bounds(VAR* v, double* t)
{
	BOOL r; /* return value */

    r = TRUE;

    if ((v->STAT & var_no_alarm)==0)
    {/* alarms/bounds are based off calc value - check and set alarms */
        if (l_compare(t, v->alarm_hi_set, 1)>0)
        {/* t > alarm_hi */
            v->STAT      &= var_alarm_lo ^ 0xFFFFFFFF;
            v->STAT      |= var_alarm_hi;
//          DIO_ALARM.val = TRUE;
        }
        else if (l_compare(t, v->alarm_lo_set, 1)<0)
        {/* t < alarm_lo */
            v->STAT      &= var_alarm_hi ^ 0xFFFFFFFF;
            v->STAT      |= var_alarm_lo;
//          DIO_ALARM.val = TRUE;
        }
        else
        {/* t within range */
            v->STAT &= var_alarm_lo ^ 0xFFFFFFFF;
            v->STAT &= var_alarm_hi ^ 0xFFFFFFFF;
        }
    }
    else
    {/* alarms disabled */
        v->STAT &= var_alarm_lo ^ 0xFFFFFFFF;
        v->STAT &= var_alarm_hi ^ 0xFFFFFFFF;
    }

    if ((v->STAT & var_roll)==var_roll)
    {/* var_roll is enabled */
        v->STAT &= var_bound_lo ^ 0xFFFFFFFF;
        v->STAT &= var_bound_hi ^ 0xFFFFFFFF;
        t[0]     =  fmod(t[0], v->bound_hi_set);

        if (t[0] < 0.0)
            t[0] = 0.0;
    }
    else if ((v->STAT & var_no_bound)==0)
    {/* bounds are enabled - check and set bounds */
        if (l_compare(t, v->bound_hi_set, 1)>0)
        {/* t > bound_hi */
            v->STAT &= var_bound_lo ^ 0xFFFFFFFF;
            v->STAT |= var_bound_hi;
            t[0]     = v->bound_hi_set;

            if ((v->STAT & var_no_alarm)==0)
			{
            	if ((DIAGNOSTICS & ERR_VAR_HI) == 0) DIAGNOSTICS |= ERR_VAR_HI;
			}

            r = FALSE;
        }
        else if (l_compare(t, v->bound_lo_set, 1)<0)
        {/* t < bound_lo */
            v->STAT &= var_bound_hi ^ 0xFFFFFFFF;
            v->STAT |= var_bound_lo;
            t[0]     = v->bound_lo_set;

			if ((v->STAT & var_no_alarm)==0)
			{
            	if ((DIAGNOSTICS & ERR_VAR_LO) == 0) DIAGNOSTICS |= ERR_VAR_LO;
			}

            r = FALSE;
        }
        else
        {/* t within range */
            v->STAT &= var_bound_lo ^ 0xFFFFFFFF;
            v->STAT &= var_bound_hi ^ 0xFFFFFFFF;
           	if (DIAGNOSTICS & ERR_VAR_LO) DIAGNOSTICS &= ~ERR_VAR_LO;
           	if (DIAGNOSTICS & ERR_VAR_HI) DIAGNOSTICS &= ~ERR_VAR_HI;
        }
    }
    else
    {/* var_roll and bounds are disabled */
        v->STAT &= var_bound_lo ^ 0xFFFFFFFF;
        v->STAT &= var_bound_hi ^ 0xFFFFFFFF;
    }

    v->calc_val = t[0];
//  v->val1     = v->calc_val;
    v->STAT    &= var_NaNum ^ 0xFFFFFFFF;

    return r;
}

/****************************************************************************/
/* VAR CHECK BOUNDS															*/
/*                                                                          */
/* Description: Checks if the value is in bounds.                           */
/*                                                                          */
/* Arguments:   VAR POINTER v		- variable								*/
/*				double POINTER t	- value to check           		        */
/*                                                                          */
/* Returns:     BOOL r - T no error, F error                                */
/*                                                                          */
/* Notes:       Only checks the bounds, does not set STAT                   */
/*                                                                          */
/****************************************************************************/
BOOL VAR_Check_Bounds(VAR* v, double* t)
{
	if ((v->STAT & var_no_bound)==0)
	{/* Check bounds enabled */
		if (l_compare(t, v->bound_hi_set, 1)>0)
			return FALSE;	/* t > bound_hi */
		else
		{ 
			if (l_compare(t, v->bound_lo_set, 1)<0)
				return FALSE;	/* t < bound_lo */
		}
	}

	return TRUE;
}

/****************************************************************************/
/* GET UNIT COEFF															*/
/*                                                                          */
/* Description: Gets the coefficients for the unit during conversion.       */
/*                                                                          */
/* Arguments:   VAR POINTER v		  - Variable							*/
/*				INT unit			  - Variable unit code					*/
/*				INT class			  - Variable unit class					*/
/*				double POINTER m - multiplier								*/
/*				double POINTER b - offset                              		*/
/*                                                                          */
/* Returns:     BOOL T if found F if not                                    */
/*                                                                          */
/* Notes:       This is used in context of conversion.                      */
/*              MASTER_UNITS has the following format:						*/
/*				{ classname, 0, 0, - indicates class header					*/
/*                 unitname, m, b, - indicates unit                         */
/*                        0, 0, 0} - end of table                           */
/*              This is horribly inefficient.  It looks at every index :(   */
/*                                                                          */
/****************************************************************************/
BOOL Get_Unit_Coeff(VAR* v, int unit, int class, double* m, double* b)
{
	int i;	/* counter */

//	_GIEP;
	i = 0;

	while(1)
	{/* Go through the MASTER_UNITS vector - INT, 0, 0 = class header */
		if ((MASTER_UNITS[3*i+0]==0) && (MASTER_UNITS[3*i+1]==0) && (MASTER_UNITS[3*i+2]==0))
		{/* if at end of list */
			m[0] = 1.0;
			b[0] = 0.0;
//			GIEP;

			return FALSE;
		}
		else if ((MASTER_UNITS[3*i+0]==class) && (MASTER_UNITS[3*i+1]==0) && (MASTER_UNITS[3*i+2]==0))
		{/* found variable class */
			i++;
			while(1)
			{/* search MASTER_UNITS one by one until unit code is found */
				if (MASTER_UNITS[3*i+0]==(unit&0xFF))
				{/* found variable unit code */
					m[0] = MASTER_UNITS[3*i+1];
					b[0] = MASTER_UNITS[3*i+2];
//					GIEP;

					return TRUE;
				}
				else if ((MASTER_UNITS[3*i+1]==0) && (MASTER_UNITS[3*i+2]==0))
				{/* didn't find variable unit */
					m[0] = 1.0;
					b[0] = 0.0;
//					GIEP;

					return FALSE;
				}
				i++;
			}
		}
		i++;
	}

//	GIEP;

//	return FALSE;
}
/****************************************************************************/
/* GET PREV UNIT															*/
/*                                                                          */
/* Description: Gets previous unit                                          */
/*                                                                          */
/* Arguments:   INT class - unit class										*/
/*				INT unit  - unit                                            */
/*                                                                          */
/* Returns:     INT unit | prev - returns the unit or previous unit         */
/*                                                                          */
/* Notes:       This is confusing.                                          */
/*							^												*/
/*              Not confusing, just unnecessarily complex.	-DHA            */
/****************************************************************************/
int Get_Prev_Unit(int class, int unit)
{
	int i,j;
	int prev;
	const Uint16 *tbl_p;

	i = 0;

	tbl_p = MASTER_UNITS_STR;

	while(1)
	{
		if (tbl_p[9*i+0]==(c_none|0x100))
			return unit;
		else if (tbl_p[9*i+0]==(class|0x100))
		{/* found variable class */
			i++;
			j 	 = i;
			prev = unit;

			while (1)
			{/* scan to end of unit list within class to get last unit */
				if ((tbl_p[9*i+0]&0x100)==0x100)
					break;

				prev = tbl_p[9*i+0];
				i++;
			}

			i = j;

			while(1)
			{
				if (tbl_p[9*i+0]==(unit&0xFF))	/* found variable unit code */
					return prev;
				else if ((tbl_p[9*i+0]&0x100)==0x100)
					return unit;

				prev = tbl_p[9*i+0];
				i++;
			}
		}
		i++;
	}
}

/****************************************************************************/
/* GET NEXT UNIT															*/
/*                                                                          */
/* Description: Gets next unit                                              */
/*                                                                          */
/* Arguments:   INT class - class code										*/
/*				INT unit  - unit code                                       */
/*                                                                          */
/* Returns:     INT unit, t or unit code									*/
/*				unit 		- if failed										*/
/*				t    		- default										*/
/*				unit code 	- found unit code                               */
/*                                                                          */
/* Notes:       As confusing as GET PREV UNIT                               */
/*                                                                          */
/****************************************************************************/
int Get_Next_Unit(int class, int unit)
{
	int i;	/* counter 			*/
	int t;	/* temp unit code	*/
	int f;	/* found unit code 	*/
	int fv;	/* found variable  	*/
	const Uint16 *tbl_p;

	i  = 0;
	f  = 0;
	fv = 0;


	tbl_p = MASTER_UNITS_STR;

	while(1)
	{
		if (tbl_p[9*i+0]==(c_none|0x100))
			return unit;
		else if (tbl_p[9*i+0]==(class|0x100))
		{/* found variable class */
			i++;

			while(1)
			{
				if (f==0)
				{
					f = 1;
					t = tbl_p[9*i+0]&0xFF;
				}

				if ((fv==0) && (tbl_p[9*i+0]==(unit&0xFF))) /* found variable unit code */
					fv = 1;
				else if ((tbl_p[9*i+0]&0x100)==0x100)
					return t;
				else if (fv==1)	/* found variable unit code */
					return (tbl_p[9*i+0]&0xFF);

				i++;
			}
		}
		i++;
	}
}

/****************************************************************************/
/* GET UNIT																	*/
/*                                                                          */
/* Description: Copies unit name to MASTER_UNITS_STR                        */
/*                                                                          */
/* Arguments:   INT class		 - class code								*/
/*				INT unit		 - unit code								*/
/*				CHAR POINTER str - string                                   */
/*                                                                          */
/* Returns:     BOOL TRUE - success; FALSE - failed                         */
/*                                                                          */
/* Notes:       none.                                                       */
/*                                                                          */
/****************************************************************************/
BOOL Get_Unit(int class, int unit, char* str)
{
	int i,j; /* counter */
	const Uint16 *tbl_p;
	const char*	tbl_pchar;

	for (i=0;i<8;i++)
		str[i] = 0;

	i = 0;

	tbl_p = MASTER_UNITS_STR;


	while(1)
	{
		if (tbl_p[9*i+0]==(c_none|0x100))
			return FALSE;
		else if (tbl_p[9*i+0]==(class|0x100))
		{/* found variable class */
			i++;

			while(1)
			{
			// NOTE: 	When the unit tables were created, the DSP had a
			// 			minimum addressable size of 16-bits. These 16-bit
			//			chars were exploited by using the MSB as flags.
			//			This code looks strange because it was made
			//			to be compatible with these legacy tables and
			//			functions. (strcpy won't work)


				if (tbl_p[9*i+0]==(unit&0xFF))
				{/* found variable unit code, copy name */
					//strcpy (str, (char*) &tbl_p[9*i+1]);
					tbl_pchar = (char*)&tbl_p[9*i+1];

					for (j=0;j<8;j++)
						str[j] = tbl_pchar[2*j]; //copy string, skipping the MSB of each "char"
					return TRUE;
				}
				else if ((tbl_p[9*i+0]&0x100)==0x100)
					return FALSE;

				i++;
			};
		}
		i++;
	}
}

BOOL Get_Unit_Clipped(int class, int unit, char* str, Uint8 total_length)
{
	BOOL is_success;

	is_success = Get_Unit(class,unit,str);

	if (total_length < 7) //clip string to specified length
		str[total_length] = 0;

	return is_success;
}

/****************************************************************************/
/* GET CLASS																*/
/*                                                                          */
/* Description: Copies class name to MASTER_UNITS_STR                       */
/*                                                                          */
/* Arguments:   INT class 		 - class code								*/
/*				CHAR POINTER str - string                                   */
/*                                                                          */
/* Returns:     BOOL TRUE - success; FALSE - failed                         */
/*                                                                          */
/* Notes:       none.                                                       */
/*                                                                          */
/****************************************************************************/
BOOL Get_Class(int class, char* str)
{
	int i; /* counter */
	const Uint16 *tbl_p;

	i 		= 0;
	str[0]  = 0;

	tbl_p = MASTER_UNITS_STR;

	while(1)
	{
		if (tbl_p[9*i+0]==(c_none|0x100))
			return FALSE;
		else if (tbl_p[9*i+0]==(class|0x100))
		{/* found variable class, copy string */
			strcpy (str, (char*) &tbl_p[9*i+1]);
			return TRUE;
		}
		i++;
	}
}

/****************************************************************************/
/* TIME SCALE FLOW															*/
/*                                                                          */
/* Description: Converts "in" value from "unit" to "flow_unit" then corrects*/
/*				for time unit.									            */
/*                                                                          */
/* Arguments:   double in  - input value								*/
/*				INT class		- class code								*/
/*				INT unit		- from unit									*/
/*				INT flow_unit	- to unit	                                */
/*                                                                          */
/* Returns:     double r - calculated return value                     */
/*                                                                          */
/* Notes:       This could easily be placed into a multi-dimensional 		*/
/*				table for lookup											*/
/*				Assume periodic update of 1sec and convert to unit's time 	*/
/*				scale                                            			*/
/*                                                                          */
/****************************************************************************/
double Time_Scale_Flow(double in, int class, int unit, int flow_unit)
{
	double r;
	int	u;

	r = 0.0;

	switch(class&0xFF)
	{/* mass or volume */
		case c_mass:
		{/* mass */
			switch(flow_unit&0xFF)
			{/* mass scale - find unit */
				case u_mfr_g_sec:
				case u_mfr_g_min:
				case u_mfr_g_hr:
				{
					u = u_m_g;

					break;
				}
				case u_mfr_metric_tons_min:
				case u_mfr_metric_tons_hr:
				case u_mfr_metric_tons_day:
				{
					u = u_m_metric_tons;

					break;
				}
				case u_mfr_lbs_sec:
				case u_mfr_lbs_min:
				case u_mfr_lbs_hr:
				case u_mfr_lbs_day:
				{
					u = u_m_lbs;

					break;
				}
				case u_mfr_short_tons_min:
				case u_mfr_short_tons_hr:
				case u_mfr_short_tons_day:
				{
					u = u_m_short_tons;

					break;
				}
				case u_mfr_long_tons_hr:
				case u_mfr_long_tons_day:
				{
					u = u_m_long_tons;

					break;
				}
				case u_mfr_kg_sec:
				case u_mfr_kg_min:
				case u_mfr_kg_hr:
				case u_mfr_kg_day:
				default:
				{
					u = u_m_kg;

					break;
				}
			}

			r = Convert(class, unit, u, in, 0, -1);	/* convert */

			switch(flow_unit&0xFF)
			{/* time scale - correct time unit */
				case u_mfr_metric_tons_day:
				case u_mfr_lbs_day:
				case u_mfr_short_tons_day:
				case u_mfr_long_tons_day:
				case u_mfr_kg_day:
				{
					r *= 86400.0;

					break;
				}
				case u_mfr_g_hr:
				case u_mfr_metric_tons_hr:
				case u_mfr_lbs_hr:
				case u_mfr_short_tons_hr:
				case u_mfr_long_tons_hr:
				case u_mfr_kg_hr:
				{
					r *= 3600.0;

					break;
				}
				case u_mfr_g_min:
				case u_mfr_metric_tons_min:
				case u_mfr_lbs_min:
				case u_mfr_short_tons_min:
				case u_mfr_kg_min:
				{
					r *= 60.0;

					break;
				}
				case u_mfr_g_sec:
				case u_mfr_lbs_sec:
				case u_mfr_kg_sec:
				default:
					break;
			}

			break;
		}
		case c_volume:
		{/* volume */
			switch(flow_unit&0xFF)
			{/* volume scale - find unit */
				case u_vfr_barrel_day:
				case u_vfr_barrel_hr:
				case u_vfr_barrel_min:
				case u_vfr_barrel_sec:
				{
					u = u_v_barrel;

					break;
				}
				case u_vfr_scf_min:
				{
					u = u_v_scf;

					break;
				}
				case u_vfr_cf_day:
				case u_vfr_cf_hr:
				case u_vfr_cf_min:
				case u_vfr_cf_sec:
				{
					u = u_v_cf;

					break;
				}
				case u_vfr_N_L_hr:
				{
					u = u_v_N_L;

					break;
				}
				case u_vfr_L_hr:
				case u_vfr_L_min:
				case u_vfr_L_sec:
				{
					u = u_v_L;

					break;
				}
				case u_vfr_N_cm_sec:
				case u_vfr_N_cm_hr:
				case u_vfr_N_cm_day:
				{
					u = u_v_N_cm;

					break;
				}
				case u_vfr_cm_day:
				case u_vfr_cm_hr:
				case u_vfr_cm_min:
				case u_vfr_cm_sec:
				{
					u = u_v_cm;

					break;
				}
				case u_vfr_imp_gal_day:
				case u_vfr_imp_gal_hr:
				case u_vfr_imp_gal_sec:
				case u_vfr_imp_gal_min:
				{
					u = u_v_imp_gal;

					break;
				}
				case u_vfr_gal_hr:
				case u_vfr_gal_day:
				case u_vfr_gal_min:
				case u_vfr_gal_sec:
				{
					u = u_v_gal;

					break;
				}
				case u_vfr_mscf_day:
				{
					u = u_v_mscf;

					break;
				}
				case u_vfr_scf_day:
				case u_vfr_scf_sec:
				{
					u = u_v_scf;

					break;
				}
				case u_vfr_Mgal_day:
				case u_vfr_ML_day:
				default:
				{
					u = 0;

					break;
				}
			}

			r = Convert(class, unit, u, in, 0, -1);	/* convert */

			switch(flow_unit&0xFF)
			{/* time scale - correct time unit */
				case u_vfr_barrel_day:
				case u_vfr_Mgal_day:
				case u_vfr_ML_day:
				case u_vfr_cf_day:
				case u_vfr_cm_day:
				case u_vfr_imp_gal_day:
				case u_vfr_gal_day:
				case u_vfr_mscf_day:
				case u_vfr_scf_day:
				{
					r *= 86400.0;

					break;
				}
				case u_vfr_cm_hr:
				case u_vfr_imp_gal_hr:
				case u_vfr_N_cm_hr:
				case u_vfr_N_L_hr:
				case u_vfr_cf_hr:
				case u_vfr_barrel_hr:
				case u_vfr_gal_hr:
				case u_vfr_L_hr:
				{
					r *= 3600.0;

					break;
				}
				case u_vfr_barrel_min:
				case u_vfr_cm_min:
				case u_vfr_cf_min:
				case u_vfr_gal_min:
				case u_vfr_L_min:
				case u_vfr_imp_gal_min:
				case u_vfr_scf_min:
				{
					r *= 60.0;

					break;
				}
				case u_vfr_gal_sec:
				case u_vfr_L_sec:
				case u_vfr_cf_sec:
				case u_vfr_cm_sec:
				case u_vfr_barrel_sec:
				case u_vfr_imp_gal_sec:
				default:
					break;
			}

			break;
		}
	}

	return r;
}

/****************************************************************************/
/* BREATOUT FLOW UNITS														*/
/*                                                                          */
/* Description: Takes in a flow unit and seperates the unit from time       */
/*                                                                          */
/* Arguments:   INT class			- class code							*/
/*				INT flow_unit		- unit source							*/
/*				INT POINTER units	- [unit code][time unit]				*/
/*				FLOAT POINTER r		- number of seconds                     */
/*                                                                          */
/* Returns:     void                                                        */
/*                                                                          */
/* Notes:       This could easily be placed into a multi-simensional table 	*/
/*				for lookup                                                  */
/*                                                                          */
/****************************************************************************/
void Breakout_Flow_Units(int class, int flow_unit, int* units, float* r)
{
	r[0] = 1.0;
	units[0] = u_generic_unknown;
	units[1] = u_generic_unknown;
	units[2] = c_unknown;

	switch(class&0xFF)
	{
		case c_mass_flow:
		{
			units[2] = c_mass;

			switch(flow_unit&0xFF)
			{/* mass scale */
				case u_mfr_g_sec:
				case u_mfr_g_min:
				case u_mfr_g_hr:
				{
					units[0] = u_m_g;

					break;
				}
				case u_mfr_metric_tons_min:
				case u_mfr_metric_tons_hr:
				case u_mfr_metric_tons_day:
				{
					units[0] = u_m_metric_tons;

					break;
				}
				case u_mfr_lbs_sec:
				case u_mfr_lbs_min:
				case u_mfr_lbs_hr:
				case u_mfr_lbs_day:
				{
					units[0] = u_m_lbs;

					break;
				}
				case u_mfr_short_tons_min:
				case u_mfr_short_tons_hr:
				case u_mfr_short_tons_day:
				{
					units[0] = u_m_short_tons;

					break;
				}
				case u_mfr_long_tons_hr:
				case u_mfr_long_tons_day:
				{
					units[0] = u_m_long_tons;

					break;
				}
				case u_mfr_kg_sec:
				case u_mfr_kg_min:
				case u_mfr_kg_hr:
				case u_mfr_kg_day:
				default:
				{
					units[0] = u_m_kg;

					break;
				}
			}

			switch(flow_unit&0xFF)
			{/* time scale */
				case u_mfr_metric_tons_day:
				case u_mfr_lbs_day:
				case u_mfr_short_tons_day:
				case u_mfr_long_tons_day:
				case u_mfr_kg_day:
				{
					units[1] = u_time_day;
					r[0] 	 = 86400.0;

					break;
				}
				case u_mfr_g_hr:
				case u_mfr_metric_tons_hr:
				case u_mfr_lbs_hr:
				case u_mfr_short_tons_hr:
				case u_mfr_long_tons_hr:
				case u_mfr_kg_hr:
				{
					units[1] = u_time_hr;
					r[0] 	 = 3600.0;

					break;
				}
				case u_mfr_g_min:
				case u_mfr_metric_tons_min:
				case u_mfr_lbs_min:
				case u_mfr_short_tons_min:
				case u_mfr_kg_min:
				{
					units[1] = u_time_min;
					r[0] 	 = 60.0;

					break;
				}
				case u_mfr_g_sec:
				case u_mfr_lbs_sec:
				case u_mfr_kg_sec:
				default:
				{
					units[1] = u_time_sec;
					r[0] 	 = 1.0;

					break;
				}
			}

			break;
		}
		case c_volumetric_flow:
		{
			units[2] = c_volume;

			switch(flow_unit&0xFF)
			{/* volume scale */
				case u_vfr_barrel_day:
				case u_vfr_barrel_hr:
				case u_vfr_barrel_min:
				case u_vfr_barrel_sec:
				{
					units[0] = u_v_barrel;

					break;
				}
				case u_vfr_scf_min:
				{
					units[0] = u_v_scf;

					break;
				}
				case u_vfr_cf_day:
				case u_vfr_cf_hr:
				case u_vfr_cf_min:
				case u_vfr_cf_sec:
				{
					units[0] = u_v_cf;

					break;
				}
				case u_vfr_N_L_hr:
				{
					units[0] = u_v_N_L;

					break;
				}
				case u_vfr_L_hr:
				case u_vfr_L_min:
				case u_vfr_L_sec:
				{
					units[0] = u_v_L;

					break;
				}
				case u_vfr_N_cm_sec:
				case u_vfr_N_cm_hr:
				case u_vfr_N_cm_day:
				{
					units[0] = u_v_N_cm;

					break;
				}
				case u_vfr_cm_day:
				case u_vfr_cm_hr:
				case u_vfr_cm_min:
				case u_vfr_cm_sec:
				{
					units[0] = u_v_cm;

					break;
				}
				case u_vfr_imp_gal_day:
				case u_vfr_imp_gal_hr:
				case u_vfr_imp_gal_sec:
				case u_vfr_imp_gal_min:
				{
					units[0] = u_v_imp_gal;

					break;
				}
				case u_vfr_gal_hr:
				case u_vfr_gal_day:
				case u_vfr_gal_min:
				case u_vfr_gal_sec:
				{
					units[0] = u_v_gal;

					break;
				}
				case u_vfr_mscf_day:
				{
					units[0] = u_v_mscf;

					break;
				}
				case u_vfr_scf_day:
				case u_vfr_scf_sec:
				{
					units[0] = u_v_scf;

					break;
				}
				case u_vfr_Mgal_day:
				case u_vfr_ML_day:
				default:
				{
					units[0] = 0;

					break;
				}
			}

			switch(flow_unit&0xFF)
			{/* time scale */
				case u_vfr_barrel_day:
				case u_vfr_Mgal_day:
				case u_vfr_ML_day:
				case u_vfr_cf_day:
				case u_vfr_cm_day:
				case u_vfr_imp_gal_day:
				case u_vfr_gal_day:
				case u_vfr_mscf_day:
				case u_vfr_scf_day:
				case u_vfr_N_cm_day:
				{
					units[1] = u_time_day;
					r[0] 	 = 86400.0;

					break;
				}
				case u_vfr_cm_hr:
				case u_vfr_imp_gal_hr:
				case u_vfr_N_cm_hr:
				case u_vfr_N_L_hr:
				case u_vfr_cf_hr:
				case u_vfr_barrel_hr:
				case u_vfr_gal_hr:
				case u_vfr_L_hr:
				{
					units[1] = u_time_hr;
					r[0] 	 = 3600.0;

					break;
				}
				case u_vfr_barrel_min:
				case u_vfr_cm_min:
				case u_vfr_cf_min:
				case u_vfr_gal_min:
				case u_vfr_L_min:
				case u_vfr_imp_gal_min:
				case u_vfr_scf_min:
				{
					units[1] = u_time_min;
					r[0] 	 = 60.0;

					break;
				}
				case u_vfr_gal_sec:
				case u_vfr_L_sec:
				case u_vfr_cf_sec:
				case u_vfr_cm_sec:
				case u_vfr_barrel_sec:
				case u_vfr_imp_gal_sec:
				case u_vfr_scf_sec:
				case u_vfr_N_cm_sec:
				default:
				{
					units[1] = u_time_sec;
					r[0] 	 = 1.0;

					break;
				}
			}

			break;
		}
	}
}

/****************************************************************************/
/* JOIN MASSTIME UNITS														*/
/*                                                                          */
/* Description: Takes in two unit codes and returns the joined unit code    */
/*                                                                          */
/* Arguments:   INT m - mass unit code										*/
/*				INT t - time unit code                                      */
/*                                                                          */
/* Returns:     INT r - returns unit                                        */
/*                                                                          */
/* Notes:       This could easily be placed into a multi-simensional table 	*/
/*				for lookup                                                  */
/*                                                                          */
/****************************************************************************/
int Join_MassTime_Units(int m, int t)
{
	int r; /* return value */

	r = u_generic_unknown;

	switch(t&0xFF)
	{/* time unit */
		case u_time_day:
		{
			switch(m&0xFF)
			{/* mass unit */
				case u_m_metric_tons: 	r = u_mfr_metric_tons_day; 	break;
				case u_m_lbs: 			r = u_mfr_lbs_day; 			break;
				case u_m_short_tons: 	r = u_mfr_short_tons_day;	break;
				case u_m_long_tons: 	r = u_mfr_long_tons_day; 	break;
				case u_m_kg: 			r = u_mfr_kg_day; 			break;
			}

			break;
		}
		case u_time_hr:
		{
			switch(m&0xFF)
			{/* mass unit */
				case u_m_g: 			r = u_mfr_g_hr; 			break;
				case u_m_metric_tons: 	r = u_mfr_metric_tons_hr; 	break;
				case u_m_lbs: 			r = u_mfr_lbs_hr; 			break;
				case u_m_short_tons: 	r = u_mfr_short_tons_hr; 	break;
				case u_m_long_tons: 	r = u_mfr_long_tons_hr; 	break;
				case u_m_kg: 			r = u_mfr_kg_hr; 			break;
			}

			break;
		}
		case u_time_min:
		{
			switch(m&0xFF)
			{/* mass unit */
				case u_m_g: 			r = u_mfr_g_min; 			break;
				case u_m_metric_tons: 	r = u_mfr_metric_tons_min; 	break;
				case u_m_lbs: 			r = u_mfr_lbs_min; 			break;
				case u_m_short_tons: 	r = u_mfr_short_tons_min; 	break;
				case u_m_kg: 			r = u_mfr_kg_min; 			break;
			}

			break;
		}
		case u_time_sec:
		{
			switch(m&0xFF)
			{/* mass unit */
				case u_m_g: 	r = u_mfr_g_sec; 	break;
				case u_m_lbs: 	r = u_mfr_lbs_sec; 	break;
				case u_m_kg: 	r = u_mfr_kg_sec; 	break;
			}

			break;
		}
	}

	return r;
}



/****************************************************************************/
/* VAR COPY																	*/
/*                                                                          */
/* Description: Can copy NULL vairable to destination or normal variable.   */
/*                                                                          */
/* Arguments:   VAR POINTER s - source variable								*/
/*				VAR POINTER d - destination variable                        */
/*                                                                          */
/* Returns:     void                                                        */
/*                                                                          */
/* Notes:       none                                                        */
/*                                                                          */
/****************************************************************************/
void VAR_Copy(VAR *s, VAR* d)
{
	if (s==(VAR*)0)	
	{	/* empty/NULL variable */
		d->val 			= 0;
		d->STAT 		= 0;
		d->aux 			= 0;
		d->calc_val 	= 0;
		d->base_val 	= 0;
		d->unit 		= u_generic_none;
		d->calc_unit 	= u_generic_none;
		d->class 		= c_not_classified;
		d->scale 		= 10;
		d->scale_long 	= 1000;
		d->bound_hi_set =  1000000000.0;
		d->bound_lo_set = -1000000000.0;
		d->alarm_hi_set =  1000000000.0;
		d->alarm_lo_set = -1000000000.0;
	}
	else
	{	/* Regular variable */
		d->val 			= s->val;
		d->STAT 		= s->STAT;
		d->aux 			= s->aux;
		d->calc_val 	= s->calc_val;
		d->base_val 	= s->base_val;
		d->unit 		= s->unit;
		d->calc_unit 	= s->calc_unit;
		d->class 		= s->class;
		d->scale 		= s->scale;
		d->scale_long 	= s->scale_long;
		d->bound_hi_set = s->bound_hi_set;
		d->bound_lo_set = s->bound_lo_set;
		d->alarm_hi_set = s->alarm_hi_set;
		d->alarm_lo_set = s->alarm_lo_set;
	}
}


