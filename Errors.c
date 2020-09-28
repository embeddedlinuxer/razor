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
* Errors.c 
*-------------------------------------------------------------------------
* Error handling
*-------------------------------------------------------------------------
* HISTORY:
*   	?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#define ERRORS_H
#include "Globals.h"

/*
void checkErrors(void)
{
	/// FREQUENCY
	if ((REG_FREQ.calc_val > 1000) || (REG_FREQ.calc_val < 0) || (FREQ_PULSE_COUNT_HI != 0) || (FREQ_U_SEC_ELAPSED == 0) || (REG_FREQ.calc_val == NAN))
    {
        if ((REG_FREQ.calc_val > 1000) || (FREQ_PULSE_COUNT_HI != 0))
        {
            if (DIAGNOSTICS & ERR_FRQ_HI) {}
            else DIAGNOSTICS |= ERR_FRQ_HI;
            if (DIAGNOSTICS & ERR_FRQ_LO) DIAGNOSTICS &= ~ERR_FRQ_LO;
        }
        else if ((REG_FREQ.calc_val < 0) || (REG_FREQ.calc_val == NAN) || (FREQ_U_SEC_ELAPSED == 0))
        {
            if (DIAGNOSTICS & ERR_FRQ_LO) {}
            else DIAGNOSTICS |= ERR_FRQ_LO;
            if (DIAGNOSTICS & ERR_FRQ_HI)   DIAGNOSTICS &= ~ERR_FRQ_HI;
        }
    }
    else
    {
        if (DIAGNOSTICS & ERR_FRQ_LO) DIAGNOSTICS &= ~ERR_FRQ_LO;
        if (DIAGNOSTICS & ERR_FRQ_HI) DIAGNOSTICS &= ~ERR_FRQ_HI;
    }

	/// TEMPERATURE
	if ((REG_TEMP_USER.calc_val > 120) || (REG_TEMP_USER.calc_val < -20))
    {
        if (REG_TEMP_USER.calc_val > 120)
        {
            if (DIAGNOSTICS & ERR_TMP_HI) {}
            else DIAGNOSTICS |= ERR_TMP_HI;
            if (DIAGNOSTICS & ERR_TMP_LO) DIAGNOSTICS &= ~ERR_TMP_LO;
        }
        else if (REG_TEMP_USER.calc_val < -20)
        {
            if (DIAGNOSTICS & ERR_TMP_LO) {}
            else DIAGNOSTICS |= ERR_TMP_LO;
            if (DIAGNOSTICS & ERR_TMP_HI) DIAGNOSTICS &= ~ERR_TMP_HI;
        }
    }
    else
    {
        if (DIAGNOSTICS & ERR_TMP_LO) DIAGNOSTICS &= ~ERR_TMP_LO;
        if (DIAGNOSTICS & ERR_TMP_HI) DIAGNOSTICS &= ~ERR_TMP_HI;
    }
}
*/


void Update_Relays(void)
{
#ifndef HAPTIC_RELAY 

/*
////////////////////////////////////////////////////////////////////////////
// don't do this if we're using the relay as haptic feedback
////////////////////////////////////////////////////////////////////////////
	if (DIAGNOSTICS & DIAGNOSTICS_MASK) //non-masked error detected
	{
		if (COIL_RELAY[0] == FALSE)
		{
			COIL_RELAY[0] = 1;
			CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT,1);
		}
	}
	else if (COIL_RELAY[0] == TRUE)
	{
		COIL_RELAY[0] = 0;
		CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT,0);
	}
*/

	// CHECK RELAY CONDITION
	switch (REG_RELAY_MODE) 
	{
		case 0	: // Watercut
   			(REG_WATERCUT.calc_val > REG_RELAY_SETPOINT.calc_val) ? (COIL_RELAY[0].val = TRUE) : (COIL_RELAY[0].val = FALSE);
			break;
		case 1	: // Phase
   		 	if (COIL_ACT_RELAY_OIL.val)
       			(COIL_OIL_PHASE.val) ? (COIL_RELAY[0].val = TRUE) : (COIL_RELAY[0].val = FALSE);
         	else 
            	(COIL_OIL_PHASE.val) ? (COIL_RELAY[0].val = FALSE) : (COIL_RELAY[0].val = TRUE);
			break;
		case 2	:  // Error
			(DIAGNOSTICS > 0) ? (COIL_RELAY[0].val = TRUE) : (COIL_RELAY[0].val = FALSE);
			break;
		case 3	: // Manual
       		(COIL_RELAY_MANUAL.val) ? (COIL_RELAY[0].val = TRUE) : (COIL_RELAY[0].val = FALSE);
			break;
		default	:
       		COIL_RELAY[0].val = FALSE;
			break;
	}
    
	// UPDATE RELAY PIN 
    if (COIL_RELAY[0].val)
    {    
    	delayTimer++;
        if ((delayTimer > REG_RELAY_DELAY) || (COIL_RELAY_MANUAL.val))
        {    
            CSL_FINS(gpioRegs->BANK[GP2].OUT_DATA, GPIO_OUT_DATA_OUT5, 1); // Relay ON
            delayTimer = 0; 
        }    
   	}    
    else 
	{
   		CSL_FINS(gpioRegs->BANK[GP2].OUT_DATA, GPIO_OUT_DATA_OUT5, 0);	// Relay OFF
		delayTimer = 0;
	}	

#endif
}
