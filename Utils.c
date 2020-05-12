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
* Util.c
*-------------------------------------------------------------------------
* Various utils. 
*-------------------------------------------------------------------------
* HISTORY:
*       Aug-29-2018 : Daniel Koh : Added.
*------------------------------------------------------------------------*/
#include "Globals.h"
#include "Security.h"
#include "nandwriter.h"
#include "Errors.h"
#include "Menu.h"
#include <assert.h>
#include <stdlib.h>
#include <time.h>

#define RSCTRL 0x01c110e8 // reset control register
#define UTIL_H
#include "Utils.h"

void
cleanDisplay(void)
{
	memset(lcdLine0, 0x20, sizeof(lcdLine0));
    memset(lcdLine1, 0x20, sizeof(lcdLine1));
    displayLcd(lcdLine0, LCD0);
    displayLcd(lcdLine1, LCD1);
}

void 
getIndex(Uint8 * i, int reg)
{
    isUpdateDisplay = FALSE;
	*i = reg;
}

void 
updateIndex(Uint8 * i)
{
    isUpdateDisplay = FALSE;
	*i = 0;
}


int
countBlinkTimes(char * text0, char * text1)
{
    static int i = 0;

    (i < 3) ? displayLcd(text1, LCD1) : displayLcd(text0, LCD1);

    if (i > 6) 
    {
        i = 0;
        return 1;
    }
    else
    {
        i++; 
        return 0;
    }
}


void 
updateDisplay(const char * text0, const char * text1)
{
    displayLcd(text0, LCD0);
	displayLcd(text1, LCD1);

	isUpdateDisplay = FALSE;
    counter = 0;
	LCD_setcursor(0,0);
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////
/////
/////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void
diagnose(Uint8 * ii, Uint8 * iindex, Uint8 * ierrorCount, Uint8 ierrors[], int * iDIAGNOSTICS_PREV)
{
    if (DIAGNOSTICS != *iDIAGNOSTICS_PREV)    // Keep updating DIAGNOSTICS
    {
        *iDIAGNOSTICS_PREV = DIAGNOSTICS;
        memset(ierrors,0,sizeof(ierrors));    // Clear array
        updateIndex(iindex);                  // Reset index = 0;
        *ierrorCount = 0;
        *ii = 0;

        while (1)
        {
            if (DIAGNOSTICS & (1 << *iindex))
            {
                ierrors[*ierrorCount] = *iindex;
                (*ierrorCount)++;
            }

            if (*iindex > MAX_ERRORS-1) break;
            else (*iindex)++;
                
        }
    }
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////
///// This fxn will be triggered upon switching stream.
/////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void
saveStreamData(void)
{
    int STREAM = (int)REG_STREAM.calc_val;

    // SAVE CURRENT REGISTER DATA TO STREAM REGISTER 
    STREAM_OIL_ADJUST[STREAM-1] = REG_OIL_ADJUST.calc_val;
}

void
getStreamData(void)
{
    int STREAM = (int)REG_STREAM.calc_val;

    // GET STREAM REGISTER DATA TO CURRENT REGISTER 
    VAR_Update(&REG_OIL_ADJUST, STREAM_OIL_ADJUST[STREAM-1], CALC_UNIT);
}
