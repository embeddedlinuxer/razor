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
* Main.c
*-------------------------------------------------------------------------
* HISTORY:
*       Jun-18-2020 : Daniel Koh : David's watchdog was all *FAKE* 
*                                  Hence, created a whole new watchdog.
*------------------------------------------------------------------------*/

#include "Watchdog.h"
#include "Globals.h"

void TimerWatchdogReactivate(unsigned int baseAddr)
{
    /* Write the first keys in the order. This order shall not change */
    HWREG(baseAddr + TMR_WDTCR) = ((HWREG(baseAddr + TMR_WDTCR) & (~TMR_WDTCR_WDKEY)) |(WDT_KEY_PRE_ACTIVE << TMR_WDTCR_WDKEY_SHIFT));
    HWREG(baseAddr + TMR_WDTCR) = ((HWREG(baseAddr + TMR_WDTCR) & (~TMR_WDTCR_WDKEY)) |(WDT_KEY_ACTIVE << TMR_WDTCR_WDKEY_SHIFT));
}

void TimerWatchdogActivate(unsigned int baseAddr)
{
    /* Enable the watchdog timer. Write the keys in the order */
    HWREG(baseAddr + TMR_WDTCR) = ((TMR_WDTCR_WDEN | TMR_WDTCR_WDFLAG) |(WDT_KEY_PRE_ACTIVE << TMR_WDTCR_WDKEY_SHIFT));
    HWREG(baseAddr + TMR_WDTCR) = ((HWREG(baseAddr + TMR_WDTCR) & (~TMR_WDTCR_WDKEY)) | (WDT_KEY_ACTIVE << TMR_WDTCR_WDKEY_SHIFT));
}

void TimerPeriodSet(unsigned int baseAddr, unsigned int timer, unsigned int period)
{
    if (TMR_TIMER12 & timer) HWREG(baseAddr + TMR_PRD12) = period; /* Write the period for Timer12 */
    if (TMR_TIMER34 & timer) HWREG(baseAddr + TMR_PRD34) = period; /* Write the period for Timer34 */
}

void TimerConfigure(unsigned int baseAddr, unsigned int config)
{
    /*
    ** Set the timer control register. This will only affect the clock
    ** selection bits. All other fields will be reset and the timer counting
    ** will be disabled.
    */
    HWREG(baseAddr + TMR_TCR) = (config & (TMR_TCR_CLKSRC12 | TMR_TCR_CLKSRC34));

    /* Clear the Timer Counters */
    HWREG(baseAddr + TMR_TIM12) = 0x0;
    HWREG(baseAddr + TMR_TIM34) = 0x0;

    /* Clear the TIMMODE bits and Reset bits */
    HWREG(baseAddr + TMR_TGCR) &= ~( TMR_TGCR_TIMMODE | TMR_TGCR_TIM34RS | TMR_TGCR_TIM12RS);

    /*
    ** Select the timer mode and disable the timer module from Reset
    ** Timer Plus features are enabled.
    */
    HWREG(baseAddr + TMR_TGCR) |= (config & 
                                   (TMR_TGCR_TIMMODE | TMR_TGCR_TIM34RS |
                                    TMR_TGCR_TIM12RS | TMR_TGCR_PLUSEN));
}

void setupWatchDog(void)
{
    /* Configuration of Timer */
    TimerConfigure(CSL_TMR_1_REGS, TMR_CFG_64BIT_WATCHDOG);

    /* Set the 64 bit timer period */
    TimerPeriodSet(CSL_TMR_1_REGS, TMR_TIMER12, TMR_PERIOD_LSB32);
    TimerPeriodSet(CSL_TMR_1_REGS, TMR_TIMER34, TMR_PERIOD_MSB32);

    /* Activate Watchdog */
    TimerWatchdogActivate(CSL_TMR_1_REGS);

    /// enable watchdog timer trigger
    isWatchDogEnabled = TRUE;
}
