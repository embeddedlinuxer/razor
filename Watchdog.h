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
* Watchdog.h
*-------------------------------------------------------------------------
* Razor watchdog is setup in Main.c and the watchdog timer is reinitialized 
* in the while loop in Menu.c. If, for any reasons, watchdog failed to 
* receive feedback, then it will do further reset.
*-------------------------------------------------------------------------
* HISTORY:
*    Jun-18-2020 : Daniel Koh : David's watchdog was all *FAKE*. I had to recreate watchdog all again. 
*------------------------------------------------------------------------*/

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <ti/csl/csl_types.h>
#include <ti/csl/src/ip/wd_timer/V0/wd_timer.h>
#include <ti/csl/src/ip/wd_timer/V0/hw_wd_timer.h>
#include <ti/csl/csl_wdt.h>
#include <ti/csl/csl_wd_timer.h>
#include <ti/csl/cslr_wd_timer.h> 
#include <ti/csl/soc.h>

///////////////////////////////////////////////////
///////////////////////////////////////////////////
/// macros used in watchdog functions
///////////////////////////////////////////////////
///////////////////////////////////////////////////

#define TMR_CFG_64BIT_WATCHDOG      (0x0000001Bu)
//#define TMR_PERIOD_LSB32          (0x06FFFFFF)
#define TMR_PERIOD_LSB32            (0x0DFFFFFE)
#define TMR_PERIOD_MSB32            (0x0)
#define TMR_PRD12                   (0x18)
#define TMR_PRD34                   (0x1C)
#define TMR_TIMER12                 (0x00003FFEu)
#define TMR_TIMER34                 (0x3FFE0000u)
#define TMR_TCR                     (0x20)
#define TMR_TCR_CLKSRC12            (0x00000100u)
#define TMR_TCR_CLKSRC34            (0x01000000u)
#define TMR_TIM12                   (0x10)
#define TMR_TIM34                   (0x14)
#define TMR_TGCR                    (0x24)
#define TMR_TGCR_PLUSEN             (0x00000010u)
#define TMR_TGCR_TIM34RS            (0x00000002u)
#define TMR_TGCR_TIM12RS            (0x00000001u)
#define TMR_TGCR_TIMMODE            (0x0000000Cu)
#define TMR_TGCR_TIM34RS            (0x00000002u)
#define TMR_TGCR_TIM12RS            (0x00000001u)
#define TMR_WDTCR                   (0x28)
#define TMR_WDTCR_WDEN              (0x00004000u)
#define TMR_WDTCR_WDFLAG            (0x00008000u)
#define WDT_KEY_PRE_ACTIVE          (0xA5C6u)
#define WDT_KEY_ACTIVE              (0xDA7Eu)
#define CMP_IDX_MASK                (0x07)
#define PRESCALE_MASK               (0x0F)
#define TDDR_MASK                   (0x0F)
#define TMR_WDTCR_WDKEY             (0xFFFF0000u)
#define TMR_WDTCR_WDKEY_SHIFT       (0x00000010u)

///////////////////////////////////////////////////
///////////////////////////////////////////////////
/// function definition
///////////////////////////////////////////////////
///////////////////////////////////////////////////

void TimerWatchdogReactivate(unsigned int baseAddr);
void TimerWatchdogActivate(unsigned int baseAddr);
void TimerPeriodSet(unsigned int baseAddr, unsigned int timer, unsigned int period);
void TimerConfigure(unsigned int baseAddr, unsigned int config);
void setupwatchdog(void);

#endif
