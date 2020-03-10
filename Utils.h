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
* Util.h
*-------------------------------------------------------------------------
* Various utils
*-------------------------------------------------------------------------
* HISTORY:
*       Aug-29-2018 : Daniel Koh : Added.
*------------------------------------------------------------------------*/

#ifndef _UTIL_H
#define _UTIL_H

#ifdef UTIL_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

_EXTERN void diagnose(Uint8 * ii, Uint8 * iindex, Uint8 * ierrorCount, Uint8 ierrors[], int * iDIAGNOSTICS_PREV);
//_EXTERN void blinkLcdLine1(char * text0, char * text1);
_EXTERN int countBlinkTimes(char * text0, char * text1);
_EXTERN void cleanDisplay(void);
_EXTERN void cancelAndReset(void);
_EXTERN void updateDisplay(const char * text0, const char * text1);
_EXTERN void updateIndex(Uint8 * i);
_EXTERN void saveStreamData(void);
_EXTERN void getStreamData(void);
_EXTERN void writeOilAdjustStreamValue(const int STREAM, const double value);
#undef _EXTERN
#undef UTIL_H
#endif 

