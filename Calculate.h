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
* Calculate.h
*-------------------------------------------------------------------------
* Most of the Razor's measurements are interpreted and calculated in this
* code. Readings are updated in Capture_Sample() once a second, and runs
* in the context of a priority 15 SWI. However the pulse count corresponding
* to oscillator frequency is measured by Count_Freq_Pulses().
* Count_Freq_Pulses() is called twice per second by the Timer module,
* so the code is run in the context of a HWI (a real-time constraint in
* measuring high frequencies). We read in the number of pulses counted by
* the counter of hardware timer 1 and divide by the sample period of 500ms
* (and multiply back the 80x frequency divider) to arrive at our oscillator
* frequency. This frequency, along with many other values and measurements,
* is used to calculate the watercut.
*-------------------------------------------------------------------------
* HISTORY:
*       Apr-13-2018 : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/
#ifndef _CALCULATE
#define _CALCULATE

#ifdef CALCULATE_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

//#define DEMO_MODE
//#define DEMO_MODE_FREQ
//#define DEMO_MODE_TEMP

// used before finally updating raw watercut value
_EXTERN float NEW_WATERCUT_RAW;
_EXTERN float Interpolate(float w1, float t1, float w2, float t2, float t);
_EXTERN void Poll(void);
_EXTERN void getAverageTemperature(void);
_EXTERN void getAverageFrequency(void);
_EXTERN void getAverageRP(void);
_EXTERN void getOilSamples(void);
_EXTERN void Update_Demo_Values(void);
_EXTERN void Bfr_Add(FP_BFR* bfr, double val);
_EXTERN Uint8 Calculate_WC(float *WC);
_EXTERN Uint8 Apply_Density_Correction(void);
_EXTERN Uint8 Update_Freq(void);
_EXTERN inline void Init_Data_Buffer(void);

#undef _EXTERN
#undef CALCULATE_H
#endif // _CALCULATE
