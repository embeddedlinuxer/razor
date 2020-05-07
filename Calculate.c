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
* Calculate.c
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

#include "Globals.h"
#include "Utils.h"
#include "stdlib.h"
#define CALCULATE_H
#include "Calculate.h"
#include <limits.h>
#include <time.h>

static int CAL_RTC_SEC, CAL_RTC_MIN, CAL_RTC_HR, CAL_RTC_DAY, CAL_RTC_MON, CAL_RTC_YR;

// This is a __HWI__ called by Count_Freq_Pulses_Clock.
// Currently, it's called once every 0.5 seconds.
void Count_Freq_Pulses(Uint32 u_sec_elapsed)
{
    // disable counter
    CSL_FINST(tmr3Regs->TCR,TMR_TCR_ENAMODE12,DISABLE); 

    //stop counter timer
    Timer_stop(counterTimerHandle); 

    //Update global variables
    FREQ_PULSE_COUNT_LO = tmr3Regs->TIM12; //store counter value to global var (lower)
    FREQ_PULSE_COUNT_HI = tmr3Regs->TIM34; //store counter value to global var (upper)
    FREQ_U_SEC_ELAPSED = u_sec_elapsed;

    //reset counter value to zero (upper & lower)
    tmr3Regs->TIM12 = 0;
    tmr3Regs->TIM34 = 0;

    //re-enable counter
    CSL_FINST(tmr3Regs->TCR,TMR_TCR_ENAMODE12,EN_ONCE); 

    //start counter timer
    Timer_start(counterTimerHandle);

    // Post Swi_Poll below
    Swi_post(Swi_Poll);
}

void Poll(void)
{
    static float WC_AVG = 0;
    static int WC_PROC_AVGING = 1;
	float WC; 			// temp watercut value
	BOOL err_f = FALSE;	// frequency calculation error
	BOOL err_w = FALSE;	// watercut calculation error
	BOOL err_d = FALSE;	// density correction error

    static Uint8 sample_clk_started = FALSE;

    // update REG_DIAGNOSTICS
    REG_DIAGNOSTICS = DIAGNOSTICS;

    // update RTC
    Read_RTC(&CAL_RTC_SEC, &CAL_RTC_MIN, &CAL_RTC_HR, &CAL_RTC_DAY, &CAL_RTC_MON, &CAL_RTC_YR);

	// get frequency
	err_f = Update_Freq();	

	// get watercut 
	if (!err_f) err_w = Calculate_WC(&WC);

	// get density
	if ((REG_OIL_DENS_CORR_MODE != 0) && !(err_f | err_w) )
	{
		double dens;

		// get density from various sources 
		     if (REG_OIL_DENS_CORR_MODE == 1) VAR_Update(&REG_OIL_DENSITY, REG_OIL_DENSITY_AI, CALC_UNIT);
		else if (REG_OIL_DENS_CORR_MODE == 2) VAR_Update(&REG_OIL_DENSITY, REG_OIL_DENSITY_MODBUS, CALC_UNIT);
		else if (REG_OIL_DENS_CORR_MODE == 3) VAR_Update(&REG_OIL_DENSITY, REG_OIL_DENSITY_MANUAL, CALC_UNIT);

		// apply density adj
		if (REG_DENSITY_ADJ != 0.0)
		{
			dens = REG_OIL_DENSITY.base_val;
			dens = Convert(REG_OIL_DENSITY.class, REG_OIL_DENSITY.calc_unit, REG_OIL_DENSITY.unit, dens, 0, 0);
			dens += REG_DENSITY_ADJ;
			dens = Convert(REG_OIL_DENSITY.class, REG_OIL_DENSITY.unit, REG_OIL_DENSITY.calc_unit, dens, 0, 0);
			VAR_Update(&REG_OIL_DENSITY,dens,CALC_UNIT);
		}

		err_d = Apply_Density_Correction();

		if (!err_d)
		{
			// add adjustment from density correction (if any)
			WC += REG_WC_ADJ_DENS;

			// max oil-phase watercut
			if (WC > REG_OIL_CALC_MAX) WC = REG_OIL_CALC_MAX;	
		}
	}
	
	if((err_f | err_w | err_d) == FALSE)
	{
		COIL_AO_ALARM.val = FALSE;

        ///////// WATERCUT AVERAGING ///////////////////

        /*********************************************** 
            [Previous Average] * (n-1) + [Next Value]
            -----------------------------------------
                                n
        ************************************************/

        if (WC_PROC_AVGING > REG_PROC_AVGING.calc_val)
        {
            WC_PROC_AVGING = 1;
            WC_AVG= 0;
        }
        
        WC_AVG *= (WC_PROC_AVGING-1);
        WC_AVG += WC;
        WC_AVG /= WC_PROC_AVGING;
        WC_PROC_AVGING++;

        VAR_Update(&REG_WATERCUT,WC_AVG,CALC_UNIT);

        /////////////////////////////////////////////////

		VAR_Update(&REG_WATERCUT_RAW,NEW_WATERCUT_RAW,CALC_UNIT);

		// [MENU 2.4] calculate analog output value
		REG_AO_OUTPUT = (16*(REG_WATERCUT.val/100)) + 4; 
	} 
	else
	{
		COIL_AO_ALARM.val = TRUE;
		if ((!COIL_AO_MANUAL.val) && (REG_AO_ALARM_MODE != 0))
			(REG_AO_ALARM_MODE == 1) ? (REG_AO_MANUAL_VAL = 20.5) : (REG_AO_MANUAL_VAL = 3.6);

		VAR_NaN(&REG_WATERCUT);
		VAR_NaN(&REG_WATERCUT_RAW);
	}

    if(sample_clk_started == FALSE)
    {
        Clock_start(Capture_Sample_Clock);
        sample_clk_started = TRUE;
    }
}


Uint8 Update_Freq(void)
{
	int key;
	double freq;

	if (FREQ_PULSE_COUNT_HI != 0) // this probably shouldn't happen
	{
		DIAGNOSTICS |= ERR_FRQ_HI;
		return 1; // either freq too high or something went wrong in the counting
	}
	else DIAGNOSTICS &= ~ERR_FRQ_HI;
		
		
	if (FREQ_U_SEC_ELAPSED == 0) // don't divide by zero
	{
		DIAGNOSTICS |= ERR_FRQ_LO;
		return 1;
	}
	else DIAGNOSTICS &= ~ERR_FRQ_LO;

	key = Swi_disable();

	// #pulses divided by #microseconds
	freq = ((double)FREQ_PULSE_COUNT_LO) / ((double)FREQ_U_SEC_ELAPSED); 

	// oscillator board uses 80x divider
	freq *= 80;	

	//update frequency
	VAR_Update(&REG_FREQ, freq + REG_OIL_INDEX.calc_val, CALC_UNIT); 	

	Swi_restore(key);

	// error checking routine
	if (REG_FREQ.calc_val == NAN)
	{
		DIAGNOSTICS |= ERR_FRQ_LO;
		return 1;
	}
	else if ((REG_FREQ.calc_val < 0))
	{
		DIAGNOSTICS |= ERR_FRQ_LO;
		return 1;
	}
	else if ((REG_FREQ.calc_val > 1000))
	{
		DIAGNOSTICS |= ERR_FRQ_HI;
		return 1;
	}
	else
		DIAGNOSTICS &= ~(ERR_FRQ_HI | ERR_FRQ_LO);

	return 0; //no errors
}


Uint8 Calculate_WC(float *WC)
{
	float 	w;
	float	ot[2];
	Uint16	i,j;

	// initialize temporary raw watercut value
	NEW_WATERCUT_RAW = REG_WATERCUT_RAW.calc_val;

	// Convert temp_adj unit to Temp_extl calc_unit for calc_value
	//t_adj_calc = Convert(REG_TEMPERATURE.class, REG_TEMP_ADJUST.unit, REG_TEMPERATURE.calc_unit, REG_TEMP_ADJUST.val, 1, REG_TEMP_ADJUST.aux);
	//VAR_Update(&REG_TEMP_USER, REG_TEMPERATURE.calc_val + t_adj_calc, CALC_UNIT);

	VAR_Update(&REG_TEMP_USER, REG_TEMPERATURE.calc_val + REG_TEMP_ADJUST.calc_val + PDI_TEMP_ADJUST.calc_val, CALC_UNIT);
	//REG_OIL_PT = (REG_OIL_P1.calc_val * REG_FREQ.calc_val) + REG_OIL_P0.calc_val + (REG_OIL_T1.calc_val * REG_TEMP_USER.calc_val) + REG_OIL_T0.calc_val;
	REG_OIL_PT = (REG_OIL_P1.calc_val * REG_FREQ.calc_val) + REG_OIL_P0.calc_val;

	///////////////////////////////////////////////
	// Check Oil/Water Phase, if Water phase, watercut = 100% (always)
	///////////////////////////////////////////////

	cycles++;

	if ((REG_FREQ.calc_val < REG_OIL_FREQ_LOW.calc_val) || (REG_FREQ.calc_val > REG_OIL_FREQ_HIGH.calc_val) || (REG_OIL_RP > REG_OIL_PT)) phase = 0;
	else phase = 1;

	if (cycles == 1) previous_phase = phase;
		
    if (phase != previous_phase) phase_rollover_count++;
        
    if (cycles > REG_PHASE_HOLD_CYCLES)
    {
        cycles = 0;
        phase_rollover_count = 0;
    }

	if ((phase_rollover_count < 2) && (cycles == REG_PHASE_HOLD_CYCLES))
	{
    	if (REG_FREQ.calc_val < REG_OIL_FREQ_LOW.calc_val) 
            COIL_OIL_PHASE.val = FALSE;
		else 
        {
            if (REG_OIL_RP > REG_OIL_PT) 
                COIL_OIL_PHASE.val = TRUE;
            else            
                COIL_OIL_PHASE.val = FALSE;
        }
	}

	if (COIL_OIL_PHASE.val)
	{
		// Find the two data point our temperature falls between
		for (i=1;i<REG_TEMP_OIL_NUM_CURVES-3;i++)
			if ( REG_TEMPS_OIL[i] > REG_TEMP_USER.calc_val ) break;

		j = i-1;

		ot[0] = REG_COEFFS_TEMP_OIL[i][3]*REG_FREQ.calc_val*REG_FREQ.calc_val*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[i][2]*REG_FREQ.calc_val*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[i][1]*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[i][0];

		ot[1] = REG_COEFFS_TEMP_OIL[j][3]*REG_FREQ.calc_val*REG_FREQ.calc_val*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[j][2]*REG_FREQ.calc_val*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[j][1]*REG_FREQ.calc_val +
				REG_COEFFS_TEMP_OIL[j][0];

		w = Interpolate(ot[0], REG_TEMPS_OIL[i], ot[1], REG_TEMPS_OIL[j], REG_TEMP_USER.calc_val);

		// Note: As with the old code, we check for the cutoff using RAW watercut calculation
		if ((w > REG_OIL_PHASE_CUTOFF) && (REG_OIL_PHASE_CUTOFF > 0)) // use the second curve (i+3)     
		{
			for (i=1;i<REG_TEMP_OIL_NUM_CURVES-3;i++)
				if (REG_TEMPS_OIL[i+3] > REG_TEMP_USER.calc_val) break;

			j 	  = i-1;

			ot[0] = REG_COEFFS_TEMP_OIL[i+3][3]*REG_FREQ.calc_val*REG_FREQ.calc_val*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[i+3][2]*REG_FREQ.calc_val*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[i+3][1]*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[i+3][0];

			ot[1] = REG_COEFFS_TEMP_OIL[j+3][3]*REG_FREQ.calc_val*REG_FREQ.calc_val*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[j+3][2]*REG_FREQ.calc_val*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[j+3][1]*REG_FREQ.calc_val +
					REG_COEFFS_TEMP_OIL[j+3][0];

			w 	  = Interpolate(ot[0], REG_TEMPS_OIL[i], ot[1], REG_TEMPS_OIL[j], REG_TEMP_USER.calc_val);

            NEW_WATERCUT_RAW = w;

            // add oil adjust
            w += REG_OIL_ADJUST.calc_val;   

            // MAX 85% in Oil Phase
            if (w > REG_OIL_CALC_MAX) w = REG_OIL_CALC_MAX;    

            *WC = w;
		}
		else
		{
			NEW_WATERCUT_RAW = w;			//use this curve and add OIL_ADJ
			w += REG_OIL_ADJUST.calc_val; 	//add oil adjust
			*WC = w;
			return 0; 						// success
		}
	}
	else
	{
		NEW_WATERCUT_RAW = MAX_WATER_PHASE;
		*WC = MAX_WATER_PHASE;
	}

 	return 0; // success
}


Uint8 Apply_Density_Correction(void)
{
	double dens = 0.0;
	
	// get current density in units of kg/m3 @ 15C
	dens = Convert(REG_OIL_DENSITY.class, REG_OIL_DENSITY.calc_unit, u_mpv_kg_cm_15C, REG_OIL_DENSITY.calc_val, 0, 0);

	// error checking
	if (dens < 750.0) DIAGNOSTICS |= ERR_DNS_LO;
	else if (dens > 998.0) DIAGNOSTICS |= ERR_DNS_HI;
	else DIAGNOSTICS &= ~(ERR_DNS_LO | ERR_DNS_HI);
		
	// get current density in same units as that of REG_DENSITY_CAL_VAL
	//dens = Convert(REG_OIL_DENSITY.class, REG_OIL_DENSITY.calc_unit, REG_DENSITY_CAL_VAL.calc_unit, REG_OIL_DENSITY.calc_val, 0, 0);

    // Razor does not use REG_DENSITY_CAL_VAL
    REG_DENSITY_CAL_VAL.calc_val = 0.0; // DKOH Oct 22, 2019

	if (REG_WATERCUT_RAW.calc_val <= 5.0)
	{// hold last value of Dadj if WC > 5.0%, otherwise calculate new Dadj
		REG_WC_ADJ_DENS = (REG_DENSITY_D2.calc_val * (dens - REG_DENSITY_CAL_VAL.calc_val) * (dens - REG_DENSITY_CAL_VAL.calc_val)) +
					      (REG_DENSITY_D1.calc_val * (dens - REG_DENSITY_CAL_VAL.calc_val)) + 
					       REG_DENSITY_D0.calc_val;
	 // [05/09/2018] Bentley requested we REMOVE 3rd-order calculations and only allow 2nd-order 
	}

	if (REG_WC_ADJ_DENS > 10.0)
	{
		REG_WC_ADJ_DENS = 10.0;
		DIAGNOSTICS |= ERR_DNS_HI;
	}
	else if (REG_WC_ADJ_DENS < -10.0)
	{
		REG_WC_ADJ_DENS = -10.0;
		DIAGNOSTICS |= ERR_DNS_LO;
	}
	else
		DIAGNOSTICS &= ~(ERR_DNS_LO | ERR_DNS_HI);

//	w = WC + REG_DENS_ADJ;
//
//	if (OIL_CALC_MODE==1)
//	{
//		if (w > REG_OIL_CALC_MAX[0])
//			w = REG_OIL_CALC_MAX[0];
//	}
//	else if (OIL_CALC_MODE==2)
//	{
//		if (w > REG_OIL_CALC_MAX[1])
//			w = REG_OIL_CALC_MAX[1];
//	}
//
//	WC = w;

	return 0; //success
}

inline float Interpolate(float w1, float t1, float w2, float t2, float t)
{
	float w;
	w = w2 - ((t2-t)*(w2-w1)/(t2-t1));
	return w;
}

void Update_Demo_Values(void)
{
	int rand_num;
	float rand_freq_delta;
	static int flip = 1;

#ifdef DEMO_MODE
	rand_num = (rand() % 100) - 51; //between -50 and 50
	rand_freq_delta = (float)flip * (float)rand_num / 200;
	#ifdef DEMO_MODE_FREQ
		VAR_Update(&REG_FREQ, REG_FREQ.calc_val+rand_freq_delta, 0);
		if ( (REG_FREQ.calc_val < 560) || (REG_FREQ.calc_val > 590) )
		{
			//keep WC within reasonable bounds
			VAR_Update(&REG_FREQ,REG_FREQ.calc_val-(rand_freq_delta*3), CALC_UNIT); //wc -= rand_freq_delta*3
			flip *= -1;
		}
	#endif
	#ifdef DEMO_MODE_TEMP
		rand_temp_delta = rand_freq_delta/-1;
		VAR_Update(&REG_TEMPERATURE, REG_TEMPERATURE.calc_val+rand_temp_delta, 0);
		// range check
			 if (REG_TEMPERATURE.calc_val < 50) VAR_Update(&REG_TEMPERATURE, 50 + abs(rand_temp_delta)*2, 0);
		else if (REG_TEMPERATURE.calc_val > 60) VAR_Update(&REG_TEMPERATURE, 60 - abs(rand_temp_delta)*2, 0);
			
	#endif
#endif
}


inline void Init_Data_Buffer(void)
{
	//note: .tail is not really necessary for our purposes
	DATALOG.WC_BUFFER.head		= 0;
	DATALOG.WC_BUFFER.tail		= 0;
	DATALOG.WC_BUFFER.n			= 0;
	DATALOG.WC_BUFFER.buff[0]	= 0;

	DATALOG.T_BUFFER.head		= 0;
	DATALOG.T_BUFFER.tail		= 0;
	DATALOG.T_BUFFER.n			= 0;
	DATALOG.T_BUFFER.buff[0]	= 0;

	DATALOG.F_BUFFER.head		= 0;
	DATALOG.F_BUFFER.tail		= 0;
	DATALOG.F_BUFFER.n			= 0;
	DATALOG.F_BUFFER.buff[0]	= 0;

	DATALOG.RP_BUFFER.head		= 0;
	DATALOG.RP_BUFFER.tail		= 0;
	DATALOG.RP_BUFFER.n			= 0;
	DATALOG.RP_BUFFER.buff[0]	= 0;
}

void Capture_Sample(void)
{
    double  sum;
    int     num_samples,i;

    /// Add new samples to buffer
    Bfr_Add(&DATALOG.WC_BUFFER, REG_WATERCUT_RAW.calc_val); // raw watercut samples
    Bfr_Add(&DATALOG.T_BUFFER,  REG_TEMPERATURE.calc_val);  // temperature samples
    Bfr_Add(&DATALOG.F_BUFFER,  REG_FREQ.calc_val);         // oscillator frequency samples
    Bfr_Add(&DATALOG.RP_BUFFER, REG_OIL_RP);                // oscillator reflected power samples

    /// Update Value Averages
    // if we haven't yet captured enough samples to compute the [REG_PROC_AVGING.calc_val]-sample mean
    if  (REG_PROC_AVGING.calc_val > DATALOG.WC_BUFFER.n)
        num_samples = DATALOG.WC_BUFFER.n;
    else
        num_samples = REG_PROC_AVGING.calc_val;

    // Average Watercut
    sum = 0;
    for (i=0;i<num_samples;i++)
    {   
        if ((DATALOG.WC_BUFFER.head-i) < 0) //wrap around
            sum += DATALOG.WC_BUFFER.buff[DATALOG.WC_BUFFER.head - i + MAX_BFR_SIZE_F];
        else
            sum += DATALOG.WC_BUFFER.buff[DATALOG.WC_BUFFER.head-i];
    }   
    REG_WATERCUT_AVG.calc_val = sum/(double)num_samples;
    //VAR_Update(&REG_WATERCUT_AVG, sum/(double)num_samples, CALC_UNIT);   //update average

	if (COIL_BEGIN_OIL_CAP.val)
    {
        // UPDATE STREAM DATA WHILE CAPTURING OIL
        STREAM_WATERCUT_AVG[(int)REG_STREAM.calc_val-1] = REG_WATERCUT_AVG.calc_val;
        STREAM_SAMPLES[(int)REG_STREAM.calc_val-1] = num_samples;
        sprintf(STREAM_TIMESTAMP[(int)REG_STREAM.calc_val-1],"%.2u:%.2u %.2u/%.2u/20%.2u",CAL_RTC_HR,CAL_RTC_MIN,CAL_RTC_MON,CAL_RTC_DAY,CAL_RTC_YR);
        Swi_post(Swi_writeNand);
    }

    //Average Temperature//
    sum = 0;
    for (i=0;i<num_samples;i++)
    {   
        if ((DATALOG.T_BUFFER.head-i) < 0) //wrap around
            sum += DATALOG.T_BUFFER.buff[DATALOG.T_BUFFER.head - i + MAX_BFR_SIZE_F];
        else
            sum += DATALOG.T_BUFFER.buff[DATALOG.T_BUFFER.head-i];

        // In 24HR mode, resets REG_TEMP_AVG between 24:00:00 and 23:59:57 everyday
        // COIL_AVG_MODE = TRUE <--- ondemand
        if (COIL_AVGTEMP_RESET.val || (!COIL_AVGTEMP_MODE.val && (CAL_RTC_SEC > 57) && (CAL_RTC_MIN == 59) && (CAL_RTC_HR == 23)))
        {
            num_samples = 0;
            DATALOG.T_BUFFER.head    = 0;
            DATALOG.T_BUFFER.tail    = 0;
            DATALOG.T_BUFFER.n       = 0;
            DATALOG.T_BUFFER.buff[0] = 0;
            COIL_AVGTEMP_RESET.val = FALSE;
        }
    }
    VAR_Update(&REG_TEMP_AVG, sum / (double)num_samples, CALC_UNIT);   //update average

    //Average Frequency//
    sum = 0;
    for (i=0;i<num_samples;i++)
    {   
        if ((DATALOG.F_BUFFER.head-i) < 0) //wrap around
            sum += DATALOG.F_BUFFER.buff[DATALOG.F_BUFFER.head - i + MAX_BFR_SIZE_F];
        else
            sum += DATALOG.F_BUFFER.buff[DATALOG.F_BUFFER.head-i];
    }
    VAR_Update(&REG_FREQ_AVG, sum / (double)num_samples, CALC_UNIT);   //update average

    //Average Reflected Power//
    sum = 0;
    for (i=0;i<num_samples;i++)
    {
        if ((DATALOG.RP_BUFFER.head-i) < 0) //wrap around
            sum += DATALOG.RP_BUFFER.buff[DATALOG.RP_BUFFER.head - i + MAX_BFR_SIZE_F];
        else
            sum += DATALOG.RP_BUFFER.buff[DATALOG.RP_BUFFER.head-i];
    }
    REG_OIL_RP_AVG =  sum / (double)num_samples;    //update average

    Clock_start(Capture_Sample_Clock); // call this again in 1 sec
}

void Calibrate_Oil(void)
{
	float t;
	double sg = 1.0;

    // FIXME: REG_DENS_ADJ needs to be an AVERAGE -- Isn't this already done??
    if ((REG_OIL_DENS_CORR_MODE != 0) && (((REG_ANALYZER_MODE&0xFF)==ANA_MODE_LOW)||((REG_ANALYZER_MODE&0xFF)==ANA_MODE_MID)))
    {   
        sg = Convert(c_mass_per_volume, FC.density.calc_unit, u_mpv_sg_15C, FC.density.calc_val, 0, FC.density.aux);
        if (sg<=0) sg = 1.0;
    }   
    else sg = 1.0;

    if (REG_STREAM.calc_val == TEMP_STREAM) // CALIBRATE SAME STREAM
    {   
        if (REG_PROC_AVGING.calc_val < DATALOG.WC_BUFFER.n)
        {
            if (COIL_OIL_PHASE.val)
            {
                if ((REG_WATERCUT.STAT & var_aux)==0) t = (REG_OIL_SAMPLE.calc_val*sg) - (REG_WATERCUT_RAW.calc_val + FC.Dadj);  // if not rollover 
                VAR_Update(&REG_OIL_ADJUST, t, CALC_UNIT);
            }
        }
        else
        {
            t = REG_OIL_SAMPLE.calc_val - (REG_WATERCUT_AVG.calc_val + REG_DENS_ADJ);
            VAR_Update(&REG_OIL_ADJUST, t, CALC_UNIT);
        }
    }   
    else // CALIBRATE "OLD" STREAM
    {   
        if (REG_PROC_AVGING.calc_val < STREAM_SAMPLES[TEMP_STREAM-1])
        {
            if ((REG_WATERCUT.STAT & var_aux)==0) STREAM_OIL_ADJUST[TEMP_STREAM-1] = (REG_OIL_SAMPLE.calc_val*sg) - (REG_WATERCUT_RAW.calc_val + FC.Dadj); // if not roll-over
        }
        else
        {
             STREAM_OIL_ADJUST[TEMP_STREAM-1] = REG_OIL_SAMPLE.calc_val - (STREAM_WATERCUT_AVG[TEMP_STREAM-1] + REG_DENS_ADJ);
        }

        // NO NEED TO KEEP TEMP VALUES
        REG_OIL_SAMPLE.val = 0;
        REG_OIL_SAMPLE.calc_val = 0;
    }   
 
    Swi_post(Swi_writeNand);
}


void Set_REG_DENSITY_CAL_Unit(void)
{
	if ((REG_DENSITY_UNIT.val==u_mpv_kg_cm_15C) || (REG_DENSITY_UNIT.val==u_mpv_deg_API_60F))
	{
		if (REG_DENSITY_CAL_VAL.calc_unit!=REG_DENSITY_UNIT.val)
		{
			REG_DENSITY_CAL_VAL.calc_unit = REG_DENSITY_UNIT.val;
			REG_DENSITY_CAL_VAL.unit = REG_DENSITY_CAL_VAL.calc_unit;
		}
	}
	else
	{
		REG_DENSITY_UNIT.val = REG_DENSITY_CAL_VAL.calc_unit;
	}

	//note:	The density coefficients are modifiable/saveable, but writing to REG_DENSITY_CAL_VAL
	//		will reset the coefficients to one of the defaults below
	if (REG_DENSITY_UNIT.val==u_mpv_kg_cm_15C)
	{
		REG_DENSITY_D1.calc_unit = u_mfgr_specific_perc_per_kgm3_15C;
		REG_DENSITY_D1.unit = u_mfgr_specific_perc_per_kgm3_15C;
		VAR_Update(&REG_DENSITY_D3, 0.0, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D2, 0.0, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D1, -0.0286, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D0, 24.6, CALC_UNIT);
		VAR_Update(&REG_DENSITY_CAL_VAL, 0.0, CALC_UNIT);;
	}
	else if (REG_DENSITY_UNIT.val==u_mpv_deg_API_60F)
	{
		REG_DENSITY_D1.calc_unit = u_mfgr_specific_perc_per_API_60F;
		REG_DENSITY_D1.unit = u_mfgr_specific_perc_per_API_60F;
		VAR_Update(&REG_DENSITY_D3, 0.0, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D2, 0.0, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D1, 0.16, CALC_UNIT);
		VAR_Update(&REG_DENSITY_D0, 0.0, CALC_UNIT);
		VAR_Update(&REG_DENSITY_CAL_VAL, 32.0, CALC_UNIT);
	}
}


// Convenience fxn adds a data sample to one of the rolling buffers
void Bfr_Add(FP_BFR* bfr, double val)
{
	bfr->head++;
	if (bfr->head >= MAX_BFR_SIZE_F)
		bfr->head -= MAX_BFR_SIZE_F;

	bfr->buff[bfr->head] = val;

	if (bfr->n < MAX_BFR_SIZE_F)
		bfr->n++;
}
