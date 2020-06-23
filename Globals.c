/*------------------------------------------------------------------------*
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
* Globals.c
*-------------------------------------------------------------------------
* HISTORY:
*       Aug-20-2013 : Daniel Koh : Created Ver 1.0.0 
*------------------------------------------------------------------------*/

#define GLOBAL_VARS

#include "Globals.h"
#include "Errors.h"

void resetGlobalVars(void)
{
    CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT5,FALSE); //set GPIO pin as output

    isWriteRTC = FALSE;
    isLogging = FALSE;

    THROW_ERROR 	                    = 0;
    DIAGNOSTICS 	                    = 0;
    DIAGNOSTICS_MASK                    = 0xFFFFFFFF;
	REG_ACTIVE_ERROR                    = 0;
	REG_WATERCUT_RAW                    = 0;
	STAT_SUCCESS 	                    = 0;
	STAT_PKT 		                    = 0;
	STAT_CMD 		                    = 0;
	STAT_RETRY 		                    = 0;
	STAT_CURRENT 	                    = 0;	// hold current stat kind

    COIL_AVGTEMP_RESET.val              = FALSE;
	COIL_AO_TRIM_MODE.val               = FALSE; 
	COIL_AI_TRIM_MODE.val               = FALSE;
	COIL_LOG_ENABLE.val                 = FALSE;
	COIL_LOCKED_SOFT_FACTORY_RESET.val  = TRUE;
	COIL_LOCKED_HARD_FACTORY_RESET.val  = TRUE;
    COIL_UPDATE_FACTORY_DEFAULT.val     = FALSE;
    COIL_UNLOCKED_FACTORY_DEFAULT.val   = FALSE;

    TEMP_STREAM                         = REG_STREAM.calc_val; // TEMP_STERAM == REG_STREAM.calc_val
}


void storeUserDataToFactoryDefault(void)
{
    // UNLOCK SAFETY GUARD FOR FCT REGISTERS
    COIL_UNLOCKED_FACTORY_DEFAULT.val = TRUE;

    isWriteRTC = FALSE;

    // REGTYPE_INT
    FCT_AO_DAMPEN           = REG_AO_DAMPEN;
    FCT_SLAVE_ADDRESS       = REG_SLAVE_ADDRESS;
    FCT_STOP_BITS           = REG_STOP_BITS;
    FCT_DENSITY_MODE        = REG_DENSITY_MODE;
    FCT_LOGGING_PERIOD      = REG_LOGGING_PERIOD;
    FCT_STATISTICS			= REG_STATISTICS;
	FCT_ACTIVE_ERROR 		= REG_ACTIVE_ERROR;
	FCT_AO_ALARM_MODE		= REG_AO_ALARM_MODE;
	FCT_PHASE_HOLD_CYCLES	= REG_PHASE_HOLD_CYCLES;
	FCT_RELAY_DELAY			= REG_RELAY_DELAY;
	FCT_AO_MODE				= REG_AO_MODE;
	FCT_OIL_DENS_CORR_MODE	= REG_OIL_DENS_CORR_MODE; 
	FCT_RELAY_MODE 			= REG_RELAY_MODE;

    // REGTYPE_DBL
    FCT_OIL_CALC_MAX        = REG_OIL_CALC_MAX;
    FCT_OIL_PHASE_CUTOFF    = REG_OIL_PHASE_CUTOFF;  // oil curve switch-over threshold
    FCT_AO_MANUAL_VAL       = REG_AO_MANUAL_VAL;
    FCT_AO_TRIMLO           = REG_AO_TRIMLO;
    FCT_AO_TRIMHI           = REG_AO_TRIMHI;
    FCT_DENSITY_ADJ         = REG_DENSITY_ADJ;
    FCT_OIL_DENSITY_MODBUS  = REG_OIL_DENSITY_MODBUS;
    FCT_OIL_DENSITY_AI      = REG_OIL_DENSITY_AI;
    FCT_OIL_DENSITY_MANUAL  = REG_OIL_DENSITY_MANUAL;
    FCT_AI_TRIMLO           = REG_AI_TRIMLO;
    FCT_AI_TRIMHI           = REG_AI_TRIMHI;
	FCT_AI_MEASURE			= REG_AI_MEASURE;
	FCT_AI_TRIMMED			= REG_AI_TRIMMED;
	FCT_DENS_ADJ 			= REG_DENS_ADJ;

    // REGTYPE_VAR
    VAR_Update(&FCT_SALINITY, REG_SALINITY.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_ADJUST, REG_OIL_ADJUST.calc_val, CALC_UNIT);
    VAR_Update(&FCT_WATER_ADJUST, REG_WATER_ADJUST.calc_val, CALC_UNIT);
    VAR_Update(&FCT_TEMP_ADJUST, REG_TEMP_ADJUST.calc_val, CALC_UNIT);
    VAR_Update(&FCT_PROC_AVGING, REG_PROC_AVGING.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_INDEX, REG_OIL_INDEX.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_P0, REG_OIL_P0.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_P1, REG_OIL_P1.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_FREQ_LOW, REG_OIL_FREQ_LOW.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_FREQ_HIGH, REG_OIL_FREQ_HIGH.calc_val, CALC_UNIT);
	VAR_Update(&FCT_SAMPLE_PERIOD, REG_SAMPLE_PERIOD.calc_val, CALC_UNIT);
    VAR_Update(&FCT_AO_LRV, REG_AO_LRV.calc_val, CALC_UNIT);
    VAR_Update(&FCT_AO_URV, REG_AO_URV.calc_val, CALC_UNIT);
	VAR_Update(&FCT_BAUD_RATE, REG_BAUD_RATE.calc_val, CALC_UNIT);
    VAR_Update(&FCT_STREAM, REG_STREAM.calc_val, CALC_UNIT);
	VAR_Update(&FCT_OIL_SAMPLE, REG_OIL_SAMPLE.calc_val, CALC_UNIT);
    VAR_Update(&FCT_DENSITY_D3, REG_DENSITY_D3.calc_val, CALC_UNIT);
    VAR_Update(&FCT_DENSITY_D2, REG_DENSITY_D2.calc_val, CALC_UNIT);
    VAR_Update(&FCT_DENSITY_D1, REG_DENSITY_D1.calc_val, CALC_UNIT);
    VAR_Update(&FCT_DENSITY_D0, REG_DENSITY_D0.calc_val, CALC_UNIT);
    VAR_Update(&FCT_DENSITY_CAL_VAL, REG_DENSITY_CAL_VAL.calc_val, CALC_UNIT);
	VAR_Update(&FCT_RELAY_SETPOINT, REG_RELAY_SETPOINT.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_DENSITY_AI_LRV, REG_OIL_DENSITY_AI_LRV.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_DENSITY_AI_URV, REG_OIL_DENSITY_AI_URV.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_T0, REG_OIL_T0.calc_val, CALC_UNIT);
    VAR_Update(&FCT_OIL_T1, REG_OIL_T1.calc_val, CALC_UNIT);

    // dislable the trigger
    COIL_UPDATE_FACTORY_DEFAULT.val = FALSE;
    COIL_UNLOCKED_FACTORY_DEFAULT.val = FALSE;

    // save to nand flash
    Swi_post(Swi_writeNand);

    // dislable usb handler
    unloadUsbDriver();

    // restart 
	_c_int00();

    // THE USER "MUST" POWER CYCLE AFTER RESET TO ENABLE USER DRIVER
}

void reloadFactoryDefault(void)
{
    isWriteRTC = FALSE;
	Uint16 i;
	char model_code[MAX_LCD_WIDTH];
	int* model_code_int;

    //isUpdateFirmware = FALSE;

	sprintf(model_code,DEFAULT_MODEL_CODE); //default model code
	model_code_int = (int*)model_code;
	for (i=0;i<4;i++) REG_MODEL_CODE[i] = model_code_int[i];

	REG_DENSITY_UNIT.val 	= u_mpv_deg_API_60F;
	PASSWORD_DELAYED 		= FALSE;

	//////////////////////////////////////
	// REGTYPE_INT 
	//////////////////////////////////////

	REG_ANALYZER_MODE 	    = ANA_MODE_MID;
	REG_AO_DAMPEN			= FCT_AO_DAMPEN;
	REG_SLAVE_ADDRESS 		= FCT_SLAVE_ADDRESS;
	REG_STOP_BITS 			= FCT_STOP_BITS;
	REG_DENSITY_MODE 		= FCT_DENSITY_MODE;
    REG_RTC_SEC				= 0;
	REG_RTC_MIN				= 0;
	REG_RTC_HR				= 0;
	REG_RTC_DAY				= 0;
	REG_RTC_MON				= 0;
	REG_RTC_YR				= 0;
	REG_RTC_SEC_IN			= 0;
	REG_RTC_MIN_IN			= 0;
	REG_RTC_HR_IN			= 0;
	REG_RTC_DAY_IN			= 0;
	REG_RTC_MON_IN			= 0;
	REG_RTC_YR_IN			= 0;
	REG_LOGGING_PERIOD		= FCT_LOGGING_PERIOD; // logging every 1 sec by default
	REG_PASSWORD			= 0;
	REG_STATISTICS			= FCT_STATISTICS;
	REG_ACTIVE_ERROR 		= FCT_ACTIVE_ERROR;
	REG_AO_ALARM_MODE		= FCT_AO_ALARM_MODE;
	REG_PHASE_HOLD_CYCLES	= FCT_PHASE_HOLD_CYCLES;
	REG_RELAY_DELAY			= FCT_RELAY_DELAY;
	REG_AO_MODE				= FCT_AO_MODE;
	REG_OIL_DENS_CORR_MODE	= FCT_OIL_DENS_CORR_MODE; 
	REG_RELAY_MODE 			= FCT_RELAY_MODE;
	REG_DIAGNOSTICS 		= 0;
	
	//////////////////////////////////////
	// REGTYPE_DBL
	//////////////////////////////////////

    REG_HARDWARE_VERSION 	= 0;
	REG_FIRMWARE_VERSION 	= 0;
	REG_OIL_RP 				= 0;
    REG_WATER_RP            = 0;
	REG_OIL_CALC_MAX 	    = FCT_OIL_CALC_MAX; 	// maximum watercut (for oil phase)
	REG_OIL_PHASE_CUTOFF 	= FCT_OIL_PHASE_CUTOFF; //oil curve switch-over threshold
	REG_OIL_RP_AVG 			= 0;
	REG_AO_MANUAL_VAL		= FCT_AO_MANUAL_VAL;
    REG_AO_TRIMLO			= FCT_AO_TRIMLO;
	REG_AO_TRIMHI 			= FCT_AO_TRIMHI;
	REG_DENSITY_ADJ 		= FCT_DENSITY_ADJ;
	REG_DENS_CORR 		= 0;
	REG_AO_OUTPUT			= 0;
	REG_OIL_DENSITY_MODBUS	= FCT_OIL_DENSITY_MODBUS;
	REG_OIL_DENSITY_AI		= FCT_OIL_DENSITY_AI;
    REG_OIL_DENSITY_MANUAL	= FCT_OIL_DENSITY_MANUAL;
	REG_AI_TRIMLO			= FCT_AI_TRIMLO;
	REG_AI_TRIMHI 			= FCT_AI_TRIMHI;
	REG_AI_MEASURE			= FCT_AI_MEASURE;
	REG_AI_TRIMMED			= FCT_AI_TRIMMED;
	REG_DENS_ADJ 			= FCT_DENS_ADJ;
	REG_OIL_PT 	            = 0;
    
    //////////////////////////////////////
   	/// Watercut - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATERCUT, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_WATERCUT, u_ana_percent, 100.0, 0.0, 105.0, -3.0);
	VAR_Update(&REG_WATERCUT, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Temperature - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMPERATURE, c_temperature, u_temp_C, 10.0, 10.0, var_no_bound);
	VAR_Setup_Unit(&REG_TEMPERATURE, u_temp_C, 350.0, -50.0, 310, 0);
	VAR_Update(&REG_TEMPERATURE, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// REG_EMULSION_PHASE - NOT USED
	//////////////////////////////////////

	VAR_Initialize(&REG_EMULSION_PHASE, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_EMULSION_PHASE, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	VAR_Update(&REG_EMULSION_PHASE, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Salinity - 0.02
	//////////////////////////////////////

	VAR_Initialize(&REG_SALINITY, c_concentration, u_conc_perc_solid_per_weight, 100.0, 100.0, 0);
	VAR_Setup_Unit(&REG_SALINITY, u_conc_perc_solid_per_weight, 30.0, 0.02, 30.0, 0.02);
	//VAR_Update(&REG_SALINITY, 0.02, CALC_UNIT);
	VAR_Update(&REG_SALINITY, FCT_SALINITY.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Phase Adjust - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	//VAR_Update(&REG_OIL_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_ADJUST, FCT_OIL_ADJUST.calc_val, CALC_UNIT);
    REG_OIL_ADJUST.swi = Swi_REG_OIL_ADJUST; // SAVE STERAM DEPENDENT DATA AFTER REG_OIL_ADJUST UPDATED.

	//////////////////////////////////////
	/// Water Phase Adjust - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATER_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_WATER_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	//VAR_Update(&REG_WATER_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_WATER_ADJUST, FCT_WATER_ADJUST.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Frequency - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_FREQ, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_FREQ, u_mfgr_specific_MHz, 1000.0, 0.0, 1001.0, -1.0);
	VAR_Update(&REG_FREQ, 0.0, 0);

	//////////////////////////////////////
	/// Average Frequency - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_FREQ_AVG, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_FREQ_AVG, u_mfgr_specific_MHz, 1000.0, 0.0, 1001.0, -1.0);
	VAR_Update(&REG_FREQ_AVG, 0.0, 0);

	//////////////////////////////////////
   	/// Average Watercut - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATERCUT_AVG, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_WATERCUT_AVG, u_ana_percent, 100.0, -100.0, 105.0, -3.0);
	VAR_Update(&REG_WATERCUT_AVG, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Average Temperature - N/A
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMP_AVG, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 10.0, 10.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_AVG, u_temp_C, 350.0, -50.0, 310, 0);
    VAR_Update(&REG_TEMP_AVG, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Temperature Adjustment - 0.0
	//////////////////////////////////////

	//VAR_Initialize(&REG_TEMP_ADJUST, c_temperature, u_mfgr_specific_none, 100.0, 100.0, var_no_bound|var_no_alarm);
	VAR_Initialize(&REG_TEMP_ADJUST, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 100.0, 100.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_ADJUST, REG_TEMPERATURE.calc_unit, 350.0, -50.0, 310, 0);
	//VAR_Update(&REG_TEMP_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_TEMP_ADJUST, FCT_TEMP_ADJUST.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// User Temperature - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMP_USER, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 10.0, 10.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_USER, REG_TEMPERATURE.calc_unit, 350.0, -50.0, 310, 0);
	VAR_Update(&REG_TEMP_USER, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Process Averaging - 5.0
	//////////////////////////////////////

	VAR_Initialize(&REG_PROC_AVGING, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&REG_PROC_AVGING, u_mfgr_specific_none, MAXBUF, 1.0, MAXBUF, 1.0);
	VAR_Update(&REG_PROC_AVGING, FCT_PROC_AVGING.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Index - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_INDEX, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_INDEX, u_mfgr_specific_MHz, 1000.0, -1000.0, 1000.0, -1000.0);
	//VAR_Update(&REG_OIL_INDEX, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_INDEX, FCT_OIL_INDEX.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil P0 - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_P0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_P0, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_P0, FCT_OIL_P0.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil P1 - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_P1, c_not_classified, u_mfgr_specific_V_per_MHz, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_P1, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_P1, FCT_OIL_P1.calc_val, CALC_UNIT);

    ////////////////////////////////////// 
    /// Oil Frequency Low - 10
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_FREQ_LOW, c_frequency, u_mfgr_specific_MHz, 10, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_FREQ_LOW, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
	//VAR_Update(&REG_OIL_FREQ_LOW, 10, CALC_UNIT);
	VAR_Update(&REG_OIL_FREQ_LOW, FCT_OIL_FREQ_LOW.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Frequency High - 1000.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_FREQ_HIGH, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_FREQ_HIGH, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
	//VAR_Update(&REG_OIL_FREQ_HIGH, 1000.0, CALC_UNIT);
	VAR_Update(&REG_OIL_FREQ_HIGH, FCT_OIL_FREQ_HIGH.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Period for Sampling - 60.0
	//////////////////////////////////////

	VAR_Initialize(&REG_SAMPLE_PERIOD, c_time, u_time_sec, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&REG_SAMPLE_PERIOD, u_time_sec, 86400, 1, 86400, 1);
	VAR_Update(&REG_SAMPLE_PERIOD, FCT_SAMPLE_PERIOD.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog out Lower Range Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_AO_LRV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_AO_LRV, u_ana_percent, 100.0, 0, 100.0, 0);
	//VAR_Update(&REG_AO_LRV, 0.0, CALC_UNIT);
	VAR_Update(&REG_AO_LRV, FCT_AO_LRV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog out Upper Range Value - 85.0
	//////////////////////////////////////

	VAR_Initialize(&REG_AO_URV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_AO_URV, u_ana_percent, 100.0, 0, 100.0, 0);
	//VAR_Update(&REG_AO_URV, 100.0, CALC_UNIT);
	VAR_Update(&REG_AO_URV, FCT_AO_URV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Modbus Baud Rate - 9600 Bps
	//////////////////////////////////////

	VAR_Initialize(&REG_BAUD_RATE, c_not_classified, u_mfgr_specific_bps, 0.01, 0.01, var_no_alarm);
	VAR_Setup_Unit(&REG_BAUD_RATE, u_mfgr_specific_bps, 115200, 300, 115200, 300);
	VAR_Update(&REG_BAUD_RATE, FCT_BAUD_RATE.calc_val, CALC_UNIT);

	//////////////////////////////////////
    /// Stream Select - 1.0
	//////////////////////////////////////

    VAR_Initialize(&REG_STREAM, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
    VAR_Setup_Unit(&REG_STREAM, u_mfgr_specific_none, SMAX, 1.0, SMAX, 1.0);
    VAR_Update(&REG_STREAM, FCT_STREAM.calc_val, CALC_UNIT); 
    REG_STREAM.swi = Swi_REG_STREAM; // RESTORE STREAM DEPENDENT DATA AFTER CHANGING STREAM 

	//////////////////////////////////////
	/// Oil Calibration Test Sample Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_SAMPLE, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_SAMPLE, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	VAR_Update(&REG_OIL_SAMPLE, FCT_OIL_SAMPLE.calc_val, CALC_UNIT);
	REG_OIL_SAMPLE.swi = Swi_REG_OIL_SAMPLE; // will be triggered after REG_OIL_SAMPLE is updated insde ModbusRTU.c

	//////////////////////////////////////
	/// Density Adjustment 3rd Order - 0.00
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D3, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D3, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D3, FCT_DENSITY_D3.calc_val, CALC_UNIT);

	//////////////////////////////////////
	///  Density Adjustment 2nd Order - 0.00
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D2, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D2, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D2, FCT_DENSITY_D2.calc_val, CALC_UNIT);

	//////////////////////////////////////
	///  Density Adjustment Slope - 0.16 
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D1, c_analytical, u_mfgr_specific_perc_per_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D1, 0.16, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D1, FCT_DENSITY_D1.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Density Adjustment Offset - 0.0 
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D0, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D0, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D0, FCT_DENSITY_D0.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Density correction zeroth-order coefficient
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_CAL_VAL, c_mass_per_volume, u_mpv_deg_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_CAL_VAL, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_CAL_VAL, FCT_DENSITY_CAL_VAL.calc_val, CALC_UNIT);
	REG_DENSITY_CAL_VAL.swi = Swi_Set_REG_DENSITY_CAL_Unit;

	//////////////////////////////////////
   	/// Relay Setpoint - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_RELAY_SETPOINT, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_RELAY_SETPOINT, u_ana_percent, 100.0, 0.0, 105.0, -3.0);
	VAR_Update(&REG_RELAY_SETPOINT, FCT_RELAY_SETPOINT.calc_val, CALC_UNIT);
	
	//////////////////////////////////////
   	/// Oil Density @ Process Temperature - 865.443
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY, c_mass_per_volume, u_mpv_kg_cm, 10.0, 1000.0, var_no_bound|var_no_alarm);
	VAR_Update(&REG_OIL_DENSITY, 865.443, CALC_UNIT);

	//////////////////////////////////////
	/// Analog In Lower Range Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY_AI_LRV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_DENSITY_AI_LRV, 750.0, CALC_UNIT);
	VAR_Update(&REG_OIL_DENSITY_AI_LRV, FCT_OIL_DENSITY_AI_LRV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog In Upper Range Value - 80.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY_AI_URV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_DENSITY_AI_URV, 998.0, CALC_UNIT);
	VAR_Update(&REG_OIL_DENSITY_AI_URV, FCT_OIL_DENSITY_AI_URV.calc_val, CALC_UNIT);

    //////////////////////////////////////
    /// Oil T0 (179) - 0.0
    //////////////////////////////////////
 
    VAR_Initialize(&REG_OIL_T0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    //VAR_Update(&REG_OIL_T0, 0.0, CALC_UNIT);
    VAR_Update(&REG_OIL_T0, FCT_OIL_T0.calc_val, CALC_UNIT);
 
    //////////////////////////////////////
    /// Oil T1 (181) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&REG_OIL_T1, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    //VAR_Update(&REG_OIL_T1, 0.0, CALC_UNIT);
    VAR_Update(&REG_OIL_T1, FCT_OIL_T1.calc_val, CALC_UNIT);

	for (i=0;i<NUMBER_OF_OSC;i++)
	{
		VAR_Initialize(&REG_FREQ, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
		VAR_Setup_Unit(&REG_FREQ, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
		VAR_Update(&REG_FREQ, 92.0, CALC_UNIT);
	}

	//////////////////////////////////////
	/// COILS
	//////////////////////////////////////

    COIL_Initialize(&COIL_RELAY[0], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[1], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[2], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[3], FALSE, 0);
	COIL_Initialize(&COIL_BEGIN_OIL_CAP, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ENABLE, FALSE, 0);
    COIL_Initialize(&COIL_LOG_ALARMS, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ERRORS, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ACTIVITY, FALSE, 0);
    COIL_Initialize(&COIL_AO_ALARM, TRUE, 0);
	COIL_Initialize(&COIL_PARITY, FALSE, 0);
	COIL_Initialize(&COIL_WRITE_RTC, FALSE, 0);
	COIL_Initialize(&COIL_OIL_DENS_CORR_EN, TRUE, 0); 
	COIL_Initialize(&COIL_AO_MANUAL, FALSE, 0);
	COIL_Initialize(&COIL_MB_AUX_SELECT_MODE, FALSE, 0);
	COIL_Initialize(&COIL_INTEGER_TABLE_SELECT, FALSE, 0);
	COIL_Initialize(&COIL_AVGTEMP_RESET, FALSE, 0); 
    COIL_Initialize(&COIL_AVGTEMP_MODE, FALSE, 0);  	
	COIL_Initialize(&COIL_OIL_PHASE, TRUE, 0);
	COIL_Initialize(&COIL_ACT_RELAY_OIL, TRUE, 0);
	COIL_Initialize(&COIL_RELAY_MANUAL, FALSE, 0);
	COIL_Initialize(&COIL_UNLOCKED, TRUE, 0);
	COIL_Initialize(&COIL_AO_TRIM_MODE, FALSE, 0);
	COIL_Initialize(&COIL_AI_TRIM_MODE, FALSE, 0);
    COIL_Initialize(&COIL_LOCKED_SOFT_FACTORY_RESET, TRUE, 0);
	COIL_Initialize(&COIL_LOCKED_HARD_FACTORY_RESET, TRUE, 0);
	COIL_Initialize(&COIL_UPDATE_FACTORY_DEFAULT, FALSE, 0);
	COIL_Initialize(&COIL_UNLOCKED_FACTORY_DEFAULT, FALSE, 0);

	CSL_FINS(gpioRegs->BANK[1].OUT_DATA,GPIO_OUT_DATA_OUT5,FALSE); //set GPIO pin as output
	THROW_ERROR 		= 0;
	DIAGNOSTICS 		= 0;
	DIAGNOSTICS_MASK 	= 0xFFFFFFFF;

}


void initializeAllRegisters(void)
{
	Uint16 i;

    //////////////////////////////////////
	// REGTYPE_INT 
	//////////////////////////////////////

	FCT_AO_DAMPEN			= 0; 
    FCT_SLAVE_ADDRESS       = 1;
    FCT_STOP_BITS           = 0;
    FCT_DENSITY_MODE        = 0;
    FCT_LOGGING_PERIOD      = 2;
    FCT_STATISTICS			= 0;
	FCT_ACTIVE_ERROR 		= 0;
	FCT_AO_ALARM_MODE		= 0;
	FCT_PHASE_HOLD_CYCLES	= 3;
	FCT_RELAY_DELAY			= 0;
	FCT_AO_MODE				= 0;
	FCT_OIL_DENS_CORR_MODE	= 0; 
	FCT_RELAY_MODE 			= 0; // WATERCUT

    REG_SN_PIPE             = 0; // (REGPERM_FCT)
	REG_ANALYZER_MODE 	    = ANA_MODE_MID;
	REG_AO_DAMPEN			= FCT_AO_DAMPEN;
	REG_SLAVE_ADDRESS 		= FCT_SLAVE_ADDRESS;
	REG_STOP_BITS 			= FCT_STOP_BITS;
	REG_DENSITY_MODE 		= FCT_DENSITY_MODE;
    REG_RTC_SEC				= 0;
	REG_RTC_MIN				= 0;
	REG_RTC_HR				= 0;
	REG_RTC_DAY				= 0;
	REG_RTC_MON				= 0;
	REG_RTC_YR				= 0;
	REG_RTC_SEC_IN			= 0;
	REG_RTC_MIN_IN			= 0;
	REG_RTC_HR_IN			= 0;
	REG_RTC_DAY_IN			= 0;
	REG_RTC_MON_IN			= 0;
	REG_RTC_YR_IN			= 0;
    REG_MODEL_CODE[0]       = 0; // (REGPERM_FCT)
    REG_MODEL_CODE[1]       = 0; // (REGPERM_FCT)
    REG_MODEL_CODE[2]       = 0; // (REGPERM_FCT)
    REG_MODEL_CODE[3]       = 0; // (REGPERM_FCT)
	REG_LOGGING_PERIOD		= FCT_LOGGING_PERIOD; // logging every 1 sec by default
	REG_PASSWORD			= 0;
	REG_STATISTICS			= FCT_STATISTICS;
	REG_ACTIVE_ERROR 		= FCT_ACTIVE_ERROR;
	REG_AO_ALARM_MODE		= FCT_AO_ALARM_MODE;
	REG_PHASE_HOLD_CYCLES	= FCT_PHASE_HOLD_CYCLES;
	REG_RELAY_DELAY			= FCT_RELAY_DELAY;
	REG_AO_MODE				= FCT_AO_MODE;
	REG_OIL_DENS_CORR_MODE	= FCT_OIL_DENS_CORR_MODE; 
	REG_RELAY_MODE 			= FCT_RELAY_MODE;
	REG_DIAGNOSTICS 		= 0;
	
	//////////////////////////////////////
	// REGTYPE_LONGINT (REGPERM_FCT)
	//////////////////////////////////////

    REG_MEASSECTION_SN      = 0; // (REGPERM_FCT)
    REG_BACKBOARD_SN        = 0; // (REGPERM_FCT)
    REG_SAFETYBARRIER_SN    = 0; // (REGPERM_FCT)
    REG_POWERSUPPLY_SN      = 0; // (REGPERM_FCT)
    REG_PROCESSOR_SN        = 0; // (REGPERM_FCT)
    REG_DISPLAY_SN          = 0; // (REGPERM_FCT)
    REG_RF_SN               = 0; // (REGPERM_FCT)
    REG_ASSEMBLY_SN         = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[0]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[1]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[2]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[3]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[4]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[5]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[6]   = 0; // (REGPERM_FCT)
    REG_ELECTRONICS_SN[7]   = 0; // (REGPERM_FCT)

	//////////////////////////////////////
	// REGTYPE_DBL
	//////////////////////////////////////

    FCT_OIL_CALC_MAX        = 85.0;
    FCT_OIL_PHASE_CUTOFF    = 0;      // oil curve switch-over threshold
    FCT_AO_MANUAL_VAL       = 0;
    FCT_AO_TRIMLO           = 4.0;
    FCT_AO_TRIMHI           = 20.0;
    FCT_DENSITY_ADJ         = 0;
    FCT_OIL_DENSITY_MODBUS  = 0;
    FCT_OIL_DENSITY_AI      = 0;
    FCT_OIL_DENSITY_MANUAL  = 0;
    FCT_AI_TRIMLO           = 0;
    FCT_AI_TRIMHI           = 0;
    FCT_AI_MEASURE			= 0;
    FCT_AI_TRIMMED			= 0;
    FCT_DENS_ADJ 			= 0;

    REG_HARDWARE_VERSION 	= 0;
	REG_FIRMWARE_VERSION 	= 0;
	REG_OIL_RP 				= 0;
	REG_SALINITY_CURVES     = 0;  // (REGTYPE_FCT)
	REG_WATER_CURVES        = 0;  // (REGTYPE_FCT)
    REG_WATER_RP            = 0;
	REG_OIL_CALC_MAX 	    = FCT_OIL_CALC_MAX; 	// maximum watercut (for oil phase)
	REG_OIL_PHASE_CUTOFF 	= FCT_OIL_PHASE_CUTOFF; //oil curve switch-over threshold
	REG_TEMP_OIL_NUM_CURVES = 5;  // (REGTYPE_FCT)
	REG_OIL_RP_AVG 			= 0;
	REG_AO_MANUAL_VAL		= FCT_AO_MANUAL_VAL;
    REG_AO_TRIMLO			= FCT_AO_TRIMLO;
	REG_AO_TRIMHI 			= FCT_AO_TRIMHI;
	REG_DENSITY_ADJ 		= FCT_DENSITY_ADJ;
	REG_DENS_CORR 		    = 0;
	REG_AO_OUTPUT			= 0;
	REG_OIL_DENSITY_MODBUS	= FCT_OIL_DENSITY_MODBUS;
	REG_OIL_DENSITY_AI		= FCT_OIL_DENSITY_AI;
    REG_OIL_DENSITY_MANUAL	= FCT_OIL_DENSITY_MANUAL;
	REG_AI_TRIMLO			= FCT_AI_TRIMLO;
	REG_AI_TRIMHI 			= FCT_AI_TRIMHI;
	REG_AI_MEASURE			= FCT_AI_MEASURE;
	REG_AI_TRIMMED			= FCT_AI_TRIMMED;
	REG_DENS_ADJ 			= FCT_DENS_ADJ;
	REG_OIL_PT 	            = 0;

    PDI_TEMP_ADJ            = 0;
    PDI_FREQ_F0             = 0;
    PDI_FREQ_F1             = 0;

    //////////////////////////////////////
    /// Salinity (9) - 0.02
    //////////////////////////////////////

    VAR_Initialize(&FCT_SALINITY, c_concentration, u_conc_perc_solid_per_weight, 100.0, 100.0, 0); 
    VAR_Setup_Unit(&FCT_SALINITY, u_conc_perc_solid_per_weight, 30.0, 0.02, 30.0, 0.02);
    VAR_Update(&FCT_SALINITY, 0.02, CALC_UNIT);

    //////////////////////////////////////
    /// Oil Phase Adjust (15) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_OIL_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
    VAR_Update(&FCT_OIL_ADJUST, 0.0, CALC_UNIT);
    //FCT_OIL_ADJUST.swi = Swi_FCT_OIL_ADJUST; // SAVE STERAM DEPENDENT DATA AFTER FCT_OIL_ADJUST UPDATED.

    //////////////////////////////////////
    /// Water Phase Adjust (17) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_WATER_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_WATER_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
    VAR_Update(&FCT_WATER_ADJUST, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Temperature Adjustment (31) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_TEMP_ADJUST, c_temperature, u_mfgr_specific_none, 100.0, 100.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_TEMP_ADJUST, 0.0, CALC_UNIT);

    //////////////////////////////////////
	/// Process Averaging - 5.0
	//////////////////////////////////////

	VAR_Initialize(&FCT_PROC_AVGING, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&FCT_PROC_AVGING, u_mfgr_specific_none, MAXBUF, 1.0, MAXBUF, 1.0);
	VAR_Update(&FCT_PROC_AVGING, 1.0, CALC_UNIT);

    //////////////////////////////////////
    /// Oil Index (37) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_INDEX, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_OIL_INDEX, u_mfgr_specific_MHz, 1000.0, -1000.0, 1000.0, -1000.0);
    VAR_Update(&FCT_OIL_INDEX, 0.0, CALC_UNIT);

    /////////////////////////////////////
    /// Oil P0 (39) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_P0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_P0, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Oil P1 (41) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_P1, c_not_classified, u_mfgr_specific_V_per_MHz, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_P1, 0.0, CALC_UNIT);

    ////////////////////////////////////// 
    /// Oil Frequency Low (43) - 10
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_FREQ_LOW, c_frequency, u_mfgr_specific_MHz, 10, 1000.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_OIL_FREQ_LOW, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
    VAR_Update(&FCT_OIL_FREQ_LOW, 10, CALC_UNIT);

    //////////////////////////////////////
    /// Oil Frequency High (45) - 1000.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_FREQ_HIGH, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_OIL_FREQ_HIGH, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
    VAR_Update(&FCT_OIL_FREQ_HIGH, 1000.0, CALC_UNIT);

    //////////////////////////////////////
	/// Period for Sampling - 60.0
	//////////////////////////////////////

	VAR_Initialize(&FCT_SAMPLE_PERIOD, c_time, u_time_sec, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&FCT_SAMPLE_PERIOD, u_time_sec, 86400, 1, 86400, 1);
	VAR_Update(&FCT_SAMPLE_PERIOD, 60.0, CALC_UNIT);

    //////////////////////////////////////
    /// Analog out Lower Range Value (49) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_AO_LRV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_AO_LRV, u_ana_percent, 100.0, 0, 100.0, 0);
    VAR_Update(&FCT_AO_LRV, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Analog out Upper Range Value (51) - 100.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_AO_URV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
    VAR_Setup_Unit(&FCT_AO_URV, u_ana_percent, 100.0, 0, 100.0, 0);
    VAR_Update(&FCT_AO_URV, 100.0, CALC_UNIT);

	//////////////////////////////////////
	/// Modbus Baud Rate - 9600 Bps
	//////////////////////////////////////

	VAR_Initialize(&FCT_BAUD_RATE, c_not_classified, u_mfgr_specific_bps, 0.01, 0.01, var_no_alarm);
	VAR_Setup_Unit(&FCT_BAUD_RATE, u_mfgr_specific_bps, 115200, 300, 115200, 300);
	VAR_Update(&FCT_BAUD_RATE, 9600.0, CALC_UNIT);

    //////////////////////////////////////
    /// Stream Select - 1.0
	//////////////////////////////////////

    VAR_Initialize(&FCT_STREAM, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
    VAR_Setup_Unit(&FCT_STREAM, u_mfgr_specific_none, SMAX, 1.0, SMAX, 1.0);
    VAR_Update(&FCT_STREAM, 1.0, CALC_UNIT);
    //REG_STREAM.swi = Swi_REG_STREAM; // RESTORE STREAM DEPENDENT DATA AFTER CHANGING STREAM 

    //////////////////////////////////////
	/// Oil Calibration Test Sample Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&FCT_OIL_SAMPLE, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&FCT_OIL_SAMPLE, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	VAR_Update(&FCT_OIL_SAMPLE, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Density Adjustment 3rd Order (117) - 0.00
    //////////////////////////////////////

    VAR_Initialize(&FCT_DENSITY_D3, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_DENSITY_D3, 0.0, CALC_UNIT);

    //////////////////////////////////////
    ///  Density Adjustment 2nd Order (119) - 0.00
    //////////////////////////////////////

    VAR_Initialize(&FCT_DENSITY_D2, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_DENSITY_D2, 0.0, CALC_UNIT);

    //////////////////////////////////////
    ///  Density Adjustment Slope (121) - 0.16 
    //////////////////////////////////////

    VAR_Initialize(&FCT_DENSITY_D1, c_analytical, u_mfgr_specific_perc_per_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_DENSITY_D1, -0.0286, CALC_UNIT);

    //////////////////////////////////////
    /// Density Adjustment Offset (123) - 0.0 
    //////////////////////////////////////

    VAR_Initialize(&FCT_DENSITY_D0, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_DENSITY_D0, 24.6, CALC_UNIT);

    //////////////////////////////////////
    /// Density correction zeroth-order coefficient (125)
    //////////////////////////////////////

    VAR_Initialize(&FCT_DENSITY_CAL_VAL, c_mass_per_volume, u_mpv_deg_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_DENSITY_CAL_VAL, 0.0, CALC_UNIT);

	//////////////////////////////////////
   	/// Relay Setpoint - 0.0
	//////////////////////////////////////

	VAR_Initialize(&FCT_RELAY_SETPOINT, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&FCT_RELAY_SETPOINT, u_ana_percent, 100.0, 0.0, 105.0, -3.0);
	VAR_Update(&FCT_RELAY_SETPOINT, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Analog In Lower Range Value (163) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_DENSITY_AI_LRV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_DENSITY_AI_LRV, 750.0, CALC_UNIT);

    //////////////////////////////////////
    /// Analog In Upper Range Value (165) - 80.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_DENSITY_AI_URV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_DENSITY_AI_URV, 998.0, CALC_UNIT);

    //////////////////////////////////////
    /// Oil T0 (179) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_T0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_T0, 0.0, CALC_UNIT);

    //////////////////////////////////////
    /// Oil T1 (181) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&FCT_OIL_T1, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    VAR_Update(&FCT_OIL_T1, 0.0, CALC_UNIT);

/* ---------- REGULAR VAR REGISTERS ----------*/
 
    //////////////////////////////////////
   	/// Watercut - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATERCUT, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_WATERCUT, u_ana_percent, 100.0, 0.0, 105.0, -3.0);
	VAR_Update(&REG_WATERCUT, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Temperature - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMPERATURE, c_temperature, u_temp_C, 10.0, 10.0, var_no_bound);
	VAR_Setup_Unit(&REG_TEMPERATURE, u_temp_C, 350.0, -50.0, 310, 0);
	VAR_Update(&REG_TEMPERATURE, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// REG_EMULSION_PHASE - NOT USED
	//////////////////////////////////////

	VAR_Initialize(&REG_EMULSION_PHASE, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_EMULSION_PHASE, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	VAR_Update(&REG_EMULSION_PHASE, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Salinity - 0.02
	//////////////////////////////////////

	VAR_Initialize(&REG_SALINITY, c_concentration, u_conc_perc_solid_per_weight, 100.0, 100.0, 0);
	VAR_Setup_Unit(&REG_SALINITY, u_conc_perc_solid_per_weight, 30.0, 0.02, 30.0, 0.02);
	//VAR_Update(&REG_SALINITY, 0.02, CALC_UNIT);
	VAR_Update(&REG_SALINITY, FCT_SALINITY.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Phase Adjust - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	//VAR_Update(&REG_OIL_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_ADJUST, FCT_OIL_ADJUST.calc_val, CALC_UNIT);
    REG_OIL_ADJUST.swi = Swi_REG_OIL_ADJUST; // SAVE STERAM DEPENDENT DATA AFTER REG_OIL_ADJUST UPDATED.

	//////////////////////////////////////
	/// Water Phase Adjust - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATER_ADJUST, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_WATER_ADJUST, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	//VAR_Update(&REG_WATER_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_WATER_ADJUST, FCT_WATER_ADJUST.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Frequency - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_FREQ, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_FREQ, u_mfgr_specific_MHz, 1000.0, 0.0, 1001.0, -1.0);
	VAR_Update(&REG_FREQ, 0.0, 0);

	//////////////////////////////////////
	/// Average Frequency - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_FREQ_AVG, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_FREQ_AVG, u_mfgr_specific_MHz, 1000.0, 0.0, 1001.0, -1.0);
	VAR_Update(&REG_FREQ_AVG, 0.0, 0);

	//////////////////////////////////////
   	/// Average Watercut - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_WATERCUT_AVG, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_WATERCUT_AVG, u_ana_percent, 100.0, -100.0, 105.0, -3.0);
	VAR_Update(&REG_WATERCUT_AVG, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Average Temperature - N/A
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMP_AVG, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 10.0, 10.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_AVG, u_temp_C, 350.0, -50.0, 310, 0);
    VAR_Update(&REG_TEMP_AVG, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Temperature Adjustment - 0.0
	//////////////////////////////////////

	//VAR_Initialize(&REG_TEMP_ADJUST, c_temperature, u_mfgr_specific_none, 100.0, 100.0, var_no_bound|var_no_alarm);
	VAR_Initialize(&REG_TEMP_ADJUST, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 100.0, 100.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_ADJUST, REG_TEMPERATURE.calc_unit, 350.0, -50.0, 310, 0);
	//VAR_Update(&REG_TEMP_ADJUST, 0.0, CALC_UNIT);
	VAR_Update(&REG_TEMP_ADJUST, FCT_TEMP_ADJUST.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// User Temperature - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_TEMP_USER, REG_TEMPERATURE.class, REG_TEMPERATURE.calc_unit, 10.0, 10.0, var_no_bound|var_no_alarm);
	//VAR_Setup_Unit(&REG_TEMP_USER, REG_TEMPERATURE.calc_unit, 350.0, -50.0, 310, 0);
	VAR_Update(&REG_TEMP_USER, 0.0, CALC_UNIT);

	//////////////////////////////////////
	/// Process Averaging - 5.0
	//////////////////////////////////////

	VAR_Initialize(&REG_PROC_AVGING, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&REG_PROC_AVGING, u_mfgr_specific_none, MAXBUF, 1.0, MAXBUF, 1.0);
	VAR_Update(&REG_PROC_AVGING, FCT_PROC_AVGING.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Index - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_INDEX, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_INDEX, u_mfgr_specific_MHz, 1000.0, -1000.0, 1000.0, -1000.0);
	//VAR_Update(&REG_OIL_INDEX, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_INDEX, FCT_OIL_INDEX.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil P0 - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_P0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_P0, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_P0, FCT_OIL_P0.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil P1 - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_P1, c_not_classified, u_mfgr_specific_V_per_MHz, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_P1, 0.0, CALC_UNIT);
	VAR_Update(&REG_OIL_P1, FCT_OIL_P1.calc_val, CALC_UNIT);

    ////////////////////////////////////// 
    /// Oil Frequency Low - 10
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_FREQ_LOW, c_frequency, u_mfgr_specific_MHz, 10, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_FREQ_LOW, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
	//VAR_Update(&REG_OIL_FREQ_LOW, 10, CALC_UNIT);
	VAR_Update(&REG_OIL_FREQ_LOW, FCT_OIL_FREQ_LOW.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Oil Frequency High - 1000.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_FREQ_HIGH, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_FREQ_HIGH, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
	//VAR_Update(&REG_OIL_FREQ_HIGH, 1000.0, CALC_UNIT);
	VAR_Update(&REG_OIL_FREQ_HIGH, FCT_OIL_FREQ_HIGH.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Period for Sampling - 60.0
	//////////////////////////////////////

	VAR_Initialize(&REG_SAMPLE_PERIOD, c_time, u_time_sec, 1.0, 1.0, var_no_alarm|var_round);
	VAR_Setup_Unit(&REG_SAMPLE_PERIOD, u_time_sec, 86400, 1, 86400, 1);
	VAR_Update(&REG_SAMPLE_PERIOD, FCT_SAMPLE_PERIOD.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog out Lower Range Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_AO_LRV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_AO_LRV, u_ana_percent, 100.0, 0, 100.0, 0);
	//VAR_Update(&REG_AO_LRV, 0.0, CALC_UNIT);
	VAR_Update(&REG_AO_LRV, FCT_AO_LRV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog out Upper Range Value - 85.0
	//////////////////////////////////////

	VAR_Initialize(&REG_AO_URV, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_alarm);
	VAR_Setup_Unit(&REG_AO_URV, u_ana_percent, 100.0, 0, 100.0, 0);
	//VAR_Update(&REG_AO_URV, 100.0, CALC_UNIT);
	VAR_Update(&REG_AO_URV, FCT_AO_URV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Modbus Baud Rate - 9600 Bps
	//////////////////////////////////////

	VAR_Initialize(&REG_BAUD_RATE, c_not_classified, u_mfgr_specific_bps, 0.01, 0.01, var_no_alarm);
	VAR_Setup_Unit(&REG_BAUD_RATE, u_mfgr_specific_bps, 115200, 300, 115200, 300);
	VAR_Update(&REG_BAUD_RATE, FCT_BAUD_RATE.calc_val, CALC_UNIT);

	//////////////////////////////////////
    /// Stream Select - 1.0
	//////////////////////////////////////

    VAR_Initialize(&REG_STREAM, c_not_classified, u_mfgr_specific_none, 1.0, 1.0, var_no_alarm|var_round);
    VAR_Setup_Unit(&REG_STREAM, u_mfgr_specific_none, SMAX, 1.0, SMAX, 1.0);
    VAR_Update(&REG_STREAM, FCT_STREAM.calc_val, CALC_UNIT); 
    REG_STREAM.swi = Swi_REG_STREAM; // RESTORE STREAM DEPENDENT DATA AFTER CHANGING STREAM 

	//////////////////////////////////////
	/// Oil Calibration Test Sample Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_SAMPLE, c_volume_per_volume, u_vpv_volume_percent, 100.0, 100.0, var_no_alarm);
	VAR_Setup_Unit(&REG_OIL_SAMPLE, u_vpv_volume_percent, 100.0, -100.0, 100.0, -100.0);
	VAR_Update(&REG_OIL_SAMPLE, FCT_OIL_SAMPLE.calc_val, CALC_UNIT);
	REG_OIL_SAMPLE.swi = Swi_REG_OIL_SAMPLE; // will be triggered after REG_OIL_SAMPLE is updated insde ModbusRTU.c

	//////////////////////////////////////
	/// Density Adjustment 3rd Order - 0.00
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D3, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D3, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D3, FCT_DENSITY_D3.calc_val, CALC_UNIT);

	//////////////////////////////////////
	///  Density Adjustment 2nd Order - 0.00
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D2, c_analytical, u_mfgr_specific_none, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D2, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D2, FCT_DENSITY_D2.calc_val, CALC_UNIT);

	//////////////////////////////////////
	///  Density Adjustment Slope - 0.16 
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D1, c_analytical, u_mfgr_specific_perc_per_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D1, 0.16, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D1, FCT_DENSITY_D1.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Density Adjustment Offset - 0.0 
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_D0, c_analytical, u_ana_percent, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_D0, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_D0, FCT_DENSITY_D0.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Density correction zeroth-order coefficient
	//////////////////////////////////////

	VAR_Initialize(&REG_DENSITY_CAL_VAL, c_mass_per_volume, u_mpv_deg_API_60F, 100.0, 10000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_DENSITY_CAL_VAL, 0.0, CALC_UNIT);
	VAR_Update(&REG_DENSITY_CAL_VAL, FCT_DENSITY_CAL_VAL.calc_val, CALC_UNIT);
	REG_DENSITY_CAL_VAL.swi = Swi_Set_REG_DENSITY_CAL_Unit;

	//////////////////////////////////////
   	/// Relay Setpoint - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_RELAY_SETPOINT, c_analytical, u_ana_percent, 100.0, 1000.0, var_dampen|var_NaNproof);
	VAR_Setup_Unit(&REG_RELAY_SETPOINT, u_ana_percent, 100.0, 0.0, 105.0, -3.0);
	VAR_Update(&REG_RELAY_SETPOINT, FCT_RELAY_SETPOINT.calc_val, CALC_UNIT);
	
	//////////////////////////////////////
   	/// Oil Density @ Process Temperature - 865.443
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY, c_mass_per_volume, u_mpv_kg_cm, 10.0, 1000.0, var_no_bound|var_no_alarm);
	VAR_Update(&REG_OIL_DENSITY, 865.443, CALC_UNIT);

	//////////////////////////////////////
	/// Analog In Lower Range Value - 0.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY_AI_LRV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_DENSITY_AI_LRV, 750.0, CALC_UNIT);
	VAR_Update(&REG_OIL_DENSITY_AI_LRV, FCT_OIL_DENSITY_AI_LRV.calc_val, CALC_UNIT);

	//////////////////////////////////////
	/// Analog In Upper Range Value - 80.0
	//////////////////////////////////////

	VAR_Initialize(&REG_OIL_DENSITY_AI_URV, c_mass_per_volume, u_mpv_kg_cm_15C, 10.0, 1000.0, var_no_bound|var_no_alarm);
	//VAR_Update(&REG_OIL_DENSITY_AI_URV, 998.0, CALC_UNIT);
	VAR_Update(&REG_OIL_DENSITY_AI_URV, FCT_OIL_DENSITY_AI_URV.calc_val, CALC_UNIT);

    //////////////////////////////////////
    /// Oil T0 (179) - 0.0
    //////////////////////////////////////
 
    VAR_Initialize(&REG_OIL_T0, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    //VAR_Update(&REG_OIL_T0, 0.0, CALC_UNIT);
    VAR_Update(&REG_OIL_T0, FCT_OIL_T0.calc_val, CALC_UNIT);
 
    //////////////////////////////////////
    /// Oil T1 (181) - 0.0
    //////////////////////////////////////

    VAR_Initialize(&REG_OIL_T1, c_emf, u_emf_V, 100.0, 10000.0, var_no_bound|var_no_alarm);
    //VAR_Update(&REG_OIL_T1, 0.0, CALC_UNIT);
    VAR_Update(&REG_OIL_T1, FCT_OIL_T1.calc_val, CALC_UNIT);

	for (i=0;i<NUMBER_OF_OSC;i++)
	{
		VAR_Initialize(&REG_FREQ, c_frequency, u_mfgr_specific_MHz, 10.0, 1000.0, var_no_alarm);
		VAR_Setup_Unit(&REG_FREQ, u_mfgr_specific_MHz, 1000.0, 0.0, 1000.0, 0.0);
		VAR_Update(&REG_FREQ, 92.0, CALC_UNIT);
	}


    //////////////////////////////////////
    /// EXTENDED 
    //////////////////////////////////////

    REG_TEMP_OIL_NUM_CURVES     = 5; // (REGPERM_FCT) used
    REG_SALINITY_CURVES         = 0; // (REGPERM_FCT) not used
    REG_WATER_CURVES            = 0; // (REGPERM_FCT) not used

    memset(REG_TEMPS_OIL, 0, sizeof(REG_TEMPS_OIL));
    memset(REG_COEFFS_TEMP_OIL, 0, sizeof(REG_COEFFS_TEMP_OIL));
    memset(REG_COEFFS_SALINITY, 0, sizeof(REG_COEFFS_SALINITY));
    memset(REG_WATER_TEMPS, 0, sizeof(REG_WATER_TEMPS));
    memset(REG_COEFFS_TEMP_WATER, 0, sizeof(REG_COEFFS_TEMP_WATER));
    memset(REG_STRING_TAG, 0, sizeof(REG_STRING_TAG));
    memset(REG_STRING_LONGTAG, 0, sizeof(REG_STRING_LONGTAG));
    memset(REG_STRING_INITIAL, 0, sizeof(REG_STRING_INITIAL));
    memset(REG_STRING_MEAS, 0, sizeof(REG_STRING_MEAS));
    memset(REG_STRING_ASSEMBLY, 0, sizeof(REG_STRING_ASSEMBLY));
    memset(REG_STRING_INFO, 0, sizeof(REG_STRING_INFO));
    memset(REG_STRING_PVNAME, 0, sizeof(REG_STRING_PVNAME));
    memset(REG_STRING_PVUNIT, 0, sizeof(REG_STRING_PVUNIT));
    memset(STREAM_TIMESTAMP, 0, sizeof(STREAM_TIMESTAMP));
    memset(STREAM_OIL_ADJUST, 0, sizeof(STREAM_OIL_ADJUST));
    memset(STREAM_WATERCUT_AVG, 0, sizeof(STREAM_WATERCUT_AVG));
    memset(STREAM_SAMPLES, 0, sizeof(STREAM_SAMPLES));

    //////////////////////////////////////
	/// COILS
	//////////////////////////////////////

    COIL_Initialize(&COIL_RELAY[0], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[1], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[2], FALSE, 0);
	COIL_Initialize(&COIL_RELAY[3], FALSE, 0);
	COIL_Initialize(&COIL_BEGIN_OIL_CAP, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ENABLE, FALSE, 0);
    COIL_Initialize(&COIL_LOG_ALARMS, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ERRORS, FALSE, 0);
	COIL_Initialize(&COIL_LOG_ACTIVITY, FALSE, 0);
    COIL_Initialize(&COIL_AO_ALARM, TRUE, 0);
	COIL_Initialize(&COIL_PARITY, FALSE, 0);
	COIL_Initialize(&COIL_WRITE_RTC, FALSE, 0);
	COIL_Initialize(&COIL_OIL_DENS_CORR_EN, TRUE, 0); 
	COIL_Initialize(&COIL_AO_MANUAL, FALSE, 0);
	COIL_Initialize(&COIL_MB_AUX_SELECT_MODE, FALSE, 0);
	COIL_Initialize(&COIL_INTEGER_TABLE_SELECT, FALSE, 0);
	COIL_Initialize(&COIL_AVGTEMP_RESET, FALSE, 0); 
    COIL_Initialize(&COIL_AVGTEMP_MODE, FALSE, 0);  	
	COIL_Initialize(&COIL_OIL_PHASE, TRUE, 0);
	COIL_Initialize(&COIL_ACT_RELAY_OIL, TRUE, 0);
	COIL_Initialize(&COIL_RELAY_MANUAL, FALSE, 0);
	COIL_Initialize(&COIL_UNLOCKED, TRUE, 0);
	COIL_Initialize(&COIL_AO_TRIM_MODE, FALSE, 0);
	COIL_Initialize(&COIL_AI_TRIM_MODE, FALSE, 0);
    COIL_Initialize(&COIL_LOCKED_SOFT_FACTORY_RESET, TRUE, 0);
	COIL_Initialize(&COIL_LOCKED_HARD_FACTORY_RESET, TRUE, 0);
	COIL_Initialize(&COIL_UPDATE_FACTORY_DEFAULT, FALSE, 0);
	COIL_Initialize(&COIL_UNLOCKED_FACTORY_DEFAULT, FALSE, 0);

	setDemoValues(); // from SN8563
}

void setDemoValues(void)
{
    Uint16 i,j;
	char model_code[MAX_LCD_WIDTH];
	int* model_code_int;

	DIAGNOSTICS = 0;
	LOCK_FACTORY = TRUE;

    REG_SN_PIPE = 8409;

    const double DEMO_P0 = +2.0000e+00;
    const double DEMO_P1 = +0.0000e-00;
    const double DEMO_T0 = +0.0000e-00;
    const double DEMO_T1 = +0.0000e-00;

    const double DEMO_FL = +2.6700e+02;
    const double DEMO_FH = +1.0000e+03;

    const double DEMO_CUTOFF = +2.0000e+01;

    const double DEMO_AO_LO = +4.0880000e+00;
    const double DEMO_AO_HI = +2.0362000e+01;
    const double DEMO_AI_LO = +9.9370870e+00;
    const double DEMO_AI_HI = +4.9848736e+01;

    const double DEMO_REG_TEMPS_OIL[10] = {+1.555556e+01,+3.777778e+01,+6.000000e+01,+1.555556e+01,+3.777778e+01,+6.000000e+01,+0.000000e+00,+0.000000e+00,+0.000000e+00,+0.000000e+00};
	const double DEMO_REG_COEFFS_TEMP_OIL[10][4] = {	
		{+2.3843453E+04,-2.6330353E+02,+9.7513338E-01,-1.2102951E-03},
		{+2.3843453E+04,-2.6330353E+02,+9.7513338E-01,-1.2102951E-03},
		{+2.3843453E+04,-2.6330353E+02,+9.7513338E-01,-1.2102951E-03},
		{+2.0000000E+01,+0.0000000E+00,+0.0000000E+00,+0.0000000E+00},
		{+2.0000000E+01,+0.0000000E+00,+0.0000000E+00,+0.0000000E+00},
		{+2.0000000E+01,+0.0000000E+00,+0.0000000E+00,+0.0000000E+00},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00}
	}; // curves modified from SN8409

    // P0 and P1 - FACTORY/USER
	VAR_Update(&FCT_OIL_P0, DEMO_P0, CALC_UNIT);
	VAR_Update(&FCT_OIL_P1, DEMO_P1, CALC_UNIT);
	VAR_Update(&REG_OIL_P0, DEMO_P0, CALC_UNIT);
	VAR_Update(&REG_OIL_P1, DEMO_P1, CALC_UNIT);

    // T0 and T1 - FACTORY/USER
    VAR_Update(&FCT_OIL_T0, DEMO_T0, CALC_UNIT);
	VAR_Update(&FCT_OIL_T1, DEMO_T1, CALC_UNIT);
    VAR_Update(&REG_OIL_T0, DEMO_T0, CALC_UNIT);
	VAR_Update(&REG_OIL_T1, DEMO_T1, CALC_UNIT);

    // OIL_FREQ_LOW - FACTORY/USER
	VAR_Update(&REG_OIL_FREQ_LOW, DEMO_FL, CALC_UNIT);
	VAR_Update(&FCT_OIL_FREQ_LOW, DEMO_FL, CALC_UNIT);

    // OIL_FREQ_HIGH - FACTORY/USER
	VAR_Update(&REG_OIL_FREQ_HIGH, DEMO_FH, CALC_UNIT);
	VAR_Update(&FCT_OIL_FREQ_HIGH, DEMO_FH, CALC_UNIT);

    // OIL_PHASE_CUTOFF - FACTORY/USER
	REG_OIL_PHASE_CUTOFF = DEMO_CUTOFF;
	FCT_OIL_PHASE_CUTOFF = DEMO_CUTOFF;

    // AO TRIM LO
    REG_AO_TRIMLO = DEMO_AO_LO;
    FCT_AO_TRIMLO = DEMO_AO_LO;

    // AO TRIM HI
    REG_AO_TRIMHI = DEMO_AO_HI;
    FCT_AO_TRIMHI = DEMO_AO_HI;

    // AI TRIM LO
    REG_AI_TRIMLO = DEMO_AI_LO;
    FCT_AI_TRIMLO = DEMO_AI_LO;

    // AI TRIM HI
    REG_AI_TRIMHI = DEMO_AI_HI;
    FCT_AI_TRIMHI = DEMO_AI_HI;

	// set up temperature curve ranges
	for(i=0;i<10;i++) 
    {
        REG_TEMPS_OIL[i] = DEMO_REG_TEMPS_OIL[i];
    }

	// set up oil temperature curves
	for(i=0;i<10;i++) 
    {
        for(j=0;j<4;j++) REG_COEFFS_TEMP_OIL[i][j] = DEMO_REG_COEFFS_TEMP_OIL[i][j];
    }

	// default value for dual-curve is '5' (0 through 5, so 6 in total)
	REG_TEMP_OIL_NUM_CURVES = 5;

    sprintf(model_code,DEFAULT_MODEL_CODE); //default model code
	model_code_int = (int*)model_code;
	for (i=0;i<4;i++) REG_MODEL_CODE[i] = model_code_int[i];
/*
    REG_SN_PIPE = 8563 ;

    Uint16 i,j;
	char model_code[MAX_LCD_WIDTH];
	int* model_code_int;

	DIAGNOSTICS = 0;
	LOCK_FACTORY = TRUE;

    const double DEMO_P0 = +1.01625e+01;
    const double DEMO_P1 = -3.25000e-02;
    const double DEMO_T0 = -1.69060e-01;
    const double DEMO_T1 = +4.48400e-03;

    const double DEMO_FL = +2.53000e+02;
    const double DEMO_FH = +1.00000e+03;

    const double DEMO_CUTOFF = 1.00000e+02;

    const double DEMO_AO_LO = +4.0880000e+00;
    const double DEMO_AO_HI = +2.0362000e+01;
    const double DEMO_AI_LO = +9.9370870e+00;
    const double DEMO_AI_HI = +4.9848736e+01;

    const double DEMO_REG_TEMPS_OIL[10] = {15,37,60,15,37,60,0,0,0,0};
	const double DEMO_REG_COEFFS_TEMP_OIL[10][4] = {	
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{+2.5079020E+04,-2.7060879E+02,+9.8012943E-01,-1.1907706E-03},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00},
		{0.00,0.00,0.00,0.00}
	}; // curves modified from SN8563

    // P0 and P1 - FACTORY/USER
	VAR_Update(&FCT_OIL_P0, DEMO_P0, CALC_UNIT);
	VAR_Update(&FCT_OIL_P1, DEMO_P1, CALC_UNIT);
	VAR_Update(&REG_OIL_P0, DEMO_P0, CALC_UNIT);
	VAR_Update(&REG_OIL_P1, DEMO_P1, CALC_UNIT);

    // T0 and T1 - FACTORY/USER
    VAR_Update(&FCT_OIL_T0, DEMO_T0, CALC_UNIT);
	VAR_Update(&FCT_OIL_T1, DEMO_T1, CALC_UNIT);
    VAR_Update(&REG_OIL_T0, DEMO_T0, CALC_UNIT);
	VAR_Update(&REG_OIL_T1, DEMO_T1, CALC_UNIT);

    // OIL_FREQ_LOW - FACTORY/USER
	VAR_Update(&REG_OIL_FREQ_LOW, DEMO_FL, CALC_UNIT);
	VAR_Update(&FCT_OIL_FREQ_LOW, DEMO_FL, CALC_UNIT);

    // OIL_FREQ_HIGH - FACTORY/USER
	VAR_Update(&REG_OIL_FREQ_HIGH, DEMO_FH, CALC_UNIT);
	VAR_Update(&FCT_OIL_FREQ_HIGH, DEMO_FH, CALC_UNIT);

    // OIL_PHASE_CUTOFF - FACTORY/USER
	REG_OIL_PHASE_CUTOFF = DEMO_CUTOFF;
	FCT_OIL_PHASE_CUTOFF = DEMO_CUTOFF;

    // AO TRIM LO
    REG_AO_TRIMLO = DEMO_AO_LO;
    FCT_AO_TRIMLO = DEMO_AO_LO;

    // AO TRIM HI
    REG_AO_TRIMHI = DEMO_AO_HI;
    FCT_AO_TRIMHI = DEMO_AO_HI;

    // AI TRIM LO
    REG_AI_TRIMLO = DEMO_AI_LO;
    FCT_AI_TRIMLO = DEMO_AI_LO;

    // AI TRIM HI
    REG_AI_TRIMHI = DEMO_AI_HI;
    FCT_AI_TRIMHI = DEMO_AI_HI;

	// set up temperature curve ranges
	for(i=0;i<10;i++) 
    {
        REG_TEMPS_OIL[i] = DEMO_REG_TEMPS_OIL[i];
    }

	// set up oil temperature curves
	for(i=0;i<10;i++) 
    {
        for(j=0;j<4;j++) REG_COEFFS_TEMP_OIL[i][j] = DEMO_REG_COEFFS_TEMP_OIL[i][j];
    }

	// default value for dual-curve is '5' (0 through 5, so 6 in total)
	REG_TEMP_OIL_NUM_CURVES = 5;

    sprintf(model_code,DEFAULT_MODEL_CODE); //default model code
	model_code_int = (int*)model_code;
	for (i=0;i<4;i++) REG_MODEL_CODE[i] = model_code_int[i];
*/
}

double Round_N(double v, int n)
{ /// Round_N to the nth decimal place
    double a, ai;

    if (n>6) return v;
    else if (n<0) return v;

    a = v*pow(10.0, (double)(n+1));

    if(v<0.0) a -= 5.0;
    else a += 5.0;

    a /= 10.0;
    modf(a, &ai);

    return ai/pow(10.0, (double)n);
}

float Round_N_Float(float v, int n)
{ /// Round_N to the nth decimal place
    double a, ai;

    if (n>6) return v;
    else if (n<0) return v;

    a = v*pow(10.0, (double)(n+1));

    if(v<0.0) a -= 5.0;
    else a += 5.0;

    a /= 10.0;
    modf(a, &ai);

    return (float)(ai/pow(10.0, (double)n));
}

double sigfig(double v, int n)
{
	int s;
	double t;
	double a;

    if(n<1) return v;
    else if (n>6) return v;
	else if(v==0.0) return v;

    if(v<0.0) s = -1;
    else s = 1;

    a = fabs(v);

    t = pow(10.0, floor(log10(a))+1.0);

    return (s*Round_N(a/t, n)*t);
}

double truncate(double v, int n)
{
    double a, ai;

    if (n>6) return v;
    else if (n<1) return v;

    a = v*pow(10.0, (double)(n+1));

    a /= 10.0;
    modf(a, &ai);

    return (ai/pow(10.0, (double)n));
}
