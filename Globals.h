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
* Globals.h
*-------------------------------------------------------------------------
* With few exceptions, virtually every global variable is declared in 
* Globals.h and defined/initialized in the Globals.c function reloadFactoryDefaultToUserData(). 
* reloadFactoryDefaultToUserData() is called during the boot-up sequence before starting 
* the TI-RTOS kernel. 
* Important note: after running reloadFactoryDefaultToUserData() these default values are 
* saved to flash memory. On subsequent boots, reloadFactoryDefaultToUserData() does NOT RUN. 
* Instead, the software pulls the saved values from flash memory. 
* (Otherwise all user settings and data would be reset to factory defaults 
* every reboot). The software uses a magic word in flash memory to determine 
* whether to run reloadFactoryDefaultToUserData() or restore data from flash. (See note at the 
* top of Main.c) To FORCE the software to run reloadFactoryDefaultToUserData(), simply #define 
* FORCE_INIT_GLOBALS and run the software with the debugger. 
* DO NOT create a boot image from the software if it has FORCE_INIT_GLOBALS defined!
*-------------------------------------------------------------------------*/

#ifndef GLOBALS_H_
#define GLOBALS_H_

#define HARDWARE_VERSION		"1.1.1"
#define FIRMWARE_VERSION 		"1.2.0"

#define BOOL Uint8
#define DEMO_MODE

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <c6x.h>

#include <xdc/cfg/global.h> 		// header file for statically defined objects/handles
#include <xdc/std.h>				// mandatory - have to include first, for BIOS types
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>		// needed for any Log_info() call
#include <xdc/runtime/Timestamp.h>	// when using Timestamp APIs (TSCL/H), 32bit, 64bit

#include <ti/sysbios/BIOS.h>		// mandatory - if you call APIs like BIOS_start()
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c64p/Cache.h>

#include <ti/csl/src/ip/usb/V3/cslr_usb_otg.h>
#include <ti/csl/src/intc/cslr_intc.h>
#include <ti/csl/src/ip/emif4/V4/cslr_emifa2.h>
#include <ti/csl/cslr.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/cslr_psc.h>
#include <ti/csl/cslr_syscfg.h>
#include <ti/csl/cslr_uart.h>
#include <ti/csl/cslr_i2c.h>
#include <ti/csl/src/ip/gpio/V3/cslr_gpio.h>
#include <ti/csl/V1/cslr_tmr.h>
#include <ti/csl/cslr_rtc.h>
#include <ti/csl/soc/omapl138/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/omapl138/src/cslr_soc.h>

#include "Errors.h"
#include "Variable.h"
#include "Units.h"
#include "Buffers.h"
#include "ModbusRTU.h"

#ifdef  GLOBAL_VARS
#define _EXTERN
#else
#define _EXTERN extern
#endif

////////////////////////////////////////////////////
///// MAKE SURE TO CHANGE VERSION UPON NEW RELEASE
////////////////////////////////////////////////////

#define DEFAULT_MODEL_CODE 			"RI1R20202RX35B25"

////////////////////////////////////////////////////
////////////////////////////////////////////////////

#define MAX_LCD_WIDTH				16
#define NUMBER_OF_OSC				1
#define DELAY						20000000	// DO **NOT** CHANGE THIS VALUE
#define ANA_MODE_FULL				4		// fullcut
#define ANA_MODE_CCM				3		// CCM
#define ANA_MODE_MID				2		// midcut
#define ANA_MODE_LOW				1		// lowcut
#define ANA_MODE_SA					0		// standalone
#define SMAX 						60 		// max number of streams
#define MAXBUF  					10		// set to 300 in old code
#define MAX_DAMP_DELAY				50		// max damping period 50 secs
#define MAX_WATER_PHASE				100		// Watercut in water phase always 100% in water phase
#define PASSWORD_LENGTH				4
#define MAX_NAME_LENGTH				20
#define MAX_CSV_ARRAY_LENGTH		2048	
#define PDI_RAZOR_PROFILE 			"0:pdi_razor_profile.csv"
#define PDI_RAZOR_FIRMWARE 			"0:pdi_razor_firmware.ais"

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
///
/// GLOBAL VARS
///
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

_EXTERN int REG_RTC_SEC;		// RTC read-only: seconds
_EXTERN int REG_RTC_MIN;		// RTC read-only: minutes
_EXTERN int REG_RTC_HR;			// RTC read-only: hours
_EXTERN int REG_RTC_DAY;		// RTC read-only: day
_EXTERN int REG_RTC_MON;		// RTC read-only: month
_EXTERN int REG_RTC_YR;			// RTC read-only: year
_EXTERN int REG_RTC_SEC_IN;		// RTC read-only: seconds
_EXTERN int REG_RTC_MIN_IN;		// RTC read-only: minutes
_EXTERN int REG_RTC_HR_IN;		// RTC read-only: hours
_EXTERN int REG_RTC_DAY_IN;		// RTC read-only: day
_EXTERN int REG_RTC_MON_IN;		// RTC read-only: month
_EXTERN int REG_RTC_YR_IN;		// RTC read-only: year
_EXTERN int REG_DIAGNOSTICS;	// diagnostics 

_EXTERN unsigned int cycles;             		    /*phase hold over RS*/
_EXTERN unsigned int previous_phase;         		/*phase hold over RS*/
_EXTERN unsigned int phase;                  		/*phase hold over RS*/
_EXTERN unsigned int phase_rollover_count;   		/*phase hold over RS*/
_EXTERN int TEMP_STREAM;
_EXTERN char lcdLine0[MAX_LCD_WIDTH];
_EXTERN char lcdLine1[MAX_LCD_WIDTH];
_EXTERN char CSV_FILES[MAX_CSV_ARRAY_LENGTH];
_EXTERN int csvCounter;
_EXTERN int usbStatus;
_EXTERN Uint32 counter;
_EXTERN Uint32 delayTimer;
_EXTERN BOOL isUpdateDisplay;
_EXTERN BOOL isWriteRTC;
_EXTERN BOOL isLogData;
_EXTERN BOOL isUpgradeFirmware;
_EXTERN BOOL isDownloadCsv;
_EXTERN BOOL isScanCsvFiles;
_EXTERN BOOL isUploadCsv;
_EXTERN BOOL isResetPower;
_EXTERN BOOL isCsvUploadSuccess;
_EXTERN BOOL isCsvDownloadSuccess;
_EXTERN BOOL isScanSuccess;
_EXTERN BOOL isPdiRazorProfile;

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/// 
/// GLOBAL FUNCTION DECLARATIION
/// 
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

_EXTERN double Round_N (double v, int n);
_EXTERN float Round_N_Float (float v, int n);
_EXTERN double sigfig (double v, int n);
_EXTERN double truncate (double v, int n);
_EXTERN void setupMenu (void);
_EXTERN void logData(void);
_EXTERN void loadUsbDriver(void);
_EXTERN void resetUsbDriver(void);
_EXTERN void resetGlobalVars(void);
_EXTERN void delayTimerSetup(void);
_EXTERN void unloadUsbDriver(void);
_EXTERN void Init_Data_Buffer(void);
_EXTERN void Init_Data_Buffer(void);
_EXTERN void initializeAllRegisters(void);
_EXTERN void Read_RTC(int* p_sec, int* p_min, int* p_hr, int* p_day, int* p_mon, int* p_yr);
_EXTERN void reloadFactoryDefault(void);
_EXTERN void storeUserDataToFactoryDefault(void);
_EXTERN void _c_int00(void);

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/// 
/// TYPE DEFINITIONS
/// 
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

typedef struct
{
	// counts the occurrence of various UART errors
	// page 1455
	Uint32 	OE; 		//RBR overrun error - bit 1
	Uint32 	PE; 		//parity error	    - bit 2
	Uint32 	FE; 		//framing error	    - bit 3
	Uint32	RXFIFOE;	//parity error	    - bit 7
} UERROR;

typedef struct
{
	// this type defines a set of data buffers of size = [MAX_BFR_SIZE_F]
	FP_BFR WC_BUFFER;	//watercut
	FP_BFR T_BUFFER;	//temperature
	FP_BFR F_BUFFER;	//frequency
	FP_BFR RP_BUFFER;	//reflected power
} DATA_BFR;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// Calculate - READ_ONLY VARIABLES
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

	_EXTERN Uint32 	 FREQ_PULSE_COUNT_LO;
	_EXTERN Uint32 	 FREQ_PULSE_COUNT_HI;
	_EXTERN Uint32 	 FREQ_U_SEC_ELAPSED; 	// microseconds - time elapsed since last frequency pulse reading
	_EXTERN DATA_BFR DATALOG;
	
////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// NAND FLASH 
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

    _EXTERN unsigned char MSG_DIG[32];

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// PDI_I2C
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

	_EXTERN Uint8	LCD_IS_INIT;
	_EXTERN Uint8	LCD_RECENT_RESET;
	_EXTERN Uint8	I2C_FINISHED_TX;
	_EXTERN Uint8	I2C_RX_BYTE_COUNT;
	_EXTERN Uint8	LCD_BUSY_FLAG;
	_EXTERN Uint8	I2C_BACKLIGHT_EN;
	_EXTERN Uint8	I2C_BUTTON_CHOOSER;
	_EXTERN Uint8	I2C_BUTTON_STEP;
	_EXTERN Uint8	I2C_BUTTON_BACK;
	_EXTERN Uint8	I2C_BUTTON_VALUE;
	_EXTERN Uint8	I2C_BUTTON_ENTER;
	_EXTERN BFR 	I2C_TXBUF;
	_EXTERN BFR 	I2C_RXBUF;
 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// ModbusRTU
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

	_EXTERN BFR 	UART_TXBUF;
	_EXTERN BFR 	UART_RXBUF;
	_EXTERN UERROR	UART_ERROR_CNT;
	_EXTERN Uint8	MB_TX_IN_PROGRESS;
	_EXTERN Uint8	THROW_ERROR;
	_EXTERN Uint16	WDOG_BYTES_TO_REMOVE;
	_EXTERN MODBUS_PACKET_LIST MB_PKT_LIST;
	_EXTERN Uint32	STAT_SUCCESS;
	_EXTERN Uint32	STAT_PKT;
	_EXTERN Uint32	STAT_CMD;
	_EXTERN Uint32	STAT_RETRY;
	_EXTERN Uint8 	STAT_CURRENT;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// GLOBAL VARIALBE DEFINITIONS 
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#ifdef GLOBAL_VARS
    // USB OTG Overlays
    CSL_Usb_otgRegsOvly usbRegs = (CSL_Usb_otgRegsOvly)CSL_USB_0_REGS;

	// Register Overlays 
	CSL_I2cRegsOvly		i2cRegs = (CSL_I2cRegsOvly)CSL_I2C_0_DATA_CFG;

	// sys config registers overlay
	CSL_SyscfgRegsOvly 	sysRegs = (CSL_SyscfgRegsOvly)(CSL_SYSCFG_0_REGS);

	// Psc register overlay       
	CSL_PscRegsOvly    	psc1Regs = (CSL_PscRegsOvly)(CSL_PSC_1_REGS);

	// Gpio register overlay            
	CSL_GpioRegsOvly    	gpioRegs = (CSL_GpioRegsOvly)(CSL_GPIO_0_REGS);

    // Interrupt Controller Register Overlay    
    CSL_IntcRegsOvly     intcRegs = (CSL_IntcRegsOvly)CSL_INTC_0_REGS;

	// Uart register overlay
	CSL_UartRegsOvly 	uartRegs = (CSL_UartRegsOvly)CSL_UART_2_REGS;

	// EMIFA register overlay
	CSL_EmifaRegsOvly 	emifaRegs = (CSL_EmifaRegsOvly)CSL_EMIFA_0_REGS;

	// Timer register overlays
	CSL_TmrRegsOvly		tmr1Regs = (CSL_TmrRegsOvly)CSL_TMR_1_REGS;
	CSL_TmrRegsOvly		tmr3Regs = (CSL_TmrRegsOvly)CSL_TMR_3_REGS;

	// RTC register overlay	
	CSL_RtcRegsOvly		rtcRegs = (CSL_RtcRegsOvly)CSL_RTC_0_REGS;
	CSL_Syscfg1RegsOvly	sys1Regs = (CSL_Syscfg1RegsOvly)(CSL_SYSCFG_1_REGS);
#else
    // Usb Overlays
    extern CSL_Usb_otgRegsOvly usbRegs;

	// Register Overlays 
	extern CSL_I2cRegsOvly 	   i2cRegs;

	// sys config registers overlay
	extern CSL_SyscfgRegsOvly  sysRegs;

	// Psc register overlay
	extern CSL_PscRegsOvly     psc1Regs;

	// Gpio register overlay
	extern CSL_GpioRegsOvly    gpioRegs;

	// Interrupt Controller Register Overlay
	extern CSL_IntcRegsOvly intcRegs;

	// Uart register overlay
	extern CSL_UartRegsOvly	   uartRegs;

	// EMIFA register overlay
	extern CSL_EmifaRegsOvly   emifaRegs;

	// Timer register overlays
	extern CSL_TmrRegsOvly 	   tmr1Regs;
	extern CSL_TmrRegsOvly 	   tmr3Regs;

	// RTC register overlay
	extern CSL_RtcRegsOvly	   rtcRegs;
	extern CSL_Syscfg1RegsOvly sys1Regs;
#endif

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// VERSION CONTROL
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(FW_REV_FIELD1,"CFG")
	_EXTERN Uint8	FW_REV_FIELD1;

#pragma DATA_SECTION(FW_REV_FIELD2,"CFG")
	_EXTERN Uint8	FW_REV_FIELD2;

#pragma DATA_SECTION(FW_REV_FIELD3,"CFG")
	_EXTERN Uint8	FW_REV_FIELD3;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// NAND Flash
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

	_EXTERN unsigned char MSG_DIG[32];

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// Modbus Tables - 16bit & 32bit registers
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(REG_WATERCUT,"CFG")
	_EXTERN far VAR REG_WATERCUT;

#pragma DATA_SECTION(REG_TEMPERATURE,"CFG")
	_EXTERN far VAR REG_TEMPERATURE;

	_EXTERN far VAR REG_EMULSION_PHASE;	// not used

#pragma DATA_SECTION(REG_SALINITY,"CFG")
	_EXTERN far VAR REG_SALINITY;

#pragma DATA_SECTION(REG_HARDWARE_VERSION,"CFG")
	_EXTERN far double REG_HARDWARE_VERSION;

#pragma DATA_SECTION(REG_FIRMWARE_VERSION,"CFG")
	_EXTERN far double REG_FIRMWARE_VERSION;

#pragma DATA_SECTION(REG_OIL_ADJUST,"CFG")
	_EXTERN far VAR REG_OIL_ADJUST;

#pragma DATA_SECTION(REG_WATER_ADJUST,"CFG")
	_EXTERN far VAR REG_WATER_ADJUST;

#pragma DATA_SECTION(REG_FREQ,"CFG")
	_EXTERN far VAR REG_FREQ;

	_EXTERN far VAR REG_FREQ_AVG;

	_EXTERN far VAR REG_WATERCUT_AVG;

	_EXTERN far double REG_WATERCUT_RAW;

#pragma DATA_SECTION(REG_TEMP_AVG,"CFG")
	_EXTERN far VAR REG_TEMP_AVG;

#pragma DATA_SECTION(REG_TEMP_ADJUST,"CFG")
	_EXTERN far VAR REG_TEMP_ADJUST;

#pragma DATA_SECTION(PDI_TEMP_ADJ,"CFG")
	_EXTERN far double PDI_TEMP_ADJ;

#pragma DATA_SECTION(PDI_FREQ_F0,"CFG")
	_EXTERN far double PDI_FREQ_F0;

#pragma DATA_SECTION(PDI_FREQ_F1,"CFG")
	_EXTERN far double PDI_FREQ_F1;

#pragma DATA_SECTION(REG_TEMP_USER,"CFG")
	_EXTERN far VAR REG_TEMP_USER;

#pragma DATA_SECTION(REG_PROC_AVGING,"CFG")
	_EXTERN far VAR REG_PROC_AVGING;

#pragma DATA_SECTION(REG_OIL_INDEX,"CFG")
	_EXTERN far VAR REG_OIL_INDEX;

#pragma DATA_SECTION(REG_OIL_P0,"CFG")
	_EXTERN far VAR REG_OIL_P0;

#pragma DATA_SECTION(REG_OIL_P1,"CFG")
	_EXTERN far VAR REG_OIL_P1;

#pragma DATA_SECTION(REG_OIL_FREQ_LOW,"CFG")
	_EXTERN far VAR REG_OIL_FREQ_LOW;

#pragma DATA_SECTION(REG_OIL_FREQ_HIGH,"CFG")
	_EXTERN far VAR REG_OIL_FREQ_HIGH;

#pragma DATA_SECTION(REG_SAMPLE_PERIOD,"CFG")
	_EXTERN far VAR REG_SAMPLE_PERIOD;

#pragma DATA_SECTION(REG_AO_LRV,"CFG")
	_EXTERN far VAR REG_AO_LRV;

#pragma DATA_SECTION(REG_AO_URV,"CFG")
	_EXTERN far VAR REG_AO_URV;

#pragma DATA_SECTION(REG_BAUD_RATE,"CFG")
	_EXTERN far VAR REG_BAUD_RATE;

    _EXTERN far double REG_OIL_RP;

	_EXTERN far double REG_WATER_RP;


#pragma DATA_SECTION(REG_OIL_CALC_MAX,"CFG")
	_EXTERN far double REG_OIL_CALC_MAX;

#pragma DATA_SECTION(REG_OIL_PHASE_CUTOFF,"CFG")
	_EXTERN far double REG_OIL_PHASE_CUTOFF;

#pragma DATA_SECTION(REG_TEMP_OIL_NUM_CURVES,"CFG")
	_EXTERN far double REG_TEMP_OIL_NUM_CURVES;

#pragma DATA_SECTION(REG_STREAM,"CFG")
	_EXTERN far VAR REG_STREAM;

    _EXTERN far double REG_OIL_RP_AVG;

	_EXTERN far VAR REG_PLACE_HOLDER;

#pragma DATA_SECTION(REG_OIL_SAMPLE,"CFG")
	_EXTERN far VAR REG_OIL_SAMPLE;

#pragma DATA_SECTION(REG_AO_MANUAL_VAL,"CFG")
	_EXTERN far double REG_AO_MANUAL_VAL;

#pragma DATA_SECTION(REG_AO_TRIMLO,"CFG")
	_EXTERN far double REG_AO_TRIMLO;

#pragma DATA_SECTION(REG_AO_TRIMHI,"CFG")
	_EXTERN far double REG_AO_TRIMHI;

#pragma DATA_SECTION(REG_DENSITY_ADJ,"CFG")
	_EXTERN far double REG_DENSITY_ADJ;

#pragma DATA_SECTION(REG_DENSITY_UNIT,"CFG")
	_EXTERN far REGSWI REG_DENSITY_UNIT;

#pragma DATA_SECTION(REG_DENS_CORR,"CFG")
	_EXTERN far double REG_DENS_CORR;

#pragma DATA_SECTION(REG_DENSITY_D3,"CFG")
	_EXTERN far VAR REG_DENSITY_D3;

#pragma DATA_SECTION(REG_DENSITY_D2,"CFG")
	_EXTERN far VAR REG_DENSITY_D2;

#pragma DATA_SECTION(REG_DENSITY_D1,"CFG")
	_EXTERN far VAR REG_DENSITY_D1;

#pragma DATA_SECTION(REG_DENSITY_D0,"CFG")
	_EXTERN far VAR REG_DENSITY_D0;

#pragma DATA_SECTION(REG_DENSITY_CAL_VAL,"CFG")
	_EXTERN far VAR REG_DENSITY_CAL_VAL;

	_EXTERN far double REG_AO_OUTPUT;

#pragma DATA_SECTION(REG_RELAY_SETPOINT,"CFG")
	_EXTERN far VAR REG_RELAY_SETPOINT;

#pragma DATA_SECTION(REG_OIL_DENSITY,"CFG")
	_EXTERN far VAR REG_OIL_DENSITY;

#pragma DATA_SECTION(REG_OIL_DENSITY_MODBUS,"CFG")
	_EXTERN far double REG_OIL_DENSITY_MODBUS;

#pragma DATA_SECTION(REG_OIL_DENSITY_AI,"CFG")
	_EXTERN far double REG_OIL_DENSITY_AI;

#pragma DATA_SECTION(REG_OIL_DENSITY_MANUAL,"CFG")
	_EXTERN far double REG_OIL_DENSITY_MANUAL;

#pragma DATA_SECTION(REG_OIL_DENSITY_AI_LRV,"CFG")
	_EXTERN far VAR REG_OIL_DENSITY_AI_LRV;

#pragma DATA_SECTION(REG_OIL_DENSITY_AI_URV,"CFG")
	_EXTERN far VAR REG_OIL_DENSITY_AI_URV;

#pragma DATA_SECTION(REG_AI_TRIMLO,"CFG")
	_EXTERN far double REG_AI_TRIMLO;

#pragma DATA_SECTION(REG_AI_TRIMHI,"CFG")
	_EXTERN far double REG_AI_TRIMHI;

    _EXTERN far double REG_AI_MEASURE;

	_EXTERN far double REG_AI_TRIMMED;

#pragma DATA_SECTION(REG_DENS_ADJ,"CFG")  
	_EXTERN far double REG_DENS_ADJ;

#pragma DATA_SECTION(REG_OIL_T0,"CFG")
    _EXTERN far VAR REG_OIL_T0;

#pragma DATA_SECTION(REG_OIL_T1,"CFG")
    _EXTERN far VAR REG_OIL_T1;

#pragma DATA_SECTION(REG_OIL_PT,"CFG")
    _EXTERN far double REG_OIL_PT;

////////////////////////////////////////////////
///// FCT VAR/DOUBLE   /////////////////////////
////////////////////////////////////////////////

#pragma DATA_SECTION(FCT_SALINITY,"CFG")
	_EXTERN far VAR FCT_SALINITY;

#pragma DATA_SECTION(FCT_OIL_ADJUST,"CFG")
	_EXTERN far VAR FCT_OIL_ADJUST;

#pragma DATA_SECTION(FCT_WATER_ADJUST,"CFG")
	_EXTERN far VAR FCT_WATER_ADJUST;

#pragma DATA_SECTION(FCT_TEMP_ADJUST,"CFG")
	_EXTERN far VAR FCT_TEMP_ADJUST;

#pragma DATA_SECTION(FCT_PROC_AVGING,"CFG")
	_EXTERN far VAR FCT_PROC_AVGING;

#pragma DATA_SECTION(FCT_OIL_INDEX,"CFG")
	_EXTERN far VAR FCT_OIL_INDEX;

#pragma DATA_SECTION(FCT_OIL_P0,"CFG")
	_EXTERN far VAR FCT_OIL_P0;

#pragma DATA_SECTION(FCT_OIL_P1,"CFG")
	_EXTERN far VAR FCT_OIL_P1;

#pragma DATA_SECTION(FCT_OIL_FREQ_LOW,"CFG")
	_EXTERN far VAR FCT_OIL_FREQ_LOW;

#pragma DATA_SECTION(FCT_OIL_FREQ_HIGH,"CFG")
	_EXTERN far VAR FCT_OIL_FREQ_HIGH;

#pragma DATA_SECTION(FCT_SAMPLE_PERIOD,"CFG")
	_EXTERN far VAR FCT_SAMPLE_PERIOD;

#pragma DATA_SECTION(FCT_AO_LRV,"CFG")
	_EXTERN far VAR FCT_AO_LRV;

#pragma DATA_SECTION(FCT_AO_URV,"CFG")
	_EXTERN far VAR FCT_AO_URV;

#pragma DATA_SECTION(FCT_BAUD_RATE,"CFG")
	_EXTERN far VAR FCT_BAUD_RATE;

#pragma DATA_SECTION(FCT_OIL_CALC_MAX,"CFG")
	_EXTERN far double FCT_OIL_CALC_MAX;

#pragma DATA_SECTION(FCT_OIL_PHASE_CUTOFF,"CFG")
	_EXTERN far double FCT_OIL_PHASE_CUTOFF;

#pragma DATA_SECTION(FCT_STREAM,"CFG")
	_EXTERN far VAR FCT_STREAM;

#pragma DATA_SECTION(FCT_OIL_SAMPLE,"CFG")
	_EXTERN far VAR FCT_OIL_SAMPLE;

#pragma DATA_SECTION(FCT_AO_MANUAL_VAL,"CFG")
	_EXTERN far double FCT_AO_MANUAL_VAL;

#pragma DATA_SECTION(FCT_AO_TRIMLO,"CFG")
	_EXTERN far double FCT_AO_TRIMLO;

#pragma DATA_SECTION(FCT_AO_TRIMHI,"CFG")
	_EXTERN far double FCT_AO_TRIMHI;

#pragma DATA_SECTION(FCT_DENSITY_ADJ,"CFG")
	_EXTERN far double FCT_DENSITY_ADJ;

#pragma DATA_SECTION(FCT_DENSITY_D3,"CFG")
	_EXTERN far VAR FCT_DENSITY_D3;

#pragma DATA_SECTION(FCT_DENSITY_D2,"CFG")
	_EXTERN far VAR FCT_DENSITY_D2;

#pragma DATA_SECTION(FCT_DENSITY_D1,"CFG")
	_EXTERN far VAR FCT_DENSITY_D1;

#pragma DATA_SECTION(FCT_DENSITY_D0,"CFG")
	_EXTERN far VAR FCT_DENSITY_D0;

#pragma DATA_SECTION(FCT_DENSITY_CAL_VAL,"CFG")
	_EXTERN far VAR FCT_DENSITY_CAL_VAL;

#pragma DATA_SECTION(FCT_RELAY_SETPOINT,"CFG")
	_EXTERN far VAR FCT_RELAY_SETPOINT;

#pragma DATA_SECTION(FCT_OIL_DENSITY,"CFG")
	_EXTERN far VAR FCT_OIL_DENSITY;

#pragma DATA_SECTION(FCT_OIL_DENSITY_MODBUS,"CFG")
	_EXTERN far double FCT_OIL_DENSITY_MODBUS;

#pragma DATA_SECTION(FCT_OIL_DENSITY_AI,"CFG")
	_EXTERN far double FCT_OIL_DENSITY_AI;

#pragma DATA_SECTION(FCT_OIL_DENSITY_MANUAL,"CFG")
	_EXTERN far double FCT_OIL_DENSITY_MANUAL;

#pragma DATA_SECTION(FCT_OIL_DENSITY_AI_LRV,"CFG")
	_EXTERN far VAR FCT_OIL_DENSITY_AI_LRV;

#pragma DATA_SECTION(FCT_OIL_DENSITY_AI_URV,"CFG")
	_EXTERN far VAR FCT_OIL_DENSITY_AI_URV;

#pragma DATA_SECTION(FCT_AI_TRIMLO,"CFG")
	_EXTERN far double FCT_AI_TRIMLO;

#pragma DATA_SECTION(FCT_AI_TRIMHI,"CFG")
	_EXTERN far double FCT_AI_TRIMHI;

#pragma DATA_SECTION(FCT_AI_MEASURE,"CFG")
	_EXTERN far double FCT_AI_MEASURE;

#pragma DATA_SECTION(FCT_AI_TRIMMED,"CFG")
	_EXTERN far double FCT_AI_TRIMMED;

#pragma DATA_SECTION(FCT_DENS_ADJ,"CFG")  
	_EXTERN far double FCT_DENS_ADJ;

#pragma DATA_SECTION(FCT_OIL_T0,"CFG")
    _EXTERN far VAR FCT_OIL_T0;

#pragma DATA_SECTION(FCT_OIL_T1,"CFG")
    _EXTERN far VAR FCT_OIL_T1;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// INTEGER
///          
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(REG_SN_PIPE,"CFG")
	_EXTERN far int REG_SN_PIPE;
 
#pragma DATA_SECTION(REG_ANALYZER_MODE,"CFG")
	_EXTERN far int REG_ANALYZER_MODE;

#pragma DATA_SECTION(REG_AO_DAMPEN,"CFG")
	_EXTERN far int REG_AO_DAMPEN;

#pragma DATA_SECTION(REG_SLAVE_ADDRESS,"CFG")
	_EXTERN far int REG_SLAVE_ADDRESS;

#pragma DATA_SECTION(REG_STOP_BITS,"CFG")
	_EXTERN far int REG_STOP_BITS;

#pragma DATA_SECTION(REG_DENSITY_MODE,"CFG")
	_EXTERN far int REG_DENSITY_MODE;

#pragma DATA_SECTION(REG_MODEL_CODE,"CFG")
	_EXTERN far int REG_MODEL_CODE[4];

#pragma DATA_SECTION(REG_LOGGING_PERIOD,"CFG")
	_EXTERN far int REG_LOGGING_PERIOD;

#pragma DATA_SECTION(REG_PASSWORD,"CFG")
	_EXTERN far int REG_PASSWORD;

#pragma DATA_SECTION(REG_STATISTICS,"CFG")
	_EXTERN far int REG_STATISTICS;

#pragma DATA_SECTION(REG_ACTIVE_ERROR,"CFG")
	_EXTERN far int REG_ACTIVE_ERROR;

#pragma DATA_SECTION(REG_AO_ALARM_MODE,"CFG")
	_EXTERN far int REG_AO_ALARM_MODE;

#pragma DATA_SECTION(REG_PHASE_HOLD_CYCLES,"CFG")   /*phase hold over RS*/
    _EXTERN far int REG_PHASE_HOLD_CYCLES;           

#pragma DATA_SECTION(REG_RELAY_DELAY,"CFG")   		
    _EXTERN far int REG_RELAY_DELAY;     	

#pragma DATA_SECTION(REG_AO_MODE,"CFG")
	_EXTERN far int REG_AO_MODE;

#pragma DATA_SECTION(REG_OIL_DENS_CORR_MODE,"CFG")
	_EXTERN far int REG_OIL_DENS_CORR_MODE;

#pragma DATA_SECTION(REG_RELAY_MODE,"CFG")
	_EXTERN far Uint8 REG_RELAY_MODE;

#pragma DATA_SECTION(REG_USB_TRY,"CFG")
	_EXTERN far int REG_USB_TRY;
 
    _EXTERN far int TMP_RTC_SEC;        // RTC read-only: seconds
    _EXTERN far int TMP_RTC_MIN;        // RTC read-only: minutes
    _EXTERN far int TMP_RTC_HR;         // RTC read-only: hours
    _EXTERN far int TMP_RTC_DAY;        // RTC read-only: day
    _EXTERN far int TMP_RTC_MON;        // RTC read-only: month
    _EXTERN far int TMP_RTC_YR;         // RTC read-only: year
    _EXTERN far int TMP_RTC_SEC_IN;     // RTC read-only: seconds
    _EXTERN far int TMP_RTC_MIN_IN;     // RTC read-only: minutes
    _EXTERN far int TMP_RTC_HR_IN;      // RTC read-only: hours
    _EXTERN far int TMP_RTC_DAY_IN;     // RTC read-only: day
    _EXTERN far int TMP_RTC_MON_IN;     // RTC read-only: month
    _EXTERN far int TMP_RTC_YR_IN;      // RTC read-only: year

/////////////////////////////////////////////////
////// FCT INTEGER //////////////////////////////
/////////////////////////////////////////////////

#pragma DATA_SECTION(FCT_AO_DAMPEN,"CFG")
	_EXTERN far int FCT_AO_DAMPEN;

#pragma DATA_SECTION(FCT_SLAVE_ADDRESS,"CFG")
	_EXTERN far int FCT_SLAVE_ADDRESS;

#pragma DATA_SECTION(FCT_STOP_BITS,"CFG")
	_EXTERN far int FCT_STOP_BITS;

#pragma DATA_SECTION(FCT_DENSITY_MODE,"CFG")
	_EXTERN far int FCT_DENSITY_MODE;

#pragma DATA_SECTION(FCT_LOGGING_PERIOD,"CFG")
	_EXTERN far int FCT_LOGGING_PERIOD;

#pragma DATA_SECTION(FCT_STATISTICS,"CFG")
	_EXTERN far int FCT_STATISTICS;

#pragma DATA_SECTION(FCT_ACTIVE_ERROR,"CFG")
	_EXTERN far int FCT_ACTIVE_ERROR;

#pragma DATA_SECTION(FCT_AO_ALARM_MODE,"CFG")
	_EXTERN far int FCT_AO_ALARM_MODE;

#pragma DATA_SECTION(FCT_PHASE_HOLD_CYCLES,"CFG")   /*phase hold over RS*/
    _EXTERN far int FCT_PHASE_HOLD_CYCLES;           

#pragma DATA_SECTION(FCT_RELAY_DELAY,"CFG")   		
    _EXTERN far int FCT_RELAY_DELAY;     	

#pragma DATA_SECTION(FCT_AO_MODE,"CFG")
	_EXTERN far int FCT_AO_MODE;

#pragma DATA_SECTION(FCT_OIL_DENS_CORR_MODE,"CFG")
	_EXTERN far int FCT_OIL_DENS_CORR_MODE;

#pragma DATA_SECTION(FCT_RELAY_MODE,"CFG")
	_EXTERN far Uint8 FCT_RELAY_MODE;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// LONG INTEGER
///          
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(REG_MEASSECTION_SN,"CFG")      // 301
    _EXTERN far long int REG_MEASSECTION_SN;

#pragma DATA_SECTION(REG_BACKBOARD_SN,"CFG")        // 303
    _EXTERN far long int REG_BACKBOARD_SN;

#pragma DATA_SECTION(REG_SAFETYBARRIER_SN,"CFG")    // 305
    _EXTERN far long int REG_SAFETYBARRIER_SN;

#pragma DATA_SECTION(REG_POWERSUPPLY_SN,"CFG")      // 307
    _EXTERN far long int REG_POWERSUPPLY_SN;

#pragma DATA_SECTION(REG_PROCESSOR_SN,"CFG")        // 309
    _EXTERN far long int REG_PROCESSOR_SN;

#pragma DATA_SECTION(REG_DISPLAY_SN,"CFG")          // 311
    _EXTERN far long int REG_DISPLAY_SN;

#pragma DATA_SECTION(REG_RF_SN,"CFG")               // 313
    _EXTERN far long int REG_RF_SN;

#pragma DATA_SECTION(REG_ASSEMBLY_SN,"CFG")         // 315
    _EXTERN far long int REG_ASSEMBLY_SN;

#pragma DATA_SECTION(REG_ELECTRONICS_SN,"CFG")      // 317
    _EXTERN far long int REG_ELECTRONICS_SN[8];

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// EXTENDED
///          
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(REG_TEMP_OIL_NUM_CURVES,"CFG") // 60001
    _EXTERN far double REG_TEMP_OIL_NUM_CURVES;

#pragma DATA_SECTION(REG_TEMPS_OIL,"CFG")           // 60003 : size = 2 x 10
    _EXTERN far double REG_TEMPS_OIL[10];

#pragma DATA_SECTION(REG_COEFFS_TEMP_OIL,"CFG")     // 60023 : size = 2 x 10 x 4
    _EXTERN far double REG_COEFFS_TEMP_OIL[10][4];

#pragma DATA_SECTION(REG_SALINITY_CURVES,"CFG")     // 60103 :
    _EXTERN far double REG_SALINITY_CURVES;

#pragma DATA_SECTION(REG_COEFFS_SALINITY,"CFG")     // 60105 : size = 2 x 20
    _EXTERN far double REG_COEFFS_SALINITY[20];

#pragma DATA_SECTION(REG_WATER_CURVES,"CFG")        // 60145 : size = 2 x 15
    _EXTERN far double REG_WATER_CURVES;

#pragma DATA_SECTION(REG_WATER_TEMPS,"CFG")         // 60147 : size = 2 x 15
    _EXTERN far double REG_WATER_TEMPS[15];

#pragma DATA_SECTION(REG_COEFFS_TEMP_WATER,"CFG")   // 60177 : size = 2 x 300 x 4
    _EXTERN far double REG_COEFFS_TEMP_WATER[300][4];

#pragma DATA_SECTION(REG_STRING_TAG,"CFG")          // 62577 : size = 1 x 8
    _EXTERN far char REG_STRING_TAG[8];

#pragma DATA_SECTION(REG_STRING_LONGTAG,"CFG")      // 62585 : size = 1 x 32
    _EXTERN far char REG_STRING_LONGTAG[32];

#pragma DATA_SECTION(REG_STRING_INITIAL,"CFG")      // 62617 : size = 1 x 4
    _EXTERN far char REG_STRING_INITIAL[4];

#pragma DATA_SECTION(REG_STRING_MEAS,"CFG")         // 62621 : size = 1 x 2
    _EXTERN far char REG_STRING_MEAS[2];

#pragma DATA_SECTION(REG_STRING_ASSEMBLY,"CFG")     // 62623 : size = 1 x 16
    _EXTERN far char REG_STRING_ASSEMBLY[16];

#pragma DATA_SECTION(REG_STRING_INFO,"CFG")         // 62639 : size = 1 x 20
    _EXTERN far char REG_STRING_INFO[20];

#pragma DATA_SECTION(REG_STRING_PVNAME,"CFG")       // 62659 : size = 1 x 20
    _EXTERN far char REG_STRING_PVNAME[20];

#pragma DATA_SECTION(REG_STRING_PVUNIT,"CFG")       // 62679 : size = 1 x 8
    _EXTERN far char REG_STRING_PVUNIT[8];

#pragma DATA_SECTION(STREAM_TIMESTAMP,"CFG")        // 62687 : size = 1 x 60 x 16;
    _EXTERN far char STREAM_TIMESTAMP[SMAX][MAX_LCD_WIDTH];

#pragma DATA_SECTION(STREAM_OIL_ADJUST,"CFG")       // 63647 : size = 2 x 60
    _EXTERN far double STREAM_OIL_ADJUST[SMAX];

#pragma DATA_SECTION(STREAM_WATERCUT_AVG,"CFG")     // 63767 : size = 2 x 60
    _EXTERN far double STREAM_WATERCUT_AVG[SMAX];

#pragma DATA_SECTION(STREAM_SAMPLES,"CFG")          // 63887 : size = 2 x 60
    _EXTERN far int STREAM_SAMPLES[SMAX];


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// 
/// COILS
/// 
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(COIL_RELAY,"CFG")
	_EXTERN far COIL COIL_RELAY[4];

#pragma DATA_SECTION(COIL_BEGIN_OIL_CAP,"CFG")
	_EXTERN far COIL COIL_BEGIN_OIL_CAP;

#pragma DATA_SECTION(COIL_LOG_ENABLE,"CFG")
	_EXTERN far COIL COIL_LOG_ENABLE;

#pragma DATA_SECTION(COIL_LOG_ALARMS,"CFG")
	_EXTERN far COIL COIL_LOG_ALARMS;

#pragma DATA_SECTION(COIL_LOG_ERRORS,"CFG")
	_EXTERN far COIL COIL_LOG_ERRORS;

#pragma DATA_SECTION(COIL_LOG_ACTIVITY,"CFG")
	_EXTERN far COIL COIL_LOG_ACTIVITY;

#pragma DATA_SECTION(COIL_AO_ALARM,"CFG")
	_EXTERN far COIL COIL_AO_ALARM;

#pragma DATA_SECTION(COIL_PARITY,"CFG")
	_EXTERN far COIL COIL_PARITY;

	_EXTERN far COIL COIL_WRITE_RTC;

#pragma DATA_SECTION(COIL_OIL_DENS_CORR_EN,"CFG")
	_EXTERN far COIL COIL_OIL_DENS_CORR_EN;

#pragma DATA_SECTION(COIL_AO_MANUAL,"CFG")
    _EXTERN far COIL COIL_AO_MANUAL;

#pragma DATA_SECTION(COIL_MB_AUX_SELECT_MODE,"CFG")
	_EXTERN far COIL COIL_MB_AUX_SELECT_MODE;

#pragma DATA_SECTION(COIL_INTEGER_TABLE_SELECT,"CFG")
	_EXTERN far COIL COIL_INTEGER_TABLE_SELECT;

#pragma DATA_SECTION(COIL_AVGTEMP_RESET,"CFG")
	_EXTERN far COIL COIL_AVGTEMP_RESET;

#pragma DATA_SECTION(COIL_AVGTEMP_MODE,"CFG")
	_EXTERN far COIL COIL_AVGTEMP_MODE;

#pragma DATA_SECTION(COIL_OIL_PHASE,"CFG")
	_EXTERN far COIL COIL_OIL_PHASE;

#pragma DATA_SECTION(COIL_ACT_RELAY_OIL,"CFG")
	_EXTERN far COIL COIL_ACT_RELAY_OIL;

#pragma DATA_SECTION(COIL_RELAY_MANUAL,"CFG")
	_EXTERN far COIL COIL_RELAY_MANUAL;

#pragma DATA_SECTION(COIL_UNLOCKED,"CFG")
	_EXTERN far COIL COIL_UNLOCKED;

#pragma DATA_SECTION(COIL_AO_TRIM_MODE,"CFG")
	_EXTERN far COIL COIL_AO_TRIM_MODE;

#pragma DATA_SECTION(COIL_AI_TRIM_MODE,"CFG")
	_EXTERN far COIL COIL_AI_TRIM_MODE;

#pragma DATA_SECTION(COIL_LOCKED_SOFT_FACTORY_RESET,"CFG")
	_EXTERN far COIL COIL_LOCKED_SOFT_FACTORY_RESET;

#pragma DATA_SECTION(COIL_LOCKED_HARD_FACTORY_RESET,"CFG")
	_EXTERN far COIL COIL_LOCKED_HARD_FACTORY_RESET;

#pragma DATA_SECTION(COIL_UPDATE_FACTORY_DEFAULT,"CFG")
	_EXTERN far COIL COIL_UPDATE_FACTORY_DEFAULT;

#pragma DATA_SECTION(COIL_UNLOCKED_FACTORY_DEFAULT,"CFG")
	_EXTERN far COIL COIL_UNLOCKED_FACTORY_DEFAULT;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// RESERVED_  
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

_EXTERN far double RESERVED_1;
_EXTERN far double RESERVED_27;
_EXTERN far double RESERVED_53;
_EXTERN far double RESERVED_57;
_EXTERN far double RESERVED_59;
_EXTERN far double RESERVED_65;
_EXTERN far double RESERVED_81;
_EXTERN far double RESERVED_83;
_EXTERN far double RESERVED_85;
_EXTERN far double RESERVED_87;
_EXTERN far double RESERVED_89;
_EXTERN far double RESERVED_91;
_EXTERN far double RESERVED_93;
_EXTERN far double RESERVED_95;
_EXTERN far double RESERVED_97;
_EXTERN far double RESERVED_99;
_EXTERN far double RESERVED_101;
_EXTERN far double RESERVED_103;
_EXTERN far double RESERVED_127;
_EXTERN far double RESERVED_129;
_EXTERN far double RESERVED_131;
_EXTERN far double RESERVED_133;
_EXTERN far double RESERVED_135;
_EXTERN far double RESERVED_137;
_EXTERN far double RESERVED_139;
_EXTERN far double RESERVED_141;
_EXTERN far double RESERVED_143;
_EXTERN far double RESERVED_147;
_EXTERN far double RESERVED_149;
_EXTERN far double RESERVED_153;
_EXTERN far double RESERVED_167;

/////////////// FACTORY DEFAULT /////////////////
_EXTERN far double FCT_RESERVED_1;
_EXTERN far double FCT_RESERVED_27;
_EXTERN far double FCT_RESERVED_53;
_EXTERN far double FCT_RESERVED_57;
_EXTERN far double FCT_RESERVED_59;
_EXTERN far double FCT_RESERVED_65;
_EXTERN far double FCT_RESERVED_81;
_EXTERN far double FCT_RESERVED_83;
_EXTERN far double FCT_RESERVED_85;
_EXTERN far double FCT_RESERVED_87;
_EXTERN far double FCT_RESERVED_89;
_EXTERN far double FCT_RESERVED_91;
_EXTERN far double FCT_RESERVED_93;
_EXTERN far double FCT_RESERVED_95;
_EXTERN far double FCT_RESERVED_97;
_EXTERN far double FCT_RESERVED_99;
_EXTERN far double FCT_RESERVED_101;
_EXTERN far double FCT_RESERVED_103;
_EXTERN far double FCT_RESERVED_127;
_EXTERN far double FCT_RESERVED_129;
_EXTERN far double FCT_RESERVED_131;
_EXTERN far double FCT_RESERVED_133;
_EXTERN far double FCT_RESERVED_135;
_EXTERN far double FCT_RESERVED_137;
_EXTERN far double FCT_RESERVED_139;
_EXTERN far double FCT_RESERVED_141;
_EXTERN far double FCT_RESERVED_143;
_EXTERN far double FCT_RESERVED_147;
_EXTERN far double FCT_RESERVED_149;
_EXTERN far double FCT_RESERVED_153;
_EXTERN far double FCT_RESERVED_167;


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// Menu 
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

	extern void setupMenu(void);

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// Security 
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#pragma DATA_SECTION(LOCK_FACTORY,"CFG")
	_EXTERN far Uint8 LOCK_FACTORY;

#pragma DATA_SECTION(LOCK_TECH,"CFG")
	_EXTERN far Uint8 LOCK_TECH;
	_EXTERN Uint32 REG_UNLOCK_PASS[2]; // 4 character max

#pragma DATA_SECTION(LOCK_BASIC,"CFG")
	_EXTERN far Uint8 LOCK_BASIC;

	_EXTERN Uint8 PASSWORD_DELAYED;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///  
/// flow computer struct
///  
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

typedef struct
{
	VAR		watercut;
	VAR		T;
	VAR		pressure;
	VAR		density;
	VAR		density_oilST;
	VAR		density_oil;
	VAR		density_waterST;
	VAR		density_water;
	VAR		salinity;
	int		API_TABLE;
	VAR		Meter_Factor;
	VAR		Shrinkage;
	float		density_adj;
	float		density_PDI_corrected;
	float		a;
	float		VCFo;
	float		VCFw;
	float		net_watercut;
	float		net_watercut_mass;
	float		Dadj;
	VAR 		NET_FLOW_OIL;
	VAR 		NET_FLOW_WATER;
	VAR 		NET_FLOW_TOTAL;
	VAR 		NET_OIL;
	VAR 		NET_WATER;
	VAR 		NET_TOTAL;
	VAR 		AVG_NET_TOTAL;
	VAR 		GROSS_OIL;
	long double 	gross_oil_whole_part;
	long double 	gross_oil_fract_part;
	VAR 		GROSS_WATER;
	VAR 		GROSS_TOTAL;
	VAR 		AVG_GROSS_TOTAL;
	VAR 		FLOW_OIL;
	VAR 		FLOW_WATER;
	VAR 		FLOW_TOTAL;
	unsigned int 	PULSES_PER_ACCUM_UNIT;
	VAR 		PULSE_TOTAL;
	VAR 		PULSE_FLOW;
	VAR 		PULSE_FREQ;
	unsigned int 	PULSE_COUNTER;
	int 		PULSE;
	int 		CNT;
	int 		PULSE_DELTA;
	float		PULSE_FACTOR;
	int 		PULSE_STAT;
	BOOL		enable_NET;
	BOOL		enable_GROSS;
	BOOL		enable_FLOW;
	BOOL		enable_DENSITY;
	BOOL		enable_PULSE;
	COIL		flow_class;
	COIL 		flow_unit;
	COIL 		accum_unit;
	COIL 		Z;

	BOOL 		USE_PDI_TEMP;
	BOOL 		USE_PDI_SALINITY;
	BOOL 		CALC_WC_DENSITY;
	BOOL 		MAN_TEMP;
	BOOL 		MAN_SALINITY;
	BOOL 		MAN_WATERCUT;
	BOOL 		MAN_DENSITY;
	BOOL 		MAN_PRESSURE;
	BOOL 		MAN_FLOW;

	///New pulse input///
	COIL 		use_totalizer;
	COIL		pulse_v;
	COIL		pulse_class;
	COIL		pulse_unit;
	VAR 		pulse_VALUE;
	///New pulse input///

} FLOW_COMP; //not used by the Razor except for API conversions

#pragma DATA_SECTION(FC,"CFG")
	_EXTERN far FLOW_COMP FC;

#undef GLOBALS_H_
#undef GLOBAL_VARS
#undef _EXTERN
#undef MENU_H

#include "API.h"
#include "PDI_I2C.h"
#endif /* GLOBALS_H_ */
