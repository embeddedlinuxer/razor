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
* Menu.c
*-------------------------------------------------------------------------
* Contains all the code relevant to the LCD mnu system. The mnu system
* uses a state-machine architecture, which is defined in Menu.h by a state
* table and a state transition table. This is the only Razor code that is
* run in the context of a Task (SYS-BIOS module) – everything else is
* interrupt-based.
* Most "end-node" states of the mnu tree have a function associated with
* them. This function is called inside of Process_Menu() via a function
* pointer. Branch or "navigational" mnu states do not have associated
* functions, and are handled entirely by Process_Menu(). These states can
* be easily recognized by the mnu coordinates displayed on the bottom
* line of the LCD (e.g.  "2.4.2.0"). Currently, we cycle through the mnu
* code with a maximum frequency of about 6.67 times per second
* (minimum period = 0.15 seconds).
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul998-2018 : Daniel Koh : Migraged to linux platform
*       Jul-29-2018 : Daniel Koh : Menu has changed 
*------------------------------------------------------------------------*/

#ifndef _MENU_H
#define _MENU_H

#ifdef MENU_H
#define _EXTERN
#else
#define _EXTERN extern
#endif

#define MNU_NRML_MODE					0x1
#define MNU_TECH_MODE					0x2
#define MNU_DIR_RIGHT					0x1
#define MNU_DIR_LEFT					0x2
#define MNU_DEPTH_MAX					10
#define MNU_INPUT_SUCCESS 				0
#define MNU_INPUT_OOB 					1
#define MNU_INPUT_CANCEL 				2
#define MNU_INPUT_WAIT 					3
#define MNU_INPUT_BAD_PASS				4
#define MNU_INPUT_GOOD_PASS				5
#define MNU_INPUT_LOCK					6
#define MNU_INPUT_NO_LOCK				7
#define MNU_INPUT_UNSIGNED				FALSE
#define MNU_INPUT_SIGNED				TRUE
#define MNU_INPUT_HOLD_COUNT		 	5
#define BTN_STEP						0
#define BTN_VALUE						1	
#define BTN_ENTER					    2		
#define BTN_BACK						3		
#define BTN_NONE						4	
#define LCD_DEGREE						0xDF
#define LCD_VERT_BAR					0x7C
#define LCD_TOGGLE_SWITCH				30
#define LCD0							0
#define LCD1							1
#define LCD_CURS_ON						0x1 //show cursor
#define LCD_CURS_OFF					0x2 //hide cursor
#define LCD_CURS_BLINK					0x4 //blink cursor (ONLY if LCD_CURS_ON)
#define LCD_CURS_NOBLINK				0x8 //don't blink cursor
#define DEGREE_CHAR						((char)0xDF)
#define EIGHT_WIDTH					 	8	
#define SEVEN_WIDTH					 	7	
#define SIX_WIDTH					 	6	
#define FIVE_WIDTH					 	5	
#define FOUR_WIDTH					 	4	
#define THREE_WIDTH					 	3	
#define TWO_WIDTH					 	2	
#define THREESIXTY						360 // 60 secs for oil capturing

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// HOMESCREEN : WaterCut->(VALUE)->Frequency->(VALUE)->...
/// 
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_HOMESCREEN_WTC				1 	// Watercut
#define MNU_HOMESCREEN_FREQ				2 	// Frequency	
#define MNU_HOMESCREEN_RP				3 	// Refelcted Power	
#define MNU_HOMESCREEN_PT				4 	// Phase threshold	
#define MNU_HOMESCREEN_AVT				5 	// Average Temp	
#define MNU_HOMESCREEN_DST				6	// Density	
#define MNU_HOMESCREEN_DGN				7 	// Diagnostics
#define FXN_HOMESCREEN_DGN				71 	// FXNDiagnostics
#define MNU_HOMESCREEN_SRN				8 	// Serial Number

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 1 : 1.0 Operation
///	 		  	| (VALUE)
///			  2.0 Config 
/// 		 	| (VALUE)
///		      3.0 Security & Info 
/// 
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_OPERATION					10 	// 1.0 Operation
#define MNU_CFG							20 	// 2.0 Config
#define MNU_SECURITYINFO				30 	// 3.0 Sec and Info

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 2: 1.0 Operation -> (STEP) ->	1.1 Stream 
///											| (VALUE)
///										1.2 Oil Adjust
///											| (VALUE)
///										1.3 Oil Capture
///											| (VALUE)
///				  					 	1.4 Sample
///											| (VALUE)
///										1.5 Average Temp
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_OPERATION_STREAM			11 	// 1.1 Stream
#define FXN_OPERATION_STREAM			110	// fxnOperation_Stream()

#define MNU_OPERATION_OILADJUST			12	// 1.2 Oil Adjust
#define FXN_OPERATION_OILADJUST			120	// fxnOperation_OilAdjust()

#define MNU_OPERATION_OILCAPTURE		13	// 1.3 Oil Capture
#define FXN_OPERATION_OILCAPTURE		130	// fxnOperation_OilCapture()

#define MNU_OPERATION_SAMPLE			14	// 1.4 Sample
#define FXN_OPERATION_SAMPLE_STREAM		140	// fxnOperation_Sample_Stream()
#define FXN_OPERATION_SAMPLE_TIMESTAMP	141	// fxnOperation_Sample_Timestamp() 
#define FXN_OPERATION_SAMPLE_VALUE		142	// fxnOperation_Sample_Value()

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 2: 2.0 Configuration->(STEP)->2.1 Analyzer
///											| (VALUE)
///										2.2 Average Temp 
///											| (VALUE)
///										2.3 Data Logger
///											| (VALUE)
///										2.4 An. Output 
///											| (VALUE)
///										2.5 Comms	
///											| (VALUE)
///										2.6 Relay
///											| (VALUE)
///					   					2.7 Density Corr.
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_ANALYZER				21 	// 2.1 Analyzer
#define MNU_CFG_AVGTEMP					22 	// 2.2 Average Temp
#define MNU_CFG_DATALOGGER				23	// 2.2 Data Logger
#define MNU_CFG_AO						24 	// 2.4 An. Output
#define MNU_CFG_COMM					25	// 2.5 Comms
#define MNU_CFG_RELAY					26	// 2.6 Relay
#define MNU_CFG_DNSCORR					27	// 2.7 Density Corr.

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 2: 3.0 Sec. & Info->(STEP) -> 3.1 Info
///											| (VALUE)
///										3.2 Time & Date
///											| (VALUE)
///										3.3. Access Tech
///											| (VALUE)
///										3.4 Diagnostics 
///											| (VALUE)
///										3.5 Change Tech
///											| (VALUE)
///										3.6 Restart
///											| (VALUE)
///										3.7 Reset Fact.
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_SECURITYINFO_INFO				31 		// 3.1 Info
#define MNU_SECURITYINFO_TIMEANDDATE 		32 		// 3.2 Time and Date
#define FXN_SECURITYINFO_TIMEANDDATE 		320 	// Node function
#define MNU_SECURITYINFO_ACCESSTECH 		33 		// 3.3 Access Tech
#define FXN_SECURITYINFO_ACCESSTECH 		330 	// Node function
#define FXN_SECURITYINFO_LOCK 			    331 	// Node function
#define MNU_SECURITYINFO_DIAGNOSTICS 		34 		// 3.4 Diagnostics
#define FXN_SECURITYINFO_DIAGNOSTICS 		340 	// Node function
#define MNU_SECURITYINFO_CHANGEPASSWORD	 	35 		// 3.5 Change Password
#define FXN_SECURITYINFO_CHANGEPASSWORD 	350 	// Node function
#define MNU_SECURITYINFO_RESTART			36 		// 3.6 Restart
#define FXN_SECURITYINFO_RESTART 			360 	// Node function
#define MNU_SECURITYINFO_FACTRESET 			37		// 3.7 Reset Fact.
#define FXN_SECURITYINFO_FACTRESET			370		// Node function
#define MNU_SECURITYINFO_UPDATEFIRMWARE 	38		// 3.8 Update firmeware.
#define FXN_SECURITYINFO_UPDATEFIRMWARE	    380		// Node function

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 3: 3.1 Info->(STEP) -> 3.1.1 Serial Number 
///							    	| (VALUE)
///							     3.1.2 Model Code
///							    	| (VALUE)
///							     3.1.3 Firmware
///							    	| (VALUE)
///							     3.1.4 Hardware 
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define FXN_SECURITYINFO_SN					311 	// Node function 
#define FXN_SECURITYINFO_MC					312 	// Node function 
#define FXN_SECURITYINFO_FW					313 	// Node function 
#define FXN_SECURITYINFO_HW					314 	// Node function 

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// 
/// LEVEL 3: 2.1 Analyzer->(STEP) -> 2.1.1 Procs Avg
///												| (VALUE)
///									 2.1.2 Temp Unit 
///												| (VALUE)
///									 2.1.3 Temp Adj. 
///												| (VALUE)
///									 2.1.4 Oil P0 
///												| (VALUE)
///									 2.1.5 Oil P1 
///												| (VALUE)
///									 2.1.6 Oil Index 
///												| (VALUE)
///									 2.1.7 Oil Freq Low 
///												| (VALUE)
///									 2.1.8 Oil Freq Hi 
///												| (VALUE)
///									 2.1.9 Phase Hold  
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_ANALYZER_PROCSAVG			211		// 2.1.1 Procs Avg
#define FXN_CFG_ANALYZER_PROCSAVG			2110	// fxnConfig_Analyzer_ProcsAvg
#define MNU_CFG_ANALYZER_TEMPUNIT			212		// 2.1.2 Temp Unit
#define FXN_CFG_ANALYZER_TEMPUNIT			2120	// fxnConfig_Analyzer_TempUnit
#define MNU_CFG_ANALYZER_TEMPADJ			213		// 2.1.3 Temp Adj
#define FXN_CFG_ANALYZER_TEMPADJ			2130	// fxnConfig_Analyzer_TempAdj
#define MNU_CFG_ANALYZER_OILP0				214 	// 2.1.4 Oil P0
#define FXN_CFG_ANALYZER_OILP0				2140	// fxnConfig_Analyzer_OilP0
#define MNU_CFG_ANALYZER_OILP1				215		// 2.1.5 Oil P1
#define FXN_CFG_ANALYZER_OILP1				2150	// fxnConfig_Analyzer_OilP1
#define MNU_CFG_ANALYZER_OILINDEX			216		// 2.1.6 Oil Index 
#define FXN_CFG_ANALYZER_OILINDEX			2160	// fxnConfig_Analyzer_OilIndex
#define MNU_CFG_ANALYZER_OILFREQLOW			217		// 2.1.7 Oil F. Low
#define FXN_CFG_ANALYZER_OILFREQLOW			2170	// fxnConfig_Analyzer_OilFreqLow
#define MNU_CFG_ANALYZER_OILFREQHI			218	    // 2.1.8 Oil F. Hi
#define FXN_CFG_ANALYZER_OILFREQHI			2180	// fxnConfig_Analyzer_OilFreqHi
#define MNU_CFG_ANALYZER_PHASEHOLD			219	    // 2.1.9 Phase Hold
#define FXN_CFG_ANALYZER_PHASEHOLD			2190	// fxnConfig_Analyzer_PhaseHold

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
/// LEVEL 3: 2.2 Avg Temp->(STEP)->	2.2.1 Mode
///									|
///									2.2.2 Avg Reset
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_AVGTEMP_MODE				221		// 2.2.1 Mode 
#define FXN_CFG_AVGTEMP_MODE				2210	// fxnConfig_AvgTemp_Mode() 
#define MNU_CFG_AVGTEMP_AVGRESET			222		// 2.2.2 Avg Reset
#define FXN_CFG_AVGTEMP_AVGRESET			2220	// fxnConfig_AvgTemp_AvgReset()

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
/// LEVEL 3: 2.3 Data Logger->(STEP) ->	2.3.1 En. Logger
///                              			| (VALUE)
///                                    	2.3.2 Period
///                              			| (VALUE)
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_DATALOGGER_ENABLELOGGER		231		// 2.3.1 En. Logger
#define FXN_CFG_DATALOGGER_ENABLELOGGER		2310	// 
#define MNU_CFG_DATALOGGER_PERIOD			232		// 2.3.2 Period
#define FXN_CFG_DATALOGGER_PERIOD			2320	// 

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///  
/// LEVEL 3: 2.4 An. Output->(STEP)-> 2.4.1 LRV
///                                    	| (VALUE)
///                                   2.4.2 URV
///                                    	| (VALUE)
///                                   2.4.3 Dampening
///                                    	| (VALUE)
///                                   2.4.4 Alarm
///                                    	| (VALUE)
///                                   2.4.5 Trim 4mA
///                                    	| (VALUE)
///                                   2.4.6 Trim 20mA
///                                    	| (VALUE)
///                                   2.4.7 Mode
///                                		| (VALUE)
///                                   2.4.8 A.O. Value
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_AO_LRV						241		// 2.4.1 LRV
#define FXN_CFG_AO_LRV						2410
#define MNU_CFG_AO_URV						242		// 2.4.2 URV
#define FXN_CFG_AO_URV						2420
#define MNU_CFG_AO_DAMPENING				243		// 2.4.3 Dampening
#define FXN_CFG_AO_DAMPENING				2430
#define MNU_CFG_AO_ALARM					244		// 2.4.4 Alarm
#define FXN_CFG_AO_ALARM					2440
#define MNU_CFG_AO_TRIMLO					245		// 2.4.5 Trim 4mA
#define FXN_CFG_AO_TRIMLO					2450
#define MNU_CFG_AO_TRIMHI					246		// 2.4.6 Trim 20mA
#define FXN_CFG_AO_TRIMHI					2460
#define MNU_CFG_AO_MODE						247		// 2.4.7 Mode
#define FXN_CFG_AO_MODE						2470
#define MNU_CFG_AO_AOVALUE					248		// 2.4.8 A. O. Value
#define FXN_CFG_AO_AOVALUE					2480

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
/// LEVEL 3: 2.5 Communication->(STEP)->2.5.1 Slave Addr
///                               			| (VALUE)
///                                    	2.4.2 Baud Rate
///                                    		| (VALUE)
///                                    	2.5.3 Parity 
///                                    		| (VALUE)
///                                    	2.5.4 Statistics
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_COMM_SLAVEADDR				251		// 2.5.1 Slave Addr
#define FXN_CFG_COMM_SLAVEADDR				2510 	// Node function
#define MNU_CFG_COMM_BAUDRATE				252		// 2.5.2 Baud Rate
#define FXN_CFG_COMM_BAUDRATE				2520	// Node function
#define MNU_CFG_COMM_PARITY					253		// 2.5.3 Parity
#define FXN_CFG_COMM_PARITY					2530  	// Node function
#define MNU_CFG_COMM_STATISTICS				254		// 2.5.4 Statistics
#define FXN_CFG_COMM_STATISTICS				2540	// Node function

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
/// LEVEL 3: 2.6 Relay->(STEP)->2.6.1 Delay
///                             	| (VALUE)
///                      		2.6.2 Mode
///                             	| (VALUE)
///                             2.6.3 Set Point
///                             	| (VALUE)
///                             2.6.3 Act. While
///                             	| (VALUE)
///								2.6.3 R. Status
///
/// *** If 2.6.2 Mode = "Error"	 , then 2.6.3 "and" 2.6.4 don't exist. ***
/// *** If 2.6.2 Mode = "Manual" , then 2.6.3 is R. Status ***
/// *** If 2.6.2 Mode = "Watercut" , then 2.6.3 is Set Point. ***
/// *** If 2.6.2 Mode = "Phase" , then 2.6.3 is Act. While. ***
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_RELAY_DELAY					261		// 2.6.1 Delay
#define FXN_CFG_RELAY_DELAY					2610	// Node function
#define MNU_CFG_RELAY_MODE					262		// 2.6.2 Mode
#define FXN_CFG_RELAY_MODE					2620	// Node function
#define MNU_CFG_RELAY_ACTWHILE				263		// 2.6.3 Act. While
#define FXN_CFG_RELAY_ACTWHILE				2630	// Node function
#define MNU_CFG_RELAY_RELAYSTATUS			264		// 2.6.3 R. Status
#define FXN_CFG_RELAY_RELAYSTATUS			2640	// Node function
#define MNU_CFG_RELAY_SETPOINT				265		// 2.6.3 Set Point
#define FXN_CFG_RELAY_SETPOINT				2650	// Node function

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
/// LEVEL 3: 2.7 Dens Correction->(STEP)->2.7.1 Corr. Enable?
///                             			| (VALUE)
///                      				  2.7.2 Disp. Unit
///                             			| (VALUE)
///                      				  2.7.3 Coeff D0
///                             			| (VALUE)
///                      				  2.7.4 Coeff D1
///                             			| (VALUE)
///                      				  2.7.5 Coeff D2
///                             			| (VALUE)
///                      				  2.7.6 Coeff D3
///                             			| (VALUE)
///                      				  2.7.7 Input Unit
///                             			| (VALUE)
///                      				  2.7.8 LRV - 2.7.8 Man Dens
///                             			| (VALUE)
///                      				  2.7.9 URV
///											| (VALUE)
///                      				  2.7.10 Trim 4mA 
///											| (VALUE)
///                      				  2.7.11 Trim 20mA 
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MNU_CFG_DNSCORR_CORRENABLE	271
#define FXN_CFG_DNSCORR_CORRENABLE	2710
#define MNU_CFG_DNSCORR_DISPUNIT	272
#define FXN_CFG_DNSCORR_DISPUNIT	2720
#define MNU_CFG_DNSCORR_COEFD0		273
#define FXN_CFG_DNSCORR_COEFD0		2730
#define MNU_CFG_DNSCORR_COEFD1		274
#define FXN_CFG_DNSCORR_COEFD1		2740
#define MNU_CFG_DNSCORR_COEFD2		275
#define FXN_CFG_DNSCORR_COEFD2		2750
#define MNU_CFG_DNSCORR_COEFD3		276
#define FXN_CFG_DNSCORR_COEFD3		2760
#define MNU_CFG_DNSCORR_INPUTUNIT 	277
#define FXN_CFG_DNSCORR_INPUTUNIT 	2770
#define MNU_CFG_DNSCORR_MANUAL	 	278
#define FXN_CFG_DNSCORR_MANUAL 		2780
#define MNU_CFG_DNSCORR_AILRV	 	2781
#define FXN_CFG_DNSCORR_AILRV 		27810
#define MNU_CFG_DNSCORR_AIURV	 	279
#define FXN_CFG_DNSCORR_AIURV 		2790
#define MNU_CFG_DNSCORR_AI_TRIMLO	280
#define FXN_CFG_DNSCORR_AI_TRIMLO	2801
#define MNU_CFG_DNSCORR_AI_TRIMHI	290
#define FXN_CFG_DNSCORR_AI_TRIMHI	2901

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////
//// MENU/FXN DECLARATIONS
////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// HOMESCREEN
_EXTERN Uint16 mnuHomescreenWaterCut(Uint16 input);
_EXTERN Uint16 mnuHomescreenFrequency(Uint16 input);
_EXTERN Uint16 mnuHomescreenReflectedPower(Uint16 input);
_EXTERN Uint16 mnuHomescreenPhaseThreshold(Uint16 input);
_EXTERN Uint16 mnuHomescreenAvgTemp(Uint16 input);
_EXTERN Uint16 mnuHomescreenDensity(Uint16 input);
_EXTERN Uint16 mnuHomescreenDiagnostics(Uint16 input);
_EXTERN Uint16 fxnHomescreenDiagnostics(Uint16 input);
_EXTERN Uint16 mnuHomescreenSerialNumber(Uint16 input);


// MENU 1.0
_EXTERN Uint16 mnuOperation(Uint16 input);
// MENU 2.0
_EXTERN Uint16 mnuConfig(Uint16 input);
// MENU 3.0
_EXTERN Uint16 mnuSecurityInfo(Uint16 input);


// MENU 1.1
_EXTERN Uint16 mnuOperation_Stream(Uint16 input);
_EXTERN Uint16 fxnOperation_Stream(Uint16 input);
// MENU 1.2
_EXTERN Uint16 mnuOperation_OilAdjust(Uint16 input);
_EXTERN Uint16 fxnOperation_OilAdjust(Uint16 input);
// MENU 1.3
_EXTERN Uint16 mnuOperation_OilCapture(Uint16 input);
_EXTERN Uint16 fxnOperation_OilCapture(Uint16 input);
// MENU 1.4
_EXTERN Uint16 mnuOperation_Sample(Uint16 input);
_EXTERN Uint16 fxnOperation_Sample_Stream(Uint16 input);
_EXTERN Uint16 fxnOperation_Sample_Timestamp(Uint16 input);
_EXTERN Uint16 fxnOperation_Sample_Value(Uint16 input);


// MENU 2.1
_EXTERN Uint16 mnuConfig_Analyzer(Uint16 input);
// MENU 2.2
_EXTERN Uint16 mnuConfig_AvgTemp(Uint16 input);
// MENU 2.3
_EXTERN Uint16 mnuConfig_DataLogger(Uint16 input);
// MENU 2.4
_EXTERN Uint16 mnuConfig_AnalogOutput(Uint16 input);
// MENU 2.5
_EXTERN Uint16 mnuConfig_Comm(Uint16 input);
// MENU 2.6
_EXTERN Uint16 mnuConfig_Relay(Uint16 input);
// MENU 2.7
_EXTERN Uint16 mnuConfig_DnsCorr(Uint16 input);


// MENU 2.1.1 
_EXTERN Uint16 mnuConfig_Analyzer_ProcsAvg(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_ProcsAvg(Uint16 input);
// MENU 2.1.2 
_EXTERN Uint16 mnuConfig_Analyzer_TempUnit(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_TempUnit(Uint16 input);
// MENU 2.1.3 
_EXTERN Uint16 mnuConfig_Analyzer_TempAdj(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_TempAdj(Uint16 input);
// MENU 2.1.4 
_EXTERN Uint16 mnuConfig_Analyzer_OilP0(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_OilP0(Uint16 input);
// MENU 2.1.5 
_EXTERN Uint16 mnuConfig_Analyzer_OilP1(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_OilP1(Uint16 input);
// MENU 2.1.6
_EXTERN Uint16 mnuConfig_Analyzer_OilIndex(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_OilIndex(Uint16 input);
// MENU 2.1.7
_EXTERN Uint16 mnuConfig_Analyzer_OilFreqLow(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_OilFreqLow(Uint16 input);
// MENU 2.1.8
_EXTERN Uint16 mnuConfig_Analyzer_OilFreqHi(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_OilFreqHi(Uint16 input);
// MENU 2.1.9
_EXTERN Uint16 mnuConfig_Analyzer_PhaseHold(Uint16 input);
_EXTERN Uint16 fxnConfig_Analyzer_PhaseHold(Uint16 input);

// MENU 2.2.1 
_EXTERN Uint16 mnuConfig_AvgTemp_Mode(Uint16 input);
_EXTERN Uint16 fxnConfig_AvgTemp_Mode(Uint16 input);
// MENU 2.2.2
_EXTERN Uint16 mnuConfig_AvgTemp_AvgReset(Uint16 input);
_EXTERN Uint16 fxnConfig_AvgTemp_AvgReset(Uint16 input);


// MENU 2.3.1
_EXTERN Uint16 mnuConfig_DataLogger_EnableLogger(Uint16 input);
_EXTERN Uint16 fxnConfig_DataLogger_EnableLogger(Uint16 input);
// MENU 2.3.2
_EXTERN Uint16 mnuConfig_DataLogger_Period(Uint16 input);
_EXTERN Uint16 fxnConfig_DataLogger_Period(Uint16 input);


// MENU 2.4.1
_EXTERN Uint16 mnuConfig_AO_LRV(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_LRV(Uint16 input);
// MENU 2.4.2  
_EXTERN Uint16 mnuConfig_AO_URV(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_URV(Uint16 input);
// MENU 2.4.3  
_EXTERN Uint16 mnuConfig_AO_Dampening(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_Dampening(Uint16 input);
// MENU 2.4.4  
_EXTERN Uint16 mnuConfig_AO_Alarm(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_Alarm(Uint16 input);
// MENU 2.4.5  
_EXTERN Uint16 mnuConfig_AO_TrimLo(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_TrimLo(Uint16 input);
// MENU 2.4.6  
_EXTERN Uint16 mnuConfig_AO_TrimHi(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_TrimHi(Uint16 input);
// MENU 2.4.7  
_EXTERN Uint16 mnuConfig_AO_Mode(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_Mode(Uint16 input);
// MENU 2.4.8  
_EXTERN Uint16 mnuConfig_AO_AoValue(Uint16 input);
_EXTERN Uint16 fxnConfig_AO_AoValue(Uint16 input);


// MENU 2.5.1  
_EXTERN Uint16 mnuConfig_Comm_SlaveAddr(Uint16 input);
_EXTERN Uint16 fxnConfig_Comm_SlaveAddr(Uint16 input);
// MENU 2.5.2  
_EXTERN Uint16 mnuConfig_Comm_BaudRate(Uint16 input);
_EXTERN Uint16 fxnConfig_Comm_BaudRate(Uint16 input);
// MENU 2.5.3  
_EXTERN Uint16 mnuConfig_Comm_Parity(Uint16 input);
_EXTERN Uint16 fxnConfig_Comm_Parity(Uint16 input);
// MENU 2.5.4
_EXTERN Uint16 mnuConfig_Comm_Statistics(Uint16 input);
_EXTERN Uint16 fxnConfig_Comm_Statistics(Uint16 input);


// MENU 2.6.1   
_EXTERN Uint16 mnuConfig_Relay_Delay(Uint16 input);
_EXTERN Uint16 fxnConfig_Relay_Delay(Uint16 input);
// MENU 2.6.2   
_EXTERN Uint16 mnuConfig_Relay_Mode(Uint16 input);
_EXTERN Uint16 fxnConfig_Relay_Mode(Uint16 input);
// MENU 2.6.3   
_EXTERN Uint16 mnuConfig_Relay_RelayStatus(Uint16 input);
_EXTERN Uint16 fxnConfig_Relay_RelayStatus(Uint16 input);
// MENU 2.6.3   
_EXTERN Uint16 mnuConfig_Relay_ActWhile(Uint16 input);
_EXTERN Uint16 fxnConfig_Relay_ActWhile(Uint16 input);
// MENU 2.6.4   
_EXTERN Uint16 mnuConfig_Relay_SetPoint(Uint16 input);
_EXTERN Uint16 fxnConfig_Relay_SetPoint(Uint16 input);


// MENU 2.7.1   
_EXTERN Uint16 mnuConfig_DnsCorr_CorrEnable(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_CorrEnable(Uint16 input);
// MENU 2.7.2
_EXTERN Uint16 mnuConfig_DnsCorr_DispUnit(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_DispUnit(Uint16 input);
// MENU 2.7.3  
_EXTERN Uint16 mnuConfig_DnsCorr_CoefD0(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_CoefD0(Uint16 input);
// MENU 2.7.4   
_EXTERN Uint16 mnuConfig_DnsCorr_CoefD1(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_CoefD1(Uint16 input);
// MENU 2.7.5   
_EXTERN Uint16 mnuConfig_DnsCorr_CoefD2(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_CoefD2(Uint16 input);
// MENU 2.7.6   
_EXTERN Uint16 mnuConfig_DnsCorr_CoefD3(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_CoefD3(Uint16 input);
// MENU 2.7.7   
_EXTERN Uint16 mnuConfig_DnsCorr_InputUnit(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_InputUnit(Uint16 input);
// MENU 2.7.8   
_EXTERN Uint16 mnuConfig_DnsCorr_Manual(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_Manual(Uint16 input);
// MENU 2.7.8   
_EXTERN Uint16 mnuConfig_DnsCorr_AiLrv(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_AiLrv(Uint16 input);
// MENU 2.7.9   
_EXTERN Uint16 mnuConfig_DnsCorr_AiUrv(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_AiUrv(Uint16 input);
// MENU 2.7.10   
_EXTERN Uint16 mnuConfig_DnsCorr_Ai_TrimLo(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_Ai_TrimLo(Uint16 input);
// MENU 2.7.11   
_EXTERN Uint16 mnuConfig_DnsCorr_Ai_TrimHi(Uint16 input);
_EXTERN Uint16 fxnConfig_DnsCorr_Ai_TrimHi(Uint16 input);


// MENU 3.1 
_EXTERN Uint16 mnuSecurityInfo_Info(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_SN(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_MC(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_FW(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_HW(Uint16 input);

// MENU 3.2  
_EXTERN Uint16 mnuSecurityInfo_TimeAndDate(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_TimeAndDate(Uint16 input);
// MENU 3.3  
_EXTERN Uint16 mnuSecurityInfo_AccessTech(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_AccessTech(Uint16 input);
// MENU 3.4  
_EXTERN Uint16 mnuSecurityInfo_Diagnostics(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_Diagnostics(Uint16 input);
// MENU 3.5  
_EXTERN Uint16 mnuSecurityInfo_ChangePassword(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_ChangePassword(Uint16 input);
// MENU 3.6  
_EXTERN Uint16 mnuSecurityInfo_Restart(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_Restart(Uint16 input);
// MENU 3.7  
_EXTERN Uint16 mnuSecurityInfo_FactReset(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_FactReset(Uint16 input);
// MENU 3.8
_EXTERN Uint16 mnuSecurityInfo_UpdateFirmware(Uint16 input);
_EXTERN Uint16 fxnSecurityInfo_UpdateFirmware(Uint16 input);


#ifdef MENU_H

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////
//// GENERAL LABELS 
////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//_EXTERN char XXXXXXXX[]						= "XXXXXXXXXXXXXXXX";
_EXTERN char BLANK[]   					    	= "                ";
_EXTERN char MENUERROR[]						= "      MENU ERROR";
_EXTERN char SUCCESS[]							= "  Value Modified";
_EXTERN char FAIL[]								= "  Out of Bounds!";
_EXTERN char INVALID[]							= "   Invalid Input";
_EXTERN char INVALID_STREAM[]					= "  Invalid Stream";
_EXTERN char INVALID_TIME[]	         			= "    Invalid Time";
_EXTERN char CANCEL[]							= "  Entry Canceled";
_EXTERN char BAD_PASS[]							= "Invalid Password";
_EXTERN char GOOD_PASS[]						= "  Unlock Success";
_EXTERN char CHANGE_SUCCESS	[]					= "  Change Success";
_EXTERN char RESET_SUCCESS	[]					= "   Reset Success";
_EXTERN char LOCK[]								= "    Lock Success";
_EXTERN char ACCEPT_SUCCESS[]					= "  Accept Success";
_EXTERN char ENABLE[]							= "          Enable";
_EXTERN char ENABLED[]							= "         Enabled";
_EXTERN char NOT_ENABLED[]						= "     Not Enabled";
_EXTERN char DISABLE[]							= "         Disable";
_EXTERN char DISABLED[]							= "        Disabled";
_EXTERN char ENGAGELOCK[]						= "    Engage Lock?";
_EXTERN char NO_LOCK[]							= "Lock Not Engaged";
_EXTERN char LOCKED[]							= "    Lock Engaged";
_EXTERN char UNLOCKED[]							= "        Unlocked";
_EXTERN char OIL_CAL_STARTED[]					= " Oil Cal Started";
_EXTERN char OIL_CAL_STOPPED[]					= " Oil Cal Stopped";
_EXTERN char AUTO_REVERSE[]						= "    Auto/Reverse";
_EXTERN char AUTOMATIC[]						= "       Automatic";
_EXTERN char WATERCUT[]							= "        Watercut";
_EXTERN char PHASE[]							= "           Phase";
_EXTERN char MANUAL[]							= "          Manual";
_EXTERN char ISTIMECORRECT[]				    = "   Time Captured";
_EXTERN char ERROR[]							= "           Error";
_EXTERN char NO_ERROR[]							= "        No Error";
_EXTERN char NONE[]								= "            None";
_EXTERN char EVEN[]								= "            Even";
_EXTERN char HARD_RESET[]						= "   HARD RESET   ";
_EXTERN char COPY_DATA[]						= " COPY USER DATA ";
_EXTERN char STOP[]						        = " !!! S T O P !!!";
_EXTERN char TAKE_SAMPLE[]						= "   TAKE A SAMPLE";
_EXTERN char ENTER_STREAM[]						= "    Enter Stream";
_EXTERN char ENTER_VALUE[]						= "Enter Sample Val";
_EXTERN char ENTER_BROWSE[]				 		= " ENTER = BROWSE ";
_EXTERN char ENTER_START[]				 		= "  ENTER = START ";
_EXTERN char ENTER_STOP[]						= "  ENTER = STOP  ";
_EXTERN char ENTER_RESTART[]					= " ENTER = RESTART";
_EXTERN char ENTER_ACCEPT[]						= " ENTER = ACCEPT ";
_EXTERN char ENTER_RESET[] 		 				= "  ENTER = RESET ";
_EXTERN char STEP_INPUT[]						= "  STEP = INPUT  ";
_EXTERN char STEP_START[]						= "  STEP = START  ";
_EXTERN char STEP_CONFIRM[]						= " STEP = CONFIRM ";
_EXTERN char STEP_CHANGE[]						= "  STEP = CHANGE ";
_EXTERN char VALUE_CHANGE[]						= " VALUE = CHANGE ";
_EXTERN char READY[]							= "           READY";
_EXTERN char NOT_READY[]						= "       NOT READY";
_EXTERN char AO_ALARM_HIGH[]                    = "          21.0mA";
_EXTERN char AO_ALARM_LOW[]                     = "           3.8mA";	
_EXTERN char AO_ALARM_OFF[]                     = "             Off";	
_EXTERN char TWENTYFOURHR[]						= "           24 HR";
_EXTERN char ONDEMAND[]							= "       On Demand";
_EXTERN char INIT_DATE[]						= "hh:mm mm/dd/20yy";  
_EXTERN char ST_SUCCESS[]						= "         Success";  
_EXTERN char ST_PKT[]						    = "         Inv Pkt";  
_EXTERN char ST_CMD[]						    = "         Inv Cmd";  
_EXTERN char ST_RETRY[]							= "           Retry";  
_EXTERN char WATER_PHASE[]						= "     Water Phase";  
_EXTERN char OIL_PHASE[]						= "       Oil Phase";  
_EXTERN char RELAY_ON[]							= "              On";  
_EXTERN char RELAY_OFF[]						= "             Off";  
_EXTERN char MODBUS[]							= "          Modbus";  
_EXTERN char ANALOG_INPUT[]						= "    Analog Input";  

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/// DENSITY UNIT STRING  ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
_EXTERN char KG_M3[]						    = "Kg/m3";  
_EXTERN char KG_M3_15C[]					    = "Kg/m3@15C";  
_EXTERN char API[]						   	    = "API";  
_EXTERN char API_60F[]						    = "API@60F";  
_EXTERN char SG[]						        = "sg";  
_EXTERN char SG_15C[]						    = "sg@15C";  
_EXTERN char LBS_FT3[]						    = "lbs/ft3";  
_EXTERN char LBS_FT3_60F[]						= "lbs/ft3@60F";  
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////
//// MENU LABELS 
////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//_EXTERN char XXXXXXXX[]						= "XXXXXXXXXXXXXXXX";
_EXTERN char FREQUENCY[]						= "Frequency";
_EXTERN char REFLECTEDPOWER[]					= "Reflect Power";
_EXTERN char PHASETHRESHOLD[]					= "Phase Threshold";
_EXTERN char AVERAGETEMP[]						= "Average Temp";
_EXTERN char DENSITY[]							= "Density";
_EXTERN char DIAGNOSTICSS[]						= "Diagnostics";
_EXTERN char SERIALNUMBER[]						= "Serial Number";

_EXTERN char OPERATION[]						= "1.0 Operation";
_EXTERN char CONFIGURATION[]					= "2.0 Config"; 
_EXTERN char SECURITYINFO[]						= "3.0 Sec & Info";

_EXTERN char STREAM[]							= "1.1 Stream";
_EXTERN char OILADJUST[]						= "1.2 Oil Adjust";
_EXTERN char OILCAPTURE[]						= "1.3 Oil Capture";
_EXTERN char SAMPLE[]							= "1.4 Sample";

_EXTERN char CFG_ANALYZER[]						= "2.1 Analyzer";
_EXTERN char CFG_AVGTEMP[]						= "2.2 Avg Temp";
_EXTERN char CFG_DATALOGGER[]					= "2.3 Data Logger";
_EXTERN char CFG_AO[]							= "2.4 An. Output";
_EXTERN char CFG_COMM[]							= "2.5 Comms";
_EXTERN char CFG_RELAY[]						= "2.6 Relay";
_EXTERN char CFG_DNSCORR[]						= "2.7 Density Corr";

_EXTERN char CFG_ANALYZER_PROCSAVG[]			= "2.1.1 Procs Avg";
_EXTERN char CFG_ANALYZER_TEMPUNIT[]			= "2.1.2 Temp Unit";
_EXTERN char CFG_ANALYZER_TEMPADJ[]				= "2.1.3 Temp Adj";
_EXTERN char CFG_ANALYZER_OILP0[]				= "2.1.4 Oil P0";
_EXTERN char CFG_ANALYZER_OILP1[]				= "2.1.5 Oil P1";
_EXTERN char CFG_ANALYZER_OILINDEX[]			= "2.1.6 Oil Index";
_EXTERN char CFG_ANALYZER_OILFREQLOW[]			= "2.1.7 Oil F. Lo";
_EXTERN char CFG_ANALYZER_OILFREQHI[]			= "2.1.8 Oil F. Hi";
_EXTERN char CFG_ANALYZER_PHASEHOLD[]			= "2.1.9 Phas Hold";

_EXTERN char CFG_AVGTEMP_MODE[]					= "2.2.1 Mode";
_EXTERN char CFG_AVGTEMP_AVGRESET[]				= "2.2.2 Avg Reset";

_EXTERN char CFG_DATALOGGER_ENABLELOGGER[]		= "2.3.1 En. Logger";
_EXTERN char CFG_DATALOGGER_PERIOD[]			= "2.3.2 Period";

_EXTERN char CFG_AO_LRV[]						= "2.4.1 LRV";
_EXTERN char CFG_AO_URV[]						= "2.4.2 URV";
_EXTERN char CFG_AO_DAMPENING[]					= "2.4.3 Dampening ";
_EXTERN char CFG_AO_ALARM[]						= "2.4.4 Alarm";
_EXTERN char CFG_AO_TRIMLO[]					= "2.4.5 Trim 4mA";
_EXTERN char CFG_AO_TRIMHI[]					= "2.4.6 Trim 20mA";
_EXTERN char CFG_AO_MODE[]						= "2.4.7 Mode";
_EXTERN char CFG_AO_AOVALUE[]					= "2.4.8 A. O. Val";
_EXTERN char CFG_MEASVAL[]						= "Supplied Value";

_EXTERN char CFG_COMM_SLAVEADDR[]				= "2.5.1 Slave Addr";
_EXTERN char CFG_COMM_BAUDRATE[]				= "2.5.2 Baud Rate";
_EXTERN char CFG_COMM_PARITY[]					= "2.5.3 Parity";
_EXTERN char CFG_COMM_STATISTICS[]				= "2.5.4 Statistics";

_EXTERN char CFG_RELAY_DELAY[]					= "2.6.1 Delay";
_EXTERN char CFG_RELAY_MODE[]					= "2.6.2 Mode";
_EXTERN char CFG_RELAY_VAR[]					= "2.6.3 Act. While";
_EXTERN char CFG_RELAY_RELAYSTATUS[]			= "2.6.3 R. Status";
_EXTERN char CFG_RELAY_SETPOINT[]				= "2.6.3 Set Point";

_EXTERN char CFG_DNSCORR_CORRENABLE[]			= "2.7.1 Corr. Mode";
_EXTERN char CFG_DNSCORR_DISPUNIT[]				= "2.7.2 Disp. Unit";
_EXTERN char CFG_DNSCORR_COEFD0[]				= "2.7.3 Coeff. D0";
_EXTERN char CFG_DNSCORR_COEFD1[]				= "2.7.4 Coeff. D1";
_EXTERN char CFG_DNSCORR_COEFD2[]				= "2.7.5 Coeff. D2";
_EXTERN char CFG_DNSCORR_COEFD3[]				= "2.7.6 Coeff. D3";
_EXTERN char CFG_DNSCORR_INPUTUNIT[]			= "2.7.7 Input Unit";
_EXTERN char CFG_DNSCORR_MANUAL[]				= "2.7.8 Man. Dens.";
_EXTERN char CFG_DNSCORR_AILRV[]				= "2.7.8 LRV";
_EXTERN char CFG_DNSCORR_AIURV[]				= "2.7.9 URV";
_EXTERN char CFG_DNSCORR_AI_TRIMLO[]			= "2.7.10 Trim 4mA";
_EXTERN char CFG_DNSCORR_AI_TRIMHI[]			= "2.7.11 Trim 20mA";

_EXTERN char SECURITYINFO_INFO[]				= "3.1 Info";
_EXTERN char SECURITYINFO_INFO_SN[]				= "3.1.1 Serial No";
_EXTERN char SECURITYINFO_INFO_MC[]				= "3.1.2 Model Code";
_EXTERN char SECURITYINFO_INFO_FW[]				= "3.1.3 Firmware";
_EXTERN char SECURITYINFO_INFO_HW[]				= "3.1.4 Hardware";
_EXTERN char SECURITYINFO_TIMEANDDATE[]			= "3.2 Time & Date";
_EXTERN char SECURITYINFO_ACCESSTECH[]			= "3.3 Access Tech";
_EXTERN char SECURITYINFO_DIAGNOSTICS[]			= "3.4 Diagnostics";
_EXTERN char SECURITYINFO_CHANGEPASSWORD[]		= "3.5 Change Paswd";
_EXTERN char SECURITYINFO_RESTART[]				= "3.6 Restart";
_EXTERN char SECURITYINFO_FACTRESET[]			= "3.7 Fact. Reset";
_EXTERN char SECURITYINFO_UPDATEFIRMWARE[]		= "3.8 Update F/W";


#endif

//////////////////////////////////////////////////////////////////////////////////////
// TYPE DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////

typedef struct
{ 
// 16x2 LCD Menu
Uint16	state;				// Current machine state of the mnu system
Uint8	pos;				// Blinking position
char 	id;					// Menu ID
Uint8	dir;				// mnu direction -- scroll either left or right
Uint8	debounceDone;
Uint8	isPressAndHold;		// set if mnu button is "held down"
Uint8	isHomeScreen;		// flag that indicatees we did a mnu reset by holding STEP button
Uint8	col;				// cursor position: column (0 to 15)
Uint8	row;				// cursor position: line   (0 to 1)
Uint8	curStat;			// cursor and blink status; 1=blink, 0=noblink
} PDI_MENU;

typedef struct
{
	Uint16	state;			// State ID
	Uint16	id;				// Menu ID
	Uint16	pos;			// Blinking position
	Uint16	(*fxnPtr)(Uint16 input);
} MENU_STATE;

// Create MENU object
_EXTERN PDI_MENU MENU;

//////////////////////////////////////////////////////////////////////////////////////
/// FUNCTION DECLARATIONS
//////////////////////////////////////////////////////////////////////////////////////

_EXTERN void blinkMenu(void);
_EXTERN void setupMenu(void);
_EXTERN void DebounceMBVE(void);
_EXTERN void Coord_Blink_ON(void);
_EXTERN void Coord_Blink_OFF(void);
_EXTERN inline void stopBlinking(void);

#ifdef MENU_H

_EXTERN MENU_STATE MENU_TABLE[] = {

//////////////////////////////////////////////////////////////////////////////////////
/// state  			 id    blink_pos   fxnPtr
/////////////////////////////////////////////////////////////////////////////////////

// HOMESCREEN
{MNU_HOMESCREEN_WTC, 99, 99, mnuHomescreenWaterCut},
{MNU_HOMESCREEN_FREQ, 99, 99, mnuHomescreenFrequency},
{MNU_HOMESCREEN_RP,  99, 99, mnuHomescreenReflectedPower},
{MNU_HOMESCREEN_PT, 99, 99, mnuHomescreenPhaseThreshold},
{MNU_HOMESCREEN_AVT, 99, 99, mnuHomescreenAvgTemp},
{MNU_HOMESCREEN_DST, 99, 99, mnuHomescreenDensity}, 
{MNU_HOMESCREEN_DGN, 99, 99, mnuHomescreenDiagnostics}, 
{FXN_HOMESCREEN_DGN, 99, 99, fxnHomescreenDiagnostics},
{MNU_HOMESCREEN_SRN, 99, 99, mnuHomescreenSerialNumber},

// MENU 1.0
{MNU_OPERATION, 1, 0, mnuOperation},
// MENU 2.0
{MNU_CFG, 2, 0, mnuConfig},
// MENU 3.0
{MNU_SECURITYINFO, 99, 99, mnuSecurityInfo},

// MENU 1.1
{MNU_OPERATION_STREAM,	1, 2, mnuOperation_Stream},
{FXN_OPERATION_STREAM, 99, 99, fxnOperation_Stream},
// MENU 1.2
{MNU_OPERATION_OILADJUST, 2, 2, mnuOperation_OilAdjust},
{FXN_OPERATION_OILADJUST, 99, 99, fxnOperation_OilAdjust},
// MENU 1.3
{MNU_OPERATION_OILCAPTURE, 3, 2, mnuOperation_OilCapture},
{FXN_OPERATION_OILCAPTURE, 99, 99, fxnOperation_OilCapture},
// MENU 1.4
{MNU_OPERATION_SAMPLE, 99, 99, mnuOperation_Sample},
{FXN_OPERATION_SAMPLE_STREAM, 99, 99, fxnOperation_Sample_Stream},
{FXN_OPERATION_SAMPLE_TIMESTAMP, 99, 99, fxnOperation_Sample_Timestamp},
{FXN_OPERATION_SAMPLE_VALUE, 99, 99, fxnOperation_Sample_Value},


// MENU 2.1
{MNU_CFG_ANALYZER, 1, 2, mnuConfig_Analyzer},
// MENU 2.2
{MNU_CFG_AVGTEMP, 2, 2, mnuConfig_AvgTemp},
// MENU 2.3
{MNU_CFG_DATALOGGER, 3, 2, mnuConfig_DataLogger},
// MENU 2.4
{MNU_CFG_AO, 4, 2, mnuConfig_AnalogOutput},
// MENU 2.5
{MNU_CFG_COMM, 5, 2, mnuConfig_Comm},
// MENU 2.6
{MNU_CFG_RELAY, 6, 2, mnuConfig_Relay},
// MENU 2.7
{MNU_CFG_DNSCORR, 99, 99, mnuConfig_DnsCorr},


// MENU 2.1.1
{MNU_CFG_ANALYZER_PROCSAVG, 1, 4, mnuConfig_Analyzer_ProcsAvg},
{FXN_CFG_ANALYZER_PROCSAVG, 99, 99, fxnConfig_Analyzer_ProcsAvg},
// MENU 2.1.2
{MNU_CFG_ANALYZER_TEMPUNIT, 2, 4, mnuConfig_Analyzer_TempUnit},
{FXN_CFG_ANALYZER_TEMPUNIT, 99, 99, fxnConfig_Analyzer_TempUnit},
// MENU 2.1.3
{MNU_CFG_ANALYZER_TEMPADJ, 3, 4, mnuConfig_Analyzer_TempAdj},
{FXN_CFG_ANALYZER_TEMPADJ, 99, 99, fxnConfig_Analyzer_TempAdj},
// MENU 2.1.4
{MNU_CFG_ANALYZER_OILP0, 4, 4, mnuConfig_Analyzer_OilP0},
{FXN_CFG_ANALYZER_OILP0, 99, 99, fxnConfig_Analyzer_OilP0},
// MENU 2.1.5
{MNU_CFG_ANALYZER_OILP1, 5, 4, mnuConfig_Analyzer_OilP1},
{FXN_CFG_ANALYZER_OILP1, 99, 99, fxnConfig_Analyzer_OilP1},
// MENU 2.1.6
{MNU_CFG_ANALYZER_OILINDEX, 6, 4, mnuConfig_Analyzer_OilIndex},
{FXN_CFG_ANALYZER_OILINDEX, 99, 99, fxnConfig_Analyzer_OilIndex},
// MENU 2.1.7
{MNU_CFG_ANALYZER_OILFREQLOW, 7, 4, mnuConfig_Analyzer_OilFreqLow},
{FXN_CFG_ANALYZER_OILFREQLOW, 99, 99, fxnConfig_Analyzer_OilFreqLow},
// MENU 2.1.8
{MNU_CFG_ANALYZER_OILFREQHI, 8, 4, mnuConfig_Analyzer_OilFreqHi},
{FXN_CFG_ANALYZER_OILFREQHI, 99, 99, fxnConfig_Analyzer_OilFreqHi},
// MENU 2.1.9
{MNU_CFG_ANALYZER_PHASEHOLD, 99, 99, mnuConfig_Analyzer_PhaseHold},
{FXN_CFG_ANALYZER_PHASEHOLD, 99, 99, fxnConfig_Analyzer_PhaseHold},


// MENU 2.2.1
{MNU_CFG_AVGTEMP_MODE, 1, 4, mnuConfig_AvgTemp_Mode},
{FXN_CFG_AVGTEMP_MODE, 99, 99, fxnConfig_AvgTemp_Mode},
// MENU 2.2.2
{MNU_CFG_AVGTEMP_AVGRESET, 99, 99, mnuConfig_AvgTemp_AvgReset},
{FXN_CFG_AVGTEMP_AVGRESET, 99, 99, fxnConfig_AvgTemp_AvgReset},


// MENU 2.3.1
{MNU_CFG_DATALOGGER_ENABLELOGGER, 1, 4, mnuConfig_DataLogger_EnableLogger},
{FXN_CFG_DATALOGGER_ENABLELOGGER, 99, 99, fxnConfig_DataLogger_EnableLogger},
// MENU 2.3.2
{MNU_CFG_DATALOGGER_PERIOD, 99, 99, mnuConfig_DataLogger_Period},
{FXN_CFG_DATALOGGER_PERIOD, 99, 99, fxnConfig_DataLogger_Period},


// MENU 2.4.1 
{MNU_CFG_AO_LRV, 1, 4, mnuConfig_AO_LRV},
{FXN_CFG_AO_LRV, 99, 99, fxnConfig_AO_LRV},
// MENU 2.4.2 
{MNU_CFG_AO_URV, 2, 4, mnuConfig_AO_URV},
{FXN_CFG_AO_URV, 99, 99, fxnConfig_AO_URV},
// MENU 2.4.3 
{MNU_CFG_AO_DAMPENING, 3, 4, mnuConfig_AO_Dampening},
{FXN_CFG_AO_DAMPENING, 99, 99, fxnConfig_AO_Dampening},
// MENU 2.4.4 
{MNU_CFG_AO_ALARM, 4, 4, mnuConfig_AO_Alarm},
{FXN_CFG_AO_ALARM, 99, 99, fxnConfig_AO_Alarm},
// MENU 2.4.5 
{MNU_CFG_AO_TRIMLO, 5, 4, mnuConfig_AO_TrimLo},
{FXN_CFG_AO_TRIMLO, 99, 99, fxnConfig_AO_TrimLo},
// MENU 2.4.6 
{MNU_CFG_AO_TRIMHI, 6, 4, mnuConfig_AO_TrimHi},
{FXN_CFG_AO_TRIMHI, 99, 99, fxnConfig_AO_TrimHi},
// MENU 2.4.7 
{MNU_CFG_AO_MODE, 7, 4, mnuConfig_AO_Mode},
{FXN_CFG_AO_MODE, 99, 99, fxnConfig_AO_Mode},
// MENU 2.4.8 
{MNU_CFG_AO_AOVALUE, 99, 99, mnuConfig_AO_AoValue},
{FXN_CFG_AO_AOVALUE, 99, 99, fxnConfig_AO_AoValue},


// MENU 2.5.1 
{MNU_CFG_COMM_SLAVEADDR, 1, 4, mnuConfig_Comm_SlaveAddr},
{FXN_CFG_COMM_SLAVEADDR, 99, 99, fxnConfig_Comm_SlaveAddr},
// MENU 2.5.2  
{MNU_CFG_COMM_BAUDRATE, 2, 4, mnuConfig_Comm_BaudRate},
{FXN_CFG_COMM_BAUDRATE, 99, 99, fxnConfig_Comm_BaudRate},
// MENU 2.5.3  
{MNU_CFG_COMM_PARITY, 3, 4, mnuConfig_Comm_Parity},
{FXN_CFG_COMM_PARITY, 99, 99, fxnConfig_Comm_Parity},
// MENU 2.5.4  
{MNU_CFG_COMM_STATISTICS, 99, 99, mnuConfig_Comm_Statistics},
{FXN_CFG_COMM_STATISTICS, 99, 99, fxnConfig_Comm_Statistics},


// MENU 2.6.1   
{MNU_CFG_RELAY_DELAY, 1, 4, mnuConfig_Relay_Delay},
{FXN_CFG_RELAY_DELAY, 99, 99, fxnConfig_Relay_Delay},
// MENU 2.6.2   
{MNU_CFG_RELAY_MODE, 2, 4, mnuConfig_Relay_Mode},
{FXN_CFG_RELAY_MODE, 99, 99, fxnConfig_Relay_Mode},
// MENU 2.6.3   
{MNU_CFG_RELAY_ACTWHILE, 3, 4, mnuConfig_Relay_ActWhile},
{FXN_CFG_RELAY_ACTWHILE, 99, 99, fxnConfig_Relay_ActWhile},
// MENU 2.6.3   
{MNU_CFG_RELAY_SETPOINT, 3, 4, mnuConfig_Relay_SetPoint},
{FXN_CFG_RELAY_SETPOINT, 99, 99, fxnConfig_Relay_SetPoint},
// MENU 2.6.3   
{MNU_CFG_RELAY_RELAYSTATUS, 99, 99, mnuConfig_Relay_RelayStatus},
{FXN_CFG_RELAY_RELAYSTATUS, 99, 99, fxnConfig_Relay_RelayStatus},


// MENU 2.7.1   
{MNU_CFG_DNSCORR_CORRENABLE, 1, 4, mnuConfig_DnsCorr_CorrEnable},
{FXN_CFG_DNSCORR_CORRENABLE, 99, 99, fxnConfig_DnsCorr_CorrEnable},
// MENU 2.7.2  
{MNU_CFG_DNSCORR_DISPUNIT, 2, 4, mnuConfig_DnsCorr_DispUnit},
{FXN_CFG_DNSCORR_DISPUNIT, 99, 99, fxnConfig_DnsCorr_DispUnit},
// MENU 2.7.3   
{MNU_CFG_DNSCORR_COEFD0, 3, 4, mnuConfig_DnsCorr_CoefD0},
{FXN_CFG_DNSCORR_COEFD0, 99, 99, fxnConfig_DnsCorr_CoefD0},
// MENU 2.7.4   
{MNU_CFG_DNSCORR_COEFD1, 4, 4, mnuConfig_DnsCorr_CoefD1},
{FXN_CFG_DNSCORR_COEFD1, 99, 99, fxnConfig_DnsCorr_CoefD1},
// MENU 2.7.5   
{MNU_CFG_DNSCORR_COEFD2, 5, 4, mnuConfig_DnsCorr_CoefD2},
{FXN_CFG_DNSCORR_COEFD2, 99, 99, fxnConfig_DnsCorr_CoefD2},
// MENU 2.7.6   
{MNU_CFG_DNSCORR_COEFD3, 6, 4, mnuConfig_DnsCorr_CoefD3},
{FXN_CFG_DNSCORR_COEFD3, 99, 99, fxnConfig_DnsCorr_CoefD3},
// MENU 2.7.7   
{MNU_CFG_DNSCORR_INPUTUNIT, 7, 4, mnuConfig_DnsCorr_InputUnit},
{FXN_CFG_DNSCORR_INPUTUNIT, 99, 99, fxnConfig_DnsCorr_InputUnit},
// MENU 2.7.8   
{MNU_CFG_DNSCORR_MANUAL, 8, 4, mnuConfig_DnsCorr_Manual},
{FXN_CFG_DNSCORR_MANUAL, 99, 99, fxnConfig_DnsCorr_Manual},
// MENU 2.7.8   
{MNU_CFG_DNSCORR_AILRV, 8, 4, mnuConfig_DnsCorr_AiLrv},
{FXN_CFG_DNSCORR_AILRV, 99, 99, fxnConfig_DnsCorr_AiLrv},
// MENU 2.7.9   
{MNU_CFG_DNSCORR_AIURV, 9, 4, mnuConfig_DnsCorr_AiUrv},
{FXN_CFG_DNSCORR_AIURV, 99, 99, fxnConfig_DnsCorr_AiUrv},
// MENU 2.7.10
{MNU_CFG_DNSCORR_AI_TRIMLO, 0, 5, mnuConfig_DnsCorr_Ai_TrimLo},
{FXN_CFG_DNSCORR_AI_TRIMLO, 99, 99, fxnConfig_DnsCorr_Ai_TrimLo},
// MENU 2.7.11
{MNU_CFG_DNSCORR_AI_TRIMHI, 99, 99, mnuConfig_DnsCorr_Ai_TrimHi},
{FXN_CFG_DNSCORR_AI_TRIMHI, 99, 99, fxnConfig_DnsCorr_Ai_TrimHi},


// MENU 3.1 
{MNU_SECURITYINFO_INFO, 1, 2, mnuSecurityInfo_Info},
{FXN_SECURITYINFO_SN, 1, 4, fxnSecurityInfo_SN},
{FXN_SECURITYINFO_MC, 2, 4, fxnSecurityInfo_MC},
{FXN_SECURITYINFO_FW, 3, 4, fxnSecurityInfo_FW},
{FXN_SECURITYINFO_HW, 4, 4, fxnSecurityInfo_HW},
// MENU 3.2 
{MNU_SECURITYINFO_TIMEANDDATE, 2, 2, mnuSecurityInfo_TimeAndDate},
{FXN_SECURITYINFO_TIMEANDDATE, 99, 99, fxnSecurityInfo_TimeAndDate},
// MENU 3.3 
{MNU_SECURITYINFO_ACCESSTECH, 3, 2, mnuSecurityInfo_AccessTech},
{FXN_SECURITYINFO_ACCESSTECH, 99, 99, fxnSecurityInfo_AccessTech},
// MENU 3.4 
{MNU_SECURITYINFO_DIAGNOSTICS, 4, 2, mnuSecurityInfo_Diagnostics},
{FXN_SECURITYINFO_DIAGNOSTICS, 99, 99, fxnSecurityInfo_Diagnostics},
// MENU 3.5 
{MNU_SECURITYINFO_CHANGEPASSWORD, 5, 2, mnuSecurityInfo_ChangePassword},
{FXN_SECURITYINFO_CHANGEPASSWORD, 99, 99, fxnSecurityInfo_ChangePassword},
// MENU 3.6 
{MNU_SECURITYINFO_RESTART, 6, 2, mnuSecurityInfo_Restart},
{FXN_SECURITYINFO_RESTART, 99, 99, fxnSecurityInfo_Restart},
// MENU 3.7 
{MNU_SECURITYINFO_FACTRESET, 7, 2, mnuSecurityInfo_FactReset},
{FXN_SECURITYINFO_FACTRESET, 99, 99, fxnSecurityInfo_FactReset},
// MENU 3.8 
{MNU_SECURITYINFO_UPDATEFIRMWARE, 99, 99, mnuSecurityInfo_UpdateFirmware},
{FXN_SECURITYINFO_UPDATEFIRMWARE, 99, 99, fxnSecurityInfo_UpdateFirmware},

//{99, 99, 99, 99}
{NULL, NULL, NULL, NULL}
};

#endif

#undef _EXTERN
#undef MENU_H
#endif
