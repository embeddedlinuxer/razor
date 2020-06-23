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
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Rewrote 90% of the entire projet
*------------------------------------------------------------------------*/

#undef MENU_H_
#include <ti/board/board.h>

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "device.h"
#include "debug.h"
#include "nandwriter.h"
#include "nand.h"
#include "device_nand.h"
#include "util.h" 
#include "Globals.h"

#define NANDWIDTH_16
#define C6748_LCDK

void Init_BoardClocks(void);
static inline void Init_Counter_3(void);
static inline void initHardwareObjects(void);
static inline void initSoftwareObjects(void);
static inline void startClocks(void);
static inline void Init_All(void);

int main (void)
{
    // SUSPEND SOURCE REGISTER (SUSPSRC)
    SYSTEM->SUSPSRC &= ((1 << 27) | (1 << 22) | (1 << 20) | (1 << 5) | (1 << 16));

    // INITIALIZE EMIF FOR CS4 FLASH MEMORY REGION
    CSL_FINST(emifaRegs->CE4CFG,EMIFA_CE4CFG_ASIZE,16BIT);
    CSL_FINST(emifaRegs->NANDFCR,EMIFA_NANDFCR_CS4NAND,NAND_ENABLE);

    // INITIALIZE C6748 SPECIFcd IC BOARD CLOCKS
    Init_BoardClocks();

    // INITIALIZE POWER AND SLEEP CONTROLLER
	Init_PSC();

    // INITIALIZE PIN MUXING
	Init_PinMux();

    // INITIALIZE EVERYTHING ELSE
	Init_All();

    // USB OSAL DELAY TIMER 
    delayTimerSetup();

    // START TI-RTOS KERNEL
	BIOS_start();

	return 0;
}

static inline void Init_All(void)
{
    // RESTORE ALL VALUES FROM NAND FLASH MEMORY
	Restore_Vars_From_NAND();

    // INITIAL FLASHING OR FACTORY RESET LEVEL
	if (!COIL_LOCKED_SOFT_FACTORY_RESET.val)
	{	
        if (!COIL_LOCKED_HARD_FACTORY_RESET.val) 
        {
            initializeAllRegisters();
        }
		reloadFactoryDefault();
		Store_Vars_in_NAND();
	}
	else
		resetGlobalVars(); 

	// INITIALIZE HARDWARES 
	initHardwareObjects();

	// INITIALIZE MENU SYSTEM
	setupMenu();

	// INITIALIZE TIMER COUNTER
	Init_Counter_3();

    // CONFIGURE UART BAUDRATE & PARITY OPTIONS
	Config_Uart(REG_BAUD_RATE.calc_val,UART_PARITY_NONE);

	// Initialize software objects
	initSoftwareObjects();

	// START VARIOUS CLOCKS 
	startClocks();
}


void Init_BoardClocks(void)
{
    Uint32 i;
    for (i=0; i < 5000000; i++);

    Board_moduleClockSyncReset(CSL_PSC_USB20);
    Board_moduleClockSyncReset(CSL_PSC_GPIO);
    Board_moduleClockSyncReset(CSL_PSC_I2C1);

    Board_moduleClockEnable(CSL_PSC_USB20); // "USB0_REFCLKIN" CLOCK SHARES WITH UART1
    Board_moduleClockEnable(CSL_PSC_GPIO);
    Board_moduleClockEnable(CSL_PSC_I2C1);

    Board_moduleClockEnable(CSL_PSC_UART2);
}


/// NOTE //////////////////////////////////////////////////////
/// A lot of the CSL macros were causing inexplicable problems
/// writing to the RTC, as well as FINST(...,RUN) causing a freeze
/// on power cycle. If it's a bug in CSL it's not immediately
/// obvious... need to look into this when I have more time.
/// Still waiting on a processor board with a working RTC battery.
//////////////////////////////////////////////////////////////

static inline void initSoftwareObjects(void)
{
	Init_Data_Buffer();
}


static inline void initHardwareObjects(void)
{
	// Setup i2c registers and start i2c running (PDI_I2C.c)
	Init_I2C();

	// initialize lcd driver (PDI_I2C.c)
	Init_LCD();

	// Initialize buttons (PDI_I2C.c)
	Init_MBVE();

	// Setup uart registers and start the uart running (ModbusRTU.c)
	Init_Uart();

	// Initialize modbus buffers and counters (ModbusRTU.c)
	Init_Modbus();
}

static inline void startClocks(void)
{
	Clock_start(Update_Relays_Clock);
	Clock_start(Capture_Sample_Clock);
}

static inline void Init_Counter_3(void)
{
    int key;

    key = Hwi_disable();

    CSL_FINST(tmr3Regs->TGCR,TMR_TGCR_TIM34RS,RESET);
    CSL_FINST(tmr3Regs->TGCR,TMR_TGCR_TIM12RS,RESET);
    CSL_FINST(tmr3Regs->TCR,TMR_TCR_ENAMODE12,DISABLE);

    //set timer mode to 64-bit
    CSL_FINST(tmr3Regs->TGCR,TMR_TGCR_TIMMODE,64BIT_GPT);

    //Set clock source to external
    CSL_FINST(tmr3Regs->TCR,TMR_TCR_CLKSRC12,TIMER);

    //take timer out of reset
    CSL_FINST(tmr3Regs->TGCR,TMR_TGCR_TIM34RS,NO_RESET);
    CSL_FINST(tmr3Regs->TGCR,TMR_TGCR_TIM12RS,NO_RESET);

    //reset counter value to zero (upper & lower)
    tmr3Regs->TIM12 = 0;
    tmr3Regs->TIM34 = 0;

    // timer period set to max
    tmr3Regs->PRD12 = 0xFFFFFFFF;
    tmr3Regs->PRD34 = 0xFFFFFFFF;

    CSL_FINS(gpioRegs->BANK[GP6].OUT_DATA,GPIO_OUT_DATA_OUT1,TRUE);

    // Enable counter
    CSL_FINST(tmr3Regs->TCR,TMR_TCR_ENAMODE12,EN_ONCE);

    Hwi_restore(key);

    // Start counter timer
    Timer_start(counterTimerHandle);
}
