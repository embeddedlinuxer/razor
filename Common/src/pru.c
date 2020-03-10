/*
 * pru.c
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/* --------------------------------------------------------------------------
  FILE        : pru.c                                                   
  PROJECT     : DA8xx/OMAP-L138 ROM Boot-Loader
  AUTHOR      : Daniel Allred
  DESC        : PRU APIs
 ----------------------------------------------------------------------------- */

/************************************************************
* Include Files                                             *
************************************************************/

// General type include
#include "tistdtypes.h"

// Device Specific CSL / Functions
#include "device.h"

// This module's header file
#include "pru.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern __FAR__ Uint32 PRU0_DATA_START, PRU0_DATA_SIZE;
extern __FAR__ Uint32 PRU0_INST_START, PRU0_INST_SIZE;
extern __FAR__ Uint32 PRU0_CODE_START, PRU0_CODE_SIZE;

extern __FAR__ Uint32 PRU1_DATA_START, PRU1_DATA_SIZE;
extern __FAR__ Uint32 PRU1_INST_START, PRU1_INST_SIZE;


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/


/************************************************************
* Global Function Definitions                               *
************************************************************/

// Load the specified PRU with code
void PRU_load (Uint32* pruCode, Uint32 codeSizeInWords)
{ 
  Uint32 i, *pruIram; 
  
  pruIram = (Uint32 *) &PRU0_INST_START;
   
  PRU_enable();

  // Copy PRU code to its instruction RAM
  for(i=0; i<codeSizeInWords; i++)
  {
    pruIram[i] = pruCode[i];
  }
}

void PRU_run ()
{ 
  // Enable PRU, let it execute the code we just copied
  PRU0->CONTROL = (PRU0->CONTROL & ~DEVICE_PRU_CONTROL_COUNTENABLE_MASK) | (0x1 << DEVICE_PRU_CONTROL_COUNTENABLE_SHIFT);
  PRU0->CONTROL = (PRU0->CONTROL & ~DEVICE_PRU_CONTROL_ENABLE_MASK) | (0x1 << DEVICE_PRU_CONTROL_ENABLE_SHIFT);
}

Uint32 PRU_waitForHalt (Int32 timeout)
{
  Int32 cnt = timeout;
  
  while ( ((PRU0->CONTROL & DEVICE_PRU_CONTROL_RUNSTATE_MASK)>> DEVICE_PRU_CONTROL_RUNSTATE_SHIFT) == 0x1 ) 
  {
    if ( cnt>0 )
    {  
      cnt--;
    }
    if (cnt == 0)
    {
      return E_TIMEOUT;
    }
  }

  return E_PASS;
}

void PRU_disable( void )
{
  // Disable PRU0
  PRU0->CONTROL = PRU0->CONTROL & ~DEVICE_PRU_CONTROL_COUNTENABLE_MASK;
  PRU0->CONTROL = PRU0->CONTROL & ~DEVICE_PRU_CONTROL_ENABLE_MASK;
  
  // Reset PRU0
  PRU0->CONTROL = PRU0->CONTROL & ~DEVICE_PRU_CONTROL_SOFTRESET_MASK;
  
  // Disable PRU SS
  DEVICE_LPSCTransition(PSCNUM0, LPSC_PRU, PD0, PSC_SWRSTDISABLE); 
}

void PRU_enable ( void )
{ 
  // Enable PRU SS
  DEVICE_LPSCTransition(PSCNUM0, LPSC_PRU, PD0, PSC_ENABLE);
  
  // Reset PRU0
  PRU0->CONTROL = 0x00000000;
}


/************************************************************
* Local Function Definitions                                *
************************************************************/



/***********************************************************
* End file                                                 *
***********************************************************/
