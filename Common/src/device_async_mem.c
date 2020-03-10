/*
 * device_async_mem.c
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
    FILE        : device_async_mem.c 				                             	 	        
    PROJECT     : TI Booting and Flashing Utilities
    AUTHOR      : Daniel Allred
    DESC        : This file descibes and implements various device-specific 
                  Async Memory components
-------------------------------------------------------------------------- */ 

// General type include
#include "tistdtypes.h"

// Device specific CSL
#include "device.h"

// Device specific EMIF details
#include "device_async_mem.h"

// Generic Async Mem header file
#include "async_mem.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

static void DEVICE_ASYNC_MEM_Init(ASYNC_MEM_InfoHandle hAsyncMemInfo);
static Uint8 DEVICE_ASYNC_MEM_IsNandReadyPin(ASYNC_MEM_InfoHandle hAsyncMemInfo);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

// AEMIF Definitions
const Uint32 DEVICE_ASYNC_MEM_regionStarts[DEVICE_ASYNC_MEM0_REGION_CNT] =
{
  0x60000000,
  0x62000000,
  0x64000000,
  0x66000000
};

const Uint32 DEVICE_ASYNC_MEM_regionSizes[DEVICE_ASYNC_MEM0_REGION_CNT] =
{
  0x02000000,
  0x02000000,
  0x02000000,
  0x02000000
};

const ASYNC_MEM_DEVICE_InterfaceObj DEVICE_ASYNC_MEM_interfaces[DEVICE_ASYNC_MEM_INTERFACE_CNT] =
{
  {
    AYSNC_MEM_INTERFACE_TYPE_EMIF2,
    (void *) AEMIF,
    DEVICE_ASYNC_MEM0_REGION_CNT,
    DEVICE_ASYNC_MEM_regionStarts,
    DEVICE_ASYNC_MEM_regionSizes
  }
};

const ASYNC_MEM_DEVICE_InfoObj DEVICE_ASYNC_MEM_info = 
{
  DEVICE_ASYNC_MEM_INTERFACE_CNT,       // interfaceCnt 
  DEVICE_ASYNC_MEM_interfaces,          // interfaces
  &(DEVICE_ASYNC_MEM_Init),             // fxnOpen
  &(DEVICE_ASYNC_MEM_IsNandReadyPin)    // fxnNandIsReadyPin;    
};


/************************************************************
* Global Function Definitions                               *
************************************************************/


/************************************************************
* Local Function Definitions                                *
************************************************************/

static void DEVICE_ASYNC_MEM_Init(ASYNC_MEM_InfoHandle hAsyncMemInfo)
{
  
  if (hAsyncMemInfo->interfaceNum == 0)
  {
    VUint32 *ABCR = NULL;
    DEVICE_Emif25Regs *EMIF = (DEVICE_Emif25Regs *) hAsyncMemInfo->regs;

    // Do width settings
    EMIF->AWCCR &= ~(DEVICE_EMIF_AWCC_WAITSTATE_MASK);
    ABCR = &(EMIF->A1CR);
    // Adjust for quicker access times   
    ABCR[hAsyncMemInfo->chipSelectNum] = 0x3FFFFFFC | ((hAsyncMemInfo->busWidth == DEVICE_BUSWIDTH_8BIT)? 0 : 1);
    
    // Do NAND enable
    if (hAsyncMemInfo->memType == AYSNC_MEM_TYPE_NAND)
    {
      // Setup  registers for NAND  
      EMIF->NANDFCR |= (0x1 << (hAsyncMemInfo->chipSelectNum));        // NAND enable for CSx
    }
  }
}

static Uint8 DEVICE_ASYNC_MEM_IsNandReadyPin(ASYNC_MEM_InfoHandle hAsyncMemInfo)
{
  return ((Uint8) ((AEMIF->NANDFSR & DEVICE_EMIF_NANDFSR_READY_MASK)>>DEVICE_EMIF_NANDFSR_READY_SHIFT));
}


/************************************************************
* End file                                                  *
************************************************************/


