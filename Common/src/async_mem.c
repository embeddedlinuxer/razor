/*
 * async_mem.c
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
  FILE      : async_mem.c 				                             	 	        
  PROJECT   : TI Boot and Flashing Utilities
  AUTHOR    : Daniel Allred
  DESC	    : Generic Asynchronous Memory driver
-------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// Device specific CSL
#include "device.h"

// Misc. utility function include
#include "util.h"

// This module's header file  
#include "async_mem.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

// The device specific async memory info structure
extern ASYNC_MEM_DEVICE_InfoObj     DEVICE_ASYNC_MEM_info;


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

#ifdef USE_IN_ROM
ASYNC_MEM_InfoObj gAsyncMemInfo;
#endif


/************************************************************
* Global Function Definitions                               *
************************************************************/

ASYNC_MEM_InfoHandle ASYNC_MEM_Open(ASYNC_MEM_Type memType, Uint32 baseAddress, Uint8 busWidth)
{
  Uint32 i,j;
  ASYNC_MEM_InfoHandle hAsyncMemInfo;
  
  // Create structure
#ifdef USE_IN_ROM
  hAsyncMemInfo = (ASYNC_MEM_InfoHandle) &gAsyncMemInfo;
#else
  hAsyncMemInfo = (ASYNC_MEM_InfoHandle) UTIL_callocMem(sizeof(ASYNC_MEM_InfoObj));
#endif

  // Fill in structure
  hAsyncMemInfo->hDeviceInfo = &DEVICE_ASYNC_MEM_info;
  hAsyncMemInfo->memType = memType;
  hAsyncMemInfo->busWidth = busWidth;
  
  hAsyncMemInfo->interfaceNum = 0xFF;
  for (i = 0; ((i<hAsyncMemInfo->hDeviceInfo->interfaceCnt) && (hAsyncMemInfo->interfaceNum == 0xFF)); i++)
  {
    for (j = 0; j<hAsyncMemInfo->hDeviceInfo->interfaces[i].regionCnt; j++)
    {
      Uint32 start, end;
      
      start = hAsyncMemInfo->hDeviceInfo->interfaces[i].regionStarts[j];
      end = start + hAsyncMemInfo->hDeviceInfo->interfaces[i].regionSizes[j];
      if ((baseAddress>=start) && (baseAddress <end))
      {
        hAsyncMemInfo->interfaceNum = i;
        hAsyncMemInfo->chipSelectNum = j;
        hAsyncMemInfo->regs = (void *) hAsyncMemInfo->hDeviceInfo->interfaces[i].regs;
        break;
      }
    }
  }
  
  // Do device level init (pinmux, power domain, etc.)
  if (DEVICE_AsyncMemInit(hAsyncMemInfo->interfaceNum) != E_PASS)
  {
    return NULL;
  }
  
  // Do device specific init for the specified memory type, memory width, etc.
  (*hAsyncMemInfo->hDeviceInfo->fxnInit)(hAsyncMemInfo);
  
  return hAsyncMemInfo;    
}


/************************************************************
* Local Function Definitions                                *
************************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

