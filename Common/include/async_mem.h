/*
 * async_mem.h
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
  FILE      : async_mem.h
  PROJECT   : TI Booting and Flashing Utilities
  AUTHOR    : Daniel Allred
  DESC      : Header to define common interface for async memory controller
-------------------------------------------------------------------------- */

#ifndef _ASYNC_MEM_H_
#define _ASYNC_MEM_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/


/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

// Supported asynchronous memory type
typedef enum _ASYNC_MEM_TYPE_
{
  AYSNC_MEM_TYPE_SRAM     = 0x01,
  AYSNC_MEM_TYPE_NOR      = 0x02,
  AYSNC_MEM_TYPE_NAND     = 0x03,
  AYSNC_MEM_TYPE_ONENAND  = 0x04
}
ASYNC_MEM_Type;

// ASYNC_MEM_INFO structure - holds pertinent info for open driver instance
typedef struct _ASYNC_MEM_INFO_
{
  ASYNC_MEM_Type    memType;            // Type of memory
  Uint8             busWidth;           // Operating bus width  
  Uint8             interfaceNum;       // Async Memory Interface Number
  Uint8             chipSelectNum;      // Operating chip select
  void              *regs;              // Configuration register overlay
  struct _ASYNC_MEM_DEVICE_INFO_ *hDeviceInfo;
} 
ASYNC_MEM_InfoObj, *ASYNC_MEM_InfoHandle;

// Supported asynchronous memory interface type
typedef enum _ASYNC_MEM_DEVICE_INTERFACE_TYPE_
{
  AYSNC_MEM_INTERFACE_TYPE_EMIF2 = 0x01,
  AYSNC_MEM_INTERFACE_TYPE_GPMC  = 0x02
}
ASYNC_MEM_DEVICE_InterfaceType;

typedef struct _ASYNC_MEM_DEVICE_INTERFACE_
{
  const ASYNC_MEM_DEVICE_InterfaceType type;
  const void *regs;
  const Uint32 regionCnt;
  const Uint32 *regionStarts;
  const Uint32 *regionSizes;
}
ASYNC_MEM_DEVICE_InterfaceObj, *ASYNC_MEM_DEVICE_InterfaceHandle;

typedef void (*ASYNC_MEM_DEVICE_Init)(ASYNC_MEM_InfoHandle hAsyncMemInfo);
typedef void (*ASYNC_MEM_DEVICE_Close)(ASYNC_MEM_InfoHandle hAsyncMemInfo);
typedef Uint8 (*ASYNC_MEM_DEVICE_IsNandReadyPin)(ASYNC_MEM_InfoHandle hAsyncMemInfo);

typedef struct _ASYNC_MEM_DEVICE_INFO_
{
  const Uint8                           interfaceCnt;
  const ASYNC_MEM_DEVICE_InterfaceObj   *interfaces;
  const ASYNC_MEM_DEVICE_Init           fxnInit;
  const ASYNC_MEM_DEVICE_IsNandReadyPin fxnNandIsReadyPin;
}
ASYNC_MEM_DEVICE_InfoObj, *ASYNC_MEM_DEVICE_InfoHandle;


/************************************************************
* Global Function Declarations                              *
************************************************************/

extern __FAR__ ASYNC_MEM_InfoHandle ASYNC_MEM_Open(ASYNC_MEM_Type memType, Uint32 baseAddress, Uint8 busWidth);


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif //_ASYNC_MEM_H_

