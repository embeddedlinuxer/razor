/*
 * nandwriter.h
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
  FILE        : nandwriter.h
  PURPOSE     : UBL main header file
  PROJECT     : DaVinci CCS NAND Flashing Utility
  AUTHOR      : Daniel Allred
  DATE        : 04-Jun-2007  
 
    HISTORY
 	     v1.00 - DJA - 04-Jun-2007
 	        Completion (with support for DM6441 and DM6441_LV)
 	     
 ----------------------------------------------------------------------------- */

/*------------------------------------------------------------------------
* nandwriter.h
*-------------------------------------------------------------------------
* This code is essentially TI's NANDWriter project integrated into our 
* firmware and adapted to run inside the TI-RTOS kernel. The code looks a bit 
* arcane because it was originally written (by TI) to work with a slew of 
* different processors and NAND chips. I've noticed some strange behavior in 
* the write-verification routine, NAND_verifyPage(), that will sometimes 
* return in a false positive result for data corruption. I mean REALLY weird 
* behavior... like the boolean expression (0xFF == 0xFF) returning FALSE. 
* Currently the fail-status of data verification is ignored until someone can 
* poke around in the disassembly (joy!) to figure out what is going on there.
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#ifndef _NANDWRITER_H_
#define _NANDWRITER_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/**************************************************
* Global Macro Declarations                       *
**************************************************/

//UBL version number
#define UBL_VERSION_STRING "1.00"
#ifdef UBL_NAND
#define UBL_FLASH_TYPE "NAND"
#else
#define UBL_FLASH_TYPE "NOR"
#endif

////////////////////////////////////////////////////////////////////
#define ADDR_DDR_CFG		(0xC7FF33EC) //beginning of CFG section
////////////////////////////////////////////////////////////////////

// Define MagicNumber constants
#define MAGIC_NUMBER_VALID          (0xA1ACED00)

// Used by UBL when doing UART boot
#define UBL_UART_BOOT               (0xA1ACED00)		/* Safe boot mode */
#define UBL_NOR_BURN                (0xA1ACED11)		/* Download via UART & Burn NOR with UBL readable header and BIN data*/
#define UBL_NOR_ERASE               (0xA1ACED22)		/* Download via UART & Global erase the NOR Flash*/
#define UBL_NAND_BURN               (0xA1ACED33)		/* Download via UART & Burn NAND - Image is binary */
#define UBL_NAND_ERASE              (0xA1ACED44)		/* Download via UART & Global erase the NAND Flash*/

// Used by RBL when doing NAND boot
#define UBL_MAGIC_SAFE              (0xA1ACED00)		/* Safe boot mode */
#define UBL_MAGIC_DMA               (0xA1ACED11)		/* DMA boot mode */
#define UBL_MAGIC_IC                (0xA1ACED22)		/* I Cache boot mode */
#define UBL_MAGIC_FAST              (0xA1ACED33)		/* Fast EMIF boot mode */
#define UBL_MAGIC_DMA_IC            (0xA1ACED44)		/* DMA + ICache boot mode */
#define UBL_MAGIC_DMA_IC_FAST       (0xA1ACED55)		/* DMA + ICache + Fast EMIF boot mode */

// Used by UBL when doing UART boot, UBL Nor Boot, or NAND boot
//#define UBL_MAGIC_BIN_IMG           (0xA1ACED66)		/* Execute in place supported*/
#define UBL_MAGIC_BIN_IMG           (0xB1ACED22)		/* Execute in place supported*/

// Used by UBL when doing UART boot
#define UBL_MAGIC_NOR_RESTORE       (0xA1ACED77)		/* Download via UART & Restore NOR with binary data */
#define UBL_MAGIC_NOR_SREC_BURN     (0xA1ACED88)		/* Download via UART & Burn NOR with UBL readable header and SREC data*/
#define UBL_MAGIC_NOR_BIN_BURN      (0xA1ACED99)		/* Download via UART & Burn NOR with UBL readable header and BIN data*/
#define UBL_MAGIC_NOR_GLOBAL_ERASE  (0xA1ACEDAA)		/* Download via UART & Global erase the NOR Flash*/
#define UBL_MAGIC_NAND_SREC_BURN    (0xA1ACEDBB)		/* Download via UART & Burn NAND - Image is S-record*/
#define UBL_MAGIC_NAND_BIN_BURN     (0xA1ACEDCC)		/* Download via UART & Burn NAND - Image is binary */
#define UBL_MAGIC_NAND_GLOBAL_ERASE (0xA1ACEDDD)		/* Download via UART & Global erase the NAND Flash*/

// Define max UBL image size
#define UBL_IMAGE_SIZE              (0x00007800u)

// Define max app image size
#define APP_IMAGE_SIZE              (0x02000000u)

// Defines for which blocks are valid for writing UBL and APP data
#define NANDWRITER_START_APP_BLOCK_NUM     (25)
#define NANDWRITER_END_APP_BLOCK_NUM       (50)

/************************************************
* Global Typedef declarations                   *
************************************************/

typedef struct
{
  Uint32 magicNum;    // Expected magic number
  Uint32 entryPoint;  // Entry point of the user application
  Uint32 numPage;     // Number of pages where boot loader is stored
  Uint32 block;       // Starting block number where User boot loader is stored
  Uint32 page;        // Starting page number where boot-loader is stored
  Uint32 ldAddress;   // Starting RAM address where image is to copied - XIP Mode
}
NANDWRITER_Boot;

//////////////////////////////////
/// swi to writeNand()
//////////////////////////////////
extern void writeNand(void);
extern void updateFirmware(void);

/******************************************************
* Global Function Declarations                        *
******************************************************/

int main(void);

///////////////////////////////
void Store_Vars_in_NAND(void);
Uint32 Restore_Vars_From_NAND(void);
///////////////////////////////

/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif //_NANDWRITER_H_

/* --------------------------------------------------------------------------
  HISTORY
    v1.00 - DJA - 02-Nov-2007
      Initial release
-------------------------------------------------------------------------- */
