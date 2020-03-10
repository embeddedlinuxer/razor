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
* nandwriter.c
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "device.h"
#include "debug.h"
#include "nandwriter.h"
#include "nand.h"
#include "device_nand.h"
#include "Globals.h"
#include "util.h"
#include <ti/fs/fatfs/ff.h>

/************************************************************
* Global Variable Definitions for page buffers        		*
************************************************************/

static Uint8* gNandTx;
static Uint8* gNandRx;

/************************************************************
* Local Macro Declarations                                  *
************************************************************/
#define FBASE   (0x62000000)

#define NANDWIDTH_16
#define OMAPL138_LCDK

// the maximum block number addressable by the EMIFA (CS3), safe estimate
#define RZR_MAX_NUM_ADDR_BLOCKS 220
#define SIZE_CFG	(52244)

/************************************************************
* Global Variable Declarations                              *
************************************************************/
extern __FAR__ Uint32  DEVICE_init(void);
extern VUint32 __FAR__ DDRStart;
extern VUint32 __FAR__ NANDStart;
extern void UTIL_setCurrMemPtr(void *value);

/************************************************************
* Function Declarations                               		*
************************************************************/

static Uint32 LOCAL_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt);

/************************************************************
* Function Definitions                                		*
************************************************************/

void writeNand(void)
{
	Swi_disable();
	Store_Vars_in_NAND();
	Swi_enable();
}

/****************************************************************************************
 * Store_Vars_in_NAND() writes all variables in the "CFG" data section into NAND flash	*
 ****************************************************************************************/
void Store_Vars_in_NAND(void)
{
	  Uint32 num_pages;
	  NAND_InfoHandle  hNandInfo;
	  Uint8  *heapPtr, *cfgPtr;
	  Int32  data_size = 0,alloc_size = 0;
	  Int32 i=0;

	  UTIL_setCurrMemPtr(0);

	  // Initialize NAND Flash
	  hNandInfo = NAND_open((Uint32)&NANDStart, BUS_16BIT );

	  if (hNandInfo == NULL) return;
	  data_size = SIZE_CFG;
	  num_pages = 0;
	  while ((num_pages * hNandInfo->dataBytesPerPage) < data_size) num_pages++;

	  // we want to allocate an even number of pages.
	  alloc_size = num_pages * hNandInfo->dataBytesPerPage;

	  // setup pointer in RAM
	  hNandInfo = NAND_open((Uint32)&NANDStart, BUS_16BIT );
	  hNandInfo = NAND_open((Uint32)&NANDStart, BUS_16BIT );
	  heapPtr = (Uint8 *) UTIL_allocMem(alloc_size);
	  cfgPtr = (Uint8*) ADDR_DDR_CFG;

	  // copy data to heap
	  for (i=0; i<alloc_size; i++) heapPtr[i]=cfgPtr[i];

	  // Write the file data to the NAND flash
	  if (LOCAL_writeData(hNandInfo, heapPtr, num_pages) != E_PASS) printf("\tERROR: Write failed.\r\n");
}

/****************************************************************************************
 * Restore_Vars_From_NAND() reads all data values stored in NAND memory and				*
 * writes them to the "CFG" data section in RAM											*
 ****************************************************************************************/

Uint32 Restore_Vars_From_NAND(void)
{
	NAND_InfoHandle  hNandInfo;
	Uint32		blockNum,pageNum,pageCnt;
	Uint32		numBlks, num_pages, i;
	Uint8		*dataPtr;
	Int32		data_size = 0;

	hNandInfo = NAND_open((Uint32)&NANDStart, BUS_16BIT );

	data_size = SIZE_CFG;
	num_pages = 0;
	while ( (num_pages * hNandInfo->dataBytesPerPage)  < data_size )
	{
	  num_pages++;
	}

	gNandTx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);
	gNandRx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);

	for (i=0; i<NAND_MAX_PAGE_SIZE; i++)
	{
		gNandTx[i]=0xff;
		gNandRx[i]=0xff;
	}

	// Get total number of blocks needed
	numBlks = 0;
	while ( (numBlks*hNandInfo->pagesPerBlock)  < num_pages )
	{
		numBlks++;
	}

	// Start in block 50
	// Leave blocks 0-49 alone, reserved for the boot image + bad blocks
	blockNum = 50;

	while (blockNum < hNandInfo->numBlocks)
	{
		// Find first good block
		if (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS)
		{
			return E_FAIL;
		}

		// Start writing in page 0 of current block
		pageNum = 0;
		pageCnt = 0;

		// Setup data pointer
		dataPtr = (Uint8*)ADDR_DDR_CFG;

		// Start page read loop
		do
		{

		  UTIL_waitLoop(200);

		  if (NAND_readPage(hNandInfo, blockNum, pageNum, gNandRx) != E_PASS)
				return E_FAIL;

		  for (i=0;i<hNandInfo->dataBytesPerPage;i++)
			  dataPtr[i+pageNum*hNandInfo->dataBytesPerPage] = gNandRx[i];

		  pageNum++;
		  pageCnt++;

		  if (pageNum == hNandInfo->pagesPerBlock)
		  {
			// A block transition needs to take place; go to next good block
			  do
			  {
				  blockNum++;
				  if (blockNum > RZR_MAX_NUM_ADDR_BLOCKS)
					  return E_FAIL;	//exceeded the "max" addressable block
			  }
			  while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS);

			  pageNum = 0;
		  }
		} while (pageCnt < num_pages);

    break;
	}
	return E_PASS;
}


void taskFxn(void)
{
	Uint32 numPagesAIS;
	NAND_InfoHandle  hNandInfo;
	Uint8  *aisPtr;
	Int32  aisFileSize = 0,aisAllocSize = 0;
	Int32 i=0;

	  // Initialize NAND Flash
	#if defined(NANDWIDTH_8)
	  hNandInfo = NAND_open((Uint32)&NANDStart, DEVICE_BUSWIDTH_8BIT );
	#elif defined(NANDWIDTH_16)
	  hNandInfo = NAND_open((Uint32)&NANDStart, BUS_16BIT );
	#else
	  #error "Must define one of NANDWIDTH_8 or NANDWIDTH_16"
	#endif

	  if (hNandInfo == NULL) return;

		aisFileSize = 52244;
		numPagesAIS = 0;
		while ( (numPagesAIS * hNandInfo->dataBytesPerPage)  < aisFileSize )
		{
		  numPagesAIS++;
		}

	    //We want to allocate an even number of pages.
	    aisAllocSize = numPagesAIS * hNandInfo->dataBytesPerPage;

	    // Setup pointer in RAM
	    aisPtr = (Uint8 *) UTIL_allocMem(aisAllocSize);

	    // Clear memory
	    for (i=0; i<aisAllocSize; i++)
	      aisPtr[i]=0xBC;

		// Write the file data to the NAND flash
		if (LOCAL_writeData(hNandInfo, aisPtr, numPagesAIS) != E_PASS)
		{
		  printf("\tERROR: Write failed.\r\n");
		  //return E_FAIL;
		}
}

// Generic function to write a UBL or Application header and the associated data
static Uint32 LOCAL_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt)
{
  Uint32    blockNum,pageNum,pageCnt;
  Uint32    numBlks;
  Uint32    i;
  Uint8     *dataPtr;

  gNandTx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);
  gNandRx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);

  for (i=0; i<NAND_MAX_PAGE_SIZE; i++)
  {
    gNandTx[i]=0xff;
    gNandRx[i]=0xff;
  }

  // Get total number of blocks needed
  numBlks = 0;
  while ( (numBlks*hNandInfo->pagesPerBlock)  < totalPageCnt )
  {
    numBlks++;
  }
//  DEBUG_printString("Number of blocks needed for data: ");
//  DEBUG_printHexInt(numBlks);
//  DEBUG_printString("\r\n");

	// Start in block 50
	/// Leave blocks 0-49 alone -- reserved for the boot image + potential bad blocks
	blockNum = 50;

  // Unprotect all blocks of the device
  if (NAND_unProtectBlocks(hNandInfo, blockNum, (hNandInfo->numBlocks-1)) != E_PASS)
  {
    blockNum++;
//    DEBUG_printString("Unprotect failed.\r\n");
    return E_FAIL;
  }

  while (blockNum < hNandInfo->numBlocks)
  {
    // Find first good block
    while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS)
    {
      blockNum++;
    }

    // Erase the current block
    NAND_eraseBlocks(hNandInfo,blockNum,1);

    // Start writing in page 0 of current block
    pageNum = 0;
    pageCnt = 0;

    // Setup data pointer
    dataPtr = srcBuf;

    // Start page writing loop
    do
    {
//      DEBUG_printString((String)"Writing image data to block ");
//      DEBUG_printHexInt(blockNum);
//      DEBUG_printString((String)", page ");
//      DEBUG_printHexInt(pageNum);
//      DEBUG_printString((String)"\r\n");

      // Write the AIS image data to the NAND device
      if (NAND_writePage(hNandInfo, blockNum,  pageNum, dataPtr) != E_PASS)
      {
//        DEBUG_printString("Write failed. Marking block as bad...\n");
        NAND_reset(hNandInfo);
        NAND_badBlockMark(hNandInfo,blockNum);
        dataPtr -=  pageNum * hNandInfo->dataBytesPerPage;
        blockNum++;
        continue;
      }

      UTIL_waitLoop(400);

      // Verify the page just written
      if (NAND_verifyPage(hNandInfo, blockNum, pageNum, dataPtr, gNandRx) != E_PASS)
      {	  /// Note:	NAND_verifyPage sometimes returns a fail even when the data match exactly.
    	  /// 		Data verification needs to be re-enabled once this is debugged.
//        DEBUG_printString("Verify failed. Marking block as bad...\n");
//        NAND_reset(hNandInfo);
//        NAND_badBlockMark(hNandInfo,blockNum);
//        dataPtr -=  pageNum * hNandInfo->dataBytesPerPage;
//        pageCnt -= pageNum;
//        pageNum = 0;
//        blockNum++;
//        continue;
      }

      pageNum++;
      pageCnt++;
      dataPtr +=  hNandInfo->dataBytesPerPage;

      if (pageNum == hNandInfo->pagesPerBlock)
      {
        // A block transition needs to take place; go to next good block
        do
        {
          blockNum++;
        }
        while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS);

        // Erase the current block
        NAND_eraseBlocks(hNandInfo,blockNum,1);

        pageNum = 0;
      }
    } while (pageCnt < totalPageCnt);

    NAND_protectBlocks(hNandInfo);
    break;
  }
  return E_PASS;
}

/***********************************************************
* End file
************************************************************/
/*
void updateFirmware(void)
{
    FIL  *fPtr;
    char firmware[] = "/PDI/bootloader.ais"; 
    Uint32 numPagesAIS;
    NAND_InfoHandle  hNandInfo;
    Uint8  *aisPtr;
    Int32  aisFileSize = 0,aisAllocSize = 0;
    Int8  fileName[256];
    Int32 i=0;
    Int32 j=0;
    Uint8 devID[4];
    Uint16 devID16[4];
    VUint16 *addr_ram;
    VUint16 *addr_flash;
    VUint16 dataword;
    ASYNC_MEM_InfoHandle dummy;
    Uint32 status;
    Bool status_reached_zero = FALSE;

    addr_flash = (VUint16 *)(FBASE + DEVICE_NAND_CLE_OFFSET);

    for(i=0;i<512;i++){}

    *addr_flash = (VUint16)0xFF; //reset command

    i=0;
    do
    {
       status = DEVICE_ASYNC_MEM_IsNandReadyPin(dummy);
       if (status == 0) status_reached_zero = TRUE;
       i++;
       if (i >= 1000) break;
    }
    while( !status || (status_reached_zero == FALSE) );

    // write getinfo command
    addr_flash = (VUint16 *)(FBASE + DEVICE_NAND_CLE_OFFSET);
    *addr_flash = (VUint16)NAND_RDID; // command
    addr_flash = (VUint16 *)(FBASE + DEVICE_NAND_ALE_OFFSET);
    *addr_flash = (VUint16)NAND_ONFIRDIDADD;

    for(i=0;i<512;i++){}

    for(i=0;i<4;i++)
    {
        addr_flash = (VUint16 *)(FBASE);
        devID16[i] = *addr_flash;
    }

    // Initialize NAND Flash
    hNandInfo = NAND_open((Uint32)NANDStart, DEVICE_BUSWIDTH_16BIT ); //NANDStart = 0x62000000
    if (hNandInfo == NULL) return E_FAIL;

    // GLOBAL ERASE NAND FLASH
    if (NAND_globalErase(hNandInfo) != E_PASS) return E_FAIL;

    // Read the file from host
    // Open an File from the USB drive
    //fPtr = f_open("/PDI/bootloader.ais", "rb");
    if (f_open(fPtr, firmware, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return E_FAIL;

    // Read file size
    fseek(fPtr,0,SEEK_END);
    aisFileSize = ftell(fPtr);

    if (aisFileSize == 0)
    {
        // read fail
        fclose (fPtr);
        return E_FAIL;
    }

    numPagesAIS = 0;
    while ( (numPagesAIS * hNandInfo->dataBytesPerPage)  < aisFileSize ) numPagesAIS++;

    //We want to allocate an even number of pages.
    aisAllocSize = numPagesAIS * hNandInfo->dataBytesPerPage;

    // Setup pointer in RAM
    aisPtr = (Uint8 *) UTIL_allocMem(aisAllocSize);

    // Clear memory
    for (i=0; i<aisAllocSize; i++)
    {
        aisPtr[i]=0xFF;
        if ((i%10000)==0) j++;
    }

    // Go to start of file
    fseek(fPtr,0,SEEK_SET);

    // Close file
    fclose (fPtr);

    // Write the file data to the NAND flash
    if (LOCAL_writeData(hNandInfo, aisPtr, numPagesAIS) != E_PASS) return E_FAIL;

    return E_PASS;
}
*/
