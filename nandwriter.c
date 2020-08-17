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
* FIRMWARE AND VARIABLE START BLOCK ADDRESS
************************************************************/
#define ACCESS_DELAY	1000
#define APP_START_BLK 	1
#define VAR_START_BLK 	50

/************************************************************
* Local Macro Declarations                                  *
************************************************************/

#define NANDWIDTH_16
#define MAX_BLK_NUM     220
#define SIZE_CFG        52244
#define FBASE           0x62000000
#define NANDStart       0x62000000

/************************************************************
* Global Variable Definitions for page buffers              *
************************************************************/

extern VUint32 __FAR__ DDRStart;
static Uint8* gNandTx;
static Uint8* gNandRx;

/************************************************************
* Function Declarations                                     *
************************************************************/

static Uint32 LOCAL_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt);
static Uint32 USB_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt);
extern void UTIL_setCurrMemPtr(void *value);

/************************************************************
* Function Definitions                                		*
************************************************************/

static Uint8 DEVICE_ASYNC_MEM_IsNandReadyPin(ASYNC_MEM_InfoHandle hAsyncMemInfo)
{
    return ((Uint8) ((AEMIF->NANDFSR & DEVICE_EMIF_NANDFSR_READY_MASK)>>DEVICE_EMIF_NANDFSR_READY_SHIFT));
}


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
    Uint8 *heapPtr, *cfgPtr;
    Int32 data_size = 0, alloc_size = 0, i = 0;

    UTIL_setCurrMemPtr(0);

    // Initialize NAND Flash
    hNandInfo = NAND_open((Uint32)NANDStart, BUS_16BIT );

    if (hNandInfo == NULL) return;
    data_size = SIZE_CFG;
    num_pages = 0;
    while ((num_pages * hNandInfo->dataBytesPerPage) < data_size) num_pages++;

    // we want to allocate an even number of pages.
    alloc_size = num_pages * hNandInfo->dataBytesPerPage;

    // setup pointer in RAM
    hNandInfo = NAND_open((Uint32)NANDStart, BUS_16BIT );
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
    Uint8 *dataPtr;
    Int32 data_size = 0;
    NAND_InfoHandle hNandInfo;
    Uint32 numBlks, num_pages, i, blockNum, pageNum, pageCnt;

    hNandInfo = NAND_open((Uint32)NANDStart, BUS_16BIT );

    data_size = SIZE_CFG;
    num_pages = 0;

    while ((num_pages * hNandInfo->dataBytesPerPage)  < data_size)  num_pages++;

    gNandTx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);
    gNandRx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);

    for (i=0; i<NAND_MAX_PAGE_SIZE; i++)
    {
        gNandTx[i]=0xff;
        gNandRx[i]=0xff;
    }

    // Get total number of blocks needed
    numBlks = 0;
    while ((numBlks*hNandInfo->pagesPerBlock) < num_pages) numBlks++;

    // Start in block 50
    // Leave blocks 0-49 alone, reserved for the boot image + bad blocks
    blockNum = VAR_START_BLK;

    while (blockNum < hNandInfo->numBlocks)
    {
        // Find first good block
        if (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS) return E_FAIL;

        // Start writing in page 0 of current block
        pageNum = 0;
        pageCnt = 0;

        // Setup data pointer
        dataPtr = (Uint8*)ADDR_DDR_CFG;

        // Start page read loop
        do
        {
            UTIL_waitLoop(200);

            if (NAND_readPage(hNandInfo, blockNum, pageNum, gNandRx) != E_PASS) return E_FAIL;

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
                    if (blockNum > MAX_BLK_NUM) return E_FAIL;  //exceeded the "max" addressable block
                }
                while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS);

                pageNum = 0;
            }
        } while (pageCnt < num_pages);

        break;
    }
    return E_PASS;
}


// Generic function to write a UBL or Application header and the associated data
static Uint32 LOCAL_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt)
{
	Uint32 blockNum,pageNum,pageCnt;
  	Uint32 numBlks;
  	Uint32 i;
  	Uint8  *dataPtr;

  	gNandTx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);
  	gNandRx = (Uint8 *) UTIL_allocMem(NAND_MAX_PAGE_SIZE);

  	for (i=0; i<NAND_MAX_PAGE_SIZE; i++)
  	{
    	gNandTx[i]=0xff;
    	gNandRx[i]=0xff;
  	}

  	// Get total number of blocks needed
  	numBlks = 0;
  	while ((numBlks*hNandInfo->pagesPerBlock) < totalPageCnt) numBlks++;

	// Start in block 50 : Leave blocks 0-49 alone -- reserved for the boot image + potential bad blocks
	blockNum = 50;

  	// Unprotect all blocks of the device
  	if (NAND_unProtectBlocks(hNandInfo, blockNum, (hNandInfo->numBlocks-1)) != E_PASS)
  	{
    	blockNum++;
    	return E_FAIL;
  	}

  	while (blockNum < hNandInfo->numBlocks)
  	{
    	// Find first good block
    	while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS) blockNum++;

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
      		// Write the AIS image data to the NAND device
      		if (NAND_writePage(hNandInfo, blockNum,  pageNum, dataPtr) != E_PASS)
      		{
        		NAND_reset(hNandInfo);
        		NAND_badBlockMark(hNandInfo,blockNum);
        		dataPtr -= pageNum * hNandInfo->dataBytesPerPage;
        		blockNum++;
        		continue;
      		}

      		UTIL_waitLoop(400);

      		// Verify the page just written
      		if (NAND_verifyPage(hNandInfo, blockNum, pageNum, dataPtr, gNandRx) != E_PASS) {}

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


static Uint32 USB_writeData(NAND_InfoHandle hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt)
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
  	while ( (numBlks*hNandInfo->pagesPerBlock) < totalPageCnt ) numBlks++;

  	DEBUG_printString("Number of blocks needed for data: ");
  	DEBUG_printHexInt(numBlks);
  	DEBUG_printString("\r\n");

  	// Start in block 1 (leave block 0 alone)
  	blockNum = 1;

  	// Unprotect all blocks of the device
  	while (NAND_unProtectBlocks(hNandInfo, blockNum, (hNandInfo->numBlocks-1)) != E_PASS) blockNum++;

   	// Find first good block
   	while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS) blockNum++;

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
   		DEBUG_printString((String)"Writing image data to block ");
   		DEBUG_printHexInt(blockNum);
   		DEBUG_printString((String)", page ");
   		DEBUG_printHexInt(pageNum);
   		DEBUG_printString((String)"\r\n");

   		/// write AIS image data to the NAND device
   		if (NAND_writePage(hNandInfo, blockNum,  pageNum, dataPtr) != E_PASS)
   		{
       		DEBUG_printString("Write failed. Marking block as bad...\n");
       		NAND_reset(hNandInfo);
       		NAND_badBlockMark(hNandInfo,blockNum);
       		dataPtr -=  pageNum * hNandInfo->dataBytesPerPage;
       		blockNum++;
       		continue;
   		}

   		for(i=0;i<ACCESS_DELAY;i++);

   		pageNum++;
   		pageCnt++;
   		dataPtr +=  hNandInfo->dataBytesPerPage;

		if (pageNum == hNandInfo->pagesPerBlock)
   		{
       		/// A block transition needs to take place; go to next good block
       		blockNum++;
       		while (NAND_badBlockCheck(hNandInfo,blockNum) != E_PASS) blockNum++;
        		
       		/// Erase the current block
       		NAND_eraseBlocks(hNandInfo,blockNum,1);
       		pageNum = 0;

   			for(i=0;i<ACCESS_DELAY;i++);
   		}

   	} while (pageCnt < totalPageCnt);

   	NAND_protectBlocks(hNandInfo);
	DEBUG_printString("\n\nNand boot preparation was successful.");
  	return E_PASS;
}


void upgradeFirmware(void)
{
	if (!isUsbActive()) return;
	isUpgradeFirmware = FALSE;

	NAND_InfoHandle hNandInfo;
	Uint32 numPagesAIS;
    Uint8 *aisPtr;
    Int32 i, status, bytesRead, aisAllocSize, aisFileSize = 0;
    VUint16 *addr_flash;
    ASYNC_MEM_InfoHandle dummy;
	FIL fPtr;

	int index=0;
	int loop = 0;
	UINT br = 0;
	BYTE buffer[4096*2] = 0;

    if (f_open(&fPtr, PDI_RAZOR_FIRMWARE, FA_READ) != FR_OK) return;

	if (!downloadCsv(PDI_RAZOR_PROFILE)) return;

    UTIL_setCurrMemPtr(0);

    /// reset command
    addr_flash = (VUint16 *)(FBASE + DEVICE_NAND_CLE_OFFSET);

    for (i=0;i<ACCESS_DELAY;i++);

    *addr_flash = (VUint16)0xFF;

	for (i=0;i<1000;i++)
	{
		if (DEVICE_ASYNC_MEM_IsNandReadyPin(dummy)) break;
	}	
    
    /// write getinfo command
    addr_flash = (VUint16 *)(FBASE + DEVICE_NAND_ALE_OFFSET);
    *addr_flash = (VUint16)NAND_ONFIRDIDADD;

    for (i=0;i<ACCESS_DELAY;i++);

    for (i=0;i<4;i++)
    {
        addr_flash = (VUint16 *)(FBASE);
    }

    /// Initialize NAND Flash
    hNandInfo = NAND_open((Uint32)NANDStart, DEVICE_BUSWIDTH_16BIT);
    if (hNandInfo == NULL) return E_FAIL;

    /// Read file size
    aisFileSize = f_size(&fPtr);

    /// get page size
    numPagesAIS = 0;
    while ( (numPagesAIS * hNandInfo->dataBytesPerPage)  < aisFileSize) numPagesAIS++;

    /// We want to allocate an even number of pages.
    aisAllocSize = numPagesAIS * hNandInfo->dataBytesPerPage;

    /// Setup pointer in RAM
    aisPtr = (Uint8 *) UTIL_allocMem(aisAllocSize);

    /// Clear memory
    for (i=0; i<aisAllocSize; i++) aisPtr[i]=0xFF;

    /// Go to start of file
    if (f_lseek(&fPtr,0) != FR_OK) return;

	isUpdateDisplay=TRUE;
	updateDisplay("FIRMWARE UPGRADE","   Loading...   ");

	/// disable all interrupt while accessing usb
	Swi_disable();

	/// read file	
	for (;;) {
     	if (f_read(&fPtr, buffer, sizeof(buffer), &br) != FR_OK) return;  /* Read a chunk of data from the source file */
        if (br == 0) break; /* error or eof */
		for (i=0;i<ACCESS_DELAY*100;i++);
		for (loop = 0; loop<sizeof(buffer); loop++) 
		{
      		aisPtr[index] = buffer[loop];
			index++;
   		}
		for (i=0;i<ACCESS_DELAY*100;i++);
    }

	/// close
    if (f_close (&fPtr) != FR_OK) return;

    Hwi_disable();

	/// global erase
    if (NAND_globalErase(hNandInfo) != E_PASS) return;
   	for(i=0;i<ACCESS_DELAY*20;i++);

    /// Write the file data to the NAND flash
    if (USB_writeData(hNandInfo, aisPtr, numPagesAIS) != E_PASS) return;
   	for(i=0;i<ACCESS_DELAY*100;i++);

	Hwi_enable();
    Swi_enable();

	isUpdateDisplay=TRUE;
	updateDisplay("FIRMWARE UPGRADE"," Upgrade Success");
	while (1);
}
