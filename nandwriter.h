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
*       Jul-18-2020 : Daniel Koh : Created this file 
*------------------------------------------------------------------------*/

#ifndef _NANDWRITER_H_
#define _NANDWRITER_H_

#include "tistdtypes.h"

#define ADDR_DDR_CFG		(0xC7FF33EC) //beginning of CFG section

void writeNand(void);
void Store_Vars_in_NAND(void);
Uint32 Restore_Vars_From_NAND(void);

#endif //_NANDWRITER_H_
