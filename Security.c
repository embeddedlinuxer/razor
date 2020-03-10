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
* Security.c
*-------------------------------------------------------------------------
* This code implements the locking and unlocking of various levels of write 
* protection. It uses the SHA256 hash function to store and verify passwords. 
* The Basic write-protection lock is disabled (unlocked) by default, and 
* when unlocked it allows the user to change a handful of innocuous 
* variables/registers that won't get your typical operator into trouble. 
* "Tech"-level write protection, when unlocked, allows for the modification 
* of most variables we expect the customer may want to change. 
* "Factory" unlock allows overwriting many variables that we typically do not 
* want customers to ever change, such as the Model Code of their analyzer. 
* These write protections are unlocked by writing the appropriate password to 
* REG_UNLOCK_PASS[0-2] as an ASCII-coded character string and then forcing the 
* appropriate coil ("COIL_UNLOCK_XXXXX") to logic high. The password fields 
* will then be cleared regardless of whether the password was accepted or denied. 
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#include "Globals.h"
#include <string.h>

#define SECURITY_H
#include "Security.h"


void Unlock_Via_Modbus(int prot_lvl)
{
	COIL_UNLOCKED.val = TRUE;
}


void Finish_Password_Delay(void)
{
	PASSWORD_DELAYED = FALSE;
}


/*
int Unlock_Basic(unsigned char* pass, Uint8 length)
{
	int is_match = FALSE;
	if (0 == memcmp(PASS_MD, BASIC_MD, 32))
		is_match = TRUE;

	if ((is_match) && !(PASSWORD_DELAYED))
	{
		LOCK_BASIC = FALSE;
		COIL_LOCK_STATUS_BASIC.val = FALSE;
		return 0; }
	else
	{
		LOCK_BASIC = TRUE;
		COIL_LOCK_STATUS_BASIC.val = TRUE;
		PASSWORD_DELAYED = TRUE;
		Clock_start(Password_Delay_Clock);
		return -1; // unlock request: DENIED
	}

}



int Unlock_Tech(unsigned char* pass, Uint8 length)
{
	int is_match = FALSE;
	if (0 == memcmp(PASS_MD, TECH_MD, 32))
			is_match = TRUE;

	if ((is_match) && !(PASSWORD_DELAYED))
	{
		LOCK_TECH = FALSE;
		COIL_LOCK_STATUS_TECH.val = FALSE;
		return 1;
	}
	else
	{
//		start_countdown_timer;
		LOCK_TECH = TRUE;
		COIL_LOCK_STATUS_TECH.val = TRUE;
		PASSWORD_DELAYED = TRUE;
		Clock_start(Password_Delay_Clock);
		return 0; // unlock request: DENIED
	}
}

int Unlock_Factory(unsigned char* pass, Uint8 length)
{
	int is_match = FALSE;
	if (memcmp(PASS_MD, FACTORY_MD, 32) == 0)
		is_match = TRUE;

	if ((is_match) && !(PASSWORD_DELAYED))
	{
		LOCK_FACTORY = FALSE;
		COIL_LOCK_STATUS_FACTORY.val = FALSE;
		return 0;
	}
	else
	{
//		start_countdown_timer;
		LOCK_FACTORY = TRUE;
		COIL_LOCK_STATUS_FACTORY.val = TRUE;
		PASSWORD_DELAYED = TRUE;
		Clock_start(Password_Delay_Clock);
		return -1; // unlock request: DENIED
	}
}

int Lock_Basic(void)
{
	LOCK_BASIC = TRUE;
	COIL_LOCK_STATUS_BASIC.val = TRUE;
	return 0;
}

int Lock_Tech(void)
{
	LOCK_TECH = TRUE;
	COIL_LOCK_STATUS_TECH.val = TRUE;
	return 0;
}

int Lock_Factory(void)
{
	LOCK_FACTORY = TRUE;
	COIL_LOCK_STATUS_FACTORY.val = TRUE;
	return 0;
}
*/
