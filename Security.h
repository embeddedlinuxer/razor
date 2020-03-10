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
* Security.h
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

#ifndef _SECURITY
#define _SECURITY

#ifdef SECURITY_H
#define _EXTERN
#else
#define _EXTERN extern
#endif


_EXTERN unsigned char PASS_MD[32];
_EXTERN int Lock_Tech(void);

#ifdef SECURITY_H
// Factory Password = 1343
const unsigned char FACTORY_MD[] = {
		0x2b,	0x0e,	0xbe,	0x24,	0x5d,	0xb5,	0x47,	0xbe,
		0x83,	0xa6,	0xfe,	0x76,	0x35,	0xf1,	0xe9,	0x20,
		0x4b,	0x23,	0x7b,	0x7e,	0x9f,	0x8e,	0xd0,	0x34,
		0x7f,	0xd9,	0x2c,	0xed,	0xe5,	0x79,	0x1c,	0x60
};

// Tech Password = 1357
unsigned char TECH_MD[] = {
		0xf3,	0xe0,	0x55,	0x91,	0x3a,	0x0b,	0x1e,	0xb0,
		0xf0,	0x73,	0x17,	0x89,	0x6f,	0x9a,	0x1b,	0xc4,
		0x66,	0xb9,	0xa5,	0x0d,	0xb8,	0x5a,	0x7f,	0x88,
		0x2f,	0x3f,	0xfd,	0xe9,	0xff,	0xb2,	0x3a,	0xca
};

// Basic Password = 1234
unsigned char BASIC_MD[] = {
		0x03,	0xac,	0x67,	0x42,	0x16,	0xf3,	0xe1,	0x5c,
		0x76,	0x1e,	0xe1,	0xa5,	0xe2,	0x55,	0xf0,	0x67,
		0x95,	0x36,	0x23,	0xc8,	0xb3,	0x88,	0xb4,	0x45,
		0x9e,	0x13,	0xf9,	0x78,	0xd7,	0xc8,	0x46,	0xf4
};
#endif

#undef _EXTERN
#undef SECURITY_H
#endif /* _SECURITY */
