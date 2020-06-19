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
* ModbusRTU.c /// MODBUS SLAVE MODE ONLY - WE DO NOT NEED MASTER MODE
*-------------------------------------------------------------------------
*** NOTE: MODBUS CURRENTLY ONLY WORKS AS A SLAVE. MASTER MODE UNNECESSARY. 
* This code comprises all Modbus routines and functions – everything but 
* the Modbus tables which are defined in ModbusTables.h, and the 
* registers/coils themselves, which are defined/declared in Globals.c and 
* Globals.h. 
* The Modbus code uses hardware interrupt 5 (UART_Hwi) to quickly post a 
* SWI and then exit.
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#include "Pinmux.h"
#include "Globals.h"
#include "ModbusTables.h"
#include <ti/csl/cslr_syscfg.h>
#include <ti/csl/src/ip/syscfg/V0/cslr_syscfg.h>

extern void delayInt(Uint32 count);

///// EXTENDED REGISTERS /////
#define SPECIAL_OFFSET 60000
#define REMAINDER      10000 // used to be 2000
#define MIN_MB_INT     201
#define MAX_MB_INT     300
#define MIN_FCT_INT    401 
#define MAX_FCT_INT    500
#define MIN_MB_LONGINT 301
#define MAX_MB_LONGINT 400

void 
delayInt(Uint32 count)
{//note: this in not an inline fxn because we actually WANT the overhead
    volatile Uint32 tempCount = 0;
    for (tempCount = 0; tempCount < count; tempCount++);
}

/***************************************************************************
 * BfrPut()
 * @param buffer	- FIFO buffer in which to Put
 * @param in_byte	- byte to put in buffer
 * @return			- 0=success, 1=buffer overwrite
 ***************************************************************************/
inline int 
BfrPut(volatile BFR* buffer, Uint8 in_byte)
{
	Uint32 key;

	key = Hwi_disableInterrupt(5);
	buffer->buff[buffer->tail] = in_byte; //write byte at tail

	//increment tail
	buffer->tail++;
	if ((buffer->tail)>=MAX_BFR_SIZE)
		buffer->tail -= MAX_BFR_SIZE;

	buffer->n++; //inc number of buffer elements
	if ((buffer->n) < MAX_BFR_SIZE)
	{
		Hwi_restoreInterrupt(5,key);
		return 0; // no buffer overwrite happened
	}
	else
	{	//buffer overwrite
		buffer->n = MAX_BFR_SIZE; //we're still at max capacity
		Hwi_restoreInterrupt(5,key);
		return ERROR_VAL;
	}
}

/***************************************************************************
 * BfrGet()
 * @param buffer	- FIFO buffer from which to Get
 * @return			- returns next byte from FIFO buffer,
 * 					  zero if buffer is empty
 ***************************************************************************/
inline Uint8 
BfrGet(volatile BFR* buffer)
{
	Uint8 out_byte;
	Uint32 key;

	key = Hwi_disableInterrupt(5);
	if (buffer->n <= 0) //if buffer empty
	{
		Hwi_restoreInterrupt(5,key);
		buffer->n = 0;
		buffer->head = buffer->tail;
		return 0;
	}

	out_byte = buffer->buff[buffer->head]; //read byte at tail

	//increment head
	buffer->head++;
	if ((buffer->head) >= MAX_BFR_SIZE)
		buffer->head -= MAX_BFR_SIZE;

	buffer->n--; //dec number of buffer elements

	Hwi_restoreInterrupt(5,key);
	return out_byte;
}

void 
Clear_Buffer(BFR* buffer)
{
	buffer->head	= 0;
	buffer->tail	= 0;
	buffer->n		= 0;
	buffer->gseed	= GSEED_DEFAULT;
}

void 
Reset_Uart_Error_Count(void)
{
	UART_ERROR_CNT.OE = 0;
	UART_ERROR_CNT.PE = 0;
	UART_ERROR_CNT.FE = 0;
	UART_ERROR_CNT.RXFIFOE = 0;
}

/*********************************************************************************
 * Init_PinMux()	- Initialize pin muxing on the SYSCFG module
 *
 ********************************************************************************/
void 
Init_PinMux(void)
{
	/////////////////////////// 
	// ENABLE PULLDOWN 
	/////////////////////////// 

	// enable pull-up/pull-down select
	CSL_FINST(sys1Regs->PUPD_ENA, SYSCFG1_PUPD_ENA_PUPDENA0, ENABLE);   

	// configure pin group 0 as pull-down (UART2_CTS)
	//CSL_FINST(sys1Regs->PUPD_ENA, SYSCFG1_PUPD_SEL_PUPDSEL0, PULLDOWN); 
	CSL_FINST(sys1Regs->PUPD_SEL, SYSCFG1_PUPD_SEL_PUPDSEL0, PULLDOWN); 

	// configure CP[18] for pull-down
	CSL_FINST(sys1Regs->PUPD_SEL, SYSCFG1_PUPD_SEL_PUPDSEL31, PULLDOWN);

	/////////////////////////// 
	// ENABLE PINMUX
	/////////////////////////// 

	// enable MUX4 for I2C0_SDA
	CSL_FINST(sysRegs->PINMUX4, SYSCFG_PINMUX4_PINMUX4_15_12,I2C0_SDA);

	// enable MUX4 for I2C0_SCL
	CSL_FINST(sysRegs->PINMUX4, SYSCFG_PINMUX4_PINMUX4_11_8,I2C0_SCL);

	// enable MUX4 for UART2_TXD
	CSL_FINST(sysRegs->PINMUX4, SYSCFG_PINMUX4_PINMUX4_23_20,UART2_TXD);

	// enable MUX4 for UART2_RXD
	CSL_FINST(sysRegs->PINMUX4, SYSCFG_PINMUX4_PINMUX4_19_16,UART2_RXD);

	// enable MUX4 for DEFAULT (TM64P0_IN12) 
	CSL_FINST(sysRegs->PINMUX4, SYSCFG_PINMUX4_PINMUX4_7_4,DEFAULT);

	// enable MUX0 for GP0[9]
	CSL_FINST(sysRegs->PINMUX0, SYSCFG_PINMUX0_PINMUX0_27_24, GPIO0_9);

	// enable MUX1 for GP0[7]	
	CSL_FINST(sysRegs->PINMUX1, SYSCFG_PINMUX1_PINMUX1_3_0, GPIO0_7);

	// enable MUX5 for DEFAULT (TM64P3_IN12)
	CSL_FINST(sysRegs->PINMUX5, SYSCFG_PINMUX5_PINMUX5_7_4, DEFAULT);

	// enable MUX19 for GP6[1]
	CSL_FINST(sysRegs->PINMUX19, SYSCFG_PINMUX19_PINMUX19_23_20, GPIO6_1);

	// enable MUX6 for GP2[5] : Relay
	CSL_FINST(sysRegs->PINMUX6, SYSCFG_PINMUX6_PINMUX6_11_8, GPIO2_5);

	// enable MUX18 for GP8[12]
	CSL_FINST(sysRegs->PINMUX18, SYSCFG_PINMUX18_PINMUX18_23_20, GPIO8_12);

	/////////////////////////// 
	// CONFIGURE I/O DIRECTION 
	/////////////////////////// 

	// configure GP0[9] as output
	CSL_FINS(gpioRegs->BANK[GP0].DIR,GPIO_DIR_DIR9,CSL_GPIO_DIR_DIR_OUT);

	// configure GP0[7] as output
	CSL_FINS(gpioRegs->BANK[GP0].DIR,GPIO_DIR_DIR7,CSL_GPIO_DIR_DIR_OUT);

	// configure GP6[1] as output
	CSL_FINS(gpioRegs->BANK[GP6].DIR,GPIO_DIR_DIR1,CSL_GPIO_DIR_DIR_OUT);

	// configure GP6[0] as input
	CSL_FINS(gpioRegs->BANK[GP6].DIR,GPIO_DIR_DIR0,CSL_GPIO_DIR_DIR_IN);

	// configure GP2[5] as output for relay
	CSL_FINS(gpioRegs->BANK[GP2].DIR,GPIO_DIR_DIR5,CSL_GPIO_DIR_DIR_OUT);
	
	// configure GP8[12] as input for LCD input
	CSL_FINS(gpioRegs->BANK[GP8].DIR,GPIO_DIR_DIR12,CSL_GPIO_DIR_DIR_IN);

	/////////////////////////// 
	// ASSIGN INITIAL VALUE
	/////////////////////////// 

	CSL_FINS(gpioRegs->BANK[GP0].OUT_DATA, GPIO_OUT_DATA_OUT9,0);
	CSL_FINS(gpioRegs->BANK[GP2].OUT_DATA, GPIO_OUT_DATA_OUT5,0);
}


void 
Config_Uart(Uint32 baudrate, Uint8 parity)
{
	Uint16 	divisor; //UART divisor value for setting baud rate
	Uint16	clock_start_val, clock_end_val; // number of clock ticks to begin/end transmission with
	Uint16	watchdog_val;
	BOOL	isBaudrate;

	//disable transmitter and receiver
	CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,RESET);
	CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,RESET);

	uartRegs->LCR = CSL_FMKT(UART_LCR_WLS,8BITS); 		//word length 8bits

	//modem control register
	uartRegs->MCR = CSL_FMKT(UART_MCR_RTS,ENABLE)  		//RTS control
                    | CSL_FMKT(UART_MCR_LOOP,DISABLE) 	//loopback mode disabled
                    | CSL_FMKT(UART_MCR_AFE,DISABLE); 	//auto flow

	//FIFO control register
	// note: FIFOEN must be set BEFORE any other bits
	uartRegs->FCR = CSL_FMKT(UART_FCR_FIFOEN,ENABLE); 	//FIFO enable
	CSL_FINST(uartRegs->FCR,UART_FCR_DMAMODE1,ENABLE);	//DMA MODE1 enable (required)
	CSL_FINST(uartRegs->FCR,UART_FCR_RXFIFTL,CHAR1);	//FIFO trigger level = 1 byte
	CSL_FINST(uartRegs->FCR,UART_FCR_TXCLR,CLR);		//clear TX FIFO
	CSL_FINST(uartRegs->FCR,UART_FCR_RXCLR,CLR);		//clear RX FIFO

	  isBaudrate = FALSE;

	  /// these numbers are ONLY for a SYS/BIOS 150us clock tick period ///
	  ///      silence interval > 3.5 character times      ///
	  switch (baudrate){ //pg.1435  // MB_Baudrate_Clock_Module.xlsx
	   	  	  case 2400:
	   	  		divisor = 3906;
	   	  		clock_start_val	= 96; //character times at this baudrate = ~4.32
				clock_end_val	= 96;
				watchdog_val 	= 48; // ~2.16 character times
	   	  		break;

	  	  	  case 4800:
	  	  		divisor = 1953;
	   	  		clock_start_val	= 48; //character times at this baudrate = ~4.32
				clock_end_val	= 48;
				watchdog_val 	= 24; // ~2.16 character times
	  	  		break;

	  	  	  case 9600:
	  	  		divisor = 977;
	  	  		clock_start_val	= 24; //character times at this baudrate = ~4.32
				clock_end_val	= 24;
				watchdog_val 	= 12; // ~2.16 character times
	  	  		break;

	  	  	  case 19200:
	  	  		divisor = 488;
	   	  		clock_start_val	= 12; //character times at this baudrate = ~4.32
				clock_end_val	= 12;
				watchdog_val 	=  6; // ~2.16 character times
	  	  		break;

	  	  	  case 38400:
	  	  		divisor = 244;
	   	  		clock_start_val	=  6; //character times at this baudrate = ~4.32
				clock_end_val	=  6;
				watchdog_val 	=  3; // ~2.16 character times
	  	  		break;

	  	  	  case 57600:
	  	  		divisor = 163;
	   	  		clock_start_val	=  4; //character times at this baudrate = ~4.32
				clock_end_val	=  4;
				watchdog_val 	=  2; // ~2.16 character times
	  	  		break;

	  	  	  case 115200:
	  	  		divisor = 81;
	   	  		clock_start_val	=  2; //character times at this baudrate = ~4.32
				clock_end_val	=  2;
				watchdog_val 	=  1; // ~2.16 character times
  	  		  	break;

	  	  	  default: // default to 9600 baud
	   	  		clock_start_val	= 14; // ~4.32 character times
				clock_end_val	= 14;
				watchdog_val 	= 12; // ~2.16 character times
	  	  		divisor = 977;
	  	  		isBaudrate = TRUE;
	  }

	  if (isBaudrate) VAR_Update(&REG_BAUD_RATE, 9600.0, 0);
	  else VAR_Update(&REG_BAUD_RATE, baudrate, 0);

	Clock_setTimeout(MB_Start_Clock_Int16,	clock_start_val);
	Clock_setTimeout(MB_Start_Clock_LongInt,clock_start_val);
	Clock_setTimeout(MB_Start_Clock_Float,	clock_start_val);
	Clock_setTimeout(MB_Start_Clock_Coil,	clock_start_val);
	Clock_setTimeout(MB_End_Clock,	        clock_end_val);

	///Note:	watchdog_val is multiplied because mdbus.exe does not send the bytes fast
	///			enough in a long write command to comply with the Modbus standard of a less than
	///			1.5 character times silent period. Multiplier (of 12) was determined experimentally.
	Clock_setTimeout(MB_Watchdog_Timeout_Clock,	watchdog_val);

	//divisor latch
	CSL_FINS(uartRegs->DLL,UART_DLL_DLL,divisor & 0xFF);	// LSB
	CSL_FINS(uartRegs->DLH,UART_DLH_DLH,divisor >> 8);	// MSB


	switch(parity){
			case UART_PARITY_EVEN:
				CSL_FINST(uartRegs->LCR,UART_LCR_PEN,ENABLE);	//parity enable
				CSL_FINST(uartRegs->LCR,UART_LCR_SP,DISABLE);	//disable stick parity
				CSL_FINST(uartRegs->LCR,UART_LCR_EPS,EVEN);		//even parity select (EVEN)
				break;

			case UART_PARITY_ODD:
				CSL_FINST(uartRegs->LCR,UART_LCR_PEN,ENABLE);	//parity enable
				CSL_FINST(uartRegs->LCR,UART_LCR_SP,DISABLE);	//disable stick parity
				CSL_FINST(uartRegs->LCR,UART_LCR_EPS,ODD);		//even parity select (ODD)
				break;

			case UART_PARITY_STICK_SET:
				CSL_FINST(uartRegs->LCR,UART_LCR_PEN,ENABLE);	//parity enable
				CSL_FINST(uartRegs->LCR,UART_LCR_SP,ENABLE);	//enable stick parity
				CSL_FINST(uartRegs->LCR,UART_LCR_EPS,ODD);		//SET stick parity
				break;

			case UART_PARITY_STICK_CLEAR:
				CSL_FINST(uartRegs->LCR,UART_LCR_PEN,ENABLE);	//parity enable
				CSL_FINST(uartRegs->LCR,UART_LCR_SP,ENABLE);	//enable stick parity
				CSL_FINST(uartRegs->LCR,UART_LCR_EPS,EVEN);		//CLEAR stick parity
				break;

			case UART_PARITY_NONE:
			default:
				CSL_FINST(uartRegs->LCR,UART_LCR_PEN,DISABLE);	//parity disable
	}

	//initialize UART buffers
	Clear_Buffer(&UART_TXBUF);
	Clear_Buffer(&UART_RXBUF);

	Reset_Uart_Error_Count();

	//enable transmitter and receiver
	CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,ENABLE);
	CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,ENABLE);

	CSL_FINST(uartRegs->IER,UART_IER_ELSI,ENABLE);	//enable line status interrupt
	CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt
	CSL_FINST(uartRegs->IER,UART_IER_ERBI,ENABLE);	//enable RX buffer interrupt
}

void 
Init_Uart(void)
{ ///note: DEVKIT uses UART2

//disable transmitter and receiver
  CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,RESET);
  CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,RESET);

  uartRegs->LCR = CSL_FMKT(UART_LCR_WLS,8BITS); 			//word length 8bits

  //modem control register
  uartRegs->MCR = CSL_FMKT(UART_MCR_RTS,ENABLE)  			//RTS control
                          | CSL_FMKT(UART_MCR_LOOP,DISABLE) //loopback mode disabled
                          | CSL_FMKT(UART_MCR_AFE,DISABLE); //auto flow disabled

  //FIFO control register
  // note: FIFOEN must be set BEFORE any other bits
  uartRegs->FCR = CSL_FMKT(UART_FCR_FIFOEN,ENABLE); 		//FIFO enable
  CSL_FINST(uartRegs->FCR,UART_FCR_DMAMODE1,ENABLE);		//DMA MODE1 enable (required)
  CSL_FINST(uartRegs->FCR,UART_FCR_RXFIFTL,CHAR1);			//FIFO trigger level = 1 byte
  CSL_FINST(uartRegs->FCR,UART_FCR_TXCLR,CLR);				//clear TX FIFO
  CSL_FINST(uartRegs->FCR,UART_FCR_RXCLR,CLR);				//clear RX FIFO

  //divisor latch = 977 --> ~9595.701 baud
  CSL_FINS(uartRegs->DLL,UART_DLL_DLL,0xD1); 				// ~9595.701 baud
  CSL_FINS(uartRegs->DLH,UART_DLH_DLH,0x03); 				// ~9595.701 baud

  //initialize UART buffers
  Clear_Buffer(&UART_TXBUF);
  Clear_Buffer(&UART_RXBUF);

  Reset_Uart_Error_Count();

  //enable transmitter and receiver
  CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,ENABLE);
  CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,ENABLE);

  CSL_FINST(uartRegs->IER,UART_IER_ELSI,ENABLE);			//enable line status interrupt
  CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);			//enable TX buffer empty interrupt
  CSL_FINST(uartRegs->IER,UART_IER_ERBI,ENABLE);			//enable RX buffer interrupt
  Hwi_enableInterrupt(5);
}

void 
Init_Modbus(void)
{
	Uint16 i;

	MB_PKT_LIST.head = 0;
	MB_PKT_LIST.tail = 0;
	MB_PKT_LIST.n = 0;

	for (i=0;i<MAX_MB_BFR;i++)
	{
		MB_PKT_LIST.BFR[i].slave 		= 0;
		MB_PKT_LIST.BFR[i].fxn 			= 0;
		MB_PKT_LIST.BFR[i].byte_cnt		= 0;
		MB_PKT_LIST.BFR[i].start_reg	= 0;
		MB_PKT_LIST.BFR[i].num_regs		= 0;
		MB_PKT_LIST.BFR[i].CRC			= 0;
		MB_PKT_LIST.BFR[i].reg_type		= 0;
	}

	WDOG_BYTES_TO_REMOVE = 0;
	MB_TX_IN_PROGRESS = FALSE;
}

void 
Init_PSC(void)
{
  // deassert UART local PSC reset and set NEXT state to ENABLE
  psc1Regs->MDCTL[CSL_PSC_UART2] = CSL_FMKT( PSC_MDCTL_NEXT, ENABLE ) | CSL_FMKT( PSC_MDCTL_LRST, DEASSERT );

  // set GPIO NEXT state to ENABLE
  psc1Regs->MDCTL[CSL_PSC_GPIO] = CSL_FMKT( PSC_MDCTL_NEXT, ENABLE );

  // move PSC to the new Next state
  psc1Regs->PTCMD = CSL_FMKT(  PSC_PTCMD_GO0, SET );

  // wait for transition
  while ( CSL_FEXT(psc1Regs->MDSTAT[CSL_PSC_UART2], PSC_MDSTAT_STATE) != CSL_PSC_MDSTAT_STATE_ENABLE );
          
  while ( CSL_FEXT(psc1Regs->MDSTAT[CSL_PSC_GPIO], PSC_MDSTAT_STATE) != CSL_PSC_MDSTAT_STATE_ENABLE );
}

unsigned int 
Calc_CRC(unsigned int* s, unsigned int n,unsigned int wrap_count)
{
	int i,j;
	unsigned int t;
	unsigned int CRC;
	Uint32 key;

	key = Hwi_disableInterrupt(5);

	CRC = 0xFFFF;

	#pragma MUST_ITERATE(2) //optimization
	for(j=0;j<n;j++)
	{
		//handle buffer wrap around -- efficient but not pretty
		if ((j >= wrap_count) && (wrap_count > 0))
			t = s[j-MAX_BFR_SIZE];
		else
			t = s[j];

		CRC ^= (t & 0xFF);

		for (i=0;i<8;i++)
		{
			if (CRC & 0x01) //if LSB is set
			{
				CRC >>= 1;	//shift right
				CRC ^= 0xA001; //XOR with 0xA001
			}
			else
			{
				CRC >>= 1;
			}
		}

	}

	Hwi_restoreInterrupt(5,key);
	return CRC;
}

/****************************************************************************
 * Uart_ISR() -- this HWI handles all UART2	interrupts						*
 *  Called by UART_Hwi. Loops until all pending interrupts are cleared.		*
 * 	Transfers bytes from HW RX FIFO to a SW RX buffer, and from	the			*
 * 	SW TX buffer to the HW TX FIFO.											*
 ****************************************************************************/
void 
Uart_ISR(void)
{
	Uint8 IIR_field 		= 0;
	Uint8 INTstatus 		= 0;
	Uint8 IPEND_bit 		= 0;
	Uint8 line_status 		= 0;
	Uint8 RX_data			= 0;
	Uint8 all_INTs_cleared 	= FALSE;
	Uint8 swi_post_needed	= FALSE;

	int i;

	while(!all_INTs_cleared) //loop until we clear all pending interrupts
	{
		//delayInt(0x1); //in place of NOPS
		IIR_field = CSL_FEXTR(uartRegs->IIR,7,0);
		INTstatus =  IIR_field & 0xE; //bits 1-3
		IPEND_bit =  IIR_field & 0x1; //bit 0 --> 1 = "no interrupts pending"

		/// note: we have to manually check this because reading IIR (as above)
		/// automatically clears (!) any THR_EMPTY interrupts that are pending
		//read LSR for empty TX EMPTY status
		if ( (CSL_FEXT(uartRegs->LSR,UART_LSR_TEMT) == 1) && (UART_TXBUF.n > 0) )
		{
			CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT9,1);
			for (i=0;i<UART_FIFO_SIZE-1;i++)
			{
				if (UART_TXBUF.n > 0) //transfer from SW TX buffer to HW TX FIFO
					CSL_FINS(uartRegs->THR,UART_THR_DATA,BfrGet(&UART_TXBUF));
				else
					break;
			}
			if (UART_TXBUF.n <= 0)
				Clock_start(MB_End_Clock);
				//MB_TX_IN_PROGRESS = FALSE;

		}

		if ( (CSL_FEXT(uartRegs->LSR,UART_LSR_TEMT) == 1) && (UART_TXBUF.n <= 0) )
		{
			CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT9,0);
		}

		switch(INTstatus)
		{
			case TX_FIFO_EMPTY_INT:
			/// TX FIFO EMPTY -- This code is rarely reached because reading IIR (as above)
			/// automatically clears (!) any THR_EMPTY interrupts that are pending
				if (UART_TXBUF.n > 0) //more bytes to send
				{
					for (i=0;i<UART_FIFO_SIZE-1;i++)
					{
						if (UART_TXBUF.n > 0) //transfer from SW TX buffer to HW TX FIFO
							CSL_FINS(uartRegs->THR,UART_THR_DATA,BfrGet(&UART_TXBUF));
						else
							break;
					}

					if (UART_TXBUF.n <= 0)
					{
						Clock_start(MB_End_Clock);
					}
				}

				break;

			case RX_DATA_RDY_INT:
			case RX_TIMEOUT_INT:
			///RX buffer has data
				line_status = CSL_FEXTR(uartRegs->LSR,7,0);

				while(line_status & 0x1) //loop until RBR is empty
				{
					RX_data = CSL_FEXT(uartRegs->RBR,UART_RBR_DATA); //get data from RX buffer register
					BfrPut(&UART_RXBUF,RX_data);

					if (Clock_isActive(MB_Watchdog_Timeout_Clock))
					{
						Clock_stop(MB_Watchdog_Timeout_Clock);
					}
					line_status = CSL_FEXTR(uartRegs->LSR,7,0);
				}
				if (UART_RXBUF.n >= 1) //need at least 8 bytes for valid frame
					swi_post_needed = TRUE;

				break;

			case LINE_STATUS_INT:
				delayInt(0x1); //in place of NOPS
				line_status = CSL_FEXTR(uartRegs->LSR,7,0);
//				Update_Uart_Error_Cnt(line_status); //add errors to error count stats

				// reset UART -- hopefully to recover from failure/infinite loop
				CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,RESET);
				CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,RESET);
				CSL_FINST(uartRegs->FCR,UART_FCR_TXCLR,CLR);			//clear TX FIFO
				CSL_FINST(uartRegs->FCR,UART_FCR_RXCLR,CLR);			//clear RX FIFO
				CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_UTRST,ENABLE);
				CSL_FINST(uartRegs->PWREMU_MGMT,UART_PWREMU_MGMT_URRST,ENABLE);
				break;

			case NOTHING_INT:
				if (IPEND_bit) //no interrupts pending
					all_INTs_cleared = TRUE;
				break;

			default:
				all_INTs_cleared = TRUE;
		}

		/// Note:	Keeping UART ISR in a while loop until the modbus tx buffer is empty was a STUPID idea.
		///			The real solution is to remove as many Hwi_disable's as possible and set a high
		///			priority for UART ISR.
//		if (UART_TXBUF.n > 0)
//			all_INTs_cleared = FALSE; //testing this... force to wait until SW TX buffer is emptied
	}

	if ( (CSL_FEXT(uartRegs->LSR,UART_LSR_TEMT) == 1) && (UART_TXBUF.n <= 0) )
	{
		CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT9,0);
	}

	if (swi_post_needed)
		Swi_post(Swi_Modbus_RX); //note: needs to be HIGH priority
}

/****************************************************************
 * Create_MB_Pkt() - 	initializes a modbus packet structure	*
 *						and adds it to the modbus packet list	*
 ****************************************************************/
MB_PKT* 
Create_MB_Pkt(MODBUS_PACKET_LIST* pkt_list, Uint8 sl, Uint8 fx, Uint16 st, Uint16 nu,
						Uint16 cr, Uint8 reg_t, Uint8 qu, Uint8 vt, Uint8 bc, Uint8 lng_addr, Uint16 reg_offset)
{
	Uint32	key;
	Uint16	special_offset;
	MB_PKT*	pkt;

	if(pkt_list->n >= MAX_MB_BFR-1)
		return (MB_PKT*)0;

	key = Hwi_disableInterrupt(5); //////////////////////////////////////////////
	pkt_list->n++;

	pkt = &pkt_list->BFR[pkt_list->tail]; // add modbus packet to buffer tail

	pkt_list->tail++;
	if (pkt_list->tail >= MAX_MB_BFR) //buffer wrap-around
		pkt_list->tail -= MAX_MB_BFR;

	Hwi_restoreInterrupt(5,key); ////////////////////////////////////////////////

	pkt->slave 			= sl;
	pkt->fxn 			= fx;
	pkt->start_reg 		= st;
	pkt->num_regs 		= nu;
	pkt->CRC 			= cr;
	pkt->reg_type 		= reg_t;
	pkt->query_rw		= qu;
	pkt->vtune			= vt;
	pkt->long_address	= lng_addr;
	pkt->is_broadcast	= bc;

	//initialize
	pkt->is_longint		= FALSE;
	pkt->byte_order		= MB_BYTE_ORDER_ABCD;
	pkt->is_special_reg = FALSE;

	special_offset = reg_offset - (reg_offset % 10000);
	if (special_offset > 0)
	{
		if (reg_offset == SPECIAL_OFFSET)
			pkt->is_special_reg = TRUE;
		reg_offset  = reg_offset % 10000;
	}

	//Determine [byte order] / [long-integer mode] from register offset
	if ( (pkt->reg_type == REG_TYPE_FLOAT) || (pkt->reg_type == REG_TYPE_LONG_INT) )
	{
		if (reg_offset >= 8000)
			pkt->is_longint	= TRUE;
		else if (reg_offset >= 6000)
			pkt->byte_order = MB_BYTE_ORDER_BADC;
		else if (reg_offset >= 4000)
			pkt->byte_order = MB_BYTE_ORDER_DCBA;
		else if (reg_offset >= 2000)
			pkt->byte_order = MB_BYTE_ORDER_CDAB;
        else if ((pkt->start_reg >= MIN_MB_LONGINT) && (pkt->start_reg < MAX_MB_LONGINT))
			pkt->is_longint	= TRUE;
	}

	///read mode
	if (pkt->query_rw == MB_READ_QRY)
	{	//using else-if instead of switch in the interest of optimization
		if (pkt->reg_type == REG_TYPE_COIL)
		{
			pkt->byte_cnt = pkt->num_regs / 8;
			if ( (pkt->num_regs % 8) > 0) // if number of coils aren't a multiple of 8
				pkt->byte_cnt++;	//need an extra byte
		}
		else if (pkt->reg_type == REG_TYPE_INTEGER)
			pkt->byte_cnt = pkt->num_regs * 2; // 2 bytes per register
		else if ( (pkt->reg_type == REG_TYPE_LONG_INT) || (pkt->reg_type == REG_TYPE_FLOAT) || (pkt->reg_type == REG_TYPE_GET_SAMPLE) )
			pkt->byte_cnt = pkt->num_regs * 4; // 4 bytes per register
	}
	///write mode
	else if (pkt->query_rw == MB_WRITE_QRY)
	{
		if (pkt->reg_type == REG_TYPE_COIL)
			pkt->byte_cnt = 2;
		else if ( (pkt->reg_type == REG_TYPE_INTEGER) || (pkt->reg_type == REG_TYPE_FLOAT) || (pkt->reg_type == REG_TYPE_LONG_INT))
			pkt->byte_cnt = pkt->num_regs * 2; // 2 bytes per register
		else if (pkt->reg_type == REG_TYPE_NO_OFFSET_FLOAT)
			pkt->byte_cnt = pkt->num_regs * 4; // 4 bytes per register
	}

	return pkt; //return a pointer to this modbus packet
}


void 
Discard_MB_Pkt_Head(MODBUS_PACKET_LIST* pkt_list)
{
	Uint32 key;

	key = Hwi_disableInterrupt(5); ////////////////////////////////////////////////
	pkt_list->BFR[pkt_list->head].slave 	= 0;
	pkt_list->BFR[pkt_list->head].fxn 		= 0;
	pkt_list->BFR[pkt_list->head].byte_cnt	= 0;
	pkt_list->BFR[pkt_list->head].start_reg	= 0;
	pkt_list->BFR[pkt_list->head].num_regs	= 0;
	pkt_list->BFR[pkt_list->head].CRC		= 0;
	pkt_list->BFR[pkt_list->head].reg_type	= 0;

	pkt_list->n--;
	pkt_list->head++;
	if (pkt_list->head >= MAX_MB_BFR) //buffer wrap-around
		pkt_list->head -= MAX_MB_BFR;
	Hwi_restoreInterrupt(5,key); /////////////////////////////////////////////////
}

void 
Discard_MB_Pkt_Tail(MODBUS_PACKET_LIST* pkt_list)
{
	Uint32 key;

	key = Hwi_disableInterrupt(5); ////////////////////////////////////////////////
	pkt_list->n--;
	pkt_list->tail--;
	if (pkt_list->tail < 0) //buffer wrap-around
		pkt_list->tail += MAX_MB_BFR;

	pkt_list->BFR[pkt_list->tail].slave 	= 0;
	pkt_list->BFR[pkt_list->tail].fxn 		= 0;
	pkt_list->BFR[pkt_list->tail].byte_cnt	= 0;
	pkt_list->BFR[pkt_list->tail].start_reg	= 0;
	pkt_list->BFR[pkt_list->tail].num_regs	= 0;
	pkt_list->BFR[pkt_list->tail].CRC		= 0;
	pkt_list->BFR[pkt_list->tail].reg_type	= 0;
	Hwi_restoreInterrupt(5,key); /////////////////////////////////////////////////
}

void 
Modbus_RX(void)
{
	Uint8	slave, fxn, using_int_offset, using_longint_offset, register_type, is_broadcast;
	Uint8	bytecnt_is_good, vtune, is_long_addr, la_offset; // <- long address: offset
	Uint16	start_reg, num_regs, reg_offset, num_data_bytes, i;
	Uint32	calc_CRC, msg_CRC, pipe_SN, la_SN; // <- long address: pipe serial number (used instead of slave number)
	Uint32	key, key2;

	// number of bytes in the message (can be for query OR response) not counting CRC bytes
	Uint32	msg_num_bytes;

	//optimization - restrict qualifier
	MB_PKT* restrict mb_pkt; 				//modbus packet pointer -- points to a packet in the modbus packet list
	unsigned int* restrict uart_pkt_ptr; 	//UART RX pointer -- points to the head of RXBUF

	//disable SWIs
	key = Hwi_disableInterrupt(5);

	if(UART_RXBUF.n < 8)
	{// not enough data in RXBUF for a complete modbus frame
		WDOG_BYTES_TO_REMOVE = UART_RXBUF.n;
		Clock_start(MB_Watchdog_Timeout_Clock); //message incomplete, start watchdog
		Hwi_restoreInterrupt(5,key);
		return;
	}
	else
		Clock_stop(MB_Watchdog_Timeout_Clock); //stop watchdog

	uart_pkt_ptr = &UART_RXBUF.buff[UART_RXBUF.head];

	if ( (uart_pkt_ptr[0] & 0xFF) == 0xFA )
	{ // long address mode
		is_long_addr = TRUE;
		la_offset = LONG_OFFSET;
	}
	else
	{ // standard address mode
		is_long_addr = FALSE;
		la_offset = 0;
	}

	slave	= uart_pkt_ptr[0+la_offset];
	fxn		= uart_pkt_ptr[1+la_offset];
	vtune	= 0;

/// NEED TO TEST THAT EVERYTHING BEHAVES ON A PARTY LINE
	if (is_long_addr)
	{
		la_SN = (Uint32) (uart_pkt_ptr[1] & 0xFF);
		la_SN = la_SN << 8;
		la_SN |= (Uint32) (uart_pkt_ptr[2] & 0xFF);
		la_SN = la_SN << 8;
		la_SN |= (Uint32) (uart_pkt_ptr[3] & 0xFF);
		la_SN = la_SN << 8;
		la_SN |= (Uint32) (uart_pkt_ptr[4] & 0xFF);

		if ( la_SN != (Uint32)REG_SN_PIPE )
		{ //wrong pipe serial number
			Clear_Buffer(&UART_RXBUF);
			Hwi_restoreInterrupt(5,key);
			return;
		}
	}
	else
	{
		if ( (slave != REG_SLAVE_ADDRESS) && (slave != 0x00) ) // ignore frames sent to other slave addresses
		{	//wrong slave address
			Clear_Buffer(&UART_RXBUF);
			Hwi_restoreInterrupt(5,key);
			return;
		}
	}

	if (slave == 0x00) //broadcast
		is_broadcast = TRUE;
	else
		is_broadcast = FALSE;

	reg_offset = 0; // default initialization

	switch (fxn)
	{	/// read functions ///
		case 0x01: //read coil
		case 0x02: //read coil (discrete input)
		case 0x03: //read holding register(s)
		case 0x04: //read input register(s) (read-only)

			if (is_broadcast)
			{	// read functions not applicable for broadcast packets
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			msg_num_bytes = 6;

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second

			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			start_reg = uart_pkt_ptr[2 + la_offset] << 8;	//MSB
			start_reg |= uart_pkt_ptr[3 + la_offset];		//LSB
			start_reg++; //convert from 0-based to 1-based

			if ( ((start_reg >= MIN_MB_INT) && (start_reg < MAX_MB_INT))
                || ((start_reg >= MIN_FCT_INT) && (start_reg < MAX_FCT_INT)) ) //integer table
			{
				using_int_offset = TRUE;
				using_longint_offset = FALSE;
				reg_offset = 0;		// register offset only applies to float table
			}
            else if ( (start_reg >= MIN_MB_LONGINT) && (start_reg < MAX_MB_LONGINT) ) //long integer table
			{
				using_longint_offset = TRUE;
				using_int_offset = FALSE;
				reg_offset = 0;		// register offset only applies to float table
			}
			else
			{ 	// floating point table
				using_int_offset = FALSE;
				using_longint_offset = FALSE;
				reg_offset 	= start_reg - (start_reg % REMAINDER); //extract register offset, if any
				start_reg 	= start_reg - reg_offset;		  //extract starting register sans any offset
			}

			num_regs = uart_pkt_ptr[4 + la_offset] << 8;	//MSB
			num_regs |= uart_pkt_ptr[5 + la_offset];		//LSB

			//determine which modbus command function is appropriate
			if ( (fxn == 0x01) || (fxn == 0x02) )
				register_type = REG_TYPE_COIL;
			else if ( (fxn == 0x03) || (fxn == 0x04) )
			{
				if (COIL_MB_AUX_SELECT_MODE.val == TRUE) //auxiliary modbus table selection mode
				{
					if (COIL_INTEGER_TABLE_SELECT.val == TRUE) //integer table selected
						register_type = REG_TYPE_INTEGER;
					else
					{	//floating-point table selected
						num_regs /= 2; // two 16-bit fields = 1 float register
						register_type = REG_TYPE_FLOAT;
					}
				}
				else //normal modbus table selection mode
				{
                    if (using_int_offset) register_type = REG_TYPE_INTEGER;
                    else if (using_longint_offset) 
                    {
						num_regs /= 2; // two 16-bit fields = 1 long int register
                        register_type = REG_TYPE_LONG_INT;
                    }
					else
					{// 40k offset indicates we use the integer table
						num_regs /= 2; // two 16-bit fields = 1 float register
						register_type = REG_TYPE_FLOAT;
					}
				}
			}
			else
				register_type = NULL;

			//create MB packet info
			Create_MB_Pkt(&MB_PKT_LIST, slave, fxn, start_reg, num_regs, 0, register_type, MB_READ_QRY, vtune, is_broadcast, is_long_addr, reg_offset);

			key2 = Hwi_disableInterrupt(5);	////////
			//remove data from RX buffer
			UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			if (UART_RXBUF.n < 0) // this shouldn't happen
				Clear_Buffer(&UART_RXBUF);
			else if (UART_RXBUF.head >= MAX_BFR_SIZE)
				UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around

			Hwi_restoreInterrupt(5,key2);		////////

			//start the clock (timer) to call MB_SendPacket after silence period
			if (register_type == REG_TYPE_COIL)
					Clock_start(MB_Start_Clock_Coil);
			else if (register_type == REG_TYPE_INTEGER)
					Clock_start(MB_Start_Clock_Int16);
            else if (register_type == REG_TYPE_LONG_INT)
					Clock_start(MB_Start_Clock_LongInt);
			else if ((register_type == REG_TYPE_FLOAT) || (register_type == REG_TYPE_NO_OFFSET_FLOAT))
					Clock_start(MB_Start_Clock_Float);
			else
			{	//something went wrong
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}
			Hwi_restoreInterrupt(5,key);
			break;

		/// write functions ///
		case 0x05: //write to coil
			msg_num_bytes = 6; // number of bytes in query (not counting CRC)

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second

			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			start_reg = uart_pkt_ptr[2 + la_offset] << 8;	//MSB
			start_reg |= uart_pkt_ptr[3 + la_offset];		//LSB
			start_reg++; //convert from 0-based to 1-based

			register_type = REG_TYPE_COIL;

			//create MB packet info
			mb_pkt = Create_MB_Pkt(&MB_PKT_LIST, slave, fxn, start_reg, 1, 0, register_type,
									 MB_WRITE_QRY, vtune, is_broadcast, is_long_addr, reg_offset);

			if ( (uart_pkt_ptr[4 + la_offset] == 0xFF) && (uart_pkt_ptr[5 + la_offset] == 0x00) ) //set coil
				mb_pkt->data[0] = TRUE;
			else if( (uart_pkt_ptr[4 + la_offset] == 0x00) && (uart_pkt_ptr[5 + la_offset] == 0x00) ) //reset coil
				mb_pkt->data[0] = FALSE;
			else // not a valid coil value
			{	///need to send an exception response
				Discard_MB_Pkt_Tail(&MB_PKT_LIST);
				Hwi_restoreInterrupt(5,key);
				return;
			}
			msg_num_bytes = 6; // number of bytes in response
            /////////////////////////////////////
			key2 = Hwi_disableInterrupt(5);	
			//remove data from RX buffer
			UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			if (UART_RXBUF.n < 0) // this shouldn't happen
				Clear_Buffer(&UART_RXBUF);
			else if (UART_RXBUF.head >= MAX_BFR_SIZE)
				UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around

			Hwi_restoreInterrupt(5,key2);		
            /////////////////////////////////////
			Clock_start(MB_Start_Clock_Coil);
			Hwi_restoreInterrupt(5,key);
			break;

		case 0x06: //write to single holding register
			msg_num_bytes = 6; // number of bytes in query (not counting CRC)

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second

			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			//starting register
			start_reg = uart_pkt_ptr[2 + la_offset] << 8;	//MSB
			start_reg |= uart_pkt_ptr[3 + la_offset];		//LSB
			start_reg++; //convert from 0-based to 1-based

			register_type = REG_TYPE_INTEGER;

			//create MB packet info
			mb_pkt = Create_MB_Pkt(&MB_PKT_LIST, slave, fxn, start_reg, 1, 0, register_type,
									MB_WRITE_QRY, vtune, is_broadcast, is_long_addr, reg_offset);
			mb_pkt->data[0] = uart_pkt_ptr[4 + la_offset];	//MSB
			mb_pkt->data[1] = uart_pkt_ptr[5 + la_offset];	//LSB (int16)


			msg_num_bytes = 6; // number of bytes in response

        	/////////////////////////////////////
			key2 = Hwi_disableInterrupt(5);
			//remove data from RX buffer
			UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			if (UART_RXBUF.n < 0) // this shouldn't happen
				Clear_Buffer(&UART_RXBUF);
			else if (UART_RXBUF.head >= MAX_BFR_SIZE)
				UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around

			Hwi_restoreInterrupt(5,key2);		
            /////////////////////////////////////
			Clock_start(MB_Start_Clock_Int16);
			Hwi_restoreInterrupt(5,key);

			break;

		case 0x10: //write to floating point OR multiple holding registers
				   //(note: 40000 offset makes this an integer value)
			num_data_bytes = uart_pkt_ptr[6];
			msg_num_bytes = 7 + num_data_bytes; // number of bytes in query (not counting CRC)

			if(UART_RXBUF.n < msg_num_bytes + 2 + la_offset) // CRC -> add 2 bytes
			{//not yet finished receiving data
				WDOG_BYTES_TO_REMOVE = UART_RXBUF.n;
				Clock_start(MB_Watchdog_Timeout_Clock); //message incomplete, start watchdog
				Hwi_restoreInterrupt(5,key);
				return;
			}
			else if (Clock_isActive(MB_Watchdog_Timeout_Clock))
				Clock_stop(MB_Watchdog_Timeout_Clock);//stop watchdog, we have the bytes we need

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC  = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second

			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			start_reg = uart_pkt_ptr[2 + la_offset] << 8;	//MSB
			start_reg |= uart_pkt_ptr[3 + la_offset];		//LSB
			start_reg++; //convert from 0-based to 1-based

			if ( (start_reg >= MIN_MB_INT) && (start_reg < MAX_MB_INT) ||
                (start_reg >= MIN_FCT_INT) && (start_reg < MAX_FCT_INT) ) //integer table
			{
				using_int_offset = TRUE;
				using_longint_offset = FALSE;
				reg_offset = 0;		// register offset only applies to float table
			}
            else if ( (start_reg >= MIN_MB_LONGINT) && (start_reg < MAX_MB_LONGINT) ) // long integer table
			{
				using_longint_offset = TRUE;
				using_int_offset = FALSE;
				reg_offset = 0;		// register offset only applies to float table
			}
			else
			{ 	// floating point table
				using_int_offset = FALSE;
				using_longint_offset = FALSE;
				reg_offset 	= start_reg - (start_reg % REMAINDER); //extract register offset, if any
				start_reg 	= start_reg - reg_offset;		  //extract starting register sans any offset
			}

			num_regs = uart_pkt_ptr[4 + la_offset] << 8;	//MSB
			num_regs |= uart_pkt_ptr[5 + la_offset];		//LSB

			bytecnt_is_good = (num_data_bytes == num_regs*2);
			//conditions for a healthy query
			// (correct number of bytes		#bytes < 2^8                #bytes > 0      )
			if ( (!bytecnt_is_good) || (num_data_bytes > 255) || (num_data_bytes == 0) )
			{//bad query
				Clear_Buffer(&UART_RXBUF);
				MB_SendException(slave, fxn, MB_EXCEP_BAD_VALUE);
				Hwi_restoreInterrupt(5,key);
				return;
			}

            if (using_int_offset) register_type = REG_TYPE_INTEGER;
            else if (using_longint_offset) 
            {
				num_regs /= 2; // long int regs are 4 bytes, not 2 bytes as with holding regs
                register_type = REG_TYPE_LONG_INT;
            }
			else
			{
				register_type = REG_TYPE_FLOAT;
				num_regs /= 2; // floating point regs are 4 bytes, not 2 bytes as with holding regs
			}

			//create MB packet info
			mb_pkt = Create_MB_Pkt	(&MB_PKT_LIST, slave, fxn, start_reg, num_regs, 0, register_type,
										 MB_READ_QRY, vtune, is_broadcast, is_long_addr, reg_offset);

			/// Copy data bytes to memory ///
			if (using_int_offset)
			{	// Always AB byte order
				for(i=0;i<=num_data_bytes;i++)
					mb_pkt->data[i] = uart_pkt_ptr[i+7 + la_offset];
			}
            else if (using_longint_offset)
			{
                for(i=0;i<=(num_data_bytes);i+=4) // for each 4-byte register
                {
                    // ABCD -> ABCD
			    	mb_pkt->data[i+0] = uart_pkt_ptr[i+7 + la_offset + 0];
					mb_pkt->data[i+1] = uart_pkt_ptr[i+7 + la_offset + 1];
					mb_pkt->data[i+2] = uart_pkt_ptr[i+7 + la_offset + 2];
					mb_pkt->data[i+3] = uart_pkt_ptr[i+7 + la_offset + 3];
                }
			}
			else // floating-point register
			{
				for(i=0;i<=(num_data_bytes);i+=4) // for each 4-byte register
				{	// write data to buffer, converting to ABCD byte order if necessary
					if (mb_pkt->byte_order == MB_BYTE_ORDER_CDAB)
					{	// CDAB -> ABCD
						mb_pkt->data[i+2] 	= uart_pkt_ptr[i+7 + la_offset + 0];
						mb_pkt->data[i+3] 	= uart_pkt_ptr[i+7 + la_offset + 1];
						mb_pkt->data[i+0] 	= uart_pkt_ptr[i+7 + la_offset + 2];
						mb_pkt->data[i+1] 	= uart_pkt_ptr[i+7 + la_offset + 3];
					}
					else if (mb_pkt->byte_order == MB_BYTE_ORDER_DCBA)
					{	// DCBA -> ABCD
						mb_pkt->data[i+3] 	= uart_pkt_ptr[i+7 + la_offset + 0];
						mb_pkt->data[i+2] 	= uart_pkt_ptr[i+7 + la_offset + 1];
						mb_pkt->data[i+1] 	= uart_pkt_ptr[i+7 + la_offset + 2];
						mb_pkt->data[i+0] 	= uart_pkt_ptr[i+7 + la_offset + 3];
					}
					else if (mb_pkt->byte_order == MB_BYTE_ORDER_BADC)
					{	// BADC -> ABCD
						mb_pkt->data[i+1] 	= uart_pkt_ptr[i+7 + la_offset + 0];
						mb_pkt->data[i+0] 	= uart_pkt_ptr[i+7 + la_offset + 1];
						mb_pkt->data[i+3] 	= uart_pkt_ptr[i+7 + la_offset + 2];
						mb_pkt->data[i+2] 	= uart_pkt_ptr[i+7 + la_offset + 3];
					}
					else //if (mb_pkt->byte_order == MB_BYTE_ORDER_ABCD)
					{	// ABCD -> ABCD
						mb_pkt->data[i+0] = uart_pkt_ptr[i+7 + la_offset + 0];
						mb_pkt->data[i+1] = uart_pkt_ptr[i+7 + la_offset + 1];
						mb_pkt->data[i+2] = uart_pkt_ptr[i+7 + la_offset + 2];
						mb_pkt->data[i+3] = uart_pkt_ptr[i+7 + la_offset + 3];
					}

				} // end for-loop
			}

			key2 = Hwi_disableInterrupt(5);	/////////////////////////////////////
			//remove data from RX buffer
			UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			if (UART_RXBUF.n < 0) // this shouldn't happen
				Clear_Buffer(&UART_RXBUF);
			else if (UART_RXBUF.head >= MAX_BFR_SIZE)
				UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around

			Hwi_restoreInterrupt(5,key2);		/////////////////////////////////////

			if (using_int_offset) Clock_start(MB_Start_Clock_Int16);
			else if(using_longint_offset) Clock_start(MB_Start_Clock_LongInt);
			else Clock_start(MB_Start_Clock_Float);
				
			Hwi_restoreInterrupt(5,key);
	
		    break;

		/// calibrate functions (non-standard) ///
		case MB_CMD_PDI_ANALYZER_SAMPLE: // mb_cmd_pdi_analyzer_sample = 66
			if (is_broadcast)
			{	// read functions not applicable for broadcast packets
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			msg_num_bytes = 7;

			//	Third byte specifies vtune, which is currently meaningless
			//	for the Razor, but must be parroted back to PC nonetheless
			vtune	= uart_pkt_ptr[2+la_offset] & 0x03; // this is the vtune the cal sw thinks it's selecting

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second

			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			register_type = REG_TYPE_GET_SAMPLE;

			//create MB packet info
			Create_MB_Pkt(&MB_PKT_LIST, slave, fxn, 0, 34, 0, register_type,
							 MB_READ_QRY, vtune, is_broadcast, is_long_addr, reg_offset);

			key2 = Hwi_disableInterrupt(5);	////////
			//remove data from RX buffer
			UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
			if (UART_RXBUF.n < 0) // this shouldn't happen
				Clear_Buffer(&UART_RXBUF);
			else if (UART_RXBUF.head >= MAX_BFR_SIZE)
				UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around

			Hwi_restoreInterrupt(5,key2);		////////

			//start the clock (timer) to call MB_SendPacket after silence period
			Clock_start(MB_Start_Clock_Sample);

			Hwi_restoreInterrupt(5,key);
			break;

		case MB_CMD_PDI_FORCE_SLAVE_PIPE: //68
			msg_num_bytes = 7;

			//query CRC
			calc_CRC = Calc_CRC(uart_pkt_ptr, msg_num_bytes + la_offset,0);
			msg_CRC = uart_pkt_ptr[msg_num_bytes + la_offset];			//LSB is first
			msg_CRC |= uart_pkt_ptr[msg_num_bytes+1 + la_offset] << 8; 	//MSB is second
			if(calc_CRC != msg_CRC)
			{//CRC mismatch
				STAT_CURRENT = 1;
				STAT_PKT++;
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			//check that the cal sw has the correct pipe SN
			pipe_SN  = uart_pkt_ptr[3 + la_offset];
			pipe_SN = pipe_SN << 8;
			pipe_SN |= uart_pkt_ptr[4 + la_offset];
			pipe_SN = pipe_SN << 8;
			pipe_SN |= uart_pkt_ptr[5 + la_offset];
			pipe_SN = pipe_SN << 8;
			pipe_SN |= uart_pkt_ptr[6 + la_offset];
			if (pipe_SN == (Uint32)REG_SN_PIPE)
			{
				//Note: some slapdash coding here. Using the start_reg parameter
				//		to carry the new slave address to MB_SendPacket_ForceSlaveAddr()
				start_reg = uart_pkt_ptr[2 + la_offset]; // new pipe SN value

				mb_pkt = Create_MB_Pkt(&MB_PKT_LIST, slave, fxn, start_reg, 0, 0, REG_TYPE_FORCE_SN,
											 MB_WRITE_QRY, 0, is_broadcast, is_long_addr, reg_offset);

				key2 = Hwi_disableInterrupt(5);	////////
				//remove data from RX buffer
				UART_RXBUF.n 	-= msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
				UART_RXBUF.head += msg_num_bytes + 2 + la_offset; //CRC is 2 bytes
				if (UART_RXBUF.n < 0) // this shouldn't happen
					Clear_Buffer(&UART_RXBUF);
				else if (UART_RXBUF.head >= MAX_BFR_SIZE)
					UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around
				Hwi_restoreInterrupt(5,key2);		////////

				//start the clock (timer) to call MB_SendPacket after silence period
				Clock_start(MB_Start_Clock_ForceSlaveAddr);
				Hwi_restoreInterrupt(5,key);
				break;
			}
			else
			{
				Clear_Buffer(&UART_RXBUF);
				Hwi_restoreInterrupt(5,key);
				return;
			}
///////////////////////////////////////////////////////////////////////

		default: //bad frame
			STAT_CURRENT = 2;
			STAT_CMD++;
			Clear_Buffer(&UART_RXBUF);
			Hwi_restoreInterrupt(5,key);
			return;
	}
}

void MB_SendPacket_Int16(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count;
	Uint16 	i, regs_written, start_reg_no_offset;
	Uint8	data_type, prot; //protection status
	Int8	rtn;
	double* mbtable_ptr_dbl = NULL;
	REGSWI* mbtable_ptr_rsw = NULL;
	int*	mbtable_ptr_int = NULL;
	VAR*	mbtable_ptr_var = NULL;
	int 	mbtable_val, old_tail, old_n;
	MB_PKT*	mb_pkt_ptr;

	// if TX is busy, come back later
	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_Int16);
		return;
	}

	if (UART_TXBUF.n > 0)
	{
		Clock_start(MB_Start_Clock_Int16);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	if ((COIL_MB_AUX_SELECT_MODE.val == FALSE) && (mb_pkt_ptr->start_reg > 40000)) //remove offset if there is one
		start_reg_no_offset = mb_pkt_ptr->start_reg - 40000;
	else
		start_reg_no_offset = mb_pkt_ptr->start_reg;

	key = Hwi_disableInterrupt(5); ////
	old_tail	= UART_TXBUF.tail;
	old_n		= UART_TXBUF.n;

	///create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		BfrPut(&UART_TXBUF,(Uint8)0xFA);
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));// MSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));// LSB
	}
    else
    {
        BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); 
    }
	//BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); // DKOH
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);

	////////////////////////////////////////////
	/// READ REGISTER
	////////////////////////////////////////////

	if ( (mb_pkt_ptr->fxn == 0x3) || (mb_pkt_ptr->fxn == 0x4) ) //read register(s)
	{
		BfrPut(&UART_TXBUF,mb_pkt_ptr->byte_cnt);

		for (i=0;i<mb_pkt_ptr->byte_cnt/2;i++)
		{
			rtn = MB_Tbl_Search_IntRegs(start_reg_no_offset+i,&mbtable_ptr_dbl,&data_type,&prot);

			if (isNoPermission(prot, MB_READ_QRY)) 	//crosscheck R/W permission of register with lock status
			{
				UART_TXBUF.n 	= old_n; 			//remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); 	//discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}

			if ((rtn == -1) || (mbtable_ptr_dbl == (double*)NULL))
			{ //register not found
				UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}

			if (data_type == REGTYPE_DBL)
				mbtable_val = (int) Round_N(*mbtable_ptr_dbl,0); 	// round to whole number and cast to int
			else if (data_type == REGTYPE_SWI)
				mbtable_val = (int) Round_N( (( (REGSWI*)mbtable_ptr_dbl)->val) ,0); 	// round to whole number and cast to int
			else if (data_type == REGTYPE_INT)
				mbtable_val = *((int*)mbtable_ptr_dbl);				// cast double* as int* then dereference
			else if (data_type == REGTYPE_VAR)
				//cast double* to VAR* and dereference; round <variable>.val to whole number then cast to int
				mbtable_val = (int) Round_N(((VAR*)mbtable_ptr_dbl)->val,0);
			else
			{
				// Something went wrong, remove everything we just added to the TX buffer
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				UART_TXBUF.n 	= old_n;
				UART_TXBUF.tail	= old_tail;
				Hwi_restoreInterrupt(5,key);
				return;
			}

			BfrPut(&UART_TXBUF,mbtable_val >> 8);	// MSB
			BfrPut(&UART_TXBUF,mbtable_val & 0xFF);	// LSB
		}

		msg_num_bytes = 3 + mb_pkt_ptr->byte_cnt; 	// slave byte + fxn byte + bytecnt byte + data bytes
	}

	/////////////////////////////////////////////////
	/// WRITE REGISTER
	/////////////////////////////////////////////////

	else if (mb_pkt_ptr->fxn == 0x6)
	{
		rtn = MB_Tbl_Search_IntRegs(start_reg_no_offset,&mbtable_ptr_dbl,&data_type,&prot);

		if ((rtn == -1) || (mbtable_ptr_dbl == (double*)NULL)) // Register not found, remove everything we just added to TX buffer
		{
			UART_TXBUF.n 	= old_n;
			UART_TXBUF.tail	= old_tail;

			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); 					//discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

        if (isNoPermission(prot,MB_WRITE_QRY)) //crosscheck R/W permission of register with lock status
    	{
			UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
			UART_TXBUF.tail	= old_tail;
    		MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		mbtable_val  = mb_pkt_ptr->data[0] << 8; 				//MSB
		mbtable_val |= mb_pkt_ptr->data[1] & 0xFF; 				//LSB

		///// WRITE TO MODBUS TABLE /////
		/*NOTE:	Although the MB master is writing 16-bit integers, our actual	//
		//		register variables are stored as a variety of data types.		//
		//		Here we	decide the appropriate way to write to those variables.	*/
		if (data_type == REGTYPE_DBL)
        {
			*mbtable_ptr_dbl = (double) mbtable_val;	//write to the double
            
            // update nand flash
            Swi_post(Swi_writeNand);
        }
		else if (data_type == REGTYPE_SWI)
		{
			mbtable_ptr_rsw->val = (double) mbtable_val;	//write to the double

			// post any REGSWI-related SWI
			if (mbtable_ptr_rsw->swi != (Swi_Handle)NULL) Swi_post(mbtable_ptr_rsw->swi);
		}
		else if (data_type == REGTYPE_INT)
		{
			mbtable_ptr_int = (int*) mbtable_ptr_dbl;	//get int* pointer to data
			*mbtable_ptr_int = mbtable_val;				//write to the integer

            // update nand flash
            Swi_post(Swi_writeNand);
		}
		else if (data_type == REGTYPE_VAR)
		{
			mbtable_ptr_var = (VAR*) mbtable_ptr_dbl;	//get VAR* pointer to data
			VAR_Update(mbtable_ptr_var, (double) mbtable_val, 0); 	//write to VAR

			// post any VAR-related SWI "AFTER" VAR_Update()
			if (mbtable_ptr_var->swi != (Swi_Handle)NULL) Swi_post(mbtable_ptr_var->swi);
            
            // update nand flash
            Swi_post(Swi_writeNand);
		}

		// send starting register -- note: need to use 0-based addressing
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) >> 8);		// MSB
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) & 0xFF);	// LSB

		//send data bytes
		BfrPut(&UART_TXBUF,mb_pkt_ptr->data[0]);	// MSB
		BfrPut(&UART_TXBUF,mb_pkt_ptr->data[1]);	// LSB

		msg_num_bytes = 6; // slave byte + fxn byte + 2 address bytes + 2 data bytes
	}

	///////////////////////////////////////////////////////
	///////////// write to multiple registers /////////////
	///////////////////////////////////////////////////////

	else if (mb_pkt_ptr->fxn == 0x10)
	{
		regs_written = 0;

		// send starting register -- note: need to use 0-based addressing
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) >> 8);		// MSB
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) & 0xFF);	// LSB

		for (i=0;i<mb_pkt_ptr->num_regs;i++)
		{
			rtn = MB_Tbl_Search_IntRegs(start_reg_no_offset+i,&mbtable_ptr_dbl,&data_type,&prot);
			if ((rtn == 0) && (mbtable_ptr_dbl != (double*)NULL))
			{
				mbtable_val  = mb_pkt_ptr->data[i*2] << 8; 		//MSB
				mbtable_val |= mb_pkt_ptr->data[i*2+1] & 0xFF; 	//LSB

				///// WRITE TO MODBUS TABLE /////
				/*NOTE:	Although the MB master is writing 16-bit integers, our actual	//
				//		register variables are stored as a variety of data types.		//
				//		Here we	decide the appropriate way to write to those variables.	*/
				if (data_type == REGTYPE_DBL)
					*mbtable_ptr_dbl = (double) mbtable_val;		//write to the double
				else if (data_type == REGTYPE_SWI)
				{
					mbtable_ptr_rsw->val = (double) mbtable_val;	//write to the double

					// post any REGSWI-related SWI
					if (mbtable_ptr_rsw->swi != (Swi_Handle)NULL)
						Swi_post(mbtable_ptr_rsw->swi);
				}
				else if (data_type == REGTYPE_INT)
				{
					mbtable_ptr_int = (int*) mbtable_ptr_dbl;	//get int* pointer to data
					*mbtable_ptr_int = mbtable_val;				//write to the integer
				}
				else if (data_type == REGTYPE_VAR)
				{
					mbtable_ptr_var = (VAR*) mbtable_ptr_dbl;	//get VAR* pointer to data
					VAR_Update(mbtable_ptr_var, (double) mbtable_val, 0); 	//write to VAR

					// post any VAR-related SWI
					if (mbtable_ptr_var->swi != (Swi_Handle)NULL)
						Swi_post(mbtable_ptr_var->swi);
				}
				regs_written++;
			}
			else
			{
				// Illegal data address, remove everything we just added to the TX buffer
				UART_TXBUF.n 	= old_n;
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}
		}

		//if we found and wrote to each register
		if (regs_written == mb_pkt_ptr->num_regs)
		{
			//send number of registers written
			BfrPut(&UART_TXBUF,regs_written >> 8); 		// MSB
			BfrPut(&UART_TXBUF,regs_written & 0xFF); 	// LSB
		}
		else
		{
			UART_TXBUF.n 	= old_n;					// Failed, remove everything we just added to the TX buffer
			UART_TXBUF.tail	= old_tail;
			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		msg_num_bytes = 6; // slave byte + fxn byte + 2 address bytes + 2 data bytes
	}
	///////////// BAD MB FXN /////////////
	else
	{
		// Something went wrong, remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;

		MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_FXN);
		Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
		Hwi_restoreInterrupt(5,key);
		return;
	}

	// calculate & send CRC
	if (UART_TXBUF.tail < old_tail) // if there was a buffer wrap around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);	// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);		// MSB
	Discard_MB_Pkt_Head(&MB_PKT_LIST); 	//discard the packet at the head of the list

	if (mb_pkt_ptr->is_broadcast == TRUE) // don't respond to broadcasts
	{
		//remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;
	}
	else
	{
		MB_TX_IN_PROGRESS = TRUE;

		if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) //if disabled
			CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt
		Hwi_restoreInterrupt(5,key); ////

		Uart_ISR(); // prime the pump
	}

	STAT_CURRENT = 0;
	STAT_SUCCESS++;
}

void 
MB_SendPacket_Coil(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count;
	Uint16 	data_byte, modulo_bits;
	Uint16  i, j, offset_cnt;
	Int8	rtn;
	Uint8	coil_val, data_type, prot; //protection status
	COIL* 	mbtable_ptr = NULL;
	int		old_tail, old_n;
	MB_PKT*	mb_pkt_ptr;

	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_Coil);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	key = Hwi_disableInterrupt(5);
	old_tail	= UART_TXBUF.tail;
	old_n		= UART_TXBUF.n;

	/// create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		BfrPut(&UART_TXBUF,(Uint8)0xFA);
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));// MSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));// LSB
	}
    else
    {
        BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS);
    }
	//BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); // DKOH
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);

    /////////////////////////////////////////////////
	/// READ COIL
	/////////////////////////////////////////////////

    if ((mb_pkt_ptr->fxn == 1) || (mb_pkt_ptr->fxn == 2)) //read coil
	{//read coil
		BfrPut(&UART_TXBUF,mb_pkt_ptr->byte_cnt);

		// we pack all coils (i.e. bits) into a minimum number of bytes
		// modulo_bits is the number of bits left over that don't completely fill a byte (bits < 8)
		modulo_bits	= mb_pkt_ptr->num_regs % 8;
		offset_cnt = 0;

		//generate data byte(s)
		for(j=1;j<=mb_pkt_ptr->byte_cnt;j++)
		{
			data_byte = 0;
			for(i=1;i<=8;i++)
			{
				coil_val = 0;
				rtn = MB_Tbl_Search_CoilRegs(mb_pkt_ptr->start_reg + offset_cnt,&mbtable_ptr,&data_type,&prot);
				offset_cnt++;

                /// coil not found && we haven't overshot the number of coils
				if ( (rtn == -1) || (mbtable_ptr == (COIL*)NULL) && (offset_cnt <= mb_pkt_ptr->num_regs) || (data_type != REGTYPE_COIL) )
				{ 
					MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_SLAVE_FAIL);
					Discard_MB_Pkt_Head(&MB_PKT_LIST); // discard the packet at the head of the list
					Hwi_restoreInterrupt(5,key);
					return;
				}

                /// checking access permission
                if (isNoPermission(prot,MB_READ_QRY))
    	        {
			        UART_TXBUF.n 	= old_n; // remove everything we just added to the TX buffer
			        UART_TXBUF.tail	= old_tail;
    		        MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
			        Discard_MB_Pkt_Head(&MB_PKT_LIST); // discard the packet at the head of the list
			        Hwi_restoreInterrupt(5,key);
			        return;
		        }

				//special case: last byte only partially filled with coils
				if ( (j==mb_pkt_ptr->byte_cnt) && (i > modulo_bits) ) break;
				else
				{ //full byte worth of coils
					coil_val = mbtable_ptr->val & 0x1; 	//get coil value
					coil_val = coil_val << (i-1);		//shift left as needed
					data_byte |= coil_val;			// OR with data_byte
				}

			}
			BfrPut(&UART_TXBUF,data_byte);	// send data_byte
		}

		msg_num_bytes = 3 + mb_pkt_ptr->byte_cnt; // address byte + fxn byte + bytecnt byte + data bytes
	}

    /////////////////////////////////////////////////
	/// WRITE COIL
	/////////////////////////////////////////////////

    else if (mb_pkt_ptr->fxn == 5)
	{
		rtn = MB_Tbl_Search_CoilRegs(mb_pkt_ptr->start_reg,&mbtable_ptr,&data_type,&prot);

		if ( (rtn == -1) || (mbtable_ptr == (COIL*)NULL) || (data_type != REGTYPE_COIL) )
		{
			/// COIL not found, remove everything we just added to the TX buffer
			UART_TXBUF.n 	= old_n;
			UART_TXBUF.tail	= old_tail;

			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

       /// checking access permission
       if (isNoPermission(prot,MB_WRITE_QRY))
       {
            UART_TXBUF.n 	= old_n; // remove everything we just added to the TX buffer
	        UART_TXBUF.tail	= old_tail;
   	        MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
	        Discard_MB_Pkt_Head(&MB_PKT_LIST); // discard the packet at the head of the list
	        Hwi_restoreInterrupt(5,key);
	        return;
        }

   		/// send the coil address -- note: need to convert back to zero-based addressing
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) >> 8);		//MSB
		BfrPut(&UART_TXBUF,(mb_pkt_ptr->start_reg-1) & 0xFF);	//LSB

		/// write to modbus table
		if (mb_pkt_ptr->data[0] == TRUE)  	// if we are setting the coil
		{
			if (mbtable_ptr->val == FALSE) // if the coil is not already set
            {
				mbtable_ptr->val = TRUE; 		// set the coil

                // update nand flash
                Swi_post(Swi_writeNand);
            }
			BfrPut(&UART_TXBUF,0xFF);
			BfrPut(&UART_TXBUF,0x00);
		}
		else if (mb_pkt_ptr->data[0] == FALSE)	//if we are resetting the coil
		{
			if (mbtable_ptr->val == TRUE)		// if the coil is not already reset
            {
				mbtable_ptr->val = FALSE;			// reset the coil

                // update nand flash
                Swi_post(Swi_writeNand);
            }

			BfrPut(&UART_TXBUF,0x00);
			BfrPut(&UART_TXBUF,0x00);
		}
		else
		{
			//remove everything we just added to the TX buffer
			UART_TXBUF.n 	= old_n;
			UART_TXBUF.tail	= old_tail;

			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_SLAVE_FAIL);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		//post the relevant SWI, if any
		if (mbtable_ptr->swi != (Swi_Handle)NULL) Swi_post(mbtable_ptr->swi);// post any COIL-related SWI

		msg_num_bytes = 6; // slave byte + fxn byte + 2 address bytes + 2 data bytes
	}
	else
	{
		// Something went wrong, remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;
		MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_FXN);
		Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
		Hwi_restoreInterrupt(5,key);
		return;
	}

	if (UART_TXBUF.tail < old_tail) // if there was a buffer wrap around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);


	BfrPut(&UART_TXBUF, CRC & 0xFF);	// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);		// MSB
	Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list

	if (mb_pkt_ptr->is_broadcast == TRUE) // don't respond to broadcasts
	{
		//remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;
	}
	else
	{
		MB_TX_IN_PROGRESS = TRUE;
		Hwi_restoreInterrupt(5,key); ////

		if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) //if disabled
			CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt

		Uart_ISR(); // prime the pump
	}

	STAT_CURRENT = 0;
	STAT_SUCCESS++;
}


void 
MB_SendPacket_LongInt(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count, mbtable_val;
	Uint16 	i, regs_written, st_reg;
	Uint8	data_type, prot; //protection status
	Int8	rtn;
	double* mbtable_ptr_dbl = NULL;
	REGSWI*	mbtable_ptr_rsw = NULL;
	double	dbl_val;
	int*	mbtable_ptr_int = NULL;
	VAR*	mbtable_ptr_var = NULL;
	float	float_val;
	int		old_tail, old_n;
	MB_PKT*	mb_pkt_ptr;

	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_LongInt);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	key = Hwi_disableInterrupt(5); ////
	old_tail	= UART_TXBUF.tail;
	old_n		= UART_TXBUF.n;

	///create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		// Do we need to worry about byte order here? ABCD for now.
		BfrPut(&UART_TXBUF,(Uint8)0xFA);
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));// MSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));// LSB
	}
    else
    {
	    BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS);
    }
	//BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); // DKOH
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);

	////////////////////////////////////////////
	//// READ REGISTER
	////////////////////////////////////////////

	if (  (mb_pkt_ptr->fxn == 0x3)	|| (mb_pkt_ptr->fxn == 0x4) )
	{ 
		BfrPut(&UART_TXBUF,mb_pkt_ptr->byte_cnt); //send byte count

		for (i=0;i<mb_pkt_ptr->byte_cnt/4;i++)
		{
			rtn = MB_Tbl_Search_LongIntRegs(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);

			if (isNoPermission(prot, MB_READ_QRY)) //crosscheck R/W permission of register with lock status
			{
				UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}

			if ((rtn == -1) || (mbtable_ptr_dbl == (double*)NULL))			// illegal data address, clean TX buffer
			{
				UART_TXBUF.n 	= old_n; 
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST);
				Hwi_restoreInterrupt(5,key);
				return;
			}

            mbtable_ptr_int = (int*) mbtable_ptr_dbl; 		// cast variable pointer to int*
			mbtable_val 	= (Uint32)(*mbtable_ptr_int);	// dereference and cast the value as Uint32

            BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));// MSB
			BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
			BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8) & 0xFF));
			BfrPut(&UART_TXBUF,(Uint8)(mbtable_val  &  0xFF));		// LSB
		}

		msg_num_bytes = 3 + mb_pkt_ptr->byte_cnt; // address byte + fxn byte + bytecnt byte + data bytes
	}


	////////////////////////////////////////////
	//// WRITE REGISTER
	////////////////////////////////////////////

	else if (mb_pkt_ptr->fxn == 0x10)
	{ 
		// write registers
		regs_written = 0;
		st_reg = mb_pkt_ptr->start_reg;

		// send starting register -- note: need to use 0-based addressing
		BfrPut(&UART_TXBUF,(st_reg-1) >> 8);	// MSB
		BfrPut(&UART_TXBUF,(st_reg-1) & 0xFF);	// LSB

		for (i=0;i<mb_pkt_ptr->byte_cnt/4;i++)
		{
			rtn = MB_Tbl_Search_LongIntRegs(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);

			if (mbtable_ptr_dbl != NULL)
			{
				if (isNoPermission(prot,MB_WRITE_QRY)) //crosscheck R/W permission of register with lock status
				{
					UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
					UART_TXBUF.tail	= old_tail;
					MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
					Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
					Hwi_restoreInterrupt(5,key);
					return;
				}

				mbtable_val  = mb_pkt_ptr->data[i*4]   << 24; 	//MSB
				mbtable_val |= mb_pkt_ptr->data[i*4+1] << 16;
				mbtable_val |= mb_pkt_ptr->data[i*4+2] << 8;
				mbtable_val |= mb_pkt_ptr->data[i*4+3]; 		//LSB

				///// WRITE TO MODBUS TABLE /////
				/*NOTE:	Although the MB master is writing 16-bit integers, our actual	//
				//		register variables are stored as a variety of data types.		//
				//		Here we	decide the appropriate way to write to those variables.	*/
				mbtable_ptr_int = (int*) mbtable_ptr_dbl;		//get int* pointer to data
				*mbtable_ptr_int = *(int*)&mbtable_val; 		//read in value as SIGNED 32-bit integer
               
                // update nand flash
                Swi_post(Swi_writeNand); 
	
				regs_written++;
			}
			else
			{
				UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}
		}

		//if we found and wrote to each register
		if (regs_written == mb_pkt_ptr->num_regs)
		{
			//send number of registers written
			BfrPut(&UART_TXBUF,regs_written*2 >> 8); 	// MSB
			BfrPut(&UART_TXBUF,regs_written*2 & 0xFF); 	// LSB
		}
		else
		{
			UART_TXBUF.n 	= old_n; 					//remove everything we just added to the TX buffer
			UART_TXBUF.tail	= old_tail;
			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); 			//discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		msg_num_bytes = 6; // slave byte + fxn byte + 2 address bytes + 2 data bytes
	}

	if (UART_TXBUF.tail < old_tail) // if there was a buffer wrap around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);						// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);							// MSB
	Discard_MB_Pkt_Head(&MB_PKT_LIST); 						// discard the packet at the head of the list

	if (mb_pkt_ptr->is_broadcast == TRUE) 					// don't respond to broadcasts
	{
		//remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;
	}
	else
	{
		MB_TX_IN_PROGRESS = TRUE;
		Hwi_restoreInterrupt(5,key);

		if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) 	//if disabled
			CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt
		Uart_ISR(); // prime the pump
	}

	STAT_CURRENT = 0;
	STAT_SUCCESS++;
}


void 
MB_SendPacket_Float(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count, mbtable_val;
	Uint16 	i, regs_written, st_reg;
	Uint8	data_type, prot; //protection status
	Int8	rtn;
	double* mbtable_ptr_dbl = NULL;
	REGSWI*	mbtable_ptr_rsw = NULL;
	double	dbl_val;
	int*	mbtable_ptr_int = NULL;
	VAR*	mbtable_ptr_var = NULL;
	float	float_val;
	int		old_tail, old_n;
	MB_PKT*	mb_pkt_ptr;

	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_Float);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	key = Hwi_disableInterrupt(5); ////
	old_tail	= UART_TXBUF.tail;
	old_n		= UART_TXBUF.n;

	///create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		// Do we need to worry about byte order here? ABCD for now.
		BfrPut(&UART_TXBUF,(Uint8)0xFA);
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));// MSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));// LSB
	}
    else
    {
	    BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS);
    }
	//BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); // DKOH
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);

	////////////////////////////////////////////
	/// READ REGISTER
	////////////////////////////////////////////

	if (  (mb_pkt_ptr->fxn == 0x3)	|| (mb_pkt_ptr->fxn == 0x4) )
	{ //read register(s)
		BfrPut(&UART_TXBUF,mb_pkt_ptr->byte_cnt); //send byte count

		for (i=0;i<mb_pkt_ptr->byte_cnt/4;i++)
		{
			if (!mb_pkt_ptr->is_special_reg)
                rtn = MB_Tbl_Search_FloatRegs(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);
			else	//extended tables
				rtn = MB_Tbl_Search_Extended(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);

			if (isNoPermission(prot, MB_READ_QRY)) //crosscheck R/W permission of register with lock status
			{
				UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}

			if ((rtn == -1) || (mbtable_ptr_dbl == (double*)NULL))			// illegal data address, clean TX buffer
			{
				UART_TXBUF.n 	= old_n; 
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST);
				Hwi_restoreInterrupt(5,key);
				return;
			}

			if (data_type == REGTYPE_VAR)
			{
				mbtable_ptr_var = (VAR*)mbtable_ptr_dbl;
				mbtable_val = *(Uint32*)(&mbtable_ptr_var->val); // treat float like a Uint32 so we can bit-shift
			}
			else if (data_type == REGTYPE_DBL)
			{
				float_val = (float)(*mbtable_ptr_dbl);	// dereference and cast the value as float
				mbtable_val = *(Uint32*)(&float_val);	// treat float like a Uint32 so we can bit-shift
			}
			else if (data_type == REGTYPE_SWI)
			{
				float_val = (float)(((REGSWI*)mbtable_ptr_dbl)->val);	// dereference as REGSWI and cast .val as float
				mbtable_val = *(Uint32*)(&float_val);	// treat float like a Uint32 so we can bit-shift
			}
			else if (data_type == REGTYPE_LONGINT)
			{
				mbtable_ptr_int = (int*) mbtable_ptr_dbl; 		// cast variable pointer to int*
				mbtable_val 	= (Uint32)(*mbtable_ptr_int);	// dereference and cast the value as Uint32
			}
			else if (data_type == REGTYPE_INT)
			{
				mbtable_ptr_int = (int*) mbtable_ptr_dbl; 	// cast variable pointer to int*
				float_val 	= (float)(*mbtable_ptr_int);	// dereference and cast the value as float
				mbtable_val = *(Uint32*)(&float_val);		// treat float like a Uint32 so we can bit-shift
			}

			// note: fortunately the C6748 uses IEEE standard floats

			/// Send data value in correct byte order ///
			if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_CDAB)
			{	// CDAB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)(mbtable_val  &  0xFF));		// LSB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));// MSB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
			}
			else if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_DCBA)
			{	// DCBA
				BfrPut(&UART_TXBUF,(Uint8)(mbtable_val  &  0xFF));		// LSB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));// MSB
			}
			else if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_BADC)
			{	// BADC
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));// MSB
				BfrPut(&UART_TXBUF,(Uint8)(mbtable_val  &  0xFF));		// LSB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8) & 0xFF));
			}
			else //if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_ABCD)
			{	// ABCD
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));// MSB
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8) & 0xFF));
				BfrPut(&UART_TXBUF,(Uint8)(mbtable_val  &  0xFF));		// LSB
			}
		}

		msg_num_bytes = 3 + mb_pkt_ptr->byte_cnt; // address byte + fxn byte + bytecnt byte + data bytes
	}

	/////////////////////////////////////////////
	/// WRITE REGISTER
	/////////////////////////////////////////////

	else if (mb_pkt_ptr->fxn == 0x10)
	{ 
		//write registers
		regs_written = 0;
		st_reg = mb_pkt_ptr->start_reg;

		//add back offset, if any
		if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_CDAB)
			st_reg += 2000;
		else if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_DCBA)
			st_reg += 4000;
		else if (mb_pkt_ptr->byte_order == MB_BYTE_ORDER_BADC)
			st_reg += 6000;

		if (mb_pkt_ptr->is_special_reg)
			st_reg += SPECIAL_OFFSET;

		// send starting register -- note: need to use 0-based addressing
		BfrPut(&UART_TXBUF,(st_reg-1) >> 8);	// MSB
		BfrPut(&UART_TXBUF,(st_reg-1) & 0xFF);	// LSB

		for (i=0;i<mb_pkt_ptr->byte_cnt/4;i++)
		{
			if (!mb_pkt_ptr->is_special_reg)
				rtn = MB_Tbl_Search_FloatRegs(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);
			else	//extended tables
				rtn = MB_Tbl_Search_Extended(mb_pkt_ptr->start_reg+(i*2),&mbtable_ptr_dbl,&data_type,&prot);

			if (mbtable_ptr_dbl != NULL)
			{
				if (isNoPermission(prot,MB_WRITE_QRY)) //crosscheck R/W permission of register with lock status
				{
					UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
					UART_TXBUF.tail	= old_tail;
					MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_VALUE);
					Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
					Hwi_restoreInterrupt(5,key);
					return;
				}

				mbtable_val  = mb_pkt_ptr->data[i*4]   << 24; 	//MSB
				mbtable_val |= mb_pkt_ptr->data[i*4+1] << 16;
				mbtable_val |= mb_pkt_ptr->data[i*4+2] << 8;
				mbtable_val |= mb_pkt_ptr->data[i*4+3]; 		//LSB

//				///// WRITE TO MODBUS TABLE /////
//

				///// WRITE TO MODBUS TABLE /////
				/*NOTE:	Although the MB master is writing 16-bit integers, our actual	//
				//		register variables are stored as a variety of data types.		//
				//		Here we	decide the appropriate way to write to those variables.	*/
				if (data_type == REGTYPE_DBL)
                {
					*mbtable_ptr_dbl = (double)*(float*)&mbtable_val;	//read in value at mbtable_val as a float, then cast to double

                    // update nand flash
                    Swi_post(Swi_writeNand); 
                }
				else if (data_type == REGTYPE_SWI)
				{
					mbtable_ptr_rsw->val = (double)*(float*)&mbtable_val;	//read in value at mbtable_val as a float, then cast to double

					// post any REGSWI-related SWI
					if (mbtable_ptr_rsw->swi != (Swi_Handle)NULL)
						Swi_post(mbtable_ptr_rsw->swi);
				}
				else if (data_type == REGTYPE_INT)
				{
					mbtable_ptr_int = (int*) mbtable_ptr_dbl;		//get int* pointer to data
					
					/// Note: 	Writing to an integer variable using the floating point register is permitted but ill-advised
					/// 		the float->int typecast effectively truncates the value being written
					*mbtable_ptr_int = (int) *(float*)&mbtable_val;	//read in value as a floating point, then cast to integer

                    // update nand flash
                    Swi_post(Swi_writeNand); 
				}
				else if (data_type == REGTYPE_LONGINT)
				{
					mbtable_ptr_int = (int*) mbtable_ptr_dbl;		//get int* pointer to data
					*mbtable_ptr_int = *(int*)&mbtable_val; 		//read in value as SIGNED 32-bit integer

                    // update nand flash
                    Swi_post(Swi_writeNand); 
				}
				else if (data_type == REGTYPE_VAR)
				{
					mbtable_ptr_var = (VAR*) mbtable_ptr_dbl;		//get VAR* pointer to data
					dbl_val = (double) *(float*)&mbtable_val;
					VAR_Update(mbtable_ptr_var, dbl_val, 0); 		//write to VAR

                    // update nand flash
                    Swi_post(Swi_writeNand); 

					if (mbtable_ptr_var->swi != (Swi_Handle)NULL)	// post any VAR-related SWI
						Swi_post(mbtable_ptr_var->swi);
				}

				regs_written++;
			}
			else
			{
				UART_TXBUF.n 	= old_n; //remove everything we just added to the TX buffer
				UART_TXBUF.tail	= old_tail;
				MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
				Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
				Hwi_restoreInterrupt(5,key);
				return;
			}
		}

		//if we found and wrote to each register
		if (regs_written == mb_pkt_ptr->num_regs)
		{
			//send number of registers written
			BfrPut(&UART_TXBUF,regs_written*2 >> 8); 	// MSB
			BfrPut(&UART_TXBUF,regs_written*2 & 0xFF); 	// LSB
		}
		else
		{
			UART_TXBUF.n 	= old_n; 					//remove everything we just added to the TX buffer
			UART_TXBUF.tail	= old_tail;
			MB_SendException(mb_pkt_ptr->slave, mb_pkt_ptr->fxn, MB_EXCEP_BAD_ADDRESS);
			Discard_MB_Pkt_Head(&MB_PKT_LIST); 			//discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		msg_num_bytes = 6; // slave byte + fxn byte + 2 address bytes + 2 data bytes
	}

	if (UART_TXBUF.tail < old_tail) // if there was a buffer wrap around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);						// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);							// MSB
	Discard_MB_Pkt_Head(&MB_PKT_LIST); 						// discard the packet at the head of the list

	if (mb_pkt_ptr->is_broadcast == TRUE) 					// don't respond to broadcasts
	{
		//remove everything we just added to the TX buffer
		UART_TXBUF.n 	= old_n;
		UART_TXBUF.tail	= old_tail;
	}
	else
	{
		MB_TX_IN_PROGRESS = TRUE;
		Hwi_restoreInterrupt(5,key);

		if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) 	//if disabled
			CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt
		Uart_ISR(); // prime the pump
	}

	STAT_CURRENT = 0;
	STAT_SUCCESS++;
}


void 
MB_SendPacket_Sample(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count, mbtable_val;
	Uint16 	i, reg;
	Uint8	vtune;
	Uint8	data_type, prot; //protection status
	Int8	rtn;
	double* mbtable_ptr_dbl = NULL;
	REGSWI*	mbtable_ptr_rsw = NULL;
	int*	mbtable_ptr_int = NULL;
	VAR*	mbtable_ptr_var = NULL;
	float	float_val;
	int		old_tail;
	MB_PKT*	mb_pkt_ptr;

	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_Sample);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	vtune = mb_pkt_ptr->vtune; // meaningless for Razor but needs to be parroted back correctly

	key = Hwi_disableInterrupt(5); ////
	old_tail = UART_TXBUF.tail;

	///create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		BfrPut(&UART_TXBUF,(Uint8)0xFA);

		// note: Cal SW wants this in DCBA order
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));	// LSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));	// MSB
	}
    else
    {
	    BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS);
    }
	//BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS); // DKOH
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);
	BfrPut(&UART_TXBUF,mb_pkt_ptr->vtune);

	////////////////////////////////////////////
	///////////// Read register(s) /////////////
	////////////////////////////////////////////

	for (i=0;i<34;i++)
	{
		if (i==0)
			reg = 19;	// REG_FREQ - frequency
		else if	(i==1)
			reg = 67;	// REG_OIL_RP - VREF
		else if	(i==2)
			reg = 67;	// Using REG_OIL_RP - "VINC"
		else if	(i==3)
		{ // just feed it whatever VTUNE the PC thinks it is using...
			float_val 	= (float)vtune;					// cast the value as float
			mbtable_val = *(Uint32*)(&float_val);		// treat float like an Uint32 so we can bit-shift

			// note: Cal SW wants this in DCBA order
			BfrPut(&UART_TXBUF,(Uint8)(mbtable_val    	   & 0xFF));	// LSB
			BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8)  & 0xFF));
			BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
			BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));	// MSB
			continue;
		}
		else if	(i==4)
			reg = 5;	// REG_TEMPERATURE - temperature (should this be REG_TEMP_USER?)
		else if	(i==5)
			reg = 5;	// REG_TEMPERATURE - temperature internal?
		else
		{	// all other sample values are don't cares to the calibration software
			BfrPut(&UART_TXBUF,0x00);// LSB
			BfrPut(&UART_TXBUF,0x00);
			BfrPut(&UART_TXBUF,0x00);
			BfrPut(&UART_TXBUF,0x00);// MSB
			continue;
		}

		rtn = MB_Tbl_Search_FloatRegs(reg,&mbtable_ptr_dbl,&data_type,&prot);
		if ((rtn == -1) || (mbtable_ptr_dbl == (double*)NULL))
		{// illegal data address
			Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
			Hwi_restoreInterrupt(5,key);
			return;
		}

		if (data_type == REGTYPE_VAR)
		{
			mbtable_ptr_var = (VAR*)mbtable_ptr_dbl;
			mbtable_val = *(Uint32*)(&mbtable_ptr_var->val); // treat float like an Uint32 so we can bit-shift
		}
		else if (data_type == REGTYPE_DBL)
		{
			float_val = (float)(*mbtable_ptr_dbl);	// dereference and cast the value as float
			mbtable_val = *(Uint32*)(&float_val);	// treat float like an Uint32 so we can bit-shift
		}
		else if (data_type == REGTYPE_SWI)
		{
			float_val = (float)(mbtable_ptr_rsw->val);	// dereference and cast the value as float
			mbtable_val = *(Uint32*)(&float_val);	// treat float like an Uint32 so we can bit-shift

		}
		else if (data_type == REGTYPE_INT)
		{
			mbtable_ptr_int = (int*) mbtable_ptr_dbl; 	// cast variable pointer to int*
			float_val 	= (float)(*mbtable_ptr_int);	// dereference and cast the value as float
			mbtable_val = *(Uint32*)(&float_val);		// treat float like an Uint32 so we can bit-shift
		}

		// note: Cal SW wants this in DCBA order
		BfrPut(&UART_TXBUF,(Uint8)( mbtable_val    	   & 0xFF));	// LSB
		BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)((mbtable_val >> 24) & 0xFF));	// MSB
	}

	msg_num_bytes = 3 + (34 * 4); // address byte + fxn byte + vtune byte + (34 registers * 4 bytes per reg)

	if (UART_TXBUF.tail < old_tail) // Calc_CRC() needs to know if there was a buffer wrap-around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);	// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);		// MSB
	MB_TX_IN_PROGRESS = TRUE;
	Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list
	Hwi_restoreInterrupt(5,key); ////

	if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) //if disabled
		CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt

	STAT_CURRENT = 0;
	STAT_SUCCESS++;

	Uart_ISR(); // prime the pump
}


void 
MB_SendPacket_ForceSlaveAddr(void)
{
	Uint32 	key,CRC, msg_num_bytes, wrap_count;
	Uint8	new_slave_addr;
	int		old_tail;
	MB_PKT*	mb_pkt_ptr;

	if (MB_TX_IN_PROGRESS == TRUE)
	{
		Clock_start(MB_Start_Clock_Sample);
		return;
	}

	mb_pkt_ptr = &MB_PKT_LIST.BFR[MB_PKT_LIST.head];

	//Note: Admittedly some slapdash coding here. Using the start_reg parameter
	//		to carry the new slave address to MB_SendPacket_ForceSlaveAddr()
	new_slave_addr = mb_pkt_ptr->start_reg;

	key = Hwi_disableInterrupt(5); ////
	old_tail	= UART_TXBUF.tail;

	///create the modbus frame
	if (mb_pkt_ptr->long_address)
	{
		BfrPut(&UART_TXBUF,(Uint8)0xFA);
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 24) & 0xFF));// MSB
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 16) & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)(((Uint32)REG_SN_PIPE >> 8)  & 0xFF));
		BfrPut(&UART_TXBUF,(Uint8)( (Uint32)REG_SN_PIPE  	   & 0xFF));// LSB
	}

	/// CHANGE SLAVE ADDRESS ///
	REG_SLAVE_ADDRESS = new_slave_addr;
	/// CHANGE SLAVE ADDRESS ///

	BfrPut(&UART_TXBUF,REG_SLAVE_ADDRESS);
	BfrPut(&UART_TXBUF,mb_pkt_ptr->fxn);
	BfrPut(&UART_TXBUF,new_slave_addr);

	BfrPut(&UART_TXBUF,(Uint8)((REG_SN_PIPE >> 24) & 0xFF));	// MSB
	BfrPut(&UART_TXBUF,(Uint8)((REG_SN_PIPE >> 16) & 0xFF));
	BfrPut(&UART_TXBUF,(Uint8)((REG_SN_PIPE >> 8)  & 0xFF));
	BfrPut(&UART_TXBUF,(Uint8)( REG_SN_PIPE  	   & 0xFF));	// LSB

	msg_num_bytes = 3 + 4; // address byte + fxn byte + new slave addr byte + SN bytes (4)

	if (UART_TXBUF.tail < old_tail) // if there was a buffer wrap around
	{
		wrap_count = MAX_BFR_SIZE - UART_TXBUF.head;
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,wrap_count);
	}
	else
		CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],msg_num_bytes,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);	// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);		// MSB
	Discard_MB_Pkt_Head(&MB_PKT_LIST); //discard the packet at the head of the list

	MB_TX_IN_PROGRESS = TRUE;

	Hwi_restoreInterrupt(5,key); ////

	if ( CSL_FEXT(uartRegs->IER,UART_IER_ETBEI) != 1) //if disabled
		CSL_FINST(uartRegs->IER,UART_IER_ETBEI,ENABLE);	//enable TX buffer empty interrupt
	
	STAT_CURRENT = 0;
	STAT_SUCCESS++;

	Uart_ISR(); // prime the pump
}

void 
MB_SendException(Uint8 slv, Uint8 fxn, Uint8 code)
{
	Uint32 key, CRC;
	Uint8  fxn_excep;

	if ((code < 0x01) || (code > 0x06)) // check bounds
	{
		return;
	}

	fxn_excep = fxn | 0x80; //set the MSB

	///////////////////////////////////////////////////////////
	key = Hwi_disableInterrupt(5); 
	Clear_Buffer(&UART_TXBUF);
	BfrPut(&UART_TXBUF,slv);
	BfrPut(&UART_TXBUF,fxn_excep);
	BfrPut(&UART_TXBUF,code);

	CRC = Calc_CRC(&UART_TXBUF.buff[UART_TXBUF.head],3,0);

	BfrPut(&UART_TXBUF, CRC & 0xFF);	// LSB
	BfrPut(&UART_TXBUF, CRC >> 8);		// MSB
	Hwi_restoreInterrupt(5,key); 
	///////////////////////////////////////////////////////////

	STAT_CURRENT = 2;
	STAT_CMD++;
	Uart_ISR(); // prime the pump
}


/****************************************************************************
 * MB_Watchdog_Timeout() -- clock module function							*
 *  Timeout on the Modbus frame -> remove the bad data from the RX buffer	*
 * 																			*
 ****************************************************************************/
void 
MB_Watchdog_Timeout(void)
{
	STAT_RETRY++;
	STAT_CURRENT = 3;
	Uint32 key;
	key = Hwi_disableInterrupt(5);	/////////////////////////////////////
	//remove data from RX buffer
	UART_RXBUF.n 	-= WDOG_BYTES_TO_REMOVE;
	UART_RXBUF.head += WDOG_BYTES_TO_REMOVE;
	if (UART_RXBUF.n < 0) // this shouldn't happen
		Clear_Buffer(&UART_RXBUF);
	else if (UART_RXBUF.head >= MAX_BFR_SIZE)
		UART_RXBUF.head -= MAX_BFR_SIZE;	//wrap around
	Hwi_restoreInterrupt(5,key);		/////////////////////////////////////
}

void 
MB_PacketDone(void)
{
	//if both THR and TSR are empty, switch back to "RX mode"
	if ( (CSL_FEXT(uartRegs->LSR,UART_LSR_TEMT) == 1) )
	{
		MB_TX_IN_PROGRESS = FALSE;
		CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT9,0);
	}
	else  //if not, keep checking until it is
		Clock_start(MB_End_Clock);
    //	CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT9,0);
}

// search the integer registers
inline Int8 
MB_Tbl_Search_IntRegs(Uint16 reg_num, double **mbtable_ptr, Uint8 *data_type, Uint8 *prot_status)
{
	Uint16 i = 0;

	//should we use a binary search?
	//linear search (for now)
	while (MB_TBL_INT[i][0] != 0) //address 0 = end of table
	{
		if ( MB_TBL_INT[i][0] == reg_num)
			break;
		else
			i++;
	}
	if (MB_TBL_INT[i][0] == 0)		// reached end of the list
	{
		*mbtable_ptr = (double*)NULL;
		data_type = (Uint8*)NULL;
		return -1; //not found
	}

	*data_type = MB_TBL_INT[i][1];
	*prot_status = (Uint8) MB_TBL_INT[i][2];
	*mbtable_ptr = (double*) MB_TBL_INT[i][3]; // typecast variable address (UINT32) to a double-type pointer

	return 0;
}

// search the long int registers
inline Int8 
MB_Tbl_Search_LongIntRegs(Uint16 reg_num, double **mbtable_ptr, Uint8 *data_type, Uint8 *prot_status)
{
	Uint16 i = 0;

	//should we use a binary search?
	//linear search (for now)
	while (MB_TBL_LONGINT[i][0] != 0) //address 0 = end of table
	{
		if ( MB_TBL_LONGINT[i][0] == reg_num)
			break;
		else
			i++;
	}

	if (MB_TBL_LONGINT[i][0] == 0)		// reached end of the list
	{
		*mbtable_ptr = (double*)NULL;
		data_type = (Uint8*)NULL;
		return -1; //not found
	}

	*data_type = MB_TBL_LONGINT[i][1];
	*prot_status = (Uint8) MB_TBL_LONGINT[i][2];
	*mbtable_ptr = (double*) MB_TBL_LONGINT[i][3]; // typecast variable address (UINT32) to a double-type pointer

	return 0;
}


// search the floating-point registers
inline Int8 
MB_Tbl_Search_FloatRegs(Uint16 reg_num, double **mbtable_ptr, Uint8 *data_type, Uint8 *prot_status)
{
	Uint16 i = 0;

	//should we use a binary search?
	//linear search (for now)
	while (MB_TBL_FLOAT[i][0] != 0) //address 0 = end of table
	{
		if ( MB_TBL_FLOAT[i][0] == reg_num) break;
		else i++;
	}
	if (MB_TBL_FLOAT[i][0] == 0)		// reached end of the list
	{
		*mbtable_ptr = (double*)NULL;
		data_type = (Uint8*)NULL;
		return -1; //not found
	}

	*data_type = MB_TBL_FLOAT[i][1];
	*prot_status = (Uint8) MB_TBL_FLOAT[i][2];
	*mbtable_ptr = (double*) MB_TBL_FLOAT[i][3]; // typecast variable address (UINT32) to a double-type pointer

	return 0;
}

// search the coil registers
inline Int8 
MB_Tbl_Search_CoilRegs(Uint16 reg_num, COIL **mbtable_ptr_coil, Uint8 *data_type, Uint8 *prot_status)
{
	Uint16 i = 0;

	//should we use a binary search?
	//linear search (for now)
	while (MB_TBL_COIL[i][0] != 0) //address 0 = end of table
	{
		if ( MB_TBL_COIL[i][0] == reg_num) break;
		else i++;
	}

	if (MB_TBL_COIL[i][0] == 0)		// reached end of the list
	{
		data_type = (Uint8*)NULL;
		*mbtable_ptr_coil = (COIL*)NULL;
		return -1; //not found
	}

	*data_type = MB_TBL_COIL[i][1];
	*prot_status = (Uint8) MB_TBL_COIL[i][2];
	*mbtable_ptr_coil = (COIL*)MB_TBL_COIL[i][3]; // typecast variable address (UINT32) to a COIL-type pointer
	return 0;
}


// search the special "extended" floating-point table
inline Int8 
MB_Tbl_Search_Extended(Uint16 reg_num, double **mbtable_ptr, Uint8 *data_type, Uint8 *prot_status)
{
	Uint16 i = 0;
	Uint8 addr_found = FALSE;
	Uint32 index;

	//should we use a binary search?
	//linear search (for now)
	while (MB_TBL_EXTENDED[i][1] != 0) //null value = end of table
	{
		//	if address is within the address range of the array
		if ( (reg_num >= MB_TBL_EXTENDED[i][0]) && (reg_num < MB_TBL_EXTENDED[i+1][0]) )
		{
			addr_found = TRUE;
			break;
		}
		else
			i++;
	}

	if (addr_found)
	{
		*data_type 		= REGTYPE_DBL; //THIS TABLE CANNOT PROPERLY HOLD A REGSWI
		*prot_status 	= REGPERM_PASSWD;

		index = (reg_num - MB_TBL_EXTENDED[i][0]) / 2; // extract array index from register address
		*mbtable_ptr = (double*) (MB_TBL_EXTENDED[i][1]) + index*sizeof(Uint8); // pointer points to the individual element in array
	}
	else
	{
		*mbtable_ptr = (double*)NULL;
		data_type = (Uint8*)NULL;
		return -1; //not found
	}

	return 0;
}

static Uint8 
isNoPermission(Uint8 prot, Uint8 isWriteCommand)
{
	if (isWriteCommand)
	{
		if (prot == REGPERM_PASSWD)
			return (COIL_UNLOCKED.val) ? 0 : 1;
        else if (prot == REGPERM_FCT)
			return (COIL_UNLOCKED_FACTORY_DEFAULT.val && COIL_UNLOCKED.val) ? 0 : 1;
		else	
			return 1;
	}
	else  // read command
		return (prot == REGPERM_WRITE_O) ? 1 : 0;
}
