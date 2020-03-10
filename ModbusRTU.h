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
* ModbusRTU.c
*-------------------------------------------------------------------------
*** NOTE: MODBUS CURRENTLY ONLY WORKS AS A SLAVE. MASTER MODE UNNECESSARY.
*-------------------------------------------------------------------------
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

#ifndef MODBUSRTU_H_
#define MODBUSRTU_H_

#define MAX_MB_BFR					(8)
#define GSEED_DEFAULT				(0xA001)
#define ERROR_VAL					(0x1)
#define TX_FIFO_EMPTY_INT			(0x2)
#define RX_TIMEOUT_INT				(0xC)
#define RX_DATA_RDY_INT				(0x4)
#define LINE_STATUS_INT				(0x6)
#define NOTHING_INT					(0x1)
#define UART_FIFO_SIZE				(16)
#define UART_PARITY_NONE			(0)
#define UART_PARITY_EVEN			(1)
#define UART_PARITY_ODD				(2)
#define UART_PARITY_STICK_SET		(3)
#define UART_PARITY_STICK_CLEAR		(4)
#define REG_TYPE_COIL				(1)
#define REG_TYPE_INTEGER			(2)
#define REG_TYPE_FLOAT				(3)	// floating point using function 0x03 or 0x04
#define REG_TYPE_NO_OFFSET_FLOAT 	(4) // floating point using function 0x45 -- NOT USED CURRENTLY
#define REG_TYPE_LONG_INT			(5)
#define REG_TYPE_GET_SAMPLE			(6)	// 'fake' type -- denotes we're getting a sample for the cal sw
#define REG_TYPE_FORCE_SN			(7) // 'fake' type -- denotes that the cal sw requests we change slave address
#define PDI_SLAVE_NUM 				(1)
#define MB_WRITE_QRY				(1)
#define MB_READ_QRY					(0)
#define MB_EXCEP_BAD_FXN			(0x01)
#define MB_EXCEP_BAD_ADDRESS		(0x02)
#define MB_EXCEP_BAD_VALUE			(0x03)
#define MB_EXCEP_SLAVE_FAIL			(0x04)
#define MB_EXCEP_ACK_WAIT			(0x05)
#define MB_EXCEP_SLAVE_BUSY			(0x06)
#define MB_CMD_PDI_ANALYZER_SAMPLE	(66)
#define MB_CMD_PDI_FORCE_SLAVE_PIPE	(68)
#define MB_BYTE_ORDER_ABCD			(0)
#define MB_BYTE_ORDER_CDAB			(1)
#define MB_BYTE_ORDER_DCBA			(2)
#define MB_BYTE_ORDER_BADC			(3)
#define LONG_OFFSET					(4)
#define NULL_PTR					((int*)0)
//#define MB_BYTE_ORDER_LONGINT		(4)

/*============================================================================*/
/*                             Type Definitions                               */
/*============================================================================*/
typedef struct
{ //modbus packet
	Uint8	slave;
	Uint8	fxn;
	Uint8	byte_cnt;
	Uint16	start_reg;
	Uint16	num_regs;
	Uint16	CRC;
	Uint8	vtune;			// N/A for Razor, but needed for calibration software compatibility
	Uint8	long_address;	// master packet used long address mode
	Uint8	is_broadcast;	// received master packet as a broadcast
	Uint8	is_longint;		// read/write using long integer format
	Uint8	is_special_reg;	// address is in "special" modbus table
	Uint8	byte_order;		// corresponds to offset used
	Uint8	reg_type; 		// 1=coil, 2=int, 3=float
	Uint8	query_rw;		// is it read mode or write mode
	Uint8	data[256]; 		// max: 128 integers/coils or 64 floats
} MB_PKT;

typedef struct
{
	int head;
	int tail;
	int n;
	MB_PKT BFR[MAX_MB_BFR];
} MODBUS_PACKET_LIST;

/*============================================================================*/
/*                           Function Declarations                            */
/*============================================================================*/

Uint8 BfrGet(volatile BFR* buffer);
int BfrPut(volatile BFR* buffer, Uint8 in_byte);
void Clear_Buffer(BFR* buffer);
void Reset_Uart_Error_Count(void);
void Init_PSC(void);
void delayInt(Uint32 count);
void Init_PinMux(void);
void Init_Uart(void);
void Init_Modbus(void);
void Config_Uart(Uint32 baudrate, Uint8 parity);
void Discard_MB_Pkt_Head(MODBUS_PACKET_LIST* pkt_list);
void Discard_MB_Pkt_Tail(MODBUS_PACKET_LIST* pkt_list);
void MB_SendException(Uint8 slv, Uint8 fxn, Uint8 code);
inline void Update_Uart_Error_Cnt(Uint8 line_status);
inline Int8 MB_Tbl_Search_IntRegs(Uint16 reg_num, double** mbtable_ptr, Uint8 *data_type, Uint8 *prot_status);
inline Int8 MB_Tbl_Search_FloatRegs(Uint16 reg_num, double** mbtable_ptr, Uint8 *data_type, Uint8 *prot_status);
inline Int8 MB_Tbl_Search_LongIntRegs(Uint16 reg_num, double** mbtable_ptr, Uint8 *data_type, Uint8 *prot_status);
inline Int8 MB_Tbl_Search_CoilRegs(Uint16 reg_num, COIL **mbtable_ptr_coil, Uint8 *data_type, Uint8 *prot_status);
inline Int8 MB_Tbl_Search_Extended(Uint16 reg_num, double **mbtable_ptr, Uint8 *data_type, Uint8 *prot_status);
unsigned int Calc_CRC(unsigned int* s, unsigned int n,unsigned int wrap_count);
//static Uint8 MB_Check_Permissions(Uint8 prot, Uint8 is_write_cmd);
static Uint8 isNoPermission(Uint8 prot, Uint8 is_write_cmd);
MB_PKT* Create_MB_Pkt(MODBUS_PACKET_LIST* pkt_list, Uint8 sl, Uint8 fx, Uint16 st, Uint16 nu,
						Uint16 cr, Uint8 reg_t, Uint8 qu, Uint8 vt, Uint8 bc, Uint8 lng_addr, Uint16 reg_offset);



#endif /* MODBUSRTU_H_ */
