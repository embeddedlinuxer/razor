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
* PDI_I2C.h
*-------------------------------------------------------------------------
* These two files contain all the source code for generic functions of I2C 
* communication, as well as many specialized functions for reading/writing 
* to particular I2C peripherals. ADCs are read and controlled via I2C, 
* and are used to acquire temperature and reflected power readings from the 
* oscillator board. I2C is also used to control the Analog Output value as 
* well as the pin expanders that operate the LCD and the infrared LEDs. 
* Note: very rarely an I2C peripheral will hang, i.e. not release the data 
* line. (That infernal ribbon cable and various connectors are likely to 
* blame.) I2C_Recover() fixes this by sending pulses on SCL until the 
* peripheral lets go of SDA.
* The I2C uses hardware interrupt 6 (I2C_Hwi) to quickly post a SWI and then exit.
*-------------------------------------------------------------------------
* HISTORY:
*       ?-?-?       : David Skew : Created
*       Jul-18-2018 : Daniel Koh : Migraged to linux platform
*------------------------------------------------------------------------*/

#ifndef PDI_I2C_H_
#define PDI_I2C_H_


/*============================================================================*/
/*                             MACRO DEFINITIONS                              */
/*============================================================================*/

/// This wasn't included in cslr_gpio.h for some reason
#define CSL_GPIO_BINTEN_EN8_MASK         (0x00000100u)
#define CSL_GPIO_BINTEN_EN8_SHIFT        (0x00000008u)
/*----EN7 Tokens----*/
#define CSL_GPIO_BINTEN_EN8_DISABLE      (0x00000000u)
#define CSL_GPIO_BINTEN_EN8_ENABLE       (0x00000001u)
///

#define I2C_INT_GENERATED_FALSE (0x00)
#define I2C_INT_GENERATED_TRUE  (0x01)

#define I2C_SLAVE_ADDR_WRITE	(0x40)
#define I2C_SLAVE_ADDR_READ		(0x41)
#define I2C_SLAVE_ADDR_XPANDR	(0x20)	// LCD expander
#define I2C_SLAVE_ADDR_MBVE		(0x21)	// MBVE expander
#define I2C_SLAVE_ADDR_DAC		(0x4C)	// DAC (AO) address
#define I2C_SLAVE_ADDR_ADC		(0x48)	// ADC address - Temperature & RP
#define I2C_SLAVE_ADDR_ADC2		(0x4A)	// ADC address - Density
#define I2C_SLAVE_ADDR_DS1340	(0x68)	// DS1340 address - RTC 
#define NO_DATA                 (0xFF)
#define I2C_CTRL_BYTE_WL		(0x10)	//DAC control byte for a write+load operation

#define I2C_BUTTON_S		    (0x1)
#define I2C_BUTTON_B		    (0x4)
#define I2C_BUTTON_V		    (0x8)
#define I2C_BUTTON_E		    (0x2)
#define I2C_BUTTON_NONE	        (0x0)

#define LCD_FUNC_SET			(0x38)
#define LCD_FUNC_SET_RS_RW		(0x00)

#define LCD_DISP_ON				(0x0E)
#define LCD_DISP_ON_RS_RW		(0x00)

#define LCD_DISP_OFF			(0x0B)
#define LCD_DISP_OFF_RS_RW		(0x00)

#define LCD_DISP_CLR			(0x01)
#define LCD_DISP_CLR_RS_RW		(0x00)

#define LCD_ENTRY_MODE			(0x06)
#define LCD_ENTRY_MODE_RS_RW	(0x00)

#define LCD_DO_NOTHING 			LCD_ENTRY_MODE

#define LCD_SET_DDRAM_ADDR		(0x80)
#define WRITE_AT_SYM			(0x40)

#define READ_BF_EPIN_OFF		(0x02)
#define READ_BF_EPIN_ON			(0x06)

#define ERROR_VAL				(0x1)

#define I2C_INIT_NUM_CHARS      (6)

#define I2C_START_SET		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STT,SET)
#define I2C_STOP_SET		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STP,SET)

#define I2C_START_CLR		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STT,CLEAR)
#define I2C_STOP_CLR		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STP,CLEAR)

#define I2C_RX_MODE			CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_TRX,RX_MODE)
#define I2C_TX_MODE			CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_TRX,TX_MODE)

#define I2C_RM_ON			CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_RM,ENABLE)
#define I2C_RM_OFF			CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_RM,DISABLE)

#define I2C_STBMODE_ON		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STB,ENABLE) 	//start byte mode enable
#define I2C_STBMODE_OFF		CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_STB,DISABLE)	//start byte mode disable

#define I2C_CNT_0BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x0) 	//Data count register = 0
#define I2C_CNT_1BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x1) 	//Data count register = 1
#define I2C_CNT_2BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x2) 	//Data count register = 2
#define I2C_CNT_3BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x3) 	//Data count register = 3
#define I2C_CNT_4BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x4) 	//Data count register = 4

#define I2C_CNT_6BYTE		i2cRegs->ICCNT = CSL_FMK(I2C_ICCNT_ICDC,0x6) 	// TESTING PURPOSES

#define I2C_MASTER_MODE	    CSL_FINST(i2cRegs->ICMDR,I2C_ICMDR_MST,MASTER_MODE)	// put I2C module in Master mode

static Uint8 ADC_BUSY_TEMP;
static Uint8 ADC_BUSY_VREF;
static Uint8 ADC_BUSY_OVERRIDE;

inline void DisableButtonInts(void);
inline void EnableButtonInts(void);

int	LCD_setaddr(int column, int line);
int LCD_printch(char c, int column, int line);

void Button_HwiFxn(void);
void GpioBank6_HwiSelect(void);
void GpioBank8_HwiSelect(void);

void I2C_HwiFxn(void);
void I2C_SwiFxn(void);

//void gpioPowerOn(void);
//void Init_PinMux(void);
void Init_I2C(void);
void Reset_I2C(void);
int  I2C_Recover(void);
void Init_LCD(void);
void Init_MBVE(void);

void I2C_SendByte(Uint8 out_byte);
void displayLcd(const char c[], int line);
void LCD_setcursor(int curs_on, int curs_blink);
void GPIO_INTpin_HwiFxn(void);

void LCD_setBlinking(int column, int line);

static inline int I2C_Wait_To_Send(void);
static inline int I2C_Wait_To_Send_ARDY(void);
static inline int I2C_Wait_To_Recieve(void);
static inline int I2C_Wait_For_Start(void);
static inline int I2C_Wait_For_Stop(void);
static inline int I2C_Wait_For_Ack(void);

//#undef PDI_I2C_H_
#endif /* PDI_I2C_H_ */
