#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/board/board.h>
#include <ti/board/src/lcdkOMAPL138/include/board_internal.h>
#include <ti/drv/usb/example/common/hardware.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/usb/usb_drv.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_usb.h>
#include <ti/csl/cslr_syscfg.h>
#include <ti/fs/fatfs/diskio.h>
#include <ti/fs/fatfs/FATFS.h>
#include "timer.h"
#include "types.h"
#include "fatfs_port_usbmsc.h"
#include "fs_shell_app_utils.h"
#include "usb_osal.h"
#include "usblib.h"
#include "usbhost.h"
#include "hw_soc.h"
#include "usbhmsc.h"
#include "Globals.h"
#include "Variable.h"
#include "Menu.h"
#include "nandwriter.h"

#define NANDWIDTH_16
#define OMAPL138_LCDK
#define USB_INSTANCE    0
#define MAX_DATA_SIZE   160 
#define MAX_HEADER_SIZE 110 
#define MAX_BUF_SIZE   	4096
#define MAX_CSV_SIZE   	4096*3

static char logFile[] = "0:PDI/LOG_01_01_2019.csv";

static Uint8 try = 0;
static Uint8 try2 = 0;
static Uint8 try3 = 0;
static Uint8 try4 = 0;

static char CSV_BUF[MAX_CSV_SIZE];
static char LOG_HEADER[MAX_HEADER_SIZE];
static char LOG_BUF[MAX_BUF_SIZE];
static Uint8 current_day = 99;

unsigned int g_ulMSCInstance = 0;
static USB_Handle usb_handle;
static USB_Params usb_host_params;
static FIL fileWriteObject;

// TIME VARS
static int USB_RTC_SEC = 0;
static int USB_RTC_MIN = 0;
static int USB_RTC_HR = 0;
static int USB_RTC_DAY = 0;
static int USB_RTC_MON = 0;
static int USB_RTC_YR = 0;
static int tmp_sec, tmp_min, tmp_hr, tmp_day, tmp_mon, tmp_yr;

/* ========================================================================== */
/*                                Prototypes                                  */
/* ========================================================================== */

void usbHostIntrConfig(USB_Params* usbParams);
void MSCCallback(uint32_t ulInstance, uint32_t ulEvent, void *pvData);
void usbCoreIntrHandler(uint32_t* pUsbParam);

/*****************************************************************************
*
* Hold the current state for the application.
*
****************************************************************************/
typedef enum
{
    // No device is present.
    STATE_NO_DEVICE,

    // Mass storage device is being enumerated.
    STATE_DEVICE_ENUM, 

    // Mass storage device is ready.
    STATE_DEVICE_READY,

    // An unsupported device has been attached.
    STATE_UNKNOWN_DEVICE,

    // A power fault has occurred.
    STATE_POWER_FAULT

} tState;

volatile tState g_eState;


/*****************************************************************************
*
* FAT fs variables.
*
*****************************************************************************/
/* USBMSC function table for USB implementation */

FATFS_DrvFxnTable FATFS_drvFxnTable = {
    FATFSPortUSBDiskClose,      /* closeDrvFxn */
    FATFSPortUSBDiskIoctl,      /* controlDrvFxn */
    FATFSPortUSBDiskInitialize, /* initDrvFxn */
    FATFSPortUSBDiskOpen,       /* openDrvFxn */
    FATFSPortUSBDiskWrite,      /* writeDrvFxn */
    FATFSPortUSBDiskRead        /* readDrvFxn */
};

/* FATFS configuration structure */
FATFS_HwAttrs FATFS_initCfg[_VOLUMES] =
{
    {0U}, {1U}, {2U}, {3U}
};

/* FATFS objects */
FATFS_Object FATFS_objects[_VOLUMES];

/* FATFS configuration structure */
const FATFS_Config FATFS_config[_VOLUMES + 1] = {
    {
        &FATFS_drvFxnTable,
        &FATFS_objects[0],
        &FATFS_initCfg[0]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[1],
         &FATFS_initCfg[1]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[2],
         &FATFS_initCfg[2]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[3],
         &FATFS_initCfg[3]
    },
    {NULL, NULL, NULL}
};

FATFS_Handle fatfsHandle = NULL;
uint32_t     g_fsHasOpened = 0;


void usbHostIntrConfig(USB_Params* usbParams)
{
    HwiP_Handle hwiHandle = NULL;
    OsalRegisterIntrParams_t interruptRegParams;

    /* Initialize with defaults */
    Osal_RegisterInterrupt_initParams(&interruptRegParams);

    /* Populate the interrupt parameters */
    interruptRegParams.corepacConfig.name=NULL;
    interruptRegParams.corepacConfig.corepacEventNum=SYS_INT_USB0; /* Event going in to CPU */
    interruptRegParams.corepacConfig.intVecNum= OSAL_REGINT_INTVEC_EVENT_COMBINER; /* Host Interrupt vector */
    interruptRegParams.corepacConfig.isrRoutine = (void (*)(uintptr_t))usbCoreIntrHandler;
    interruptRegParams.corepacConfig.arg = (uintptr_t)usbParams;

    Osal_RegisterInterrupt(&interruptRegParams,&hwiHandle);
    USB_irqConfig(usbParams->usbHandle, usbParams);
}

/*****************************************************************************
*
* This is the callback from the MSC driver.
*
* \param ulInstance is the driver instance which is needed when communicating
* with the driver.
* \param ulEvent is one of the events defined by the driver.
* \param pvData is a pointer to data passed into the initial call to register
* the callback.
*
* This function handles callback events from the MSC driver.  The only events
* currently handled are the MSC_EVENT_OPEN and MSC_EVENT_CLOSE.  This allows
* the main routine to know when an MSC device has been detected and
* enumerated and when an MSC device has been removed from the system.
*
* \return Returns \e true on success or \e false on failure.
*
*****************************************************************************/
void
MSCCallback(uint32_t ulInstance, uint32_t ulEvent, void *pvData)
{
    /*
    * Determine the event.
    */
    switch(ulEvent)
    {
        // Called when the device driver has successfully enumerated an MSC
        case MSC_EVENT_OPEN:
        {
            // Proceed to the enumeration state.
            g_eState = STATE_DEVICE_ENUM;

            break;
        }

        // Called when the device driver has been unloaded due to error or
        // the device is no longer present.
        case MSC_EVENT_CLOSE:
        {
            // Go back to the "no device" state and wait for a new connection.
            g_eState = STATE_NO_DEVICE;
            g_fsHasOpened = 0;
			resetUsbStaticVars();

            break;
        }

        default:
        {
            break;
        }
    }
}

/* main entry point for USB host core interrupt handler with USB Wrapper setup
* Matching interrupt call-back function API */
void usbCoreIntrHandler(uint32_t* pUsbParam)
{
    USB_coreIrqHandler(((USB_Params*)pUsbParam)->usbHandle, (USB_Params*)pUsbParam);
}

void resetUsbDriver(void)
{
   unloadUsbDriver();
   loadUsbDriver();
}


void 
unloadUsbDriver(void)
{
    USBHCDReset(USB_INSTANCE);
    USBHMSCDriveClose(g_ulMSCInstance);
    usb_handle->isOpened = 0;
    g_fsHasOpened = 0;
    if (g_fsHasOpened) FATFS_close(fatfsHandle);

    usb_osalDelayMs(500);
}


void loadUsbDriver(void)
{
    usb_host_params.usbMode      = USB_HOST_MSC_MODE;
    usb_host_params.instanceNo   = USB_INSTANCE;
    usb_handle = USB_open(usb_host_params.instanceNo, &usb_host_params);

    // failed to open
    if (usb_handle == 0) 
    {
        return;
    }

    // Setup the INT Controller
	usbHostIntrConfig (&usb_host_params);

    // Initialize the file system.
    FATFS_init();

    // Open an instance of the mass storage class driver.
    g_ulMSCInstance = USBHMSCDriveOpen(usb_host_params.instanceNo, 0, MSCCallback);

    usb_osalDelayMs(500);
}

void resetCsvStaticVars(void)
{
	isUpdateDisplay = FALSE;
	isWriteRTC = FALSE;
	isUpgradeFirmware = FALSE;
	isDownloadCsv = FALSE;
	isScanCsvFiles = FALSE;
	isUploadCsv = FALSE;
	isResetPower = FALSE;
	isCsvUploadSuccess = FALSE;
	isCsvDownloadSuccess = FALSE;
	isScanSuccess = FALSE;
	isPdiUpgradeMode = FALSE;
	isLogData = FALSE;

	/// disable usb access flags
	CSV_FILES[0] = '\0';
	CSV_BUF[0] = '\0'; 
	csvCounter = 0;
}

void resetUsbStaticVars(void)
{
	current_day = 99;
	try = 0;
	try2 = 0;
	try3 = 0;
	try4 = 0;
	usbStatus = 0;

	LOG_HEADER[0] = '\0';
	LOG_BUF[0] = '\0';
}

void stopAccessingUsb(FRESULT fr)
{
	resetCsvStaticVars();
	resetUsbStaticVars();

    /// find out why it failed
         if (fr == FR_DISK_ERR) usbStatus = 2;
    else if (fr == FR_INT_ERR) usbStatus = 3;
    else if (fr == FR_NOT_READY) usbStatus = 4;
    else if (fr == FR_NO_FILE) usbStatus = 5;
    else if (fr == FR_NO_PATH) usbStatus = 6;
    else if (fr == FR_INVALID_NAME) usbStatus = 7;
    else if (fr == FR_DENIED) usbStatus = 8;
    else if (fr == FR_INVALID_OBJECT) usbStatus = 9;
    else if (fr == FR_WRITE_PROTECTED) usbStatus = 10;
    else if (fr == FR_INVALID_DRIVE) usbStatus = 11;
    else if (fr == FR_NOT_ENABLED) usbStatus = 12;
    else if (fr == FR_NO_FILESYSTEM) usbStatus = 13;
    else if (fr == FR_TIMEOUT) usbStatus = 14;
    else if (fr == FR_LOCKED) usbStatus = 15;
    else if (fr == FR_NOT_ENOUGH_CORE) usbStatus = 16;
    else usbStatus = 2;

    return;
}


BOOL isUsbActive(void)
{
    if (USBHCDMain(USB_INSTANCE, g_ulMSCInstance) != 0) 
    {
		/// no need to check unreachable triggers
		try = 0; 
		try2 = 0;
		try3 = 0;

		if (try4 > REG_USB_TRY) stopAccessingUsb(FR_INVALID_DRIVE);
		else try4++;

		return FALSE;
    }
	else
	{
		try4 = 0; /// no need to check unreachable trigger 

    	if (g_eState == STATE_DEVICE_ENUM)
    	{
			try3 = 0; /// no need to check unreachable trigger 
   
        	if (USBHMSCDriveReady(g_ulMSCInstance) != 0) 
        	{
				try2 = 0; /// no need to check unreachable trigger;

				if (try > REG_USB_TRY) stopAccessingUsb(FR_INVALID_DRIVE);
				else try++;
            	
				return FALSE;
        	}
			else
			{
				try = 0; /// no need to check unreachable trigger

				if (!g_fsHasOpened)
        		{
            		if (FATFS_open(0U, NULL, &fatfsHandle) == FR_OK) 
            		{
						try2 = 0; /// no need to check unreachable trigger
                		g_fsHasOpened = 1;
            		}
            		else 
            		{
                		if (try2 > REG_USB_TRY) stopAccessingUsb(FR_INVALID_DRIVE);
                		else try2++;
                    	
						return FALSE;
            		}
        		}
			}

			return TRUE;
    	}
    	else 
    	{
			/// no need to check unreachable triggers
			try = 0;
			try2 = 0;

        	if (try3 > REG_USB_TRY) stopAccessingUsb(FR_INVALID_DRIVE);
        	else try3++;

        	return FALSE;
    	}
	}		
}


void logData(void)
{
	if (!isUsbActive()) return;

    FRESULT fresult;
	int data_delay;

   	/// read rtc
   	Read_RTC(&tmp_sec, &tmp_min, &tmp_hr, &tmp_day, &tmp_mon, &tmp_yr);

	/// VALIDATE PERIOD
   	if ((tmp_sec % REG_LOGGING_PERIOD != 0) || (tmp_sec == USB_RTC_SEC)) return;

	/// UPDATE TIME	
	if (USB_RTC_SEC != tmp_sec) USB_RTC_SEC = tmp_sec;
	if (USB_RTC_MIN != tmp_min) USB_RTC_MIN = tmp_min;
	if (USB_RTC_HR != tmp_hr)   USB_RTC_HR = tmp_hr;
	if (USB_RTC_DAY != tmp_day) USB_RTC_DAY = tmp_day;
	if (USB_RTC_MON != tmp_mon) USB_RTC_MON = tmp_mon;
	if (USB_RTC_YR != tmp_yr)   USB_RTC_YR = tmp_yr;

   	/// A NEW FILE? 
   	if (current_day != USB_RTC_DAY) 
   	{   
		FILINFO fno;
       	current_day = USB_RTC_DAY;

       	LOG_BUF[0] = '\0';

       	// mkdir PDI
       	fresult = f_mkdir("0:PDI");
       	if ((fresult != FR_EXIST) && (fresult != FR_OK)) 
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

        // get a file name
        logFile[0] = '\0';
        sprintf(logFile,"0:PDI/LOG_%02d_%02d_20%02d.csv",USB_RTC_MON, USB_RTC_DAY, USB_RTC_YR); 

        if (f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING) == FR_OK) 
        {
            fresult = f_close(&fileWriteObject);
            if (fresult == FR_OK) return;
        }

		/// open file
       	fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_CREATE_ALWAYS);
       	if (fresult != FR_OK) 
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header1
		LOG_HEADER[0] = '\0';
		sprintf(LOG_HEADER,"\nFirmware:,%5s\nSerial Number:,%5d\n\nDate,Time,Alarm,Stream,Watercut,Watercut_Raw,", FIRMWARE_VERSION, REG_SN_PIPE);

       	if (f_puts(LOG_HEADER,&fileWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&fileWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header2
		LOG_HEADER[0] = '\0';
       	sprintf(LOG_HEADER,"Temp(C),Avg_Temp(C),Temp_Adj,Freq(Mhz),Oil_Index,RP(V),Oil_PT,Oil_P0,Oil_P1,");

       	if (f_puts(LOG_HEADER,&fileWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&fileWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header3
		LOG_HEADER[0] = '\0';
       	sprintf(LOG_HEADER,"Density,Oil_Freq_Low,Oil_Freq_Hi,AO_LRV,AO_URV,AO_MANUAL_VAL,Relay_Setpoint\n");

       	if (f_puts(LOG_HEADER,&fileWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&fileWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// close file
       	fresult = f_close(&fileWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// flush LOG_BUF 
       	LOG_BUF[0] = '\0';

       	return;
   	}   

	printf("%s\n","new DATA_BUF");

	/// new DATA_BUF
   	char DATA_BUF[1024];
	DATA_BUF[0] = '\0';

	/// get modbus data
   	sprintf(DATA_BUF,"%02d-%02d-20%02d,%02d:%02d:%02d,%10d,%2.0f,%6.2f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%6.3f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%5.1f,%5.1f,%5.2f,%8.1f,\n",USB_RTC_MON,USB_RTC_DAY,USB_RTC_YR,USB_RTC_HR,USB_RTC_MIN,USB_RTC_SEC,DIAGNOSTICS,REG_STREAM.calc_val,REG_WATERCUT.calc_val,REG_WATERCUT_RAW,REG_TEMP_USER.calc_val,REG_TEMP_AVG.calc_val,REG_TEMP_ADJUST.calc_val,REG_FREQ.calc_val,REG_OIL_INDEX.calc_val,REG_OIL_RP,REG_OIL_PT,REG_OIL_P0.calc_val,REG_OIL_P1.calc_val, REG_OIL_DENSITY.calc_val, REG_OIL_FREQ_LOW.calc_val, REG_OIL_FREQ_HIGH.calc_val, REG_AO_LRV.calc_val, REG_AO_URV.calc_val, REG_AO_MANUAL_VAL,REG_RELAY_SETPOINT.calc_val);

	printf("LOG_BUF LENGTH : %d\n", strlen(LOG_BUF));

    /// fill data upto MAX_DATA_SIZE
    if ((MAX_BUF_SIZE - strlen(LOG_BUF)) > strlen(DATA_BUF))
    {
        strcat(LOG_BUF,DATA_BUF);
        return;
    }

	printf("open\n");

    /// open file
    fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
    if (fresult != FR_OK)
    {
        stopAccessingUsb(fresult);
        return;
    }

	printf("lseek\n");

    /// move pointer to end of file 
    fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
    if (fresult != FR_OK)
    {
        stopAccessingUsb(fresult);
        return;
    }

	printf("Swi_disable\n");
	Swi_disable();

	printf("f_puts\n");
    /// write
    if (f_puts(LOG_BUF,&fileWriteObject) == EOF)
    {
        stopAccessingUsb(FR_DISK_ERR);
        return;
    }

	printf("Swi_enable\n");
	Swi_enable();

    /// close file
	printf ("verify: file size: %d, LOG_BUF size: %d\n", f_size(&fileWriteObject), strlen(LOG_BUF));
	printf ("verify: file size: %d, LOG_BUF size: %d\n", f_size(&fileWriteObject), strlen(LOG_BUF));
	printf ("verify: file size: %d, LOG_BUF size: %d\n", f_size(&fileWriteObject), strlen(LOG_BUF));
	printf ("close\n");
    fresult = f_close(&fileWriteObject);
    if (fresult != FR_OK)
    {
        stopAccessingUsb(fresult);
        return;
    }

	printf ("LOG_BUF=0\n");
	LOG_BUF[0] = '\0';
	LOG_BUF[0] = '\0';
	LOG_BUF[0] = '\0';
	LOG_BUF[0] = '\0';
	LOG_BUF[0] = '\0';
	LOG_BUF[0] = '\0';
}


BOOL downloadCsv(char* fname)
{
	if (!isUsbActive()) return FALSE;
	isDownloadCsv = FALSE;
	
	FRESULT fr;	
	FIL csvWriteObject;
	CSV_BUF[0] = '\0';
	char csvFileName[50] = 0;
	int csv_buf_delay;
	int data_index;

	/// get file name
	(strcmp(PDI_RAZOR_PROFILE,fname)==0) ? sprintf(csvFileName,"0:%s.csv",fname) : sprintf(csvFileName,"0:P%06d.csv",REG_SN_PIPE);

	/// open file
	printf("f_open...");
	if (f_open(&csvWriteObject, csvFileName, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) 
	{
		isUpgradeFirmware = FALSE;
		return FALSE;;
	}

	/// disable all interrupts
	printf("Swi_disable...");
	Swi_disable();

	/// integer
	printf("data copy...");

    sprintf(CSV_BUF+strlen(CSV_BUF),"Serial,,201,int,1,RW,1,%d,\n",REG_SN_PIPE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Dampen,,203,int,1,RW,1,%d,\n",REG_AO_DAMPEN); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Slave Address,,204,int,1,RW,1,%d,\n",REG_SLAVE_ADDRESS); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stop Bits,,205,int,1,RW,1,%d,\n",REG_STOP_BITS); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Mode,,206,int,1,RW,1,%d,\n",REG_DENSITY_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Model Code 0,,219,int,1,RW,1,%d,\n",REG_MODEL_CODE[0]); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Model Code 1,,220,int,1,RW,1,%d,\n",REG_MODEL_CODE[1]); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Model Code 2,,221,int,1,RW,1,%d,\n",REG_MODEL_CODE[2]); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Model Code 3,,222,int,1,RW,1,%d,\n",REG_MODEL_CODE[3]); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Logging Period,,223,int,1,RW,1,%d,\n",REG_LOGGING_PERIOD); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Alarm Mode,,227,int,1,RW,1,%d,\n",REG_AO_ALARM_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Phase Hold Over,,228,int,1,RW,1,%d,\n",REG_PHASE_HOLD_CYCLES); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Delay,,229,int,1,RW,1,%d,\n",REG_RELAY_DELAY); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Mode,,230,int,1,RW,1,%d,\n",REG_AO_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Correction Mode,,231,int,1,RW,1,%d,\n",REG_OIL_DENS_CORR_MODE);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Mode,,232,int,1,RW,1,%d,\n",REG_RELAY_MODE); 

	/// float or double
	sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Adjust,,15,float,1,RW,1,%15.7f\n",REG_OIL_ADJUST.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Temp Adjust,,31,float,1,RW,1,%15.7f\n",REG_TEMP_ADJUST.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Proc Avg,,35,float,1,RW,1,%15.7f\n",REG_PROC_AVGING.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Index,,37,float,1,RW,1,%15.7f\n",REG_OIL_INDEX.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P0,,39,float,1,RW,1,%15.7f\n",REG_OIL_P0.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P1,,41,float,1,RW,1,%15.7f\n",REG_OIL_P1.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Low,,43,float,1,RW,1,%15.7f\n",REG_OIL_FREQ_LOW.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil High,,45,float,1,RW,1,%15.7f\n",REG_OIL_FREQ_HIGH.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Sample Period,,47,float,1,RW,1,%5.1f,\n",REG_SAMPLE_PERIOD.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO LRV,,49,float,1,RW,1,%15.7f\n",REG_AO_LRV.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO URV,,51,float,1,RW,1,%15.7f\n",REG_AO_URV.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Baud Rate,,55,float,1,RW,1,%10.1f,\n",REG_BAUD_RATE.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Calc Max,,67,float,1,RW,1,%15.7f,\n",REG_OIL_CALC_MAX); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Dual Curve Cutoff,,69,float,1,RW,1,%15.7f\n",REG_OIL_PHASE_CUTOFF);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stream,,73,float,1,RW,1,%10.1f,\n",REG_STREAM.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim Low,,107,float,1,RW,1,%15.7f\n",REG_AO_TRIMLO); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim High,,109,float,1,RW,1,%15.7f\n",REG_AO_TRIMHI); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Adj,,111,float,1,RW,1,%15.7f\n",REG_DENSITY_ADJ);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Unit,,113,float,1,RW,1,%10.1f,\n",REG_DENSITY_UNIT); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"D3,,117,float,1,RW,1,%15.7f\n",REG_DENSITY_D3.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D2,,119,float,1,RW,1,%15.7f\n",REG_DENSITY_D2.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D1,,121,float,1,RW,1,%15.7f\n",REG_DENSITY_D1.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D0,,123,float,1,RW,1,%15.7f\n",REG_DENSITY_D0.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Dens Calibration Val,,125,float,1,RW,1,%15.7f,\n",REG_DENSITY_CAL_VAL.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Setpoint,,151,float,1,RW,1,%15.7f,\n",REG_RELAY_SETPOINT.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density Manual,,161,float,1,RW,1,%15.7f,\n",REG_OIL_DENSITY_MANUAL); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density AI LRV,,163,float,1,RW,1,%15.7f,\n",REG_OIL_DENSITY_AI_LRV.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density AI URV,,165,float,1,RW,1,%15.7f,\n",REG_OIL_DENSITY_AI_URV.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim Low,,169,float,1,RW,1,%15.7f\n",REG_AI_TRIMLO); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim High,,171,float,1,RW,1,%15.7f\n",REG_AI_TRIMHI); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T0,,179,float,1,RW,1,%15.7f\n",REG_OIL_T0.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T1,,181,float,1,RW,1,%15.7f\n",REG_OIL_T1.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Temp Adj,,781,float,1,RW,1,%15.7f\n",PDI_TEMP_ADJ);
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Freq F0,,783,float,1,RW,1,%15.7f\n",PDI_FREQ_F0);
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Freq F1,,785,float,1,RW,1,%15.7f\n",PDI_FREQ_F1);

	/// extended 60K
    sprintf(CSV_BUF+strlen(CSV_BUF),"Number of Oil Temperature Curves,,60001,float,1,RW,1,%15.7f\n",REG_TEMP_OIL_NUM_CURVES);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Temperature List,,60003,float,1,RW,10,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_TEMPS_OIL[0],REG_TEMPS_OIL[1],REG_TEMPS_OIL[2],REG_TEMPS_OIL[3],REG_TEMPS_OIL[4],REG_TEMPS_OIL[5],REG_TEMPS_OIL[6],REG_TEMPS_OIL[7],REG_TEMPS_OIL[8],REG_TEMPS_OIL[9]);

	for (data_index=0;data_index<10;data_index++)
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve %d,,%d,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",data_index,60023+data_index*4,REG_COEFFS_TEMP_OIL[data_index][0],REG_COEFFS_TEMP_OIL[data_index][1],REG_COEFFS_TEMP_OIL[data_index][2],REG_COEFFS_TEMP_OIL[data_index][3]);

	/// long int	
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Measurement Section,,301,long,1,RW,1,%d\n",REG_MEASSECTION_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Back Board,,303,long,1,RW,1,%d\n",REG_BACKBOARD_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Safety Barrier,,305,long,1,RW,1,%d\n",REG_SAFETYBARRIER_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Power Supply,,307,long,1,RW,1,%d\n",REG_POWERSUPPLY_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Processor Board,,309,long,1,RW,1,%d\n",REG_PROCESSOR_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Display Board,,311,long,1,RW,1,%d\n",REG_DISPLAY_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - RF Board,,313,long,1,RW,1,%d\n",REG_RF_SN);
    sprintf(CSV_BUF+strlen(CSV_BUF),"SN - Assembly,,315,long,1,RW,1,%d\n",REG_ASSEMBLY_SN);

	/// hardware part serial number
	for (data_index=0;data_index<8;data_index++)
    sprintf(CSV_BUF+strlen(CSV_BUF),"Electronic SN %d,,%d,long,1,RW,1,%d\n",data_index,317+2*data_index,REG_ELECTRONICS_SN[data_index]);

	/// stream dependent data
	for (data_index=0;data_index<60;data_index++)
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stream Oil Adjust %d,,%d,float,1,RW,1,%15.7f\n",data_index,63647+2*data_index,STREAM_OIL_ADJUST[data_index]);
	
	printf("data ready\n");

	/// enable all interrupts back
	printf("Swi_enable\n");
	Swi_enable();

	/// write
	printf("writing file\n");
	UINT bw;
	fr = f_write(&csvWriteObject,CSV_BUF,strlen(CSV_BUF),&bw);
	if (fr != FR_OK) 
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		Swi_enable();
		return FALSE;
	}

	/// sync
	printf("syncing file\n");
	fr = f_sync(&csvWriteObject);
	if (fr != FR_OK)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		Swi_enable();
		return FALSE;
	}

	/// close file
	printf("closing file\n");
	fr = f_close(&csvWriteObject);
	if (fr != FR_OK)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		Swi_enable();
		return FALSE;
	}

	/// set global var true
	printf("set flags\n");
    isCsvDownloadSuccess = TRUE;
    isCsvUploadSuccess = FALSE;
    
    return TRUE;
}


void scanCsvFiles(void)
{
	if (!isUsbActive()) return;
	isScanCsvFiles = FALSE;

	FRESULT res;
    static DIR dir;
    static FILINFO fno;
	const char path[] = "0:";
	csvCounter = 0;
	CSV_FILES[0] = '\0';

	/// disable all interrupts
	printf("Swi_disable...");
	Swi_disable();

	printf("f_opendir...");
	if (f_opendir(&dir, path) != FR_OK) 
	{
		Swi_enable();
		return;
	}

	printf("Looping start...");
    for (;;) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;
        if (fno.fattrib & AM_DIR) {} // directory
		else // file
		{ 
			if (strstr(fno.fname, ".csv") != NULL) 
			{
				strcat(CSV_FILES,fno.fname);
				csvCounter++;
				isScanSuccess = TRUE;
			}
        }
    }

	/// close dir
	printf("Closing dir...");
	f_closedir(&dir);

	/// enable all interrupts back
	printf("Swi_enable...");
	Swi_enable();
    return;
}


BOOL uploadCsv(void)
{
	if (!isUsbActive()) return FALSE;
	isUploadCsv = FALSE;
	
	FIL fil;
	FRESULT result;
	char line[1024];
	char csvFileName[50];
	csvFileName[0] = '\0';
	line[0] = '\0';

	/// get file name
	(isPdiUpgradeMode) ? sprintf(csvFileName,"0:%s.csv",PDI_RAZOR_PROFILE) : sprintf(csvFileName,"0:%s.csv",CSV_FILES);

	/// open file
	printf("Open file...");
	if (f_open(&fil, csvFileName, FA_READ) != FR_OK) return FALSE;

	/// do not upgrade firmware after profiling
	isUpgradeFirmware = FALSE;

	/// swi disable
	printf("Swi_disable...");
	Swi_disable();

	/// read line
	printf("f_get looping starts...");
    while (f_gets(line, sizeof(line), &fil)) 
	{
		int i = 0; 
		char* regid;
		char* regval;
		char* regval1;
		char* regval2;
		char* regval3;
		char* regval4;
		char* regval5;
		char* regval6;
		char* regval7;
		char* regval8;
		char* regval9;

		/// remove trailing \n
		line[strcspn( line,"\n")] = '\0';

		/// split line
        char* ptr = strtok(line, ",");
        while (ptr != NULL)
        {   
			/// get value
			if (i==1) regid = ptr;
			else if (i==6) regval = ptr;
			else if (i==7) regval1 = ptr;
			else if (i==8) regval2 = ptr;
			else if (i==9) regval3 = ptr;
			else if (i==10) regval4 = ptr;
			else if (i==11) regval5 = ptr;
			else if (i==12) regval6 = ptr;
			else if (i==13) regval7 = ptr;
			else if (i==14) regval8 = ptr;
			else if (i==15) regval9 = ptr;

			/// next 
            ptr = strtok(NULL, ",");
            i++; 
        } 

		printf("updating register %s\n", regid);

		int ivalue = atoi(regval);
		float fvalue = atof(regval);
		float fvalue1 = atof(regval1);
		float fvalue2 = atof(regval2);
		float fvalue3 = atof(regval3);
		float fvalue4 = atof(regval4);
		float fvalue5 = atof(regval5);
		float fvalue6 = atof(regval6);
		float fvalue7 = atof(regval7);
		float fvalue8 = atof(regval8);
		float fvalue9 = atof(regval9);

		/// integer	
		if      (strcmp(regid, "201") == 0) REG_SN_PIPE = ivalue;
		else if (strcmp(regid, "203") == 0) REG_AO_DAMPEN = ivalue;
		else if (strcmp(regid, "204") == 0) REG_SLAVE_ADDRESS = ivalue;
		else if (strcmp(regid, "205") == 0) REG_STOP_BITS = ivalue;
		else if (strcmp(regid, "206") == 0) REG_DENSITY_MODE = ivalue;
		else if (strcmp(regid, "219") == 0) REG_MODEL_CODE[0] = ivalue;
		else if (strcmp(regid, "220") == 0) REG_MODEL_CODE[1] = ivalue;
		else if (strcmp(regid, "221") == 0) REG_MODEL_CODE[2] = ivalue;
		else if (strcmp(regid, "222") == 0) REG_MODEL_CODE[3] = ivalue;
		else if (strcmp(regid, "223") == 0) REG_LOGGING_PERIOD = ivalue;
		else if (strcmp(regid, "227") == 0) REG_AO_ALARM_MODE = ivalue;
		else if (strcmp(regid, "228") == 0) REG_PHASE_HOLD_CYCLES = ivalue;
		else if (strcmp(regid, "229") == 0) REG_RELAY_DELAY = ivalue;
		else if (strcmp(regid, "230") == 0) REG_AO_MODE = ivalue;
		else if (strcmp(regid, "231") == 0) REG_OIL_DENS_CORR_MODE = ivalue;
		else if (strcmp(regid, "232") == 0) REG_RELAY_MODE = ivalue;

		/// long int
		else if (strcmp(regid, "301") == 0) REG_MEASSECTION_SN = ivalue;
		else if (strcmp(regid, "303") == 0) REG_BACKBOARD_SN = ivalue;
		else if (strcmp(regid, "305") == 0) REG_SAFETYBARRIER_SN = ivalue;
		else if (strcmp(regid, "307") == 0) REG_POWERSUPPLY_SN = ivalue;
		else if (strcmp(regid, "309") == 0) REG_PROCESSOR_SN = ivalue;
		else if (strcmp(regid, "311") == 0) REG_DISPLAY_SN = ivalue;
		else if (strcmp(regid, "313") == 0) REG_RF_SN = ivalue;
		else if (strcmp(regid, "315") == 0) REG_ASSEMBLY_SN = ivalue;
		else if (strcmp(regid, "15") == 0) VAR_Update(&REG_OIL_ADJUST,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "31") == 0) VAR_Update(&REG_TEMP_ADJUST,fvalue,CALC_UNIT);
		else if (strcmp(regid, "35") == 0) VAR_Update(&REG_PROC_AVGING,fvalue,CALC_UNIT);
		else if (strcmp(regid, "37") == 0) VAR_Update(&REG_OIL_INDEX,fvalue,CALC_UNIT);
		else if (strcmp(regid, "39") == 0) VAR_Update(&REG_OIL_P0,fvalue,CALC_UNIT);
		else if (strcmp(regid, "41") == 0) VAR_Update(&REG_OIL_P1,fvalue,CALC_UNIT);
		else if (strcmp(regid, "43") == 0) VAR_Update(&REG_OIL_FREQ_LOW,fvalue,CALC_UNIT);
		else if (strcmp(regid, "45") == 0) VAR_Update(&REG_OIL_FREQ_HIGH,fvalue,CALC_UNIT);
		else if (strcmp(regid, "47") == 0) VAR_Update(&REG_SAMPLE_PERIOD,fvalue,CALC_UNIT);
		else if (strcmp(regid, "49") == 0) VAR_Update(&REG_AO_URV,fvalue,CALC_UNIT);
		else if (strcmp(regid, "51") == 0) VAR_Update(&REG_AO_LRV,fvalue,CALC_UNIT);
		else if (strcmp(regid, "55") == 0) VAR_Update(&REG_BAUD_RATE,fvalue,CALC_UNIT);
		else if (strcmp(regid, "67") == 0) REG_OIL_CALC_MAX = fvalue;
		else if (strcmp(regid, "69") == 0) REG_OIL_PHASE_CUTOFF = fvalue;
		else if (strcmp(regid, "73") == 0) VAR_Update(&REG_STREAM,fvalue,CALC_UNIT);
		else if (strcmp(regid, "107") == 0) REG_AO_TRIMLO = fvalue;
		else if (strcmp(regid, "109") == 0) REG_AO_TRIMHI = fvalue;
		else if (strcmp(regid, "111") == 0) REG_DENSITY_ADJ = fvalue;
		else if (strcmp(regid, "117") == 0) VAR_Update(&REG_DENSITY_D3,fvalue,CALC_UNIT);
		else if (strcmp(regid, "119") == 0) VAR_Update(&REG_DENSITY_D2,fvalue,CALC_UNIT);
		else if (strcmp(regid, "121") == 0) VAR_Update(&REG_DENSITY_D1,fvalue,CALC_UNIT);
		else if (strcmp(regid, "123") == 0) VAR_Update(&REG_DENSITY_D0,fvalue,CALC_UNIT);
		else if (strcmp(regid, "125") == 0) VAR_Update(&REG_DENSITY_CAL_VAL,fvalue,CALC_UNIT);
		else if (strcmp(regid, "151") == 0) VAR_Update(&REG_RELAY_SETPOINT,fvalue,CALC_UNIT);
		else if (strcmp(regid, "161") == 0) REG_OIL_DENSITY_MANUAL = fvalue;
		else if (strcmp(regid, "163") == 0) VAR_Update(&REG_OIL_DENSITY_AI_LRV,fvalue,CALC_UNIT);
		else if (strcmp(regid, "165") == 0) VAR_Update(&REG_OIL_DENSITY_AI_URV,fvalue,CALC_UNIT);
		else if (strcmp(regid, "169") == 0) REG_AI_TRIMLO = fvalue;
		else if (strcmp(regid, "171") == 0) REG_AI_TRIMHI = fvalue;
		else if (strcmp(regid, "179") == 0) VAR_Update(&REG_OIL_T0,fvalue,CALC_UNIT);
		else if (strcmp(regid, "181") == 0) VAR_Update(&REG_OIL_T1,fvalue,CALC_UNIT);
		else if (strcmp(regid, "781") == 0) PDI_TEMP_ADJ = fvalue;
		else if (strcmp(regid, "783") == 0) PDI_FREQ_F0 = fvalue;
		else if (strcmp(regid, "785") == 0) PDI_FREQ_F1 = fvalue;

		/// extended
		else if (strcmp(regid, "60001") == 0) REG_TEMP_OIL_NUM_CURVES = fvalue; 
		else if (strcmp(regid, "60003") == 0) 
		{
			REG_TEMPS_OIL[0] = fvalue;
			REG_TEMPS_OIL[1] = fvalue1;
			REG_TEMPS_OIL[2] = fvalue2;
			REG_TEMPS_OIL[3] = fvalue3;
			REG_TEMPS_OIL[4] = fvalue4;
			REG_TEMPS_OIL[5] = fvalue5;
			REG_TEMPS_OIL[6] = fvalue6;
			REG_TEMPS_OIL[7] = fvalue7;
			REG_TEMPS_OIL[8] = fvalue8;
			REG_TEMPS_OIL[9] = fvalue9;
		}
		else if (strcmp(regid, "60023") == 0) 
		{
			REG_COEFFS_TEMP_OIL[0][0] = fvalue;
			REG_COEFFS_TEMP_OIL[0][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[0][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[0][3] = fvalue3;
		}
		else if (strcmp(regid, "60031") == 0) 
		{
			REG_COEFFS_TEMP_OIL[1][0] = fvalue;
			REG_COEFFS_TEMP_OIL[1][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[1][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[1][3] = fvalue3;
		}
		else if (strcmp(regid, "60039") == 0)
		{
			REG_COEFFS_TEMP_OIL[2][0] = fvalue;
			REG_COEFFS_TEMP_OIL[2][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[2][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[2][3] = fvalue3;
		}
		else if (strcmp(regid, "60047") == 0)
		{
			REG_COEFFS_TEMP_OIL[3][0] = fvalue;
			REG_COEFFS_TEMP_OIL[3][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[3][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[3][3] = fvalue3;
		}
		else if (strcmp(regid, "60055") == 0)
		{
			REG_COEFFS_TEMP_OIL[4][0] = fvalue;
			REG_COEFFS_TEMP_OIL[4][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[4][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[4][3] = fvalue3;
		}
		else if (strcmp(regid, "60063") == 0)
		{
			REG_COEFFS_TEMP_OIL[5][0] = fvalue;
			REG_COEFFS_TEMP_OIL[5][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[5][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[5][3] = fvalue3;
		}

		/// stream dependent data
		else if (strcmp(regid, "63647") == 0) STREAM_OIL_ADJUST[0] = fvalue;
		else if (strcmp(regid, "63649") == 0) STREAM_OIL_ADJUST[1] = fvalue;
		else if (strcmp(regid, "63651") == 0) STREAM_OIL_ADJUST[2] = fvalue;
		else if (strcmp(regid, "63653") == 0) STREAM_OIL_ADJUST[3] = fvalue;
		else if (strcmp(regid, "63655") == 0) STREAM_OIL_ADJUST[4] = fvalue;
		else if (strcmp(regid, "63657") == 0) STREAM_OIL_ADJUST[5] = fvalue;
		else if (strcmp(regid, "63659") == 0) STREAM_OIL_ADJUST[6] = fvalue;
		else if (strcmp(regid, "63661") == 0) STREAM_OIL_ADJUST[7] = fvalue;
		else if (strcmp(regid, "63663") == 0) STREAM_OIL_ADJUST[8] = fvalue;
		else if (strcmp(regid, "63665") == 0) STREAM_OIL_ADJUST[9] = fvalue;
		else if (strcmp(regid, "63667") == 0) STREAM_OIL_ADJUST[10] = fvalue;
		else if (strcmp(regid, "63669") == 0) STREAM_OIL_ADJUST[11] = fvalue;
		else if (strcmp(regid, "63671") == 0) STREAM_OIL_ADJUST[12] = fvalue;
		else if (strcmp(regid, "63673") == 0) STREAM_OIL_ADJUST[13] = fvalue;
		else if (strcmp(regid, "63675") == 0) STREAM_OIL_ADJUST[14] = fvalue;
		else if (strcmp(regid, "63677") == 0) STREAM_OIL_ADJUST[15] = fvalue;
		else if (strcmp(regid, "63679") == 0) STREAM_OIL_ADJUST[16] = fvalue;
		else if (strcmp(regid, "63681") == 0) STREAM_OIL_ADJUST[17] = fvalue;
		else if (strcmp(regid, "63683") == 0) STREAM_OIL_ADJUST[18] = fvalue;
		else if (strcmp(regid, "63685") == 0) STREAM_OIL_ADJUST[19] = fvalue;
		else if (strcmp(regid, "63687") == 0) STREAM_OIL_ADJUST[20] = fvalue;
		else if (strcmp(regid, "63689") == 0) STREAM_OIL_ADJUST[21] = fvalue;
		else if (strcmp(regid, "63691") == 0) STREAM_OIL_ADJUST[22] = fvalue;
		else if (strcmp(regid, "63693") == 0) STREAM_OIL_ADJUST[23] = fvalue;
		else if (strcmp(regid, "63695") == 0) STREAM_OIL_ADJUST[24] = fvalue;
		else if (strcmp(regid, "63697") == 0) STREAM_OIL_ADJUST[25] = fvalue;
		else if (strcmp(regid, "63699") == 0) STREAM_OIL_ADJUST[26] = fvalue;
		else if (strcmp(regid, "63701") == 0) STREAM_OIL_ADJUST[27] = fvalue;
		else if (strcmp(regid, "63703") == 0) STREAM_OIL_ADJUST[28] = fvalue;
		else if (strcmp(regid, "63705") == 0) STREAM_OIL_ADJUST[29] = fvalue;
		else if (strcmp(regid, "63707") == 0) STREAM_OIL_ADJUST[30] = fvalue;
		else if (strcmp(regid, "63709") == 0) STREAM_OIL_ADJUST[31] = fvalue;
		else if (strcmp(regid, "63711") == 0) STREAM_OIL_ADJUST[32] = fvalue;
		else if (strcmp(regid, "63713") == 0) STREAM_OIL_ADJUST[33] = fvalue;
		else if (strcmp(regid, "63715") == 0) STREAM_OIL_ADJUST[34] = fvalue;
		else if (strcmp(regid, "63717") == 0) STREAM_OIL_ADJUST[35] = fvalue;
		else if (strcmp(regid, "63719") == 0) STREAM_OIL_ADJUST[36] = fvalue;
		else if (strcmp(regid, "63721") == 0) STREAM_OIL_ADJUST[37] = fvalue;
		else if (strcmp(regid, "63723") == 0) STREAM_OIL_ADJUST[38] = fvalue;
		else if (strcmp(regid, "63725") == 0) STREAM_OIL_ADJUST[39] = fvalue;
		else if (strcmp(regid, "63727") == 0) STREAM_OIL_ADJUST[40] = fvalue;
		else if (strcmp(regid, "63729") == 0) STREAM_OIL_ADJUST[41] = fvalue;
		else if (strcmp(regid, "63731") == 0) STREAM_OIL_ADJUST[42] = fvalue;
		else if (strcmp(regid, "63733") == 0) STREAM_OIL_ADJUST[43] = fvalue;
		else if (strcmp(regid, "63735") == 0) STREAM_OIL_ADJUST[44] = fvalue;
		else if (strcmp(regid, "63737") == 0) STREAM_OIL_ADJUST[45] = fvalue;
		else if (strcmp(regid, "63739") == 0) STREAM_OIL_ADJUST[46] = fvalue;
		else if (strcmp(regid, "63741") == 0) STREAM_OIL_ADJUST[47] = fvalue;
		else if (strcmp(regid, "63743") == 0) STREAM_OIL_ADJUST[48] = fvalue;
		else if (strcmp(regid, "63745") == 0) STREAM_OIL_ADJUST[49] = fvalue;
		else if (strcmp(regid, "63747") == 0) STREAM_OIL_ADJUST[50] = fvalue;
		else if (strcmp(regid, "63749") == 0) STREAM_OIL_ADJUST[51] = fvalue;
		else if (strcmp(regid, "63751") == 0) STREAM_OIL_ADJUST[52] = fvalue;
		else if (strcmp(regid, "63753") == 0) STREAM_OIL_ADJUST[53] = fvalue;
		else if (strcmp(regid, "63755") == 0) STREAM_OIL_ADJUST[54] = fvalue;
		else if (strcmp(regid, "63757") == 0) STREAM_OIL_ADJUST[55] = fvalue;
		else if (strcmp(regid, "63759") == 0) STREAM_OIL_ADJUST[56] = fvalue;
		else if (strcmp(regid, "63761") == 0) STREAM_OIL_ADJUST[57] = fvalue;
		else if (strcmp(regid, "63763") == 0) STREAM_OIL_ADJUST[58] = fvalue;
		else if (strcmp(regid, "63765") == 0) STREAM_OIL_ADJUST[59] = fvalue;

		line[0] = '\0';
		line[0] = '\0';

		/// print status -- we use print as an intended "delay"
		if (isPdiUpgradeMode) 
		{
			LCD_setcursor(0,0);
			displayLcd(" PROFILE UPLOAD ",0);
		}

		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
		displayLcd("    Loading...  ",1);
	}	

	printf("Swi_enable\n");
	Swi_enable();

	/// close file
	printf("closing file\n");
	f_close(&fil);

	/// set global var true
	printf("set global flags\n");
    isCsvUploadSuccess = TRUE;
    isCsvDownloadSuccess = FALSE;

	/// update factory default
	printf("FACTORY_DEFAULT\n");
    COIL_UPDATE_FACTORY_DEFAULT.val = TRUE;

	/// write to flash
	printf("Swi_post\n");
	Swi_post(Swi_writeNand);	

	/// delete PDI_RAZOR_PROFILE
	if (isPdiUpgradeMode) 
	{
		displayLcd("PROFILE UPLOADED",0);
		f_unlink(csvFileName);
	}

    while (1) displayLcd("   REMOVE USB   ", 1);

	return TRUE;
}
