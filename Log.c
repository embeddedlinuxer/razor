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
#define MAX_BUF_SIZE   	4096*2

static char logFile[] = "0:PDI/LOG_01_01_2019.csv";

static Uint8 try = 0;
static Uint8 try2 = 0;
static Uint8 try3 = 0;
static Uint8 try4 = 0;

static char CSV_BUF[MAX_BUF_SIZE];
static char LOG_HEADER[MAX_HEADER_SIZE];
static char LOG_BUF[MAX_BUF_SIZE];
static char LOG_BUF1[MAX_BUF_SIZE];
static char LOG_BUF2[MAX_BUF_SIZE];
static char LOG_BUF3[MAX_BUF_SIZE];
static char LOG_BUF4[MAX_BUF_SIZE];
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
	isPdiRazorProfile = FALSE;
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

	LOG_HEADER[0] = '\0';
	LOG_BUF[0] = '\0';
	LOG_BUF1[0] = '\0';
	LOG_BUF2[0] = '\0';
	LOG_BUF3[0] = '\0';
	LOG_BUF4[0] = '\0';
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

	FIL logReadObject;
    FRESULT fresult;
	int write_file_size = 0;
	static int buf_index = 0;

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
       	// TIMESTAMP VALIDATOR
       	static int delayLogging = 0;
       	delayLogging++;
       	if (delayLogging < 10) return;
       	delayLogging = 0;

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

   	char DATA_BUF[MAX_DATA_SIZE];
	DATA_BUF[0] = '\0';
   	sprintf(DATA_BUF,"%02d-%02d-20%02d,%02d:%02d:%02d,%10d,%2.0f,%6.2f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%6.3f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%5.1f,%5.1f,%5.2f,%8.1f,\n",USB_RTC_MON,USB_RTC_DAY,USB_RTC_YR,USB_RTC_HR,USB_RTC_MIN,USB_RTC_SEC,DIAGNOSTICS,REG_STREAM.calc_val,REG_WATERCUT.calc_val,REG_WATERCUT_RAW,REG_TEMP_USER.calc_val,REG_TEMP_AVG.calc_val,REG_TEMP_ADJUST.calc_val,REG_FREQ.calc_val,REG_OIL_INDEX.calc_val,REG_OIL_RP,REG_OIL_PT,REG_OIL_P0.calc_val,REG_OIL_P1.calc_val, REG_OIL_DENSITY.calc_val, REG_OIL_FREQ_LOW.calc_val, REG_OIL_FREQ_HIGH.calc_val, REG_AO_LRV.calc_val, REG_AO_URV.calc_val, REG_AO_MANUAL_VAL,REG_RELAY_SETPOINT.calc_val);

 	if (buf_index == 0)
    {
        /// fill data upto MAX_DATA_SIZE
        if ((MAX_BUF_SIZE - strlen(LOG_BUF)) > strlen(DATA_BUF))
        {
            strcat(LOG_BUF,DATA_BUF);
            return;
        }

        /// open file
        fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// move to entry point
        fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// write
        if (f_puts(LOG_BUF,&fileWriteObject) == EOF)
        {
            stopAccessingUsb(FR_DISK_ERR);
            return;
        }

        /// reset log buf
        LOG_BUF1[0] = '\0';
        buf_index = 1;
    }
	else if (buf_index == 1)
    {
        /// fill data upto MAX_DATA_SIZE
        if ((MAX_BUF_SIZE - strlen(LOG_BUF1)) > strlen(DATA_BUF))
        {
            strcat(LOG_BUF1,DATA_BUF);
            return;
        }

        /// open file
        fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// move to entry point
        fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// write
        if (f_puts(LOG_BUF1,&fileWriteObject) == EOF)
        {
            stopAccessingUsb(FR_DISK_ERR);
            return;
        }

        /// reset log buf
        LOG_BUF2[0] = '\0';
        buf_index = 2;
    }
    else if (buf_index == 2)
    {
        /// fill data upto MAX_DATA_SIZE
        if ((MAX_BUF_SIZE - strlen(LOG_BUF2)) > strlen(DATA_BUF))
        {
            strcat(LOG_BUF2,DATA_BUF);
            return;
        }

        /// open file
        fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// move to entry point
        fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// write
        if (f_puts(LOG_BUF2,&fileWriteObject) == EOF)
        {
            stopAccessingUsb(FR_DISK_ERR);
            return;
        }

        /// reset log buf
        LOG_BUF3[0] = '\0';
        buf_index = 3;
    }
    else if (buf_index == 3)
    {
        /// fill data upto MAX_DATA_SIZE
        if ((MAX_BUF_SIZE - strlen(LOG_BUF3)) > strlen(DATA_BUF))
        {
            strcat(LOG_BUF3,DATA_BUF);
            return;
        }

        /// open file
        fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// move to entry point
        fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
		if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// write
        if (f_puts(LOG_BUF3,&fileWriteObject) == EOF)
        {
            stopAccessingUsb(FR_DISK_ERR);
            return;
        }

        /// reset log buf
        LOG_BUF4[0] = '\0';
        buf_index = 4;
    }
    else if (buf_index == 4)
    {
        /// fill data upto MAX_DATA_SIZE
        if ((MAX_BUF_SIZE - strlen(LOG_BUF4)) > strlen(DATA_BUF))
        {
            strcat(LOG_BUF4,DATA_BUF);
	        return;
        }

        /// open file
        fresult = f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

        /// move to entry point
        fresult = f_lseek(&fileWriteObject,f_size(&fileWriteObject));
        if (fresult != FR_OK)
        {
            stopAccessingUsb(fresult);
            return;
        }

		/// write
        if (f_puts(LOG_BUF4,&fileWriteObject) == EOF)
        {
            stopAccessingUsb(FR_DISK_ERR);
            return;
        }

        /// reset log buf
        LOG_BUF[0] = '\0';
        buf_index = 0;
    }

    /// sync with usb drive
    fresult = f_sync(&fileWriteObject);
    if (fresult != FR_OK)
    {
        stopAccessingUsb(fresult);
        return;
    }

	/// update file size before closing
	write_file_size = f_size(&fileWriteObject);

    /// close file
    fresult = f_close(&fileWriteObject);
    if (fresult != FR_OK)
    {
        stopAccessingUsb(fresult);
        return;
    }

	/// verify the written file in USB
	f_open(&logReadObject,logFile,FA_READ);
	if (f_size(&logReadObject) != write_file_size)
	{
		f_close(&logReadObject); 
		stopAccessingUsb(FR_DISK_ERR);
		return;
	}

	f_close(&logReadObject); 
}


BOOL downloadCsv(void) 
{
	if (!isUsbActive()) return;
	isDownloadCsv = FALSE;
	
	FRESULT fr;	
	FIL csvWriteObject;
	CSV_BUF[0] = '\0';
	char csvFileName[16] = 0;

    if (isPdiRazorProfile)
	{
		isPdiRazorProfile = FALSE;
		if (f_open(&csvWriteObject, PDI_RAZOR_PROFILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return FALSE;
	}
	else  
	{
		sprintf(csvFileName,"0:P%06d.csv",REG_SN_PIPE);
		if (f_open(&csvWriteObject, csvFileName, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return FALSE;
	}

    /// REG_SN_PIPE
    sprintf(CSV_BUF,"Serial,,201,int,1,RW,1,%d,\n",REG_SN_PIPE); 

    /// REG_OIL_P0
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P0,,39,float,1,RW,1,%15.7f\n",REG_OIL_P0.calc_val); 

    /// REG_OIL_P1
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P1,,41,float,1,RW,1,%15.7f\n",REG_OIL_P1.calc_val); 

    /// REG_OIL_T0
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T0,,179,float,1,RW,1,%15.7f\n",REG_OIL_T0.calc_val); 

    /// REG_OIL_T1
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T1,,181,float,1,RW,1,%15.7f\n",REG_OIL_T1.calc_val); 

    /// REG_OIL_FREQ_LOW
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Low,,43,float,1,RW,1,%15.7f\n",REG_OIL_FREQ_LOW.calc_val); 

    /// REG_OIL_FREQ_HIGH
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil High,,45,float,1,RW,1,%15.7f\n",REG_OIL_FREQ_HIGH.calc_val); 

    /// REG_AO_TRIMLO
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim Low,,107,float,1,RW,1,%15.7f\n",REG_AO_TRIMLO); 

    /// REG_AO_TRIMHI
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim High,,109,float,1,RW,1,%15.7f\n",REG_AO_TRIMHI); 

    /// REG_AI_TRIMLO
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim Low,,169,float,1,RW,1,%15.7f\n",REG_AI_TRIMLO); 

    /// REG_AI_TRIMHI
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim High,,171,float,1,RW,1,%15.7f\n",REG_AI_TRIMHI); 

	/// REG_DENSITY_D3
    sprintf(CSV_BUF+strlen(CSV_BUF),"D3,,117,float,1,RW,1,%15.7f\n",REG_DENSITY_D3.calc_val);

    /// REG_DENSITY_2D
    sprintf(CSV_BUF+strlen(CSV_BUF),"D2,,119,float,1,RW,1,%15.7f\n",REG_DENSITY_D2.calc_val);

    /// REG_DENSITY_D1
    sprintf(CSV_BUF+strlen(CSV_BUF),"D1,,121,float,1,RW,1,%15.7f\n",REG_DENSITY_D1.calc_val);

    /// REG_DENSITY_0D
    sprintf(CSV_BUF+strlen(CSV_BUF),"D0,,123,float,1,RW,1,%15.7f\n",REG_DENSITY_D0.calc_val);

    /// REG_OIL_PHASE_CUTFF
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Dual Curve Cutoff,,69,float,1,RW,1,%15.7f\n",REG_OIL_PHASE_CUTOFF);

    /// REG_TEMP_OIL_NUM_CURVES
    sprintf(CSV_BUF+strlen(CSV_BUF),"Number of Oil Temperature Curves,,60001,float,1,RW,1,%15.7f\n",REG_TEMP_OIL_NUM_CURVES);

    /// REG_TEMPS_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Temperature List,,60003,float,1,RW,10,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_TEMPS_OIL[0],REG_TEMPS_OIL[1],REG_TEMPS_OIL[2],REG_TEMPS_OIL[3],REG_TEMPS_OIL[4],REG_TEMPS_OIL[5],REG_TEMPS_OIL[6],REG_TEMPS_OIL[7],REG_TEMPS_OIL[8],REG_TEMPS_OIL[9]);

    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve0,,60023,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[0][0],REG_COEFFS_TEMP_OIL[0][1],REG_COEFFS_TEMP_OIL[0][2],REG_COEFFS_TEMP_OIL[0][3])
;
    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve1,,60031,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[1][0],REG_COEFFS_TEMP_OIL[1][1],REG_COEFFS_TEMP_OIL[1][2],REG_COEFFS_TEMP_OIL[1][3]);

    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve2,,60039,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[2][0],REG_COEFFS_TEMP_OIL[2][1],REG_COEFFS_TEMP_OIL[2][2],REG_COEFFS_TEMP_OIL[2][3]);

    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve3,,60047,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[3][0],REG_COEFFS_TEMP_OIL[3][1],REG_COEFFS_TEMP_OIL[3][2],REG_COEFFS_TEMP_OIL[3][3]);

    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve4,,60055,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[4][0],REG_COEFFS_TEMP_OIL[4][1],REG_COEFFS_TEMP_OIL[4][2],REG_COEFFS_TEMP_OIL[4][3]);

    /// REG_COEFFS_TEMP_OIL
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve5,,60063,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",REG_COEFFS_TEMP_OIL[5][0],REG_COEFFS_TEMP_OIL[5][1],REG_COEFFS_TEMP_OIL[5][2],REG_COEFFS_TEMP_OIL[5][3]);

    /// REG_OIL_ADJUSTA
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Adjust,,15,float,1,RW,1,%15.7f\n",REG_OIL_ADJUST.calc_val);

    /// REG_TEMP_ADJUSTA
    sprintf(CSV_BUF+strlen(CSV_BUF),"Temp Adjust,,31,float,1,RW,1,%15.7f\n",REG_TEMP_ADJUST.calc_val);

    /// REG_PROC_AVGING
    sprintf(CSV_BUF+strlen(CSV_BUF),"Proc Avg,,35,float,1,RW,1,%15.7f\n",REG_PROC_AVGING.calc_val);

	/// REG_OIL_INDEX
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Index,,37,float,1,RW,1,%15.7f\n",REG_OIL_INDEX.calc_val);

    /// REG_AO_LRV
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO LRV,,49,float,1,RW,1,%15.7f\n",REG_AO_LRV.calc_val);

    /// REG_AO_URV
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO URV,,51,float,1,RW,1,%15.7f\n",REG_AO_URV.calc_val);

    /// REG_AO_URV
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Adj,,111,float,1,RW,1,%15.7f\0",REG_DENSITY_ADJ);

	UINT bw;
	fr = f_write(&csvWriteObject,CSV_BUF,strlen(CSV_BUF),&bw);
	if (fr != FR_OK) 
	{
		stopAccessingUsb(fr);
		return FALSE;
	}

	fr = f_sync(&csvWriteObject);
	if (fr != FR_OK)
	{
		stopAccessingUsb(fr);
		return FALSE;
	}

	fr = f_close(&csvWriteObject);
	if (fr != FR_OK)
	{
		stopAccessingUsb(fr);
		return FALSE;
	}

	FIL csvReadObject;
	f_open(&csvReadObject,csvFileName,FA_READ);
	if (f_size(&csvReadObject) != strlen(CSV_BUF))
	{
		f_close(&csvReadObject); 
		stopAccessingUsb(FR_DISK_ERR);
		return FALSE;
	}

	f_close(&csvReadObject); 

	/// set global var true
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

	if (f_opendir(&dir, path) != FR_OK) return;

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

	f_closedir(&dir);

    return;
}


BOOL uploadCsv(void)
{
	if (!isUsbActive()) return FALSE;
	if (!isPdiRazorProfile) isUploadCsv = FALSE;
	
	FIL fil;
	FRESULT result;
	char line[250];
	char csvFileToUpload[50];

	/// open csv file to upload
	if (isPdiRazorProfile) 
	{
		isPdiRazorProfile = FALSE;
		if (f_open(&fil, PDI_RAZOR_PROFILE, FA_READ) != FR_OK) 
		{
			f_close(&fil);
			return FALSE;
		}
		displayLcd(" UPLOAD PROFILE ",0);
	}

	/// make the file name
	csvFileToUpload[0] = '\0';
	strcat(csvFileToUpload,"0:");
	strcat(csvFileToUpload,CSV_FILES);
	strcat(csvFileToUpload,".csv");
	
	/// open csv file in normal mode
	if (f_open(&fil, csvFileToUpload, FA_READ) != FR_OK) return FALSE;

	/// read line
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

    	/// upload 
		if      (strcmp(regid, "201") == 0) REG_SN_PIPE = atoi(regval);
		else if (strcmp(regid, "39") == 0) VAR_Update(&REG_OIL_P0,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "41") == 0) VAR_Update(&REG_OIL_P1,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "43") == 0) VAR_Update(&REG_OIL_FREQ_LOW,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "45") == 0) VAR_Update(&REG_OIL_FREQ_HIGH,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "69") == 0) REG_OIL_PHASE_CUTOFF = atof(regval);
		else if (strcmp(regid, "107") == 0) REG_AO_TRIMLO = atof(regval);
		else if (strcmp(regid, "109") == 0) REG_AO_TRIMHI = atof(regval);
		else if (strcmp(regid, "117") == 0) VAR_Update(&REG_DENSITY_D3,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "119") == 0) VAR_Update(&REG_DENSITY_D2,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "121") == 0) VAR_Update(&REG_DENSITY_D1,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "123") == 0) VAR_Update(&REG_DENSITY_D0,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "169") == 0) REG_AI_TRIMLO = atof(regval);
		else if (strcmp(regid, "171") == 0) REG_AI_TRIMHI = atof(regval);
		else if (strcmp(regid, "179") == 0) VAR_Update(&REG_OIL_T0,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "181") == 0) VAR_Update(&REG_OIL_T1,atof(regval),CALC_UNIT);
		else if (strcmp(regid, "781") == 0) PDI_TEMP_ADJ = atof(regval);
		else if (strcmp(regid, "783") == 0) PDI_FREQ_F0 = atof(regval);
		else if (strcmp(regid, "785") == 0) PDI_FREQ_F1 = atof(regval);
		else if (strcmp(regid, "60001") == 0) REG_TEMP_OIL_NUM_CURVES = atof(regval); 
		else if (strcmp(regid, "60003") == 0) 
		{
			REG_TEMPS_OIL[0] = atof(regval);
			REG_TEMPS_OIL[1] = atof(regval1);
			REG_TEMPS_OIL[2] = atof(regval2);
			REG_TEMPS_OIL[3] = atof(regval3);
			REG_TEMPS_OIL[4] = atof(regval4);
			REG_TEMPS_OIL[5] = atof(regval5);
			REG_TEMPS_OIL[6] = atof(regval6);
			REG_TEMPS_OIL[7] = atof(regval7);
			REG_TEMPS_OIL[8] = atof(regval8);
			REG_TEMPS_OIL[9] = atof(regval9);
		}
		else if (strcmp(regid, "60023") == 0) 
		{
			REG_COEFFS_TEMP_OIL[0][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[0][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[0][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[0][3] = atof(regval3);
		}
		else if (strcmp(regid, "60031") == 0) 
		{
			REG_COEFFS_TEMP_OIL[1][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[1][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[1][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[1][3] = atof(regval3);
		}	
		else if (strcmp(regid, "60039") == 0)
		{
			REG_COEFFS_TEMP_OIL[2][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[2][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[2][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[2][3] = atof(regval3);
		}
		else if (strcmp(regid, "60047") == 0)
		{
			REG_COEFFS_TEMP_OIL[3][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[3][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[3][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[3][3] = atof(regval3);
		}
		else if (strcmp(regid, "60055") == 0)
		{
			REG_COEFFS_TEMP_OIL[4][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[4][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[4][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[4][3] = atof(regval3);
		}
		else if (strcmp(regid, "60063") == 0)
		{
			REG_COEFFS_TEMP_OIL[5][0] = atof(regval);
			REG_COEFFS_TEMP_OIL[5][1] = atof(regval1);
			REG_COEFFS_TEMP_OIL[5][2] = atof(regval2);
			REG_COEFFS_TEMP_OIL[5][3] = atof(regval3);
		}

		line[0] = '\0';
	}

	f_close(&fil);

	/// set global var true
    isCsvUploadSuccess = TRUE;
    isCsvDownloadSuccess = FALSE;

	/// write to flash
	Swi_post(Swi_writeNand);	

	/// delete PDI_RAZOR_PROFILE
	f_unlink(PDI_RAZOR_PROFILE);

    for(;;) displayLcd("   Remove USB   ", 1);

	return TRUE;
}
