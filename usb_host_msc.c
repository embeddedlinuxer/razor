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

#define USB_INSTANCE    0
#define MIN_DISK_SPACE  10240 // 10MB
#define NANDWIDTH_16
#define OMAPL138_LCDK
#define MAX_DATA_SIZE   8192

unsigned int g_ulMSCInstance = 0;
static USB_Handle usb_handle;
static USB_Params usb_host_params;
static FIL fileWriteObject;
static char logFile[] = "0:PDI/LOG_01_01_2019.csv";
static Uint8 current_day = 99;
static char LOG_BUF[MAX_DATA_SIZE];
static char LOG_HEADER[110];
static BOOL isWriteFile = TRUE;
static BOOL isResetUsbDriver = TRUE;

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

void resetUsbDriver(void);
void usbHostIntrConfig(USB_Params* usbParams);
void MSCCallback(uint32_t ulInstance, uint32_t ulEvent, void *pvData);
void usbCoreIntrHandler(uint32_t* pUsbParam);
void checkFreeSpace(void);
extern void unloadUsbDriver(void);
extern void loadUsbDriver(void);
extern void logUsbFxn(void);

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

/*********************************************
*
*   Check remaining free USB drive space
*
**********************************************/
void
checkFreeSpace(void)
{
    uint32_t totalSize = 0U;
    FATFS *pFatFs;

    f_getfree("0:", (DWORD *)&totalSize, &pFatFs);
    if ((totalSize*pFatFs->csize)/2 < MIN_DISK_SPACE) COIL_LOG_ENABLE.val = FALSE;
}


void logUsbFxn(void)
{
    if (USBHCDMain(USB_INSTANCE, g_ulMSCInstance) != 0) return;
    if (g_eState == STATE_DEVICE_ENUM)
    {   
        if (USBHMSCDriveReady(g_ulMSCInstance) != 0) usb_osalDelayMs(300);

        if (!g_fsHasOpened)
        {
            if (FATFS_open(0U, NULL, &fatfsHandle) == FR_OK) 
                g_fsHasOpened = 1;
            else 
            {
                g_fsHasOpened = 0;
                return;
            }
        }
    }
    else return;
    
   	/// STOP LOGGING?	
    if (!COIL_LOG_ENABLE.val)
   	{
        isLogging = FALSE;
		current_day = 99;
       	return;
   	}

    /// READ RTC
    Read_RTC(&tmp_sec, &tmp_min, &tmp_hr, &tmp_day, &tmp_mon, &tmp_yr);

    /// TAKES TIME TO RELOAD USB DRIVER
    if ((tmp_min == 59) && (tmp_sec < 30) && (isResetUsbDriver == FALSE)) return;

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

        FRESULT fresult;
		FILINFO fno;
        current_day = USB_RTC_DAY;

        logFile[0] = '\0';

        // get a file name
        sprintf(logFile,"0:PDI/LOG_%02d_%02d_20%02d.csv",USB_RTC_MON, USB_RTC_DAY, USB_RTC_YR); 

		// file already exists?
		if (f_stat(logFile, &fno) == FR_OK) return;

        // mkdir PDI
        fresult = f_mkdir("/PDI");
        if ((fresult != FR_EXIST) && (fresult != FR_OK)) return;

		// open file
        if (f_open(&fileWriteObject, logFile, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;

        // write header1
		sprintf(LOG_HEADER,"\nFirmware:,%5s\nSerial Number:,%5d\n\nDate,Time,Alarm,Stream,Watercut,Watercut_Raw,", FIRMWARE_VERSION, REG_SN_PIPE);
        if (f_puts(LOG_HEADER,&fileWriteObject) == EOF)
        {
            f_close(&fileWriteObject);
            return;
        } 
	    f_sync(&fileWriteObject);
        usb_osalDelayMs(500);

        // write header2
        sprintf(LOG_HEADER,"Temp(C),Avg_Temp(C),Temp_Adj,Freq(Mhz),Oil_Index,RP(V),Oil_PT,Oil_P0,Oil_P1,");
        if (f_puts(LOG_HEADER,&fileWriteObject) == EOF)
        {
            f_close(&fileWriteObject);
            return;
        } 
        f_sync(&fileWriteObject);
        usb_osalDelayMs(500);

        // write header3
        sprintf(LOG_HEADER,"Density,Oil_Freq_Low,Oil_Freq_Hi,AO_LRV,AO_URV,AO_MANUAL_VAL,Relay_Setpoint\n");
        if (f_puts(LOG_HEADER,&fileWriteObject) == EOF)
        {
            f_close(&fileWriteObject);
            return;
        } 
        f_sync(&fileWriteObject);
        usb_osalDelayMs(500);

        // close file
        f_close(&fileWriteObject);

        // check remaining disk space
        checkFreeSpace();

        return;
    }   

    char DATA_BUF[160];

    sprintf(DATA_BUF,"\n%02d-%02d-20%02d,%02d:%02d:%02d,%10d,%2.0f,%6.2f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%6.3f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%5.1f,%5.1f,%5.2f,%8.1f,",USB_RTC_MON,USB_RTC_DAY,USB_RTC_YR,USB_RTC_HR,USB_RTC_MIN,USB_RTC_SEC,DIAGNOSTICS,REG_STREAM.calc_val,REG_WATERCUT.calc_val,REG_WATERCUT_RAW.calc_val,REG_TEMP_USER.calc_val,REG_TEMP_AVG.calc_val,REG_TEMP_ADJUST.calc_val,REG_FREQ.calc_val,REG_OIL_INDEX.calc_val,REG_OIL_RP,REG_OIL_PT,REG_OIL_P0.calc_val,REG_OIL_P1.calc_val, REG_OIL_DENSITY.calc_val, REG_OIL_FREQ_LOW.calc_val, REG_OIL_FREQ_HIGH.calc_val, REG_AO_LRV.calc_val, REG_AO_URV.calc_val, REG_AO_MANUAL_VAL,REG_RELAY_SETPOINT.calc_val);

    // FILL DATA UPTO MAX_DATA_SIZE 2048 BYTES
    strcat(LOG_BUF,DATA_BUF);
    if (MAX_DATA_SIZE - strlen(LOG_BUF) > 160) return;
    while (MAX_DATA_SIZE > strlen(LOG_BUF)) strcat (LOG_BUF,",");

    if (f_open(&fileWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING) != FR_OK) 
    {
        current_day = 99;
        LOG_BUF[0] = '\0';
        return;
    }

    if (f_lseek(&fileWriteObject,f_size(&fileWriteObject)) != FR_OK)
    {
        f_close(&fileWriteObject);
        LOG_BUF[0] = '\0';
        return;
    }

    /// WRITE 
    if(f_puts(LOG_BUF,&fileWriteObject) != strlen(LOG_BUF))
    {
        f_close(&fileWriteObject);
        LOG_BUF[0] = '\0';
        return;
    }

    /// SYNC WITH USB DRIVE
    if (f_sync(&fileWriteObject) != FR_OK) 
    {
        f_close(&fileWriteObject);
        LOG_BUF[0] = '\0';
        return;
    }
   
    /// Needs time to sync because f_sync() is extremely slow) 
    usb_osalDelayMs(500); 

    /// CLOSE 
    f_close(&fileWriteObject); 

    /// RESET LOG_BUF   
    LOG_BUF[0] = '\0';

    /// RESET USB DRIVER EVERY HOUR 
    if (tmp_min == 58) isResetUsbDriver = TRUE;
    if (isResetUsbDriver)
    {
        if (tmp_min == 59)
        {
            isResetUsbDriver = FALSE;
            resetUsbDriver();
        }
    } 
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
    if (usb_handle == 0) return;

    // Setup the INT Controller
	usbHostIntrConfig (&usb_host_params);

    // Initialize the file system.
    FATFS_init();

    // Open an instance of the mass storage class driver.
    g_ulMSCInstance = USBHMSCDriveOpen(usb_host_params.instanceNo, 0, MSCCallback);

    usb_osalDelayMs(500);
}
