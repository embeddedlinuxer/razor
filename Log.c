
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
#include <ti/csl/soc.h>
#include <ti/csl/cslr_usb.h>
#include <ti/csl/cslr_syscfg.h>
#include <ti/fs/fatfs/diskio.h>
#include <ti/fs/fatfs/FATFS.h>
#include "ti/csl/arch/arm9/V0/csl_cp15.h"
#include "fatfs_port_usbmsc.h"
#include "fs_shell_app_utils.h"
#include "hw_soc.h"
#include "usblib.h"
#include "usbhost.h"
#include "usbhmsc.h"
#include "usb_drv.h"
#include "usb_osal.h"
#include "timer.h"
#include "types.h"
#include "Globals.h"
#include "Variable.h"
#include "Menu.h"
#include "nandwriter.h"

#define USB3SS_EN
#define NANDWIDTH_16
#define OMAPL138_LCDK
#define USB_INSTANCE    0
#define MAX_HEAD_SIZE   110 
#define MAX_DATA_SIZE  	256
#define MAX_BUF_SIZE	4096
#define MAX_CSV_SIZE   	4096*3

static char LOG_HEAD[MAX_HEAD_SIZE];
static char LOG_BUF[MAX_BUF_SIZE];
static char logFile[] = "0:PDI/LOG_01_01_2019.csv";
static USB_Handle usb_handle;
static USB_Params usb_host_params;
static FIL logWriteObject;
unsigned int g_ulMSCInstance = 0; 

// TIME VARS
static Uint8 current_day = 99;
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
			usbStatus = 0;
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


void setupMMUForUSB()
{
#if defined(evmAM437x)
    /* this is the USB IP memory. Map it just in case the MMU example has not done so */
    static mmuMemRegionConfig_t regionDev =
            {
                0x48380000,     /* USB0 & USB1 */
                2,              /* Number of pages */
                1U*MEM_SIZE_MB, /* Page size - 1MB */
                MMU_MEM_ATTR_DEVICE_SHAREABLE,
                MMU_CACHE_POLICY_WB_WA, /* Inner - Invalid here */
                MMU_CACHE_POLICY_WB_WA, /* Outer - Invalid here */
                MMU_ACCESS_CTRL_PRV_RW_USR_RW,
                FALSE /* Non Secure memory */
            };

    /* this is the noncached area that the DWC driver uses */
    static mmuMemRegionConfig_t regionDDR =
            {
                0xA0000000,     /* APP_UNCACHED_DATA_BLK3_MEM */
                2,              /* Number of pages */
                1U*MEM_SIZE_MB, /* Page size - 1MB */
                MMU_MEM_ATTR_DEVICE_SHAREABLE,
                MMU_CACHE_POLICY_WB_WA, /* Inner - Invalid here */
                MMU_CACHE_POLICY_WB_WA, /* Outer - Invalid here */
                MMU_ACCESS_CTRL_PRV_RW_USR_RW,
                FALSE /* Non Secure memory */
            };

    MMUMemRegionMap(&regionDev, (uint32_t*)pageTable);
    MMUMemRegionMap(&regionDDR, (uint32_t*)pageTable);

#endif

#if defined(BUILD_ARM) && (defined(SOC_OMAPL137) || defined(SOC_OMAPL138))
    /* Sets up 'Level 1" page table entries.*/
    uint32_t index;
    for(index = 0; index < (4*1024); index++)
    {
        pageTable[index] = (index << 20) | 0x00000C12;
    }

    /* Disable Instruction Cache*/
    CSL_CP15ICacheDisable();

    /* Configures translation table base register
    * with pagetable base address.
    */
    CSL_CP15TtbSet((unsigned int )pageTable);

    /* Enables MMU */
    CSL_CP15MMUEnable();

    /* Enable Data Cache */
    CSL_CP15DCacheEnable();
#endif
}

void loadUsbDriver(void)
{
	/* set up MMU page for APP_UNCACHED_DATA_BLK3_MEM and for USB */
    setupMMUForUSB();

	USB_Config* usb_config;

    usb_host_params.usbMode      = USB_HOST_MSC_MODE;
    usb_host_params.instanceNo   = USB_INSTANCE;
    usb_handle = USB_open(usb_host_params.instanceNo, &usb_host_params);

    // failed to open
    if (usb_handle == 0) return;

    // Setup the INT Controller
	usbHostIntrConfig (&usb_host_params);

	/// enable usb 3.0 super speed && DMA MODE
    usb_config->usb30Enabled = TRUE;
	usb_handle->usb30Enabled = TRUE;
	usb_handle->dmaEnabled = TRUE;
    usb_handle->handleCppiDmaInApp = TRUE;

    // Initialize the file system.
    FATFS_init();

    // Open an instance of the mass storage class driver.
    g_ulMSCInstance = USBHMSCDriveOpen(usb_host_params.instanceNo, 0, MSCCallback);

    // watchdog timer reset
    TimerWatchdogReactivate(CSL_TMR_1_REGS);

    usb_osalDelayMs(500);
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
	csvCounter = 0;
}

void resetUsbStaticVars(void)
{
	current_day = 99;
	usbStatus = 0;
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
	static int stop_usb = 0;
    if (stop_usb > REG_USB_TRY)
    {
        stopAccessingUsb(FR_TIMEOUT);
        stop_usb = 0;
    }
    else stop_usb++;

    if (USBHCDMain(USB_INSTANCE, g_ulMSCInstance) != 0) return FALSE;
    else
    {
        if (g_eState == STATE_DEVICE_ENUM)
        {
            if (USBHMSCDriveReady(g_ulMSCInstance) != 0) return FALSE;

            if (!g_fsHasOpened)
            {
                if (FATFS_open(0U, NULL, &fatfsHandle) != FR_OK) return FALSE;
                else g_fsHasOpened = 1;
            }

            stop_usb = 0;
            return TRUE;
        }

        return FALSE;
    }
}


void logData(void)
{
    static FRESULT fresult;
	static int time_counter = 1;
	static int prev_sec = 0;
	int i = 0;
	
   	/// read rtc
   	Read_RTC(&tmp_sec, &tmp_min, &tmp_hr, &tmp_day, &tmp_mon, &tmp_yr);

	/// valid timestamp?
   	if (tmp_sec == prev_sec) return;
	else 
	{
		prev_sec = tmp_sec;
		time_counter++;
	}

	if (time_counter % REG_LOGGING_PERIOD != 0) return;
	else time_counter = 0;

	/// UPDATE TIME	
	USB_RTC_SEC = tmp_sec;
	if (USB_RTC_MIN != tmp_min) USB_RTC_MIN = tmp_min;
	if (USB_RTC_HR != tmp_hr)   USB_RTC_HR = tmp_hr;
	if (USB_RTC_DAY != tmp_day) USB_RTC_DAY = tmp_day;
	if (USB_RTC_MON != tmp_mon) USB_RTC_MON = tmp_mon;
	if (USB_RTC_YR != tmp_yr)   USB_RTC_YR = tmp_yr;

	/// check usb driver
	if (!isUsbActive()) 
	{
		TimerWatchdogReactivate(CSL_TMR_1_REGS);
		return;
	}

   	/// A NEW FILE? 
   	if (current_day != USB_RTC_DAY) 
   	{   
       	current_day = USB_RTC_DAY;

		// mkdir PDI
        fresult = f_mkdir("0:/PDI");
        if ((fresult != FR_EXIST) && (fresult != FR_OK))
        {
            stopAccessingUsb(fresult);
            return;
        }

        // get a file name
        sprintf(logFile,"0:/PDI/LOG_%02d_%02d_20%02d.csv",USB_RTC_MON, USB_RTC_DAY, USB_RTC_YR); 

        if (f_open(&logWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING) == FR_OK) 
        {
            fresult = f_close(&logWriteObject);
            if (fresult == FR_OK) return;
        }

		/// open file
       	fresult = f_open(&logWriteObject, logFile, FA_WRITE | FA_CREATE_ALWAYS);
       	if (fresult != FR_OK) 
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header1
		sprintf(LOG_HEAD,"\nFirmware:,%5s\nSerial Number:,%5d\n\nDate,Time,Alarm,Stream,Watercut,Watercut_Raw,", FIRMWARE_VERSION, REG_SN_PIPE);

       	if (f_puts(LOG_HEAD,&logWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&logWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header2
       	sprintf(LOG_HEAD,"Temp(C),Avg_Temp(C),Temp_Adj,Freq(Mhz),Oil_Index,RP(V),Oil_PT,Oil_P0,Oil_P1,");

       	if (f_puts(LOG_HEAD,&logWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&logWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// write header3
       	sprintf(LOG_HEAD,"Density,Oil_Freq_Low,Oil_Freq_Hi,AO_LRV,AO_URV,AO_MANUAL_VAL,Relay_Setpoint\n");

       	if (f_puts(LOG_HEAD,&logWriteObject) == EOF) 
       	{
           	stopAccessingUsb(FR_DISK_ERR);
           	return;
       	}

       	fresult = f_sync(&logWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

       	/// close file
       	fresult = f_close(&logWriteObject);
       	if (fresult != FR_OK)
       	{
           	stopAccessingUsb(fresult);
           	return;
       	}

		TimerWatchdogReactivate(CSL_TMR_1_REGS);
		return;
   	}   

	/// checking highspeed mode for debugging purpose only
    //if (CSL_FEXT(usbRegs->POWER, USB_OTG_POWER_HSMODE)) printf ("USB HIGH SPEED ENABLED\n");
    //else printf ("USB HIGH SPEED NOT ENABLED\n");
        
	/// new DATA_BUF
	char *DATA_BUF;
    DATA_BUF=(char *)malloc(MAX_DATA_SIZE*sizeof(char));

	/// error checking
	if (DATA_BUF == NULL) return;

	/// get modbus data
	Swi_disable();

	i = snprintf(DATA_BUF,MAX_DATA_SIZE,"%02d-%02d-20%02d,%02d:%02d:%02d,%10d,%2.0f,%6.2f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%6.3f,%5.1f,%5.1f,%5.1f,%5.1f,%6.3f,%6.3f,%5.1f,%5.1f,%5.2f,%8.1f,\n",USB_RTC_MON,USB_RTC_DAY,USB_RTC_YR,USB_RTC_HR,USB_RTC_MIN,USB_RTC_SEC,DIAGNOSTICS,REG_STREAM.calc_val,REG_WATERCUT.calc_val,REG_WATERCUT_RAW,REG_TEMP_USER.calc_val,REG_TEMP_AVG.calc_val,REG_TEMP_ADJUST.calc_val,REG_FREQ.calc_val,REG_OIL_INDEX.calc_val,REG_OIL_RP,REG_OIL_PT,REG_OIL_P0.calc_val,REG_OIL_P1.calc_val, REG_OIL_DENSITY.calc_val, REG_OIL_FREQ_LOW.calc_val, REG_OIL_FREQ_HIGH.calc_val, REG_AO_LRV.calc_val, REG_AO_URV.calc_val, REG_AO_MANUAL_VAL,REG_RELAY_SETPOINT.calc_val);

   	Swi_enable();

	if ((i>200) || (i<150))
	{
		free(DATA_BUF);
		return;
	}

	/// check max_buf_size = 4096
	if (MAX_BUF_SIZE > (strlen(LOG_BUF) + strlen(DATA_BUF))) 
	{
		TimerWatchdogReactivate(CSL_TMR_1_REGS);
		strcat(LOG_BUF,DATA_BUF);
		free(DATA_BUF);
		return;
	}
	else free(DATA_BUF);

	/// open
   	fresult = f_open(&logWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
   	if (fresult != FR_OK)
   	{
		f_close(&logWriteObject); 
       	stopAccessingUsb(fresult);
       	return;
   	}

	/// append mode 
  	fresult = f_lseek(&logWriteObject,f_size(&logWriteObject));
  	if (fresult != FR_OK)
  	{
		f_close(&logWriteObject); 
       	stopAccessingUsb(fresult);
       	return;
   	}

  	/// write
	if (f_puts(LOG_BUF,&logWriteObject) == EOF)
   	{
		f_close(&logWriteObject); 
   		stopAccessingUsb(FR_DISK_ERR);
   		return;
   	}

	/// close
   	fresult = f_close(&logWriteObject);
	if (fresult != FR_OK)
   	{    
   		stopAccessingUsb(fresult);
   		return;
   	} 

	TimerWatchdogReactivate(CSL_TMR_1_REGS);
	LOG_BUF[0] = '\0';
   	return;
}


BOOL downloadCsv(void)
{
	if (!isUsbActive()) return FALSE;
	isDownloadCsv = FALSE;

	FRESULT fr;	
	FIL csvWriteObject;
	char csvFileName[50] = {""};
	char CSV_BUF[MAX_CSV_SIZE] = {""};
	int i, data_index;

	/// get file name
	(isPdiUpgradeMode) ? sprintf(csvFileName,"0:%s.csv",PDI_RAZOR_PROFILE) : sprintf(csvFileName,"0:P%06d.csv",REG_SN_PIPE);

	usb_osalDelayMs(500);

	/// open file
	if (f_open(&csvWriteObject, csvFileName, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) 
	{
		isUpgradeFirmware = FALSE;
		return FALSE;;
	}

	/// integer
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
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Unit,,113,float,1,RW,1,%10.0f,\n",REG_DENSITY_UNIT.val); 
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
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve %d,,%d,float,1,RW,4,%15.7f,%15.7f,%15.7f,%15.7f\n",data_index,60023+data_index*8,REG_COEFFS_TEMP_OIL[data_index][0],REG_COEFFS_TEMP_OIL[data_index][1],REG_COEFFS_TEMP_OIL[data_index][2],REG_COEFFS_TEMP_OIL[data_index][3]);

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
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stream Oil Adjust %d,,%d,float,1,RW,1,%15.7f\n",data_index,63647+4*data_index,STREAM_OIL_ADJUST[data_index]);

	/// write
	fr = f_puts(CSV_BUF,&csvWriteObject);
	if (fr == EOF)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		return FALSE;
	}

	fr = f_sync(&csvWriteObject);
	if (fr != FR_OK)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		return FALSE;
	}
	for (i=0;i<1000;i++) displayLcd("    Loading...  ",LCD1);

	/// close file
	fr = f_close(&csvWriteObject);
	if (fr != FR_OK)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		return FALSE;
	}
	printf("closing file%d\n",i);

	/// set global var true
	printf("set flags\n");
    isCsvDownloadSuccess = TRUE;
    isCsvUploadSuccess = FALSE;
    
	TimerWatchdogReactivate(CSL_TMR_1_REGS);
    return TRUE;
}


void scanCsvFiles(void)
{
	if (!isUsbActive()) return;
	isScanCsvFiles = FALSE;

	int i;
	FRESULT res;
    static DIR dir;
    static FILINFO fno;
	const char path[] = "0:";
	csvCounter = 0;
	CSV_FILES[0] = '\0';

	/// disable all interrupts
	Swi_disable();
	for (i=0;i<10;i++) printf("Swi_disable...");

	/// opendir
	if (f_opendir(&dir, path) != FR_OK) 
	{
		Swi_enable();
		return;
	}
	for (i=0;i<10;i++) printf("f_opendir...");

	/// read file names
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
		TimerWatchdogReactivate(CSL_TMR_1_REGS);
    }
	for (i=0;i<10;i++) printf("read files...");

	/// close dir
	f_closedir(&dir);
	for (i=0;i<10;i++) printf("Closing dir...");

	/// enable all interrupts back
	Swi_enable();
	for (i=0;i<10;i++) printf("Swi_enable...");

    return;
}


BOOL uploadCsv(void)
{
	if (!isUsbActive()) return FALSE;
	isUploadCsv = FALSE;

	FIL fil;
	int i, id;
	char line[1024] = {""};
	char csvFileName[50] = {""};

	/// get file name
	(isPdiUpgradeMode) ? sprintf(csvFileName,"0:%s.csv",PDI_RAZOR_PROFILE) : sprintf(csvFileName,"0:%s.csv",CSV_FILES);

	/// open file
	if (f_open(&fil, csvFileName, FA_READ) != FR_OK) return FALSE;
	printf("Open file...");

	Swi_disable();
	printf("Swi_disable...");

	/// do not upgrade firmware after profiling
	isUpgradeFirmware = FALSE;

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


		/// 1-dimensional array
		id = atoi(regid);
		if ((id>0) && (id<1000)) updateVars(id,fvalue);

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
		else if (strcmp(regid, "60071") == 0)
		{
			REG_COEFFS_TEMP_OIL[6][0] = fvalue;
			REG_COEFFS_TEMP_OIL[6][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[6][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[6][3] = fvalue3;
		}
		else if (strcmp(regid, "60079") == 0)
		{
			REG_COEFFS_TEMP_OIL[7][0] = fvalue;
			REG_COEFFS_TEMP_OIL[7][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[7][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[7][3] = fvalue3;
		}
		else if (strcmp(regid, "60087") == 0)
		{
			REG_COEFFS_TEMP_OIL[8][0] = fvalue;
			REG_COEFFS_TEMP_OIL[8][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[8][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[8][3] = fvalue3;
		}
		else if (strcmp(regid, "60095") == 0)
		{
			REG_COEFFS_TEMP_OIL[9][0] = fvalue;
			REG_COEFFS_TEMP_OIL[9][1] = fvalue1;
			REG_COEFFS_TEMP_OIL[9][2] = fvalue2;
			REG_COEFFS_TEMP_OIL[9][3] = fvalue3;
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

		for (i=0;i<10;i++) line[0] = '\0';

		/// print status -- we use print as an intended "delay"
		if (isPdiUpgradeMode) 
		{
			LCD_setcursor(0,0);
			displayLcd(" PROFILE UPLOAD ",0);
		}

		TimerWatchdogReactivate(CSL_TMR_1_REGS);
		displayLcd("    Loading...  ",1);
	}	

	/// close file
	f_close(&fil);
	for (i=0;i<10;i++) printf("Closing%d...\n",i);

	Swi_enable();
	for (i=0;i<10;i++) printf("Swi_enable%d\n",i);

	/// update FACTORY DEFAULT
   	storeUserDataToFactoryDefault();
	Swi_post(Swi_writeNand);	
	isCsvUploadSuccess = TRUE;
    isCsvDownloadSuccess = FALSE;
	for (i=0;i<10;i++) printf("Swi_post%d\n",i);

	/// delete PDI_RAZOR_PROFILE
	if (isPdiUpgradeMode) 
	{
		displayLcd("PROFILE UPLOADED",0);
		f_unlink(csvFileName);
	}

    while (1) 
	{
		TimerWatchdogReactivate(CSL_TMR_1_REGS);
		displayLcd("   REMOVE USB   ", 1);
	}

	return TRUE;
}
