
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
#include "nandwriter.h"
#include "Menu.h"

#define USB3SS_EN
#define NANDWIDTH_16
#define OMAPL138_LCDK
#define USB_INSTANCE    0
#define MAX_HEAD_SIZE   110 
#define MAX_DATA_SIZE  	256
#define MAX_ENTRY_SIZE  50 
#define MAX_CSV_SIZE   	4096*3

static char LOG_HEAD[MAX_HEAD_SIZE];
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
    if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);

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
		if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
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

		if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
		return;
   	}   

	/// new DATA_BUF
	char *DATA_BUF;
    DATA_BUF=(char *)malloc(MAX_DATA_SIZE*sizeof(char));

    /// temp data holder
    char entry[MAX_ENTRY_SIZE];

	/// error checking
	if (DATA_BUF == NULL) return;

	/// get modbus data
	Swi_disable();

    /// read registers
    double LOG_REGS[] = {
        DIAGNOSTICS,
        REG_STREAM.calc_val,
        REG_WATERCUT.calc_val,
        REG_WATERCUT_RAW,
        REG_TEMP_USER.calc_val,
        REG_TEMP_AVG.calc_val,
        REG_TEMP_ADJUST.calc_val,
        REG_FREQ.calc_val,
        REG_OIL_INDEX.calc_val,
        REG_OIL_RP,
        REG_OIL_PT,
        REG_OIL_P0.calc_val,
        REG_OIL_P1.calc_val,
        REG_OIL_DENSITY.calc_val,
        REG_OIL_FREQ_LOW.calc_val,
        REG_OIL_FREQ_HIGH.calc_val,
        REG_AO_LRV.calc_val,
        REG_AO_URV.calc_val,
        REG_AO_MANUAL_VAL,
        REG_RELAY_SETPOINT.calc_val
    };

    int ARRAY_SIZE = sizeof LOG_REGS / sizeof LOG_REGS[0];
    int index;

    /// read integer type variables
    sprintf(DATA_BUF,"%02d-%02d-20%02d,%02d:%02d:%02d,",USB_RTC_MON,USB_RTC_DAY,USB_RTC_YR,USB_RTC_HR,USB_RTC_MIN,USB_RTC_SEC);

    /// read double type variables 
    for (index=0;index<ARRAY_SIZE;index++)
    {
        sprintf(entry,"%g,",LOG_REGS[index]);
        strcat(DATA_BUF,entry);
    }

    strcat(DATA_BUF,"\n");

   	Swi_enable();

	/// open
   	fresult = f_open(&logWriteObject, logFile, FA_WRITE | FA_OPEN_EXISTING);
   	if (fresult != FR_OK)
   	{
		f_close(&logWriteObject); 
       	stopAccessingUsb(fresult);
	    free(DATA_BUF);
       	return;
   	}

	/// append mode 
  	fresult = f_lseek(&logWriteObject,f_size(&logWriteObject));
  	if (fresult != FR_OK)
  	{
		f_close(&logWriteObject); 
       	stopAccessingUsb(fresult);
	    free(DATA_BUF);
       	return;
   	}

  	/// write
	if (f_puts(DATA_BUF,&logWriteObject) == EOF)
   	{
		f_close(&logWriteObject); 
   		stopAccessingUsb(FR_DISK_ERR);
	    free(DATA_BUF);
   		return;
   	}

	/// close
   	fresult = f_close(&logWriteObject);
	if (fresult != FR_OK)
   	{    
   		stopAccessingUsb(fresult);
	    free(DATA_BUF);
   		return;
   	} 

	if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
	free(DATA_BUF);

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
    static int tmp_sec, tmp_min, tmp_hr, tmp_day, tmp_mon, tmp_yr;
    Read_RTC(&tmp_sec, &tmp_min, &tmp_hr, &tmp_day, &tmp_mon, &tmp_yr);

	/// get file name
	(isPdiUpgradeMode) ? sprintf(csvFileName,"0:%s.csv",PDI_RAZOR_PROFILE) : sprintf(csvFileName,"0:P%06d.csv",REG_SN_PIPE);

	usb_osalDelayMs(500);

	/// open file
	if (f_open(&csvWriteObject, csvFileName, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) 
	{
		isUpgradeFirmware = FALSE;
		return FALSE;;
	}

    char lcdModelCode[] = "INCDYNAMICSPHASE";

    for (i=0;i<4;i++)
    {
        lcdModelCode[i*4+3] = (REG_MODEL_CODE[i] >> 24) & 0xFF;
        lcdModelCode[i*4+2] = (REG_MODEL_CODE[i] >> 16) & 0xFF;
        lcdModelCode[i*4+1] = (REG_MODEL_CODE[i] >> 8)  & 0xFF;
        lcdModelCode[i*4+0] = (REG_MODEL_CODE[i] >> 0)  & 0xFF;
    }
     
    /// status header
    sprintf(CSV_BUF+strlen(CSV_BUF),"# RAZOR CONFIGURATION FILE\n"); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"# FIRMWARE VERSION: %s\n",FIRMWARE_VERSION);
    sprintf(CSV_BUF+strlen(CSV_BUF),"# MODEL CODE: %s\n",lcdModelCode);
    sprintf(CSV_BUF+strlen(CSV_BUF),"# SERIAL NUMBER: %d\n",REG_SN_PIPE);
    sprintf(CSV_BUF+strlen(CSV_BUF),"# %02d:%02d %02d/%02d/20%02d\n\n",tmp_hr,tmp_min,tmp_mon,tmp_day,tmp_yr);

	/// integer
    sprintf(CSV_BUF+strlen(CSV_BUF),"Serial,,201,int,1,RW,1,%d\n",REG_SN_PIPE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Dampen,,203,int,1,RW,1,%d\n",REG_AO_DAMPEN); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Slave Address,,204,int,1,RW,1,%d\n",REG_SLAVE_ADDRESS); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stop Bits,,205,int,1,RW,1,%d\n",REG_STOP_BITS); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Mode,,206,int,1,RW,1,%d\n",REG_DENSITY_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Model Code,,219,int,1,RW,1,%s\n",lcdModelCode); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Logging Period,,223,int,1,RW,1,%d\n",REG_LOGGING_PERIOD); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Alarm Mode,,227,int,1,RW,1,%d\n",REG_AO_ALARM_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Phase Hold Over,,228,int,1,RW,1,%d\n",REG_PHASE_HOLD_CYCLES); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Delay,,229,int,1,RW,1,%d\n",REG_RELAY_DELAY); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Mode,,230,int,1,RW,1,%d\n",REG_AO_MODE); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Correction Mode,,231,int,1,RW,1,%d\n",REG_OIL_DENS_CORR_MODE);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Mode,,232,int,1,RW,1,%d\n",REG_RELAY_MODE); 

	/// float or double
	sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Adjust,,15,float,1,RW,1,%015.7f\n",REG_OIL_ADJUST.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Temp Adjust,,31,float,1,RW,1,%015.7f\n",REG_TEMP_ADJUST.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Proc Avg,,35,float,1,RW,1,%015.7f\n",REG_PROC_AVGING.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Index,,37,float,1,RW,1,%015.7f\n",REG_OIL_INDEX.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P0,,39,float,1,RW,1,%015.7f\n",REG_OIL_P0.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil P1,,41,float,1,RW,1,%015.7f\n",REG_OIL_P1.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Low,,43,float,1,RW,1,%015.7f\n",REG_OIL_FREQ_LOW.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil High,,45,float,1,RW,1,%015.7f\n",REG_OIL_FREQ_HIGH.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Sample Period,,47,float,1,RW,1,%05.1f\n",REG_SAMPLE_PERIOD.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO LRV,,49,float,1,RW,1,%015.7f\n",REG_AO_LRV.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO URV,,51,float,1,RW,1,%015.7f\n",REG_AO_URV.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Baud Rate,,55,float,1,RW,1,%010.1f\n",REG_BAUD_RATE.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Calc Max,,67,float,1,RW,1,%015.7f\n",REG_OIL_CALC_MAX); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Dual Curve Cutoff,,69,float,1,RW,1,%015.7f\n",REG_OIL_PHASE_CUTOFF);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stream,,73,float,1,RW,1,%010.1f\n",REG_STREAM.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim Low,,107,float,1,RW,1,%015.7f\n",REG_AO_TRIMLO); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AO Trim High,,109,float,1,RW,1,%015.7f\n",REG_AO_TRIMHI); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Adj,,111,float,1,RW,1,%015.7f\n",REG_DENSITY_ADJ);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Density Unit,,113,float,1,RW,1,%010.0f\n",REG_DENSITY_UNIT.val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"D3,,117,float,1,RW,1,%015.7f\n",REG_DENSITY_D3.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D2,,119,float,1,RW,1,%015.7f\n",REG_DENSITY_D2.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D1,,121,float,1,RW,1,%015.7f\n",REG_DENSITY_D1.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"D0,,123,float,1,RW,1,%015.7f\n",REG_DENSITY_D0.calc_val);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Dens Calibration Val,,125,float,1,RW,1,%015.7f\n",REG_DENSITY_CAL_VAL.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Relay Setpoint,,151,float,1,RW,1,%015.7f\n",REG_RELAY_SETPOINT.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density Manual,,161,float,1,RW,1,%015.7f\n",REG_OIL_DENSITY_MANUAL); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density AI LRV,,163,float,1,RW,1,%015.7f\n",REG_OIL_DENSITY_AI_LRV.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Density AI URV,,165,float,1,RW,1,%015.7f\n",REG_OIL_DENSITY_AI_URV.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim Low,,169,float,1,RW,1,%015.7f\n",REG_AI_TRIMLO); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"AI Trim High,,171,float,1,RW,1,%015.7f\n",REG_AI_TRIMHI); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T0,,179,float,1,RW,1,%015.7f\n",REG_OIL_T0.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil T1,,181,float,1,RW,1,%015.7f\n",REG_OIL_T1.calc_val); 
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Temp Adj,,781,float,1,RW,1,%015.7f\n",PDI_TEMP_ADJ);
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Freq F0,,783,float,1,RW,1,%015.7f\n",PDI_FREQ_F0);
    sprintf(CSV_BUF+strlen(CSV_BUF),"PDI Freq F1,,785,float,1,RW,1,%015.7f\n",PDI_FREQ_F1);

	/// extended 60K
    sprintf(CSV_BUF+strlen(CSV_BUF),"Number of Oil Temperature Curves,,60001,float,1,RW,1,%015.7f\n",REG_TEMP_OIL_NUM_CURVES);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Temperature List,,60003,float,1,RW,10,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f,%015.7f\n",REG_TEMPS_OIL[0],REG_TEMPS_OIL[1],REG_TEMPS_OIL[2],REG_TEMPS_OIL[3],REG_TEMPS_OIL[4],REG_TEMPS_OIL[5],REG_TEMPS_OIL[6],REG_TEMPS_OIL[7],REG_TEMPS_OIL[8],REG_TEMPS_OIL[9]);

	for (data_index=0;data_index<10;data_index++)
    sprintf(CSV_BUF+strlen(CSV_BUF),"Oil Curve %d,,%d,float,1,RW,4,%015.7f,%015.7f,%015.7f,%015.7f\n",data_index,60023+data_index*8,REG_COEFFS_TEMP_OIL[data_index][0],REG_COEFFS_TEMP_OIL[data_index][1],REG_COEFFS_TEMP_OIL[data_index][2],REG_COEFFS_TEMP_OIL[data_index][3]);

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
    //sprintf(CSV_BUF+strlen(CSV_BUF),"Stream Oil Adjust %d,,%d,float,1,RW,1,%015.7f\n",data_index,63647+4*data_index,STREAM_OIL_ADJUST[data_index]);
    sprintf(CSV_BUF+strlen(CSV_BUF),"Stream Oil Adjust %d,,%d,float,1,RW,1,%015.7f\n",data_index,63647+2*data_index,STREAM_OIL_ADJUST[data_index]);

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
	for (i=0;i<1000;i++) displayLcd("     Loading    ",LCD1);

	/// close file
	fr = f_close(&csvWriteObject);
	if (fr != FR_OK)
	{
		resetUsbDriver();
		stopAccessingUsb(fr);
		return FALSE;
	}

	/// set global var true
    isCsvDownloadSuccess = TRUE;
    isCsvUploadSuccess = FALSE;
    
	if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
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
		if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
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

	///Swi_disable();

	/// do not upgrade firmware after profiling
	isUpgradeFirmware = FALSE;

	/// read line
    while (f_gets(line, sizeof(line), &fil)) 
	{
		int i = 0; 
        char * splitValue[17];
        float value[17];

		/// remove trailing \n
		line[strcspn( line,"\n")] = '\0';

		/// split line
        char* ptr = strtok(line, ",");
        while (ptr != NULL)
        {   
            splitValue[i] = ptr;
            value[i] = atof(ptr);
            ptr = strtok(NULL, ",");

            i++; 
        } 

	    if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);

        /// start read values
		id = (int)value[1];

        /// update registers 
        if (id==219) // <------- MODEL_CODE
        {
            char model_code[MAX_LCD_WIDTH];
            int* model_code_int;
            sprintf(model_code,splitValue[6]); //default model code
            model_code_int = (int*)model_code;
            for (i=0;i<4;i++) REG_MODEL_CODE[i] = model_code_int[i];
        }
		else if ((id>0) && (id<1000)) updateVars(id,value[6]);
		else if (id==60001) REG_TEMP_OIL_NUM_CURVES = value[6]; 
		else if (id==60003) for (i=0;i<10;i++) REG_TEMPS_OIL[i] = value[i+6];
        else if ((id>60022) && (id<60096)) for (i=0;i<4;i++) REG_COEFFS_TEMP_OIL[(id-60023)/8][i] = value[i+6];
        else if ((id>63646) && (id<63766)) STREAM_OIL_ADJUST[(id-63647)/2] = value[6];

        /// reset line[]
		for (i=0;i<10;i++) line[0] = '\0';

		/// print status -- we use print as an intended "delay"
		if (isPdiUpgradeMode) 
		{
			LCD_setcursor(0,0);
			displayLcd(" PROFILE UPLOAD ",0);
		}

		if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
		displayLcd("     Loading    ",LCD1);
	}	

	/// close file
	f_close(&fil);

	/// update FACTORY DEFAULT
   	storeUserDataToFactoryDefault();

    /// reset CSV triggers
	isCsvUploadSuccess = TRUE;
    isCsvDownloadSuccess = FALSE;

	/// delete PDI_RAZOR_PROFILE
	if (isPdiUpgradeMode) 
	{
	    if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
		displayLcd(" PROFILE UPLOADED ",0);
		f_unlink(csvFileName);
	}

    while (1) 
	{
		if (isWatchDogEnabled) TimerWatchdogReactivate(CSL_TMR_1_REGS);
		displayLcd("   REMOVE USB   ", 1);
	}

	return TRUE;
}
