/*
 *
 * bbu_driver.c
 * Description:
 * Kernel driver and Kernel thread for BBU
 *
 * Copyright (C) 2008 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*! \file bbu_driver.c
    \brief Kernel driver and Kernel thread for BBU
*/

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <pal.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>

#include "_tistdtypes.h"
#include "bbu_persistent_data.h"
#include "bbu_driver.h"
#include "bbu_hw_api.h"
#include "bbu_board_def.h"
#include "bbu_float_support.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/
/*! \def BBU_ADC_DATA_MAX_FAILURES
 *  \brief bbu max number of adc samples failures
 */
#define BBU_ADC_DATA_MAX_FAILURES 10

/*! \def BBU_CDEV_MIN_MINOR, BBU_CDEV_NUM_DEVICES, BBU_CDEV_NAME
 *  \brief bbu character device info
 */
#define BBU_CDEV_MIN_MINOR       0
#define BBU_CDEV_NUM_DEVICES     1
#define BBU_CDEV_NAME            "bbu_driver"

/*! \def BBU_TO_P5_DATA, BBU_FROM_P5_DATA
 *  \brief direction of writing/reading persistent data to/from memory on P5
 */
#define BBU_TO_P5_DATA (0)
#define BBU_FROM_P5_DATA (1)

/*! \variable BbuIoctlCmdDisplay
*  /brief   table of bbu ioctl cmds and cmd names
*/
#define BBU_IOCTL_CMD(cmd, cmdstring) cmdstring,
const Char *BbuIoctlCmdDisplay[] =
{
    BBU_IOCTL_CMDS
    ""
};

#undef BBU_IOCTL_CMD

/*! \variable BbuSafetyViolationCauses
*  /brief   table of  bbu violation causes
*/
#define BBU_SAFETY_VIOL(cause, causeString) causeString,

const Char *BbuSafetyViolationCauses[] =
{
    BBU_SAFETY_VIOLATION_REASONS
    ""
};

#undef BBU_SAFETY_VIOL

/*! \def BBU_RESET_SAFETY_VIOLATION
 *  \brief macro to reset violation cause in violation list
 */
#define BBU_RESET_SAFETY_VIOLATION(cause) ( bbuDriverLocalData.safetyViolation.safetyViolationList[cause] = False)

/*! \struct typedef struct BbuDriverData_t
 *  /brief all the local data that bbu driver uses
 */
typedef struct
{
    Bbu_BatteryPersistentData_t bbuBatPersistentData; /* mirror of data saved in P5 */
    BbuAdcStateData_t   bbuAdcData;                   /* last scanning results and the current accumulator*/
    BbuPwmParameters_t  bbuPwmData;                   /* pwm relevant data */
    wait_queue_head_t   bbuRqueue[2];                 /* used for synchronizing between thread and ioctl looking for safety violation */

    struct semaphore    bbuPersistDataSema;          /* used for updating persistent data */
    struct semaphore    bbuDataSema;                 /* used for synchronizing between thread and ioctls*/
    Uint32              currentViolationCounter;     /* counts number of negative current limit violations */
    Uint32              calibrationTimerCounter;     /* counts number of periodic kernel thread passes before calibration needs to be done*/
    Uint32              calibrationTimerMaxCount;    /* max number of kernel thread passes before calibration needs to be done*/
    Uint32              temperatureValidCounter;     /* counter of valid temperature measurements after an invalid was detected,
                                                         in order to remove alarm*/
    BbuSafetyViolationData_t    safetyViolation;     /* safety violation data*/
} BbuDriverData_t;


/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/

static Int32 bbuCreateCharDev(void);
static void bbuDeleteCdev(void);
static ssize_t bbuRead(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t bbuWrite(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static Int32 bbuOpen(struct inode *inode, struct file *filp);
static long bbuIoctl(struct file *file, Uint32 cmd, unsigned long arg);
static Int32 bbuRelease(struct inode *inode, struct file *filp);

static Int32 bbuCopyPersistentData(Int16 direction);

/* kernel thread main*/
static Int32 BbuPeriodicHwScan(void *data);

static Int32 bbuStartPeriodicScan(void);
static Int32 bbuStopPeriodicScan(void);
static Int32 bbuIoctlGetUserData(Uint32 userData, Int32 *localData, Uint32 cmd);
static Int32 bbuSetWorkingParamsData(BbuIoctlDataS_t *data);
static Int32 bbuGetWorkingData(BbuIoctlDataS_t *data);
static Int32 bbuGetAdcData(BbuIoctlDataS_t *data);
static Int32 bbuSetPwmData(BbuIoctlDataS_t *data);
static Int32 bbuGetPwmData(BbuIoctlDataS_t *data);
static Int32 bbuEnableBatteryCmd(Int32 data);
static Int32 BbuGetOneShot(BbuIoctlDataS_t *data);
static Int32 BbuTreatSafetyLimits(Bbu_OneShotDataType_t *adcDataP);
static void  BbuNotifySafetyViolation(BbuSafetyViolationCauses_e cause);
static Int32 BbuTreatViolationNotifyRequest(BbuIoctlDataS_t *data);
static Int32 BbuDbgCalcFLoat(BbuIoctlDataS_t *data);
static Int32 BbuCorrectWorkingParams(void);
static Int32 BbuGetHwEnabledBat(BbuIoctlDataS_t *data);
static Int32 BbuValidateAdcData(Bbu_OneShotDataType_t *adcDataP);

static void bbuInitData(void);
static void bbuHwDisable(void);

/* mutex */

static Int32 bbuDriverGetDataAccess(void);
static Int32 bbuDriverReleaseDataAccess(void);
static Int32 bbuGetAccessToP5Mem(void);
static Int32 bbuFreeAccessToP5Mem(void);

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/
/*! \variable bbuAdcDataDebug
*  /brief   indicates the adc last sample
*/
static Bbu_OneShotDataType_t bbuAdcDataDebug;

/*! \variable adcDebugCounter
*  /brief   indicates number of samples failures
*/
static Uint32 adcDebugCounter = 0;

/*! \variable bbuWorkDataInitialized
*  /brief   indicates working data has been initialized, kernel thread may start running
*/
static atomic_t     bbuWorkDataInitialized = ATOMIC_INIT(False);

/*! \variable bbuDriverLocalData
*  /brief   internal data used by driver
*/
static BbuDriverData_t bbuDriverLocalData;

/*! \variable bbuCdev, bbuDevNumbers, bbuCdevFops
*  /brief   bbu character device and related data
*/
static struct cdev *bbuCdev = NULL;
static dev_t        bbuDevNumbers;
static struct file_operations bbuCdevFops =
{
    .owner = THIS_MODULE,
    .unlocked_ioctl = bbuIoctl,
    .read =  bbuRead, /* adc read and p5 sram read - can be the same read or need ioctl?*/
    .write = bbuWrite, /* pwm data update and p5 sram write - can be the same or need ioctl?*/
    .open  = bbuOpen,
    .release = bbuRelease,
};

/*! \variable bbuKernThread
*  /brief  Kernel thread that periodically scans input , performs calibration and tests safety
*/
static struct task_struct *bbuKernThread = NULL;

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/



/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/

/**************************************************************************/
/*! \fn  Int32 bbuDriverInit(void)
 **************************************************************************
 *  \brief BBU driver kernel module init function
 *  \return  0 or error code
 */
static Int32 bbuDriverInit(void)
{
    Int32 retVal;

    bbuInitData();/* todo maybe need be moved after check of support*/

    if (!Bbu_IsHwEnabled())
    {
        printk(KERN_ERR "BBU not supported on board\n");
        return -EFAULT;
    }

    retVal = bbuCreateCharDev();
    if (retVal != 0)
    {
        printk(KERN_ERR "Failed to create BBU char device\n");
        return retVal;
    }
    BBU_KDEBUG_PRINT(KERN_INFO, "Succeeded to create BBU char device maj %d \n", MAJOR(bbuDevNumbers));

    Bbu_HalInit();

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU DLM is up!\n");

    return retVal;
}

/**************************************************************************/
/*! \fn static Int32 bbuCreateCharDev(void)
 **************************************************************************
 *  \brief Create bbu kernel module character device
 *  \return  0 or error code
 */
static Int32 bbuCreateCharDev(void)
{
    /*Allocate major and minor numbers for the device*/
    if (alloc_chrdev_region(&bbuDevNumbers, BBU_CDEV_MIN_MINOR, BBU_CDEV_NUM_DEVICES,
                            BBU_CDEV_NAME))
    {
        printk(KERN_ERR "Failed to allocate BBU char device numbers\n");
        return -ENODEV;
    }
    /*Allocate the device*/
    bbuCdev = cdev_alloc();
    if (!bbuCdev)
    {
        printk(KERN_ERR "Failed to allocate BBU char device\n");
        unregister_chrdev_region(bbuDevNumbers, BBU_CDEV_NUM_DEVICES);
        return -ENOMEM;
    }

    /*Init device structure*/
    bbuCdev->ops = &bbuCdevFops;
    bbuCdev->owner = THIS_MODULE;

    /*Add the device to the kernel*/
    if (cdev_add(bbuCdev, bbuDevNumbers, BBU_CDEV_NUM_DEVICES))
    {
        printk(KERN_ERR "Failed to add BBU char device\n");
        kfree(bbuCdev);
        unregister_chrdev_region(bbuDevNumbers, BBU_CDEV_NUM_DEVICES);
        return -ENODEV;
    }
    return 0;
}

/**************************************************************************/
/*! \fn bbuIoctl
 **************************************************************************
 *  \brief IOCTL routine for the bbu driver
 *  \param[in] inode
 *  \param[in] filp
 *  \param[in] cmd - specific ioctl cmd
 *  \param[in,out] arg - pointer to user data if present
 *  \return 0 if ok, or error if error is encountered */
static long bbuIoctl(struct file *file, Uint32 cmd, unsigned long arg)
{
    BbuIoctlDataS_t data;
    Int32 ret = 0;

    /*
       * extract the type and number bitfields, and don't decode
       * wrong cmds: return ENOTTY (inappropriate ioctl) 
       */
    if (_IOC_TYPE(cmd) != BBU_IOCTL_MAGIC)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s wrong cmd type, magic  %d\n", __FUNCTION__, _IOC_TYPE(cmd));
        return -ENOTTY;
    }

    if (_IOC_NR(cmd) > BBU_IOC_MAX_NUM)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s wrong cmd type: %d\n", __FUNCTION__, _IOC_NR(cmd));
        return -ENOTTY;
    }

    /* cmd valid, continue */
    BBU_KDEBUG_PRINT(KERN_INFO, "%s: cmd %s\n", __FUNCTION__, BbuIoctlCmdDisplay[_IOC_NR(cmd)]);

    /* for debug !!!*/
    if (cmd == BBUIOC_TEST)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s: get user data\n", __FUNCTION__);
    }

    data.bufLen = 0;
    if (_IOC_SIZE(cmd) != 0) /* there is actually data , get it for all subcommands that may need it*/
    {
        if (copy_from_user(&data, (void __user *)arg, sizeof(BbuIoctlDataS_t))) /* only pointer is copied here*/
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy from user\n", __FUNCTION__);
            return -EFAULT;
        }
    }
    /*Process the data*/
    switch (cmd)
    {
    /*start periodic scanning if data initialized*/
    case BBUIOC_START:
        {
            bbuStartPeriodicScan();
            break;
        }
    case BBUIOC_STP:
        {
            bbuStopPeriodicScan();
            break;
        }
    case BBUIOC_CALIB: /* todo move to a separate routine*/
        {
            BbuCurrentMode_e calibMode;
            Int32 currentCalibStatus;

            if (bbuIoctlGetUserData(arg, (Int32 *)&calibMode, cmd)!= 0)
            {
                break;
            }

            currentCalibStatus = atomic_read(&bbuNeedCalibrate);

            if ((calibMode == BBU_SingleEnded) 
                && (!BBU_SINGLE_CALIBRATION_SET(currentCalibStatus)))
            {
                BBU_SET_SINGLE_CALIBRATION_NEEDED(currentCalibStatus);
                atomic_set(&bbuNeedCalibrate, currentCalibStatus);
                BBU_KDEBUG_PRINT(KERN_INFO, "%s  calibrate single, bbuNeedCalibrate: %d \n", __FUNCTION__, atomic_read(&bbuNeedCalibrate));
            }

            if ((calibMode == BBU_Differential) 
                && (!BBU_DIFF_CALIBRATION_SET(currentCalibStatus)))
            {
                BBU_SET_DIFF_CALIBRATION_NEEDED(currentCalibStatus);
                atomic_set(&bbuNeedCalibrate, currentCalibStatus);
                BBU_KDEBUG_PRINT(KERN_INFO, "%s  calibrate diff, bbuNeedCalibrate: %d \n", __FUNCTION__, atomic_read(&bbuNeedCalibrate));
            }

            /* kernel thread must do the calibration on the next run*/
            BBU_KDEBUG_PRINT(KERN_INFO, "%s  calibrate requested \n", __FUNCTION__);

            if (bbuKernThread == NULL) /* manual dbg */
            {
                Bbu_AdcCalibrate(calibMode);
            }

            break;
        }
    case BBUIOC_ENABLE_BAT: 
        {
            bbuEnableBatteryCmd(arg);
            break;
        }
    case BBUIOC_SWITCH_BAT: /* todo move to a separate routine*/
        {
            Int32 batIndexAndStat;
            Bool batteriesEnabledTogether;
            Int32 nextActiveBat;

            if (bbuIoctlGetUserData(arg, &batIndexAndStat, cmd)!= 0)
            {
                break;
            }

            batteriesEnabledTogether = (Bool) BAT_CHARGE_STATUS(batIndexAndStat);/* True - discharge state*/
            nextActiveBat = BAT_ENABLE_INDEX(batIndexAndStat);
            BBU_KDEBUG_PRINT(KERN_INFO, "%s:  %s bat %d \n", __FUNCTION__, (BAT_ENABLE_STATUS(batIndexAndStat) == True)? "Enable" : "Disable", BAT_ENABLE_INDEX(batIndexAndStat));

            if ((ret = bbuDriverGetDataAccess()) != 0)
            {
                BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
                return ret;
            }

            if (batteriesEnabledTogether)
            {
                ret = Bbu_SetActiveBattery_discharge(nextActiveBat);
            }
            else
            {
                ret = Bbu_SetActiveBattery(nextActiveBat);
            }
            bbuDriverReleaseDataAccess();
            break;
        }
    case BBUIOC_UPDATE_WORKING_PARAMS:
        {
            ret = bbuSetWorkingParamsData(&data);
            atomic_set(&bbuWorkDataInitialized, True);

            break;
        }
    case BBUIOC_GET_WORKING_DATA:
        {
            ret = bbuGetWorkingData(&data);
            break;
        }
    case BBUIOC_GET_ADC_DATA: /* takes latest data polled, corrects (calibration and averaging) */
        {
            ret = bbuGetAdcData(&data);
            break;
        }
    case BBUIOC_UPDATE_PWM_DATA:
        {
            ret = bbuSetPwmData(&data);
            break;
        }
    case BBUIOC_GET_PWM_DATA:
        {
            ret = bbuGetPwmData(&data);
            break;
        }

    case BBUIOC_GET_READ_REG_CMD:
        {

            break;
        }
    case BBUIOC_GET_WRITE_REG_CMD:
        {
            break;
        }
    case BBUIOC_TEST:
        {
            ret = BbuDbgCalcFLoat(&data);
            break;
        }
    case BBUIOC_ONE_SHOT:
        {
            ret = BbuGetOneShot(&data);
            break;
        }
    case BBUIOC_READ_VIOLATION:
        {
            ret = BbuTreatViolationNotifyRequest(&data);
            break;
        }
    case BBUIOC_GET_ENABLED_BAT:
        {
            ret = BbuGetHwEnabledBat(&data);
            break;
        }
    default:
        {
            ret = -ENOTTY;
            break;
        }
    }
    return ret;
}

/**************************************************************************/
/*! \fn BbuPeriodicHwScan
 **************************************************************************
 *  \brief kernel thread main routine :
 *  periodically scans ADC ports, stores ADC data, performs safety checks
 *  \param[in] data - not used
 *  \return Not supposed to return, may return error
 */
static Int32 BbuPeriodicHwScan(void *data)
{
    Bbu_OneShotDataType_t adcData;
    Int32 retVal = 0;
    Int32 calibrationNeeded;
    Int32 currResult;
    Bool needBbuReset = False;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s : start dbg print retval %d\n",  __FUNCTION__, retVal);

    if (bbuDriverGetDataAccess() != 0)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to get AccessAllow err %d ,  continuing\n", __FUNCTION__, retVal);
    }
    /* request scanning once*/

    if ((retVal = Bbu_SetUpOneShotSample()) != 0)
    {
        printk(KERN_ERR "%s : failed to set one shot sample ret %d\n", __FUNCTION__, retVal);
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s : Set one shot sample\n", __FUNCTION__);      
    }

    memset(bbuAdcDataDebug.vals, 0, sizeof (Bbu_OneShotDataType_t));

    /* scan periodically */
    do
    {
        /* release is here at the start of loop so that if exited in 'continue' free sema */
        bbuDriverReleaseDataAccess();

        BBU_KDEBUG_PRINT(KERN_INFO, "%s TO BE REMOVED  : Set periodic timer %d\n", __FUNCTION__, bbuWorkingParams.bbuScanningPeriod);   
        schedule_timeout_interruptible(bbuWorkingParams.bbuScanningPeriod);

        if (bbuDriverGetDataAccess() != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to get AccessAllow err %d ,  continuing\n", __FUNCTION__, retVal);
            continue;
        }
        /* need to do caibration? if so - calibrate otherwise poll and check safety*/
        /* check timer - if calibration is needed. if needed, set bbuNeedCalibrate to BBU_NEED_CALIBRATE_BOTH*/

        if ((bbuDriverLocalData.calibrationTimerCounter++) == bbuDriverLocalData.calibrationTimerMaxCount)
        {
            bbuDriverLocalData.calibrationTimerCounter = 0; /* reset counter */
            atomic_set(&bbuNeedCalibrate, BBU_NEED_CALIBRATE_BOTH);
        }

        needBbuReset = False;

        if ((retVal = (Bbu_OneShotSamplePoll(&adcData))) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to read one  ADC sample\n", __FUNCTION__);
            needBbuReset = True; 
        }

        if (BbuValidateAdcData(&adcData) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s sample multiple failures\n", __FUNCTION__);
            needBbuReset = True; 
        }

        if(needBbuReset == True)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s performing BBU reset\n", __FUNCTION__);
            Bbu_AdcReset(True);
            Bbu_AdcReset(False);
            Bbu_HalInit();
            Bbu_SetUpOneShotSample();
            continue;
        }

        BbuTreatSafetyLimits(&adcData);
        BBU_KDEBUG_PRINT(KERN_INFO, " safety done , store ADC data\n");

        /* store  ADC data for application */
        memcpy(&(bbuDriverLocalData.bbuAdcData.vals), &(adcData.vals), sizeof (Bbu_OneShotDataType_t));

        if (bbuWorkingParams.bbuNeedToAccumulateCurrent)
        {
            /* Accumulator += ADCdata{diffCHannel] */
            /*  convert hw to pure*/
            currResult = Bbu_ConvertHwToPure_Diff(BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_BATT_CURRENT, (&adcData)), BBU_Differential);
            bbuDriverLocalData.bbuAdcData.bbuCurrentChannelAcc += currResult;
            bbuDriverLocalData.bbuAdcData.samplesCounter++;
            BBU_KDEBUG_PRINT(KERN_INFO, " acc updated in periodic current 0x%X acc 0x%X counter %d\n", currResult, bbuDriverLocalData.bbuAdcData.bbuCurrentChannelAcc,  bbuDriverLocalData.bbuAdcData.samplesCounter);
        }

        if (Bbu_GainControl( &(adcData.vals[BBU_ADC_CHANNEL_BATT_CURRENT]) ) == True)
        {
            bbuDriverLocalData.calibrationTimerCounter = 0; /* reset counter */
            atomic_set(&bbuNeedCalibrate, BBU_NEED_CALIBRATE_BOTH);
        }

        if ((calibrationNeeded = atomic_read(&bbuNeedCalibrate)) != BBU_CALIBRATION_DONE)
        /* calibration to be done because time has come or driver marked it (gain changed)*/
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "thread needs to calibrate value %d", calibrationNeeded);

            /* perform needed calibration*/
            if (BBU_SINGLE_CALIBRATION_SET(calibrationNeeded))
            {
                if ((Bbu_AdcCalibrate(BBU_SingleEnded)) != 0)
                {
                    BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to calibrate single \n", __FUNCTION__);
                }
            }

            if (BBU_DIFF_CALIBRATION_SET(calibrationNeeded))
            {
                if ((Bbu_AdcCalibrate(BBU_Differential)) != 0)
                {
                    BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to calibrate diff \n", __FUNCTION__);
                }

            }
            atomic_set(&bbuNeedCalibrate, BBU_CALIBRATION_DONE);
            BbuCorrectWorkingParams();
        }
        /* setup next poll */
        if ((retVal = Bbu_SetUpOneShotSample()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to set one shot sample ret %d\n", __FUNCTION__, retVal);
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_DEBUG, "%s : suceeded to set one shot sample back to loop \n", __FUNCTION__);
        }

    }while (1); 

    return retVal;
}

/**************************************************************************/
/*! \fn bbuDriverExit
 **************************************************************************
 *  \brief driver exit routine
 *  stops all activity, disables hardware and destroys bbu device
 *  \param[in] none
 *  \return  none
 */
static void bbuDriverExit(void)
{
    /* must stop batteries/adc/pwm*/
    bbuHwDisable();

    /* dealloc device*/
    bbuDeleteCdev();
    printk(KERN_WARNING "BBU module is down!\n");
    return;
}

/**************************************************************************/
/*! \fn bbuDeleteCdev
 **************************************************************************
 *  \brief deallocate bbu character device
 *  \param[in] none
 *  \return  none
 */
static void bbuDeleteCdev(void)
{
    cdev_del(bbuCdev);
    unregister_chrdev_region(bbuDevNumbers, BBU_CDEV_NUM_DEVICES);
}

/**************************************************************************/
/*! \fn bbuOpen
 **************************************************************************
 *  \brief bbu driver open routine
 *  opens device.
 * example of using open: if ((fd = open("/dev/" DI_NAME "0", O_RDWR)) < 0) {
   * TODO consider removing open and release procs
 *  \param[in] node, file pointer
 *  \return  none
 */
static int bbuOpen(struct inode *inode, struct file *filp)
{
    Int32 retVal = 0;

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU DLM open \n");

    return retVal;
}

/**************************************************************************/
/*! \fn bbuRelease
 **************************************************************************
 *  \brief bbu driver release routine
 *  closes device.
 *   TODO consider removing open and release procs
 *  \param[in] node, file pointer
 *  \return  none
 */
static Int32 bbuRelease(struct inode *inode, struct file *filp)
{
    Int32 retVal = 0;

    BBU_KDEBUG_PRINT(KERN_INFO, "bbu release \n");

    return retVal;
}

/**************************************************************************/
/*! \fn bbuRead
 **************************************************************************
 *  \brief bbu driver read routine
 *  read P5 persistent data.
 *  read is isually done once after reset, then copy kept in user space
 *  \param[in] count and pos are ignored for BBU 
 *  \param[in]  file pointer
 *  \param[in, out]  buf - buffer to be filled with data
 *  \return  size of data read or error (negative)
 */
static ssize_t bbuRead(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    Int32 retVal = 0;

    if (bbuGetAccessToP5Mem() != 0)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "BBU driver get access failed  \n");
        return  -EFAULT;
    }

    bbuCopyPersistentData(BBU_FROM_P5_DATA);
    bbuFreeAccessToP5Mem();

    if (copy_to_user(buf, &(bbuDriverLocalData.bbuBatPersistentData), sizeof(Bbu_BatteryPersistentData_t)))
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "BBU driver read failed  \n");
        retVal = -EFAULT;
    }
    else
    {
        retVal = sizeof(Bbu_BatteryPersistentData_t);
    }

    return retVal; 
}

/**************************************************************************/
/*! \fn bbuWrite
 **************************************************************************
 *  \brief bbu driver write routine
 *  write to P5 persistent data.
 *  \param[in] count and pos are ignored for BBU 
 *  \param[in]  file pointer
 *  \param[in]  buf - buffer to be copied from
 *  \return  size of data written or error (negative)
 */
static ssize_t bbuWrite(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    Int32 retVal = 0; 

    if ((retVal = copy_from_user(&(bbuDriverLocalData.bbuBatPersistentData), buf, sizeof(Bbu_BatteryPersistentData_t))))
    {
        retVal = -EFAULT;
        return retVal;
    }

    if (bbuGetAccessToP5Mem() != 0)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "BBU driver get access failed  \n");
        return  -EFAULT;
    }

    /* put data to chip */
    bbuCopyPersistentData(BBU_TO_P5_DATA);
    bbuFreeAccessToP5Mem();

    retVal = sizeof(Bbu_BatteryPersistentData_t);

    return retVal;
}

/**************************************************************************/
/*! \fn bbuCopyPersistentData
 **************************************************************************
 *  \brief copy persistent data to /from P5 ram 
 *  \param[in] write direction 
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuCopyPersistentData(Int16 direction)
{
    Int32 retval = 0;
    Int32 index =  0;
    Int32 limit = (sizeof (Bbu_BatteryPersistentData_t) >> 2) + 1; /* number of Int32 */
    Int32 reg = BBU_P5_STORAGE;

    BBU_KDEBUG_PRINT(KERN_DEBUG, "persistent real data size %d words %d \n", sizeof (Bbu_BatteryPersistentData_t), limit);
    for (index = 0; index < limit; index++)
    {
        reg+=index*4;

        if (direction == BBU_TO_P5_DATA)
        {
            Bbu_SetRegValue(reg, ((Uint32 *) &(bbuDriverLocalData.bbuBatPersistentData))[index]);
        }
        else
        {
            ((Uint32 *) &(bbuDriverLocalData.bbuBatPersistentData))[index] = Bbu_GetRegValue(reg);
        }
    }
    return retval;
}

/**************************************************************************/
/*! \fn bbuIoctlGetUserData
 **************************************************************************
 *  \brief copy user data (integer size )to local buffer
 *  \param[in] userData pointer to user data to copy data from 
 *  \param[in] localData pointer to local buffer to copy data to 
 *  \param[in] cmd - ioctl cmd to be logged in case of failure
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuIoctlGetUserData(Uint32 userData, Int32 *localData, Uint32 cmd)
{
    Int32 ret = 0;

    if (access_ok(VERIFY_READ, (void __user *)userData, sizeof(Int32)))
    {
        if ((ret = get_user(*localData, (Int32 __user*)userData)) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to get data from user cmd %s ret %d \n", __FUNCTION__, BbuIoctlCmdDisplay[_IOC_NR(cmd)], ret);
        }
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s  access denied for user data", __FUNCTION__);
        ret = -EACCES;
    }
    return ret;
}

/**************************************************************************/
/*! \fn bbuSetWorkingParamsData
 **************************************************************************
 *  \brief set working data from user to local copy,
 *   this  includes converting safety limits to hw form using calibration coefficients;
 *    resetting periodic counter and calibration counter
 *  \param[in] data - user space buffer that includes working parameters, buffer expected to be 'BbuWorkingParams_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuSetWorkingParamsData(BbuIoctlDataS_t *data)
{
    BbuWorkingParams_t localBuf;
    bbu_float64  a_single_coeff = bbuWorkingParams.bbuCurrentCalValues.single_a;
    bbu_float64  b_single_coeff = bbuWorkingParams.bbuCurrentCalValues.single_b;
    bbu_float64  a_diff_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_a;
    bbu_float64  b_diff_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_b;
    Int32 ret = 0;

    BBU_KDEBUG_PRINT(KERN_DEBUG, "%s start \n", __FUNCTION__);

    /*Copy data buffer contents from user space  */
    if (data->buf && (data->bufLen == sizeof(BbuWorkingParams_t)))
    {

        if (copy_from_user((Uint8 *)&localBuf, data->buf, data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy from user\n", __FUNCTION__);
            return -EFAULT;
        }

        BBU_KDEBUG_PRINT(KERN_INFO, "%s data copied to local buf \n", __FUNCTION__);

        BBU_KDEBUG_PRINT(KERN_INFO, "%s wait for thread to stop \n", __FUNCTION__);
        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        BBU_KDEBUG_PRINT(KERN_INFO, "%s copy datap \n", __FUNCTION__);
        memcpy(&(bbuWorkingParams), &localBuf, data->bufLen);
        /* update calibration counter count */
        bbuDriverLocalData.calibrationTimerMaxCount = (bbuWorkingParams.bbuCalibrationPeriod >= bbuWorkingParams.bbuScanningPeriod) ?
                                                      (bbuWorkingParams.bbuCalibrationPeriod / bbuWorkingParams.bbuScanningPeriod) : 1;

        /* restore the calibration coefficients */
        bbuWorkingParams.bbuCurrentCalValues.single_a = a_single_coeff;
        bbuWorkingParams.bbuCurrentCalValues.single_b = b_single_coeff;
        bbuWorkingParams.bbuCurrentCalValues.diff_a = a_diff_coeff;
        bbuWorkingParams.bbuCurrentCalValues.diff_b = b_diff_coeff ;
        /* correct the values according to the calibration coeffificents*/ 
        BbuCorrectWorkingParams();
        BBU_KDEBUG_PRINT(KERN_INFO, "%s calibration timer %d max count %d need accumulate %d current dir %d \n", __FUNCTION__, bbuWorkingParams.bbuCalibrationPeriod, bbuDriverLocalData.calibrationTimerMaxCount,
                         bbuWorkingParams.bbuNeedToAccumulateCurrent, bbuWorkingParams.bbuCurrentDirection);

        BBU_KDEBUG_PRINT(KERN_INFO, "%s free thread to run \n", __FUNCTION__);
        bbuDriverReleaseDataAccess();

    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuWorkingParams_t));
    }

    return ret;
}

/**************************************************************************/
/*! \fn bbuGetWorkingData
 **************************************************************************
 *  \brief get working data from local copy to user buffer,
 *   copy of working data is kept in user space and is downloaded from there.
 * this routine is  for debug  and after calibration to see the calib values
 *  \param[in, out] data - user space buffer, buffer expected to be 'BbuWorkingParams_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuGetWorkingData(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;

    if (data->buf && (data->bufLen == sizeof(BbuWorkingParams_t)))
    {
        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        if (copy_to_user(data->buf, (Uint8 *)&(bbuWorkingParams), data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
            return -EFAULT;
        }

        bbuDriverReleaseDataAccess();

    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuWorkingParams_t));
    }

    return ret;
}

/**************************************************************************/
/*! \fn bbuGetAdcData
 **************************************************************************
 *  \brief get adc data - latest status of adc ports, and of accumulator - from local copy to user buffer,
 *   if requested, current accumulator is reset
 *   samples are converted to pure form before copying to user
 *  \param[in, out] data - user space buffer, buffer expected to be 'BbuAdcStateData_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuGetAdcData(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;
    Bool needToResetAccumulator = False;
    BbuAdcStateData_t localData;
    Uint32 ch;

    if (data->buf && (data->bufLen == sizeof(BbuAdcStateData_t)))
    {
        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        memcpy(&localData, &(bbuDriverLocalData.bbuAdcData), sizeof(BbuAdcStateData_t));

        needToResetAccumulator = (Bool)data->param1;

        if (needToResetAccumulator)/* need to reset accum?*/
        {
            bbuDriverLocalData.bbuAdcData.bbuCurrentChannelAcc = 0;
            bbuDriverLocalData.bbuAdcData.samplesCounter = 0;
            BBU_KDEBUG_PRINT(KERN_DEBUG, "%s accumulator reset \n", __FUNCTION__);
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_DEBUG, "%s no need reset acc\n", __FUNCTION__);
        }

        bbuDriverReleaseDataAccess();

        /* convert to pure data for user */
        for (ch = 0; ch < MAX_ADC_CHANNEL_INDEX ; ch++)
        {
            localData.vals[ch] = (ch != BBU_ADC_CHANNEL_BATT_CURRENT) ? Bbu_ConvertHwToPure_Single(localData.vals[ch])
                                 : Bbu_ConvertHwToPure_Diff(localData.vals[ch], BBU_Differential);
        }
        localData.currentVal = Bbu_ConvertHwToPure_Diff(localData.vals[BBU_ADC_CHANNEL_BATT_CURRENT], BBU_Differential);


        if (copy_to_user(data->buf, (Uint8 *)&localData, data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
            return -EFAULT;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "%s buf addr %x accumulator %d counter %u \n", __FUNCTION__, (Int32)data->buf, localData.bbuCurrentChannelAcc,
                             localData.samplesCounter);
        }
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuAdcStateData_t));
        ret = EINVAL;
    }

    return ret;
}

/**************************************************************************/
/*! \fn bbuSetPwmData
 **************************************************************************
 *  \brief configure PWM upon data requested
 *  \param[in] data - user space buffer,  buffer expected to be 'BbuPwmParameters_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuSetPwmData(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;
    BbuPwmParameters_t  bbuPwmData;

    if (data->buf && (data->bufLen == sizeof(BbuPwmParameters_t)))
    {

        if (copy_from_user(&bbuPwmData, data->buf, data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy from user\n", __FUNCTION__);
            return -EFAULT;
        }

        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }
        memcpy(&bbuDriverLocalData.bbuPwmData, &bbuPwmData, data->bufLen); 

        Bbu_EnablePWM(bbuDriverLocalData.bbuPwmData.active);

        if (bbuDriverLocalData.bbuPwmData.active)
        {
            Bbu_SetPwmDutyCycle(bbuDriverLocalData.bbuPwmData.pwmDutyCycle); 
        }

        bbuDriverReleaseDataAccess();
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuPwmParameters_t));
    }

    return ret;
}

/**************************************************************************/
/*! \fn bbuGetPwmData
 **************************************************************************
 *  \brief get PWM  data from hardware
 *  \param[in, out] data - user space buffer, buffer expected to be 'BbuPwmParameters_t' size 
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuGetPwmData(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;

    if (data->buf && (data->bufLen == sizeof(BbuPwmParameters_t)))
    {

        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        /* can check if pwm enabled??? TODO if can, update bbuDriverLocalData.bbuPwmData.active*/
        bbuDriverLocalData.bbuPwmData.pwmDutyCycle = Bbu_GetPwmDutyCycle();
        bbuDriverLocalData.bbuPwmData.pwmGain = Bbu_GetD2SAmplification(); 

        if (copy_to_user(data->buf, (Uint8 *)&(bbuDriverLocalData.bbuPwmData), data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
            return -EFAULT;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "%s pwm gain 0x%x  \n", __FUNCTION__, (Int32)bbuDriverLocalData.bbuPwmData.pwmGain);
        }
        bbuDriverReleaseDataAccess();
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuPwmParameters_t));
    }

    return ret;
}

/**************************************************************************/
/*! \fn bbuEnableBatteryCmd
 **************************************************************************
 *  \brief change battery enabled status
 *  \param[in, out] data - user space buffer, content  expected to be 'int' size, that includes battery index and 
 *                 battery desired status 
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuEnableBatteryCmd(Int32 data)
{
    Int32 batIndexAndStat;
    Int32 retVal = 0;

    if ((retVal = bbuIoctlGetUserData(data, &batIndexAndStat, BBUIOC_ENABLE_BAT)) != 0)
    {
        return retVal;
    }

    if ((retVal = bbuDriverGetDataAccess()) != 0)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, retVal);
        return retVal;
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s:  %s bat %d\n", __FUNCTION__, (BAT_ENABLE_STATUS(batIndexAndStat) == True)? "Enable" : "Disable", BAT_ENABLE_INDEX(batIndexAndStat));
    Bbu_EnableBattery(BAT_ENABLE_INDEX(batIndexAndStat), BAT_ENABLE_STATUS(batIndexAndStat));

    /*  for debugging: verify battery in correct status*/
    BBU_KDEBUG_PRINT(KERN_INFO, "%s:  bat %d is enabled\n", __FUNCTION__, Bbu_GetEnabledBattery());

    bbuDriverReleaseDataAccess();

    return retVal;
}

/**************************************************************************/
/*! \fn bbuDriverGetDataAccess
 **************************************************************************
 *  \brief try to get access to sema. waits till access granted
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuDriverGetDataAccess(void)
{
    Int32 retVal = 0;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: get access \n", __FUNCTION__);
    if (down_interruptible(&(bbuDriverLocalData.bbuDataSema)))
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s failed to get AccessAllow err %d ,  continuing\n", __FUNCTION__, retVal);
        retVal = -ERESTARTSYS;
    }
    BBU_KDEBUG_PRINT(KERN_DEBUG, "%s: GOT access \n", __FUNCTION__);
    return retVal;
}

/**************************************************************************/
/*! \fn BbuGetHwEnabledBat
 **************************************************************************
 *  \brief get  hardware 'battery enabled' status
 *  \param[in, out] user buffer, expected to be of int size
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuGetHwEnabledBat(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;
    Int32 batEnabled;

    if (data->buf && (data->bufLen == sizeof(batEnabled)))
    {
        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        batEnabled = Bbu_GetEnabledBattery();

        if (copy_to_user(data->buf, (Uint8 *)&(batEnabled), data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
            return -EFAULT;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "%s bat enabled index %d\n", __FUNCTION__, batEnabled);
        }
        bbuDriverReleaseDataAccess();
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(batEnabled));
    }

    return ret;

}

/**************************************************************************/
/*! \fn BbuDbgCalcFLoat
 **************************************************************************
 *  \brief debug routine to test float_64 calculations
 *  \param[in, out] user buffer, expected to be  'BbuFloatCalc_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuDbgCalcFLoat(BbuIoctlDataS_t *data)
{
    Int32 src1 ;
    Int32 src2 ;
    Int32 operation ;
    bbu_float64 src1_64;
    bbu_float64 src2_64;
    bbu_float64 res_64;
    Int32 result;
    BbuFloatCalc_t dataLocal;
    Int32 retVal = 0;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: start \n", __FUNCTION__);
    if (data->buf && (data->bufLen == sizeof(BbuFloatCalc_t)))
    {
        if (copy_from_user(&dataLocal, data->buf, data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy from user\n", __FUNCTION__);
            return -EFAULT;
        }
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s : Invalid data buffer\n", __FUNCTION__);
        return -EFAULT;
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: read data \n", __FUNCTION__);

    src1 = dataLocal.operand1;
    src2 = dataLocal.operand2;
    operation = dataLocal.operation;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: start calc \n", __FUNCTION__);

    Int32ToFloat64(src1, &src1_64);
    Int32ToFloat64(src2, &src2_64);

    BBU_KDEBUG_PRINT(KERN_INFO, "\n bbucal64 src1 %d src2 %d  src1.man %d src1_64.exp %d src2.man %d src2_64.exp %d \n",
                     src1, src2, src1_64.mantissa, src1_64.exponent, src2_64.mantissa, src2_64.exponent);

    switch (operation)
    {
    case 0:
        {
            AddFloat64(src1_64, src2_64, &res_64);
            break;
        }
    case 1:
        {
            SubFloat64(src1_64, src2_64, &res_64);
            break;
        }
    case 2:
        {
            MulFloat64(src1_64, src2_64, &res_64);
            break;
        }
    case 3:
        {
            DivideFloat64(src1_64, src2_64, &res_64);
            break;
        }
    default:
        /* Invalid operation */
        return -EFAULT;
    }

    result = Float64ToInt(res_64);
    BBU_KDEBUG_PRINT(KERN_INFO, "\n bbucal64 result mant %d exp %d result int %d \n",
                     res_64.mantissa, res_64.exponent, result);

    dataLocal.result = result;
    dataLocal.resultFloat = res_64;
    if (copy_to_user(data->buf, (Uint8 *)&dataLocal, data->bufLen))
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
        return -EFAULT;
    }
    return retVal;
}

/**************************************************************************/
/*! \fn BbuGetOneShot
 **************************************************************************
 *  \brief  routine to get one shot. May be used for debug. If kernel thread is not running,
 *   requests for one shot sample and waits for reply. If kernel thread runs, takes its last result
 *   results are converted to pure form
 *  \param[in, out] user buffer, expected to be  'BbuAdcStateData_t' size
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuGetOneShot(BbuIoctlDataS_t *data)
{
    Int32 ret = 0;
    BbuAdcStateData_t tdata;
    Int32 ch;
    Int32 currResult;

    if (data->buf && (data->bufLen == sizeof(BbuAdcStateData_t)))
    {

        if ((ret = bbuDriverGetDataAccess()) != 0)
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to stop thread err %d \n", __FUNCTION__, ret);
            return ret;
        }

        if (bbuKernThread == NULL) /* manual dbg */
        {
            /* Set ADC to Idle state. */

            Bbu_AdcSetToIdle();

            if (( ret = Bbu_SetUpOneShotSampleAndPoll((Bbu_OneShotDataType_t *)&(tdata.vals)))  != 0)
            {
                BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed do one shot polling err %d \n", __FUNCTION__, ret);
            }

            /* convert to software format */
            for (ch = 0; ch < MAX_ADC_CHANNEL_INDEX ; ch ++)
            {
                tdata.vals[ch] = (ch != BBU_ADC_CHANNEL_BATT_CURRENT) ? Bbu_ConvertHwToPure_Single(tdata.vals[ch])
                                 : Bbu_ConvertHwToPure_Diff(tdata.vals[ch], BBU_Differential);
            }

            tdata.currentVal = Bbu_ConvertHwToPure_Diff(tdata.vals[BBU_ADC_CHANNEL_BATT_CURRENT], BBU_Differential);

            memcpy(&(bbuDriverLocalData.bbuAdcData.vals), tdata.vals, sizeof(Uint16)*(MAX_ADC_CHANNEL_INDEX + 1));
            /* Accumulator += ADCdata{diffCHannel] */
            currResult = BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_BATT_CURRENT, (&tdata));

            bbuDriverLocalData.bbuAdcData.bbuCurrentChannelAcc += currResult;
            bbuDriverLocalData.bbuAdcData.samplesCounter++;
        }
        else /* use last shot*/
        {
            memcpy(tdata.vals, &(bbuDriverLocalData.bbuAdcData.vals), sizeof(Uint16)*(MAX_ADC_CHANNEL_INDEX + 1));
            for (ch = 0; ch <= MAX_ADC_CHANNEL_INDEX ; ch++)
            {
                tdata.vals[ch] = (ch != BBU_ADC_CHANNEL_BATT_CURRENT) ? Bbu_ConvertHwToPure_Single(tdata.vals[ch])
                                 : Bbu_ConvertHwToPure_Diff(tdata.vals[ch], BBU_Differential);
            }
            tdata.currentVal = Bbu_ConvertHwToPure_Diff(tdata.vals[BBU_ADC_CHANNEL_BATT_CURRENT], BBU_Differential);
        }

        bbuDriverReleaseDataAccess();

        BBU_KDEBUG_PRINT(KERN_DEBUG, "%s: converted to pure 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", __FUNCTION__,
                         tdata.vals[0], tdata.vals[1], tdata.vals[2], tdata.vals[3], tdata.vals[4], tdata.vals[5], tdata.vals[6], tdata.vals[7]);

        if (copy_to_user(data->buf, (Uint8 *)&tdata, data->bufLen))
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
            return -EFAULT;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "%s succeeded \n", __FUNCTION__);
        }
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: %s bufLen %d datasize %d\n", __FUNCTION__, (data->buf == NULL) ? "user buffer NULL" : "wrong buff len",
                         data->bufLen, sizeof(BbuAdcStateData_t));
    }

    return ret;
}

/**************************************************************************/
/*! \fn BbuTreatViolationNotifyRequest
 **************************************************************************
 *  \brief  routine to wait for safety violation notification.
 *   waits on rd queue for being waked up when safety violation notification is set
 *   being waked up, sends to violation data to user
 *   hopefully, if no safety violation, may wait here for ever!
 *  \param[in, out] user buffer, expected to be  'BbuSafetyViolationCauses_e' size
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuTreatViolationNotifyRequest(BbuIoctlDataS_t *data)
{
    BbuSafetyViolationCauses_e localCause;
    Int32 ret;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: start \n", __FUNCTION__);
    /* wait on q*/
    if (wait_event_interruptible(bbuDriverLocalData.bbuRqueue[0],  (bbuDriverLocalData.safetyViolation.newViolationCauseDetected != BBU_VIOLATION_NONE)))
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s: restart \n", __FUNCTION__);
        return -ERESTARTSYS;
    }

    if ((ret = bbuDriverGetDataAccess()) != 0)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to lock data err %d \n", __FUNCTION__, ret);
        return ret;
    }
    localCause = bbuDriverLocalData.safetyViolation.newViolationCauseDetected;
    bbuDriverLocalData.safetyViolation.newViolationCauseDetected = BBU_VIOLATION_NONE;

    bbuDriverReleaseDataAccess();

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: waked \n", __FUNCTION__);

    /* copy the reason, todo additional information */
    if (copy_to_user(data->buf, &localCause, sizeof(localCause)))
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s : failed to copy to user\n", __FUNCTION__);
        return -EFAULT;
    }
    else
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s succeeded \n", __FUNCTION__);
    }
    return 0;
}

/**************************************************************************/
/*! \fn BbuTreatSafetyLimits
 **************************************************************************
 *  \brief  routine to check for presence of safety violation notification.
 *   tests the last sample values vs working parameter limits. If violation found,
 *   notifies possible waiting entity
 *  \param[in, out] adc buffer with latest samples results
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuTreatSafetyLimits(Bbu_OneShotDataType_t *adcDataP)
{
    Uint16 sample;
    Uint16 currentSample;
    Int32 retVal =  0; 

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: test safety \n", __FUNCTION__);

    /* check safety. if fail -- notify aplication */

    /* check temperature */

    sample = BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_TEMPERATURE, adcDataP);

    if (BBU_IS_TEMPERATURE_ABOVE_THRESHOLD(sample, bbuWorkingParams.bbuMaxTempLimit))
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, " bbu temp above max: sample %x limit %x\n", sample, bbuWorkingParams.bbuMaxTempLimit);
        BbuNotifySafetyViolation(BBU_TEMPERATURE_HIGH);
    }
    else
    {
        BBU_RESET_SAFETY_VIOLATION(BBU_TEMPERATURE_HIGH);
    }

    if (BBU_IS_TEMPERATURE_BELOW_THRESHOLD(sample, bbuWorkingParams.bbuMinTempLimit))
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, " bbu temp below min: sample %x limit %x\n", sample, bbuWorkingParams.bbuMinTempLimit);
        BbuNotifySafetyViolation(BBU_TEMPERATURE_LOW);
    }
    else
    {
        BBU_RESET_SAFETY_VIOLATION(BBU_TEMPERATURE_LOW);
    }

    /* check if need notify temperature is ok now */
    if (bbuWorkingParams.bbuNeedToCheckTemperatureBackToNormal)
    {
        if ((bbuDriverLocalData.safetyViolation.safetyViolationList[BBU_TEMPERATURE_LOW] == False) &&
            (bbuDriverLocalData.safetyViolation.safetyViolationList[BBU_TEMPERATURE_HIGH] == False))
        {
            /* check for how many times temperature ok */
            bbuDriverLocalData.temperatureValidCounter++;
            if (bbuDriverLocalData.temperatureValidCounter == bbuWorkingParams.bbuMinCountTemperatureNormal)
            {
                bbuDriverLocalData.temperatureValidCounter = 0;
                BbuNotifySafetyViolation(BBU_TEMPERATURE_NORMAL);
            }
        }
        else
        {
            /* reset the counter, need to be sequence of succeessful measurements */
            bbuDriverLocalData.temperatureValidCounter = 0;
        }
    }
    /*  check voltage  */
    sample = BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_BATT_VOLTAGE, adcDataP);
/*    sample = Bbu_ConvertHwToPure_Single(sample);*/

    if (sample > bbuWorkingParams.bbuMaxVoltLimit)
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, " bbu volt above limit sample %x limit %x\n", sample, bbuWorkingParams.bbuMaxVoltLimit );
        BbuNotifySafetyViolation(BBU_VOLTAGE_HIGH);

        /* Note: The safety-failure of battery over-voltage can only occur on a   */
        /* battery that is inserted and enabled. A similar indication might */
        /* be received if a battery is removed while it is being charged.   */
        /* In order to prevent this possibility, the conditions below need  */
        /* to be checked before declaring a safety violation.               */
        /*                                                                   */
        /*  It is  left to application                                      */
        /* responsibility to do the checks                                   */
    }
    else
    {
        BBU_RESET_SAFETY_VIOLATION(BBU_VOLTAGE_HIGH);
    }

    /* check current limits */

    BBU_KDEBUG_PRINT(KERN_DEBUG, " bbu determine current direction\n");

    sample = BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_BATT_CURRENT, adcDataP);
    currentSample = sample;

    if (bbuWorkingParams.bbuCurrentDirection == BBU_POSITIVE_CURRENT)
    {
        if (currentSample > bbuWorkingParams.bbuMaxCurrLimitPositive)
        {
            BBU_KDEBUG_PRINT(KERN_WARNING, "bbu positive current above limit sample %x limit %x\n", sample, bbuWorkingParams.bbuMaxCurrLimitPositive);
            BbuNotifySafetyViolation(BBU_CURRENT_HIGH_CHARGE);
        }
        else
        {
            BBU_RESET_SAFETY_VIOLATION(BBU_CURRENT_HIGH_CHARGE);
        }
    }
    else
    {
        if (currentSample < bbuWorkingParams.bbuMaxCurrLimitNegative)
        {
            bbuDriverLocalData.currentViolationCounter++;
        }
        else
        {
            bbuDriverLocalData.currentViolationCounter = 0;
        }

        /* several consecutive over-current events are considered a safety violation */
        if (bbuDriverLocalData.currentViolationCounter > bbuWorkingParams.bbuMaxCurrentViolationCount)
        {
            BBU_KDEBUG_PRINT(KERN_WARNING, "bbu negative current above limit: sample %x limit %x\n", sample, bbuWorkingParams.bbuMaxCurrLimitNegative);
            BbuNotifySafetyViolation(BBU_CURRENT_HIGH_DISCHARGE);
        }
        else
        {
            BBU_RESET_SAFETY_VIOLATION(BBU_CURRENT_HIGH_DISCHARGE);
        }
    }

    if (bbuWorkingParams.bbuNeedToCheckBuckClosedVoltage)
    {
        sample = BBU_GET_CURRENT_SAMPLE(BBU_ADC_CHANNEL_BUCK_VOLTAGE, adcDataP);
        /*      sample = Bbu_ConvertHwToPure_Single(sample);*/

        if (sample > bbuWorkingParams.bbuMaxBuckClosedVoltage)
        {
            BBU_KDEBUG_PRINT(KERN_WARNING, "bbu  Buck Voltage is Over Maximal limit: sample %x limit %x\n", sample, bbuWorkingParams.bbuMaxBuckClosedVoltage);
            BbuNotifySafetyViolation(BBU_CLOSED_BUCK_VOLTAGE);
        }
        else
        {
            BBU_RESET_SAFETY_VIOLATION(BBU_CLOSED_BUCK_VOLTAGE);
        }
    }
    return retVal;
}

/**************************************************************************/
/*! \fn BbuNotifySafetyViolation
 **************************************************************************
 *  \brief  mark local driver data safety violation found, notify whoever waits on rd queue
 *   Update BbuSafetyViolationCauses table and newViolationCauseDetected with violation found. 
 *   if this type of violation is freshly detected, notifiespossible waiting entity
 *  \param[in] safety violation cause
 *  \return none
 */
static void BbuNotifySafetyViolation(BbuSafetyViolationCauses_e cause)
{
    static Bool notified = False; /* for debug*/
    Int32 i;

    /* set violation status*/
    BBU_KDEBUG_PRINT(KERN_INFO, "bbu violation cause %s \n", BbuSafetyViolationCauses[cause]);
    if (!notified)
    {

        BBU_KDEBUG_PRINT(KERN_INFO, "bbu violations curr %d  %d %d %d %d %d %d\n", bbuDriverLocalData.safetyViolation.safetyViolationList[cause], 
                         bbuDriverLocalData.safetyViolation.safetyViolationList[1], bbuDriverLocalData.safetyViolation.safetyViolationList[2], bbuDriverLocalData.safetyViolation.safetyViolationList[3],
                         bbuDriverLocalData.safetyViolation.safetyViolationList[4], bbuDriverLocalData.safetyViolation.safetyViolationList[5], bbuDriverLocalData.safetyViolation.safetyViolationList[6]);

        /* dbug  walk around - initialize the list for debug - todo */
        for (i=0; i<BBU_SAFETY_VIOLATION_REASON_LAST ; i++)
        {
            bbuDriverLocalData.safetyViolation.safetyViolationList[i] = False;
        }
        notified = True;
    }

    if (bbuDriverLocalData.safetyViolation.safetyViolationList[cause] == False)
    {
        bbuDriverLocalData.safetyViolation.safetyViolationList[cause] = True;
        bbuDriverLocalData.safetyViolation.newViolationCauseDetected = cause;
        BBU_KDEBUG_PRINT(KERN_INFO, " bbu violation wakeup user \n");
        wake_up_interruptible(&bbuDriverLocalData.bbuRqueue[0]);
    }
}

/**************************************************************************/
/*! \fn BbuCorrectWorkingParams
 **************************************************************************
 *  \brief  update working params thresholds according to the calibration coefficients
 *   use the pure values in working parameter limits and calibration coefficients for the conversion,
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuCorrectWorkingParams(void)
{
    Int32 retVal = 0;

    /* convert pure to hw all thresholds  */

    bbuWorkingParams.bbuMaxVoltLimit = Bbu_ConvertPureToHw_Single(bbuWorkingParams.bbuMaxVoltLimitPure);

    bbuWorkingParams.bbuMinVoltLimit = Bbu_ConvertPureToHw_Single(bbuWorkingParams.bbuMinVoltLimitPure);
    bbuWorkingParams.bbuMaxBuckClosedVoltage = Bbu_ConvertPureToHw_Single(bbuWorkingParams.bbuMaxBuckClosedVoltagePure);
    bbuWorkingParams.bbuMaxTempLimit = Bbu_ConvertPureToHw_Single(bbuWorkingParams.bbuMaxTempLimitPure);
    bbuWorkingParams.bbuMinTempLimit = Bbu_ConvertPureToHw_Single(bbuWorkingParams.bbuMinTempLimitPure);

    /* current values  conversion*/
    bbuWorkingParams.bbuMaxCurrLimitPositive = Bbu_ConvertPureToHw_Diff(bbuWorkingParams.bbuMaxCurrLimitPositivePure, BBU_Differential);
    BBU_KDEBUG_PRINT(KERN_WARNING, "%s convert positive curr limit: pure 0x%x  (d %d) hw 0x%x\n", __FUNCTION__, bbuWorkingParams.bbuMaxCurrLimitPositivePure, bbuWorkingParams.bbuMaxCurrLimitPositivePure,  bbuWorkingParams.bbuMaxCurrLimitPositive);
    bbuWorkingParams.bbuMaxCurrLimitNegative = Bbu_ConvertPureToHw_Diff(bbuWorkingParams.bbuMaxCurrLimitNegativePure, BBU_Differential) ;

    return retVal;
}

/**************************************************************************/
/*! \fn bbuDriverReleaseDataAccess
 **************************************************************************
 *  \brief  free data access sema
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuDriverReleaseDataAccess(void)
{
    Int32 retVal = 0;

    up (&(bbuDriverLocalData.bbuDataSema));
    BBU_KDEBUG_PRINT(KERN_INFO, "%s  \n", __FUNCTION__);

    return retVal;
}

/**************************************************************************/
/*! \fn bbuGetAccessToP5Mem
 **************************************************************************
 *  \brief  get sema access to P5 memory
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuGetAccessToP5Mem(void)
{

    if (down_interruptible(&(bbuDriverLocalData.bbuPersistDataSema)))
    {
        return -ERESTARTSYS;
    }
    return 0;
}

/**************************************************************************/
/*! \fn bbuFreeAccessToP5Mem
 **************************************************************************
 *  \brief  free sema access to P5 memory
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuFreeAccessToP5Mem(void)
{
    up (&(bbuDriverLocalData.bbuPersistDataSema));
    return 0;
}

/**************************************************************************/
/*! \fn bbuStartPeriodicScan
 **************************************************************************
 *  \brief create kernel thread for periodic scanning
 *   thread is not created if working parameters are not initialized or if the thread already exists
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuStartPeriodicScan(void)
{
    Int32 retVal = 0;

    if (atomic_read(&bbuWorkDataInitialized) == False)
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "BBU periodic attempt to run but no working data, ignoring \n");
        return retVal;
    }
    if (bbuKernThread != NULL)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "BBU thread already runs \n");
    }
    else
    {
        bbuKernThread = kthread_run(BbuPeriodicHwScan, NULL, "bbukern");
        if (IS_ERR(bbuKernThread))
        {
            retVal = PTR_ERR(bbuKernThread);
            printk(KERN_ERR "Error %d create bbu kern thread\n", retVal);
            bbuKernThread = NULL;
        }
    }
    return retVal;
}

/**************************************************************************/
/*! \fn bbuStartPeriodicScan
 **************************************************************************
 *  \brief stop kernel thread if exists
 *  \param[in, out] none
 *  \return 0 if ok, error otherwise
 */
static Int32 bbuStopPeriodicScan(void)
{
    Int32 retVal = 0;

    if (bbuKernThread != NULL)
    {
        /* stop thread*/
        printk(KERN_INFO "retVal val stopping bbu kern thread\n");

        retVal = kthread_stop(bbuKernThread);
        printk(KERN_INFO "retVal val %d stopping bbu kern thread\n", retVal);

        bbuKernThread = NULL;
    }
    return retVal;
}

/**************************************************************************/
/*! \fn bbuInitData
 **************************************************************************
 *  \brief initialize driver data
 *  \param[in, out] none
 *  \return none
 */
static void bbuInitData(void)
{
    Int32 i = 0;

    memset(&bbuDriverLocalData, 0, sizeof(BbuDriverData_t));
    memset(&bbuWorkingParams, 0, sizeof(BbuWorkingParams_t));

    /* bbuWorkingParams.bbuCurrentCalValues.diff_a = bbuWorkingParams.bbuCurrentCalValues.single_a = 1;*/
    InitFloat64(ONE_MANTISSA, ONE_EXPONENT, &bbuWorkingParams.bbuCurrentCalValues.diff_a);
    InitFloat64(ONE_MANTISSA, ONE_EXPONENT, &bbuWorkingParams.bbuCurrentCalValues.single_a);

    /* bbuWorkingParams.bbuCurrentCalValues.diff_b = bbuWorkingParams.bbuCurrentCalValues.single_b = 0;*/
    InitFloat64(ZERO_MANTISSA, ZERO_EXPONENT, &bbuWorkingParams.bbuCurrentCalValues.diff_b);
    InitFloat64(ZERO_MANTISSA, ZERO_EXPONENT, &bbuWorkingParams.bbuCurrentCalValues.single_b);

    bbuDriverLocalData.safetyViolation.newViolationCauseDetected = BBU_VIOLATION_NONE;

    for (i=0; i<BBU_SAFETY_VIOLATION_REASON_LAST ; i++)
    {
        bbuDriverLocalData.safetyViolation.safetyViolationList[i] = False;
    }

	sema_init(&bbuDriverLocalData.bbuDataSema,1);
    sema_init(&bbuDriverLocalData.bbuPersistDataSema,1);

    init_waitqueue_head(&bbuDriverLocalData.bbuRqueue[0]);
    init_waitqueue_head(&bbuDriverLocalData.bbuRqueue[1]);
}

/**************************************************************************/
/*! \fn BbuTreatSafetyLimits
 **************************************************************************
 *  \brief  routine to check for presence of safety violation notification.
 *   tests the last sample values vs working parameter limits. If violation found,
 *   notifies possible waiting entity
 *  \param[in, out] adc buffer with latest samples results
 *  \return 0 if ok, error otherwise
 */
static Int32 BbuValidateAdcData(Bbu_OneShotDataType_t *adcDataP)
{
    if(memcmp(bbuAdcDataDebug.vals, adcDataP->vals, sizeof (Bbu_OneShotDataType_t) == 0))
    {
        adcDebugCounter++;
    }
    else
    {
        adcDebugCounter = 0;
        memcpy(bbuAdcDataDebug.vals, adcDataP->vals, sizeof (Bbu_OneShotDataType_t));
    }

    if (adcDebugCounter >= BBU_ADC_DATA_MAX_FAILURES)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s adc data violation\n", __FUNCTION__);
        return -1;
    }
    return 0;
}

/**************************************************************************/
/*! \fn bbuHwDisable
 **************************************************************************
 *  \brief stop bbu hardware, disable batteries and stop PWM
 *  \param[in, out] none
 *  \return none
 */
static void bbuHwDisable(void)
{
    Int32 batIndex;

    /* disable batteries*/
    for (batIndex = BBU_BAT1_INDEX; batIndex<BBU_NUM_OF_BATTERIES_SUPPORTED; batIndex++)
    {
        Bbu_EnableBattery(batIndex, False);
    }

    /* disable pwm*/
    Bbu_EnablePWM(0);
}

MODULE_AUTHOR("Texas Instruments, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BBU Driver");

module_init(bbuDriverInit);
module_exit(bbuDriverExit);

