/*
 *
 * bbu_hw_api.c
 * Description:
 * BBU Hardware API implementation
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

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <pal.h>

#include "bbu_hw_api.h"
#include "bbu_driver.h"
#include "bbu_types.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

/*! \def MAX_ADC_VALUE
 *  \brief maximum number that may be measured/returned by ADC
 *  12 bits max work (4095), multiplied by 16 (max averaging range)  
 */
#define MAX_ADC_VALUE 0xffff  

typedef enum PAL_SYS_PSC_MODULE_tag
{
    PSC_ARM = 0,        /* direct from pllctrl */
    PSC_VLYNQ,
    PSC_DSPSS,          /* async reset */
    PSC_MPEG_OUT,
    PSC_DDR_PHY,
    PSC_EMIF3E,         /* soft reset to emif3e */
    PSC_CPSPDMA,
    PSC_MCDMA,
    PSC_TDM,
    PSC_BBU,
    PSC_PERF_MON,
    PSC_DOCSIS_MAC,
    PSC_GROUP1, /* Group 1 - slow peripherals, uart0, wdt, i2c, dspintc */
    PSC_UART1,
    PSC_GPIO,
    PSC_TIMER0,
    PSC_TIMER1,
    PSC_TIMER2,
    PSC_ROM,
    PSC_ASYNC_EMIF,
    PSC_MMAP_SPI,
    PSC_DEBUG_SS,
    PSC_SRAM,
    PSC_GROUP2,         /* Buses - bootcfg, scr and bridges */
    PSC_SR_CLK0,        /* cpdsp */
    PSC_SR_CLK1,        /* mpdsp */
    PSC_SR_CLK2,        /* qpdsp */
    PSC_SR_CLK3,        /* apdsp */
    PSC_SR_CLK4,        /* LUT 400 Mhz */
    PSC_SR_CLK5,            /* usb */
    PSC_SR_CLK6,        /* ethernet + mdio */
    PSC_SR_CLK7,        /* new clock for dma1 */
    PSC_ADC_DUMP,
    PSC_RESERVED,       /* Reserved */
    PSC_USB_PHY,
    PSC_DSP_PROXY,
    PSC_EMIF3E_VRST,    /* generate vrst (por) to emif3 */
    PAL_SYS_MAX_PSC_MODULES
}PAL_SYS_PSC_MODULE_T;

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/

static void Bbu_DetermineCalibrationData(void);
static Bool BbuIsRev2(void);
static Int32 Bbu_CalculateCalibrationCoefficients(Uint16 sampleLow,
                                              Uint16 sampleHigh,
                                              bbu_float64 *a_coeff,
                                              bbu_float64 *b_coeff,
                                              BbuCurrentMode_e    diffMode);
static Uint16 Bbu_CalculateEqSample(Uint16 sample, Int32 newGain);
#if 0 /* todo to be done - configurable from working params */
static void BbuChangeGainSettings(Uint16 sample, Uint16 *pSample, Bbu_D2sGainType_e newGain);
#endif

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/

/**********************************************************************************
 *            BBU data -calibration data and safety limits                       */
/*********************************************************************************/

/*! \variable bbuWorkingParams
*  /brief   contains all working params needed by the driver and determined by the user -
*  timer values, security limits etc
*  download by bbu driver ioctl
*/
BbuWorkingParams_t bbuWorkingParams; 

/*! \variable bbuNeedCalibrate
*  /brief   bitmask to notify kernel thread whether calibration needs to be performed
*  bit 0  specifies if single calibration needed, bit 1 if diff calibration neeeded
*/
atomic_t     bbuNeedCalibrate = ATOMIC_INIT(BBU_CALIBRATION_DONE);

/*! \variable activeChannelBitmask
*  /brief   bitmask to specify which ADC channels are active
*/
static char activeChannelBitmask = (char)DEFAULT_ACTIVE_ADC_CHANNELS;

/*! \variable gainEquivalents
*  /brief  actual gain values, associated with the gain enum type
*/
static const Int32 gainEquivalents[BBU_D2S_GAIN_20+1] = {1, 10, 15, 20};

/*! \variable calibration variables for diff calibration
*  /brief set duriing initalization after chip version is determined
*    using constants specific for P5 version
*   constants for differential calibration,  are determined upon P5 version
*   (constants for single_ended calibration are the same for both versions) 
*/

/*! \variable diffMantissa, diffExponent
*  /brief set duriing initalization,  used in float64 calculations
*    represent /lowConst - highConst/ for diff calibration
*/
static Int32 diffMantissa = 0;
static Int32 diffExponent = 0;

/*! \variable lowWordMantissa, lowWordExponent
*  /brief set duriing initalization, used in float64 calculations 
*    represent /lowConst  Word/ for diff calibration 
*/
static Int32 lowWordMantissa = 0;
static Int32 lowWordExponent = 0;

/*! \variable revision2
*  /brief set duriing initalization 
* True when board with P5 rev 2 is identified
*/
static Bool revision2 = True;

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/


/**************************************************************************/
/*                  Bbu_GetRegValue()                                     */
/**************************************************************************/
/* DESCRIPTION: Reads the value in a specified register.                  */
/*                                                                        */
/* INPUT:       regAddr - address of the register                         */
/*                                                                        */
/* OUTPUT:      the value that was read from the register.                */
/*                                                                        */
/**************************************************************************/
Uint32 Bbu_GetRegValue(Uint32 regAddr)
{
    Uint32 val;

    val = *(volatile Uint32*)(regAddr);

    BBU_KDEBUG_PRINT(KERN_DEBUG, "%s: read register at offset 0x%lX, value is 0x%lX\n", __FUNCTION__, (unsigned long)regAddr, (unsigned long)val);

    return(Uint32)val;
}

/**************************************************************************/
/*                  Bbu_SetRegValue()                                     */
/**************************************************************************/
/* DESCRIPTION: Writes a given value to a specified register.             */
/*                                                                        */
/* INPUT:       regAddr - address of the register.                        */
/*              val     - value to write to the register.                 */
/*                                                                        */
/* OUTPUT:      none                                                      */
/*                                                                        */
/**************************************************************************/
void Bbu_SetRegValue(Uint32 regAddr, Uint32 val)
{
    (*(volatile Uint32 *) (regAddr)) = val;

    BBU_KDEBUG_PRINT(KERN_DEBUG, "%s: wrote register at offset 0x%lX, value is 0x%lx\n", __FUNCTION__, (unsigned long)regAddr, (unsigned long)val);
    return;
}

/**************************************************************************/
/*                    Bbu_IsHwEnabled()                                   */
/**************************************************************************/
/* DESCRIPTION: Checks if BBU is supported on chip          .             */
/* OUTPUT:      True - BBU supported.                                     */
/*              False - BBU not supported                                 */
/*                                                                        */
/**************************************************************************/
Bool Bbu_IsHwEnabled(void) 
{
    Uint32 bbuEnabledReg;
    Bool enabled = True;

    bbuEnabledReg = Bbu_GetRegValue(BBU_ENABLED_REG_PUMA5);
    BBU_KDEBUG_PRINT(KERN_INFO, "%s: read bbu enabled 0x%X, value is 0x%X\n", __FUNCTION__, BBU_ENABLED_REG_PUMA5, bbuEnabledReg);
    if ((bbuEnabledReg) & BBU_ENABLED_MASK)
    {
        BBU_KDEBUG_PRINT(KERN_WARNING, "%s: bbu not enabled !!\n", __FUNCTION__);
        enabled = False; 
    }
    return enabled;
}

/**************************************************************************/
/*                        Bbu_HalInit()                                   */
/**************************************************************************/
/* DESCRIPTION: Performs the necessary initializations and settings in    */
/*              order for the BBU hardware to operate correctly.          */
/*                                                                        */
/* INPUT:       none                                                      */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - failure.                                             */
/**************************************************************************/
void Bbu_HalInit(void)
{
    Uint32 regVal;
    Uint16 dummy;
    Int32 i;

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s start\n", __FUNCTION__);

    /* Taking the BBU module out of reset in PRCR */
    PAL_sysResetCtrl((Int32)PSC_BBU, OUT_OF_RESET);

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s device out of reset\n", __FUNCTION__);

    /* set initial clock rate to maximal rate of ~2MHz (25MHz/13) */
    Bbu_SetBbuClockDivider(PCLKCR1_PUMA5_BBU_CLK_DIV_MIN);

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s clk divider set\n", __FUNCTION__);

    /* Set ADC to Idle state. */
    Bbu_AdcSetToIdle();

    /* set averaging mode to the default - x16 */
    Bbu_SetAveragingMode(AVERAGING_MODE_16);

    /* Analog Control Register settings */
    regVal = Bbu_GetRegValue(BBU_ACTRL_PUMA5); /* read ACTRL register value */
    regVal |= BBU_ACTRL_PUMA5_DIFF_MASK;  /* set 'DIFF' to 1 */ 

    regVal &= ~BBU_ACTRL_PUMA5_GAINSEL_MASK; /* zero gain bits */
    regVal |= (BBU_D2S_GAIN_20 << BBU_ACTRL_PUMA5_GAINSEL_SHIFT); /* set gain to x20 */

    Bbu_SetRegValue(BBU_ACTRL_PUMA5, regVal); /* write ADC register */

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s average mode and gain set\n", __FUNCTION__);

    /* Enable necessary GPIOs - only PWM is relevant */

    regVal = Bbu_GetRegValue(GPIO_PUMA5_EN);/* read AUX GPIO_EN register value */

    /* disable GPIO coupled with PWM, to allow PWM signal to be outputed */
    regVal &= ~(1<<(BBU_GPIO_PWM));/* disable */

    Bbu_SetRegValue(GPIO_PUMA5_EN, regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s: GPIO  disabled (to enable pwm)\n", __FUNCTION__);

    /* PWM */
    Bbu_EnablePWM(0); /* Disable PWM */
    Bbu_SetPwmRange(BBU_PWMCR_RANGE_DEFAULT); /* Set to 200KHz */

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s pwm configured and disabled\n", __FUNCTION__);

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU generate dummy sample:\n");

    /* Generate  2 "Dummy" samples - workaround for ADC first-sample issue. */
    for (i=0; i<2; i++)
    {
        Bbu_SetUpSingleSampleAndPoll(0, &dummy);
    }

    /* determine revision and coefficients */
    Bbu_DetermineCalibrationData();

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s calibrate:\n", __FUNCTION__);

    /* Perform initial calibration */
    Bbu_AdcCalibrate(BBU_SingleEnded); /* single-ended calibration */
    Bbu_AdcCalibrate(BBU_Differential); /* differential calibration */

    BBU_KDEBUG_PRINT(KERN_INFO, "BBU %s end\n", __FUNCTION__);

    return;
}

/******************************************************************************
 *                                  PWM Control                               *
 ******************************************************************************/

/**************************************************************************/
/*                  Bbu_EnablePWM()                                       */
/**************************************************************************/
/* DESCRIPTION: Enables the PWM signal. Enabling the PWM will generate    */
/*              a signal according to the parameters in the PWM control   */
/*              register (BBU_PWMCR_PUMA5).                               */
/*                                                                        */
/* INPUT:       enable : 1 = enable PWM.                                  */
/*                       0 = disable PWM                                  */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - undefined input value.                               */
/**************************************************************************/
Int32 Bbu_EnablePWM(Int32 enable)
{
    Uint32 regVal;

    if ((enable < 0) || (enable > 1))
    {
        return -1;
    }

    /* When disabling PWM - enable appropriate AUX-GPIO , and set it to input */
    if (!enable)
    {
        regVal = Bbu_GetRegValue(GPIO_PUMA5_DIR); /* read GPIO_DIR register value */
        regVal |= (1 << (BBU_GPIO_PWM));/* input */
        Bbu_SetRegValue(GPIO_PUMA5_DIR, regVal);

        regVal = Bbu_GetRegValue(GPIO_PUMA5_EN); /* read GPIO_EN register value */
        regVal |= (1<<(BBU_GPIO_PWM));/* enable  GPIO - disables pwm*/
        Bbu_SetRegValue(GPIO_PUMA5_EN, regVal);
    }
    /* When enabling PWM - disable appropraite GPIO */
    else
    {
        regVal = Bbu_GetRegValue(GPIO_PUMA5_EN);/* read GPIO_EN register value */
        /* disable GPIO #28, to allow PWM signal to be output */
        regVal &= ~(1<<(BBU_GPIO_PWM));/* disable */
        Bbu_SetRegValue(GPIO_PUMA5_EN, regVal);
    }

    /* write PWM-enable register */
    Bbu_SetRegValue((Uint32)BBU_PWMEN_PUMA5, enable);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s PWM %s  \n", __FUNCTION__, (enable) ? "enabled" : "disabled" );

    return 0;
}

/**************************************************************************/
/*                  Bbu_SetPwmDutyCycle()                                 */
/**************************************************************************/
/* DESCRIPTION: Sets the Duty-Cycle parameter in the PWM config register. */
/*                                                                        */
/* INPUT:       val : requested value of duty-cycle parameter.            */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid input value.                                 */
/**************************************************************************/
Int32 Bbu_SetPwmDutyCycle(Uint32 val)
{
    Uint32 regVal;

    if ((val < 0) || (val > BBU_PWMCR_PUMA5_MAX_VAL))
    {
        return -1;
    }

    regVal = Bbu_GetRegValue(BBU_PWMCR_PUMA5);  /* read current register value */
    regVal &= ~(BBU_PWMCR_PUMA5_DC_MASK);        /* reset duty-cycle bits*/
    regVal |= (val<<BBU_PWMCR_PUMA5_DC_SHIFT);   /* set duty-cycle bits to requested value */
    Bbu_SetRegValue(BBU_PWMCR_PUMA5, regVal); /* write to register*/

    BBU_KDEBUG_PRINT(KERN_INFO, "%s PWM duty cycle  0x%lx\n", __FUNCTION__, (unsigned long)val);

    return 0;
}

/**************************************************************************/
/*                  Bbu_GetPwmDutyCycle()                                 */
/**************************************************************************/
/* DESCRIPTION: Reads the Duty-Cycle parameter from the PWM config        */
/*              register.                                                 */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      Value of the Duty-Cycle parameter.                        */
/**************************************************************************/
Uint32 Bbu_GetPwmDutyCycle(void)
{
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_PWMCR_PUMA5);  /* read current register value */
    regVal = (regVal >> BBU_PWMCR_PUMA5_DC_SHIFT); /* shift duty-cycle bits */
    regVal &= BBU_PWMCR_PUMA5_MAX_VAL;             /* reset upper bits */

    BBU_KDEBUG_PRINT(KERN_INFO, "%s PWM duty cycle 0x%lx \n", __FUNCTION__, (unsigned long)regVal);

    return regVal;
}

/**************************************************************************/
/*                     Bbu_SetPwmRange()                                  */
/**************************************************************************/
/* DESCRIPTION: Sets the Range parameter in the PWM config register.      */
/*                                                                        */
/* INPUT:       val : requested value of range parameter.                 */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid input value.                                 */
/**************************************************************************/
Int32 Bbu_SetPwmRange(Uint32 val)
{
    Uint32 regVal;

    /* note : value 0 is restricted! */
    if ((val <= 0) || (val > BBU_PWMCR_PUMA5_MAX_VAL))
    {
        return -1;
    }

    regVal = Bbu_GetRegValue(BBU_PWMCR_PUMA5);  /* read current register value */             
    regVal &= ~(BBU_PWMCR_PUMA5_RANGE_MASK);     /* reset range bits*/                    
    regVal |= val;                               /* set range bits to requested value */  
    Bbu_SetRegValue(BBU_PWMCR_PUMA5, regVal); /* write to register*/                        

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: set PWM Range parameter to 0x%lx\n", __FUNCTION__, (unsigned long)val);
    return 0;
}

/**************************************************************************/
/*                  Bbu_GetPwmRange()                                     */
/**************************************************************************/
/* DESCRIPTION: Reads the range parameter from the PWM config             */
/*              register.                                                 */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      Value of the range parameter.                             */
/**************************************************************************/
Uint32 Bbu_GetPwmRange(void)
{
    Uint32 regVal;

    regVal=Bbu_GetRegValue(BBU_PWMCR_PUMA5);  /* read current register value */
    regVal &= BBU_PWMCR_PUMA5_MAX_VAL;                            /* reset upper bits */

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: PWM range parameter is 0x%lx\n", __FUNCTION__, (unsigned long)regVal);

    return regVal;
}

/******************************************************************************
 *                                  ADC Control                               *
 ******************************************************************************/

/**************************************************************************/
/*                         Bbu_AdcGetMode()                               */
/**************************************************************************/
/* DESCRIPTION: Get the current ADC mode (by reading the "mode" variable).*/
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      adcMode                                                   */
/**************************************************************************/
Bbu_AdcModeType_e Bbu_AdcGetMode(void)
{
    Uint32 regVal;
    Bbu_AdcModeType_e adcMode = ADC_MODE_UNKNOWN;

    regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);

    /* check if HW returned to IDLE mode, and update variable if necessary. */
    if (regVal & BBU_MODECR_PUMA5_IDLE_MASK)
    {
        adcMode = ADC_MODE_IDLE;
    }
    return adcMode;
}

/**************************************************************************/
/*                        Bbu_AdcSetToIdle()                              */
/**************************************************************************/
/* DESCRIPTION: Sets the ADC to Idle mode.                                */
/*              Note - The function polls the ADC status, and returns     */  
/*                     only after the ADC is in Idle mode, or if a        */
/*                     polling-timeout expires.                           */
/*                                                                        */
/* INPUT:       none                                                      */
/*                                                                        */
/* OUTPUT:      0  - ADC has been set to Idle mode                        */
/*              -1 - Polling timeout has expired                          */
/**************************************************************************/
Int32 Bbu_AdcSetToIdle(void)
{
    Int32 count = 0;
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);
    if (regVal & BBU_MODECR_PUMA5_IDLE_MASK)
    {
        return 0;
    }

    /* keeping the current avaraging configuration*/
    regVal &= (BBU_MODECR_PUMA5_AVERAGE_MASK | BBU_MODECR_PUMA5_AVGNUM_MASK);

    /* setting the "stop" bit */
    regVal |= BBU_MODECR_PUMA5_STOP_MASK;

    Bbu_SetRegValue(BBU_MODECR_PUMA5, regVal);

    /* polling on the Idle bit */
    while (count < MAX_ADC_IDLE_POLL*2)
    {
        count++;
        regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);
        if (regVal & BBU_MODECR_PUMA5_IDLE_MASK)
        {
            BBU_KDEBUG_PRINT(KERN_INFO, "%s: ADC set to Idle mode\n", __FUNCTION__);
            return 0;
        }
    }

    printk(KERN_WARNING "%s: failed to set ADC to Idle mode, count = %d\n", __FUNCTION__, count);

    return -1;
}

/**************************************************************************/
/*                        Bbu_AdcReset()                                  */
/**************************************************************************/
/* DESCRIPTION: reset the ADC and BBU control.                           */
/*                                                                        */
/* INPUT:       reset - indicates if setting IN Reset or OUT of Reset                                                       */
/*                                                                        */
/* OUTPUT:      0  - reset successfully                                   */
/*              -1 - Error                                                */
/**************************************************************************/
Int32 Bbu_AdcReset(Bool reset)
{
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_ADC_CNTL);

    if (reset)
    {        
        /* Set Next state as reset for the module */
        regVal = (regVal & 0xffffffe0) | 0x01;
        Bbu_SetRegValue(BBU_ADC_CNTL, regVal);
    }
    else
    {   
        /* Set Next state as Enabled for the module */
        regVal = (regVal & 0xffffffe0) | 0x03;
        Bbu_SetRegValue(BBU_ADC_CNTL, regVal);
    }

    /* Enable the Power Domain Transition Command */
    Bbu_SetRegValue(BBU_ADC_ACT, 1);

    /* Check for Transition Complete(PTSTAT) */
    while (Bbu_GetRegValue(BBU_ADC_STS) & 1);
   
    return(0);
}

/**************************************************************************/
/*                        Bbu_SetAdcDftMode()                             */
/**************************************************************************/
/* DESCRIPTION: Sets the ADC to DFT operating mode.                       */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - failure.                                             */
/**************************************************************************/
Int32 Bbu_SetAdcDftMode(void)
{
    Uint32 regVal;

    /* in order to change the ADC operating mode, it must first be set to idle */
    if (Bbu_AdcSetToIdle() == -1)
    {
        return -1;
    }

    regVal=Bbu_GetRegValue(BBU_MODECR_PUMA5);
    regVal &= (BBU_MODECR_PUMA5_AVERAGE_MASK | BBU_MODECR_PUMA5_AVGNUM_MASK);
    regVal |= BBU_MODECR_PUMA5_DFT_MASK;

    Bbu_SetRegValue(BBU_MODECR_PUMA5,regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: ADC set to DFT mode\n", __FUNCTION__);

    return 0;
}

/**************************************************************************/
/*                     Bbu_SetUpSingleSample()                            */
/**************************************************************************/
/* DESCRIPTION: Sets the ADC to perform one sample on a single channel.   */
/*              Note: This function sets up the ADC, but does not retrieve*/
/*                    the sampled value.                                  */
/*                                                                        */
/* INPUT:       channel - the index of the channel to be sampled.         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid channel number or other failure.             */
/**************************************************************************/
Int32 Bbu_SetUpSingleSample(Int32 channel)
{
    Uint32 regVal;

    if ((channel < 0) || (channel > MAX_ADC_CHANNEL_INDEX))
    {
        return -1;
    }

    if (!((Int32)activeChannelBitmask & (1<<channel)))
    {
        return -1;
    }

    /* make sure ADC is idle */
    if (Bbu_AdcSetToIdle() == -1)
    {
        return -1;
    }

    /* set the channel to be sampled */
    Bbu_SetRegValue(BBU_ADCCHNL_PUMA5, channel);

    regVal =  Bbu_GetRegValue(BBU_MODECR_PUMA5);
    regVal &= (BBU_MODECR_PUMA5_AVERAGE_MASK | BBU_MODECR_PUMA5_AVGNUM_MASK);
    regVal |= BBU_MODECR_PUMA5_SINGLE_MASK;

    Bbu_SetRegValue(BBU_MODECR_PUMA5, regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: set up single sample on channel #%d\n", __FUNCTION__, channel);

    return 0;
}

/**************************************************************************/
/*                   Bbu_SetUpSingleSampleAndPoll()                       */
/**************************************************************************/
/* DESCRIPTION: Sets the ADC to perform one sample on a single channel,   */
/*              polls the valid bit until data is ready, and returns the  */
/*              sampled value.                                            */
/*                                                                        */
/* INPUT:       channel - the index of the channel to be sampled.         */
/*              pData   - pointer to where the sampled data should be     */
/*                        stored.                                         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid channel number or other failure.             */
/**************************************************************************/
Int32 Bbu_SetUpSingleSampleAndPoll(Int32 channel, Uint16 *pData)
{
    Uint32 regVal;
    Int32 count = 0;
    Int32 idleCount = 0;

    if (Bbu_SetUpSingleSample(channel) == -1)
    {
        return -1;
    }

    while (count < MAX_ADC_SAMPLE_POLL)
    {
        regVal = Bbu_GetRegValue(BBU_ADCDATA_PUMA5);

        if (regVal & BBU_ADCDATA_PUMA5_VALID_MASK)
        {
            (*pData) = (Uint16)(regVal & BBU_ADCDATA_PUMA5_DATA_MASK);

            BBU_KDEBUG_PRINT(KERN_INFO, "%s: single sample retrieved, channel = %d ; value = %lX ; count = 0x%x\n", __FUNCTION__, channel, (unsigned long)(*pData), count);

            /* making sure that the controller returns to Idle state */
            regVal = 0;
            while ((!regVal) && (idleCount < MAX_ADC_SAMPLE_POLL))
            {
                regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);
                regVal &= BBU_MODECR_PUMA5_IDLE_MASK;
                idleCount++;
            }

            if (idleCount >= MAX_ADC_SAMPLE_POLL)
            {
                return -1;
            }

            return 0;
        }
        count++;
    }
    return -1;
}

/**************************************************************************/
/*                      Bbu_GetSingleSampleValue()                        */
/**************************************************************************/
/* DESCRIPTION: Returns the sampled value after a single sample.          */
/*              Note: ADC must be in Idle mode for this function.         */
/*                                                                        */
/* INPUT:       channel - the index of the channel to be sampled.         */
/*              pData   - pointer to where the sampled data should be     */
/*                        stored.                                         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid channel number or other failure.             */
/**************************************************************************/
Int32 Bbu_GetSingleSampleValue(Uint16 *pData)
{
    Uint32 regVal; 

    if (Bbu_AdcGetMode() != ADC_MODE_IDLE)
    {
        return -1;
    }

    regVal = Bbu_GetRegValue(BBU_ADCDATA_PUMA5);
    (*pData) = (Uint16)(regVal & BBU_ADCDATA_PUMA5_DATA_MASK);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: single sample retrieved, value = 0x%x\n", __FUNCTION__, (unsigned int)(*pData));

    return 0;
}

/**************************************************************************/
/*                    Bbu_SetUpOneShotSample()                            */
/**************************************************************************/
/* DESCRIPTION: Set the ADC to perform a sample on each channel (one-Shot)*/
/*              Note: This function sets up the ADC, but does not retrieve*/
/*                    the sampled data.                                   */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - invalid channel number or other failure.             */
/**************************************************************************/
Int32 Bbu_SetUpOneShotSample(void)
{
    Uint32 regVal;

    /* make sure ADC is idle */
    if (Bbu_AdcSetToIdle() == -1)
    {
        return -1;
    }

    regVal =  Bbu_GetRegValue(BBU_MODECR_PUMA5);
    regVal &= (BBU_MODECR_PUMA5_AVERAGE_MASK | BBU_MODECR_PUMA5_AVGNUM_MASK);
    regVal |= BBU_MODECR_PUMA5_ONE_SHOT_MASK;

    Bbu_SetRegValue(BBU_MODECR_PUMA5, regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: set up one-shot sample\n", __FUNCTION__);

    return 0;
}

/**************************************************************************/
/*                      Bbu_OneShotSamplePoll()                           */
/**************************************************************************/
/* DESCRIPTION: Polls the valid bit until data is ready, and returns the  */
/*              sampled value.                                            */
/*                                                                        */
/* INPUT:       pData   - pointer to where the sampled data should be     */
/*                        stored.                                         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - failure.                                             */
/**************************************************************************/
Int32 Bbu_OneShotSamplePoll(Bbu_OneShotDataType_t *pData)
{
    Uint32 regVal;
    Int32 count = 0;
    Int32 channel;
    Int32 idleCount = 0;

    while (count < MAX_ADC_SAMPLE_POLL)
    {
        regVal = Bbu_GetRegValue(BBU_ONE_SHOT_VALID_REG_PUMA5);

        if (regVal & BBU_ONE_SHOT_VALID_PUMA5_MASK)
        {
            for (channel=0; channel<=MAX_ADC_CHANNEL_INDEX; channel++)
            {
                regVal = Bbu_GetRegValue(BBU_ONE_SHOT_CHANNEL_REG_PUMA5(channel));
                (pData->vals)[channel] = (Uint16)(regVal & BBU_ONE_SHOT_PUMA5_DATA_MASK);
            }

            BBU_KDEBUG_PRINT(KERN_INFO, "%s: one-Shot sample retrieved\n", __FUNCTION__);

            /* making sure that the controller returns to Idle state */
            regVal = 0;
            while ((!regVal) && (idleCount < MAX_ADC_SAMPLE_POLL))
            {
                regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);
                regVal &= BBU_MODECR_PUMA5_IDLE_MASK;
                idleCount++;
            }

            if (idleCount >= MAX_ADC_SAMPLE_POLL)
            {
                return -1;
            }

            return 0;
        }
    }
    return -1;
}

/**************************************************************************/
/*                  Bbu_SetUpOneShotSampleAndPoll()                       */
/**************************************************************************/
/* DESCRIPTION: Sets the ADC to perform one-shot sampling.                */
/*              polls the valid bit until data is ready, and returns the  */
/*              sampled value.                                            */
/*                                                                        */
/* INPUT:       pData   - pointer to where the sampled data should be     */
/*                        stored.                                         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - failure.                                            */
/**************************************************************************/
Int32 Bbu_SetUpOneShotSampleAndPoll(Bbu_OneShotDataType_t *pData)
{
    if (Bbu_SetUpOneShotSample() == -1)
    {
        return -1;
    }

    return Bbu_OneShotSamplePoll(pData);
}

/**************************************************************************/
/*                    Bbu_GetOneShotSampleValues()                        */
/**************************************************************************/
/* DESCRIPTION: Returns the sampled values after a one-shot sample.       */
/*              Note: ADC must be in Idle mode for this function.         */
/*                                                                        */
/* INPUT:       pData   - pointer to where the sampled data should be     */
/*                        stored.                                         */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - ADC not in Idle mode.                                */
/**************************************************************************/
Int32 Bbu_GetOneShotSampleValues(Bbu_OneShotDataType_t *pData)
{
    Int32 channel; 
    Uint32 regVal; 

    if (Bbu_AdcGetMode() != ADC_MODE_IDLE)
    {
        return -1;
    }

    for (channel=0; channel <= MAX_ADC_CHANNEL_INDEX; channel++)
    {
        regVal = Bbu_GetRegValue(BBU_ONE_SHOT_CHANNEL_REG_PUMA5(channel));
        (pData->vals)[channel] = (Uint16)(regVal & BBU_ONE_SHOT_PUMA5_DATA_MASK);
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: one-Shot sample retrieved\n", __FUNCTION__);

    return 0;
}

/**************************************************************************/
/*                  Bbu_SetAveragingMode()                                */
/**************************************************************************/
/* DESCRIPTION: Configures the BBU to a specified averaging mode          */
/*              Note: Updates the continuous-limits after setting the new */
/*                    averaging mode.                                     */
/*                                                                        */
/* INPUT:       mode - Requested averaging mode.                          */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - undefined input value.                               */
/**************************************************************************/
Int32 Bbu_SetAveragingMode(Bbu_AveragingModeType_e mode)
{
    Uint32 regVal;
    Bbu_AveragingModeType_e oldMode;

    if ((mode != AVERAGING_MODE_1) &&
        (mode != AVERAGING_MODE_2) &&
        (mode != AVERAGING_MODE_4) &&
        (mode != AVERAGING_MODE_8) &&
        (mode != AVERAGING_MODE_16))
    {
        printk(KERN_ERR "%s: wrong averaging mode %d\n", __FUNCTION__, mode);
        return -1;
    }

    /* get the current mode */
    regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5); 
    if (!(regVal & BBU_MODECR_PUMA5_AVERAGE_MASK))
    {
        oldMode = AVERAGING_MODE_1;
    }
    else
    {
        oldMode = (regVal & BBU_MODECR_PUMA5_AVGNUM_MASK) >> BBU_MODECR_PUMA5_AVGNUM_SHIFT;
    }

    /* check if the mode needs to be changed */
    if (oldMode == mode)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s:  new averaging mode is the same as previous mode.\n", __FUNCTION__);
        return 0;
    }

    if (mode == AVERAGING_MODE_1) /* no averaging */
    {
        regVal &= (~BBU_MODECR_PUMA5_AVERAGE_MASK);
        Bbu_SetRegValue(BBU_MODECR_PUMA5, regVal);
        BBU_KDEBUG_PRINT(KERN_INFO, "%s: set averaging mode\n", __FUNCTION__);

        return 0;
    }

    regVal |= BBU_MODECR_PUMA5_AVERAGE_MASK;         /* turn averaging on                     */
    regVal &= (~BBU_MODECR_PUMA5_AVGNUM_MASK);       /* set averaging bits to 0.              */
    regVal |= (mode<<BBU_MODECR_PUMA5_AVGNUM_SHIFT); /* set averaging bits to requested mode. */
    Bbu_SetRegValue(BBU_MODECR_PUMA5, regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: Set averaging mode\n", __FUNCTION__);

    return 0;
}

/**************************************************************************/
/*                  Bbu_GetAveragingMode()                                */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU-ADC averaging mode                          */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Bbu_AveragingModeType_e Bbu_GetAveragingMode(void)
{
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_MODECR_PUMA5);

    if (!(regVal & BBU_MODECR_PUMA5_AVERAGE_MASK))
    {
        return AVERAGING_MODE_1;
    }

    regVal &= BBU_MODECR_PUMA5_AVGNUM_MASK;
    regVal >>= BBU_MODECR_PUMA5_AVGNUM_SHIFT;

    return(Bbu_AveragingModeType_e)regVal;
}

/**************************************************************************/
/*                  Bbu_SetBbuClockDivider()                              */
/**************************************************************************/
/* DESCRIPTION: Configures the BBU clock divider to a given value.        */
/*              Note: in order to set the divider the BBU clock must be   */
/*              disabled (enable bit = 0). tbd                            */
/*                                                                        */
/* INPUT:       divider - Requested divider value.                        */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - undefined input value.                               */
/**************************************************************************/
Int32 Bbu_SetBbuClockDivider(Uint16 divider)
{
    Uint32 regVal;

    if ((divider < PCLKCR1_PUMA5_BBU_CLK_DIV_MIN) ||
        (divider > PCLKCR1_PUMA5_BBU_CLK_DIV_MAX))
    {
        printk(KERN_ERR"%s: invalid divider value (%x)\n", __FUNCTION__, divider);
        return -1;
    }

    regVal = Bbu_GetRegValue(BBU_CR);
    regVal &= (~PCLKCR1_PUMA5_BBU_CLK_DIV_MASK);     /* set divider bits to 0.               */
    regVal |= (divider<<BBU_CR_DIV_SHIFT);           /* set divider bits to requested value. */
    Bbu_SetRegValue(BBU_CR, regVal);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: set clock divider to value 0x%x\n", __FUNCTION__, divider);

    return 0;
}

/**************************************************************************/
/*                  Bbu_GetBbuClockDivider()                              */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU clock divider.                              */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Uint16 Bbu_GetBbuClockDivider(void)
{
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_CR);
    regVal &= PCLKCR1_PUMA5_BBU_CLK_DIV_MASK;
    regVal = (regVal >> BBU_CR_DIV_SHIFT);

    return(Uint16)regVal;
}

/**************************************************************************/
/*                  Bbu_SetD2SAmplification()                             */
/**************************************************************************/
/* DESCRIPTION: Configures the Gain level of the D2S to a given value.    */
/*              Note: This function assumes that the D2S Amp is powered-up*/
/*              and that the differential mode is enabled (both settings  */
/*              are performed in the BBU_ACTRL_PUMA5 register.)           */
/*              Also - the ADC must be in Idle mode before calling this   */
/*                     function.                                          */
/*                                                                        */
/* INPUT:       level - Requested amplification level.                    */
/*                                                                        */
/* OUTPUT:      0  - success.                                             */
/*              -1 - undefined input value.                               */
/**************************************************************************/
Int32 Bbu_SetD2SAmplification(Bbu_D2sGainType_e level)
{
    Uint32 regVal;
    Uint32 newGain;
    Uint32 currentCalibStatus;

    if ((level != BBU_D2S_GAIN_1) &&
        (level != BBU_D2S_GAIN_10) &&
        (level != BBU_D2S_GAIN_15) &&
        (level != BBU_D2S_GAIN_20))
    {
        printk(KERN_ERR "%s: failed to set D2S gain - unknown level #%d\n", __FUNCTION__, level);

        return -1;
    }

    /* Check if BBU ADC control HW is in IDLE mode. */
    if (Bbu_AdcGetMode() != ADC_MODE_IDLE)
    {
        printk(KERN_ERR "%s: failed to set D2S gain - ADC not in IDLE mode\n", __FUNCTION__);
        return -1;
    }

    regVal = Bbu_GetRegValue(BBU_ACTRL_PUMA5); 
    newGain = ((regVal & BBU_ACTRL_PUMA5_GAINSEL_MASK) >> BBU_ACTRL_PUMA5_GAINSEL_SHIFT);
    if (level == newGain)
    {
        BBU_KDEBUG_PRINT(KERN_INFO, "%s: D2S gain already at level #%d\n", __FUNCTION__, level);
        return 0;
    }

    regVal &= (~BBU_ACTRL_PUMA5_GAINSEL_MASK);        /* set gain bits to 0.                    */
    regVal |= (level << BBU_ACTRL_PUMA5_GAINSEL_SHIFT); /* set averaging bits to requested level. */
    Bbu_SetRegValue(BBU_ACTRL_PUMA5, regVal);

    /* after changing the amplification level, differential calibration MUST be performed */

    currentCalibStatus = atomic_read(&bbuNeedCalibrate);

    if (!BBU_DIFF_CALIBRATION_SET(currentCalibStatus))
    {
        BBU_SET_DIFF_CALIBRATION_NEEDED(currentCalibStatus);
        atomic_set(&bbuNeedCalibrate, currentCalibStatus);
        BBU_KDEBUG_PRINT(KERN_INFO, "%s  calibrate diff, bbuNeedCalibrate: %d \n", __FUNCTION__, atomic_read(&bbuNeedCalibrate));
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: set D2S gain to level #%d\n", __FUNCTION__, level);

    return 0;
}

/**************************************************************************/
/*                 Bbu_GetD2SAmplification()                              */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU D2 amplicifation.                           */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Bbu_D2sGainType_e Bbu_GetD2SAmplification(void)
{
    Uint32 regVal;

    regVal = Bbu_GetRegValue(BBU_ACTRL_PUMA5);
    regVal &= BBU_ACTRL_PUMA5_GAINSEL_MASK;     
    regVal >>= BBU_ACTRL_PUMA5_GAINSEL_SHIFT;

    return(Bbu_D2sGainType_e)regVal;
}



/* todo see if routine can be made more concise*/
/**************************************************************************/
/*                       Bbu_GainControl()                                */
/**************************************************************************/
/* DESCRIPTION: Received a sample from the differential channel, and      */
/*              checks if the gain-setting needs to change.               */
/*              If required, the gain setting is changed, and an          */
/*              equivalent sample is calculated according to the new gain.*/
/*                                                                        */
/* INPUT:       sample - last sample from the differential channel.       */
/*                                                                        */
/* OUTPUT:      True  - setting was changed.                              */
/*              FALSE - setting was NOT changed.                          */
/**************************************************************************/
Int32 Bbu_GainControl(Uint16 *pSample)
{
    Bbu_D2sGainType_e gainLevel;    
    Int32             rc = False; 
    Uint16            sample;

    sample = Bbu_AveragingCompensation(*pSample, AVERAGED_TO_PURE_CONVERSION);
    gainLevel = Bbu_GetD2SAmplification();

    switch (gainLevel)
    {
    case BBU_D2S_GAIN_20:
        {
            /* do we need to change gain to x15? */
            if (sample >= bbuWorkingParams.bbuGainLimits[BBU_GAIN_20_MAX_VALUE])
            {
                /* Set ADC to Idle state. */
                Bbu_AdcSetToIdle();

                /* change gain setting */
                Bbu_SetD2SAmplification(BBU_D2S_GAIN_15);  

                /* indicate that the setting has been changed */
                rc = True;

                /* calculate an equivalent sample in the new gain */
                *pSample = Bbu_CalculateEqSample(sample, 15);
                *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);
            }
#if 0 
            BbuChangeGainSettings(sample, pSample, BBU_D2S_GAIN_15);
            rc = True;
#endif
            break;
        }

    case BBU_D2S_GAIN_15:
        {
            /* do we need to change gain to x10? */
            if (sample >= bbuWorkingParams.bbuGainLimits[BBU_GAIN_15_MAX_VALUE])
            {
                /* Set ADC to Idle state. */
                Bbu_AdcSetToIdle();

                /* change gain setting */
                Bbu_SetD2SAmplification(BBU_D2S_GAIN_10);  

                /* indicate that the setting has been changed */
                rc = True;

                /* calculate an equivalent sample in the new gain */
                *pSample = Bbu_CalculateEqSample(sample, 10);
                *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);
            }

            /* do we need to change gain to x20? */
            else if (sample <= bbuWorkingParams.bbuGainLimits[BBU_GAIN_15_MIN_VALUE])
            {
                /* Set ADC to Idle state. */
                Bbu_AdcSetToIdle();

                /* change gain setting */
                Bbu_SetD2SAmplification(BBU_D2S_GAIN_20);

                /* indicate that the setting has been changed */
                rc = True;

                /* calculate an equivalent sample in the new gain */
                *pSample = Bbu_CalculateEqSample(sample, 20);
                *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);
            }
            break;
        }

    case BBU_D2S_GAIN_10:
        {
            /* do we need to change gain to x1? */
            if (sample >= bbuWorkingParams.bbuGainLimits[BBU_GAIN_10_MAX_VALUE])
            {
                /* Set ADC to Idle state. */
                Bbu_AdcSetToIdle();

                /* change gain setting */
                Bbu_SetD2SAmplification(BBU_D2S_GAIN_1);  

                /* indicate that the setting has been changed */
                rc = True;

                /* calculate an equivalent sample in the new gain */
                *pSample = Bbu_CalculateEqSample(sample, 1);
                *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);

            }

            /* do we need to change gain to x15? */
            else
            {
                if (sample <= bbuWorkingParams.bbuGainLimits[BBU_GAIN_10_MIN_VALUE])
                {
                    /* Set ADC to Idle state. */
                    Bbu_AdcSetToIdle();

                    /* change gain setting */
                    Bbu_SetD2SAmplification(BBU_D2S_GAIN_15);

                    /* indicate that the setting has been changed */
                    rc = True;

                    /* calculate an equivalent sample in the new gain */
                    *pSample = Bbu_CalculateEqSample(sample, 15);
                    *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);

                }
            }
            break;
        }
    case BBU_D2S_GAIN_1:
        {
            /* do we need to change gain to x10? */
            if (sample <= bbuWorkingParams.bbuGainLimits[BBU_GAIN_1_MIN_VALUE])
            {
                /* Set ADC to Idle state. */
                Bbu_AdcSetToIdle();

                /* change gain setting */
                Bbu_SetD2SAmplification(BBU_D2S_GAIN_10);

                /* indicate that the setting has been changed */
                rc = True;

                /* calculate an equivalent sample in the new gain */
                *pSample = Bbu_CalculateEqSample(sample, 10);
                *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);
            }
            break;
        }
    default:
        break;
    }

    return rc;
}

/******************************************************************************
 *                              Battery Interface                             *
 ******************************************************************************/

/**************************************************************************/
/*                  Bbu_EnableBatteryDelay()                              */
/**************************************************************************/
/* DESCRIPTION: implements a delay, used for the battery-switching during */
/*              charge and discharge.                                     */
/*              Note: can not handle delay that is longer than one cycle  */
/*                    of the MIPS free-running counter.                   */
/*                                                                        */
/* INPUT:       microSec - number of micro second to delay.               */
/*                                                                        */
/* OUTPUT:      none.                                                     */
/*                                                                        */
/**************************************************************************/

/*todo: need to update this function for 2 batteries */
void Bbu_EnableBatteryDelay(Uint16 microSec)
{
#if 0/* need to update this function, currently not used*/
    Uint32 numOfCounterIncs/*, endCount*/;

    if (!microSec)
    {
        return;
    }

    /* TODO count the correct time*/

    /* MIPS reg increments once every TWO CPU clocks. therefore:          */
    /* number of reg increments in usec =                                 */
    /*      number of CPU clocks in on second / 2 / number of usec in sec */
    numOfCounterIncs = (/*TNETD53XX_MIPS_FREQ/*/(2*1000000))* microSec;

    /*endCount = ((Uint32)sysCountGet()) + numOfCounterIncs;*/

    /* If there was a wrap when endCount was calculated */
    /* while(endCount < (Uint32)sysCountGet());*/

    /*while(endCount > (Uint32)sysCountGet());*/
#else
 #if 0 /* cant be done in interrupt disable - review when 2 batteries are used*/
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(microSec/1000);
   #endif
#endif
}

/**************************************************************************/
/*                  Bbu_EnableBattery()                                   */
/**************************************************************************/
/* DESCRIPTION: Enables/disables the requested battery                    */
/*              Note: for battery #1 reverse the logic of the enable line.*/
/*                                                                        */
/* INPUT:       index - index of battery to be enabled.                   */
/*              enable - 1=enable; 0=disable                              */
/*                                                                        */
/* OUTPUT:      0 - success                                               */
/*              -1 - invalid input value.                                 */
/**************************************************************************/
Int32 Bbu_EnableBattery(Uint16 index, Uint16 enable)
{
    Uint32 bbuCrVal;
    Uint32 activeBat;

    if ((index != BBU_BAT1_INDEX) && (index != BBU_BAT2_INDEX))
    {
        return -1;
    }
    if ((enable != 0) && (enable != 1))
    {
        return -1;
    }

    bbuCrVal = Bbu_GetRegValue(BBU_CR);

    activeBat = (index == BBU_BAT1_INDEX) ? BBU_EN_BAT1_MASK : BBU_EN_BAT2_MASK;

    if (enable == 1)
    {
        bbuCrVal |= activeBat;
    }
    else /*enable = 0*/
    {
        bbuCrVal &= ~activeBat;
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s:  battery #%d to status %d \n", __FUNCTION__, index, enable);

    Bbu_SetRegValue((Uint32)BBU_CR, bbuCrVal);

    return 0;
}

/**************************************************************************/
/*            Bbu_SetActiveBattery_discharge()                            */
/**************************************************************************/
/* DESCRIPTION: Enables the requested battery, and disables the other.    */
/*              Note: During discharge, this process allows for a         */
/*                    momentary state where both batteries are enabled.   */
/*                                                                        */
/* INPUT:       index - index of battery to be set to active (0 or 1).    */
/*                                                                        */
/* OUTPUT:      0 - success                                               */
/*              -1 - invalid battery index.                               */
/**************************************************************************/
Int32 Bbu_SetActiveBattery_discharge(Uint16 index)
{
    Int32 newActiveBattery = index;
    Int32 prevActiveBattery; 
    unsigned long flags;

    if (newActiveBattery == BBU_BAT1_INDEX)
    {
        prevActiveBattery = BBU_BAT2_INDEX;
    }
    else
    {
        if (index == BBU_BAT2_INDEX)
        {
            prevActiveBattery = BBU_BAT1_INDEX;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s: invalid battery #%d \n", __FUNCTION__, index);
            return -1;
        }
    }

    /* During discharge this action MUST be atomic */

    /*disable interrupts */

    local_irq_save(flags); /* to disable */

    Bbu_EnableBattery(newActiveBattery, True);
    Bbu_EnableBatteryDelay(BBU_BAT_EN_DELAY_USEC);
    Bbu_EnableBattery(prevActiveBattery, False);

    /*enable interrupts*/
    local_irq_restore(flags); /* to enable */ 

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: enabled battery #%d (discharge mode)\n", __FUNCTION__, index);

    return 0;
}

/**************************************************************************/
/*                    Bbu_SetActiveBattery()                              */
/**************************************************************************/
/* DESCRIPTION: Enables the requested battery, and disables the other.    */
/*              Note: used during maintain and charge. This process       */
/*                    first closes the other battery, and only then opens */
/*                    the requested one.                                  */
/*                                                                        */
/* INPUT:       index - index of battery to be set to active (0 or 1).    */
/*                                                                        */
/* OUTPUT:      0 - success                                               */
/*              -1 - invalid battery index.                               */
/**************************************************************************/
Int32 Bbu_SetActiveBattery(Uint16 index)
{
    Int32 newActiveBattery = index;
    Int32 prevActiveBattery;

    if (index == BBU_BAT1_INDEX)
    {
        prevActiveBattery = BBU_BAT2_INDEX;
    }
    else
    {
        if (index == BBU_BAT2_INDEX)
        {
            prevActiveBattery = BBU_BAT1_INDEX;
        }
        else
        {
            BBU_KDEBUG_PRINT(KERN_ERR, "%s: invalid battery #%d \n", __FUNCTION__, index);
            return -1;
        }
    }

    /* During charge this action does NOT need to be atomic */

    Bbu_EnableBattery(prevActiveBattery, False);
    Bbu_EnableBatteryDelay(BBU_BAT_EN_DELAY_USEC);
    Bbu_EnableBattery(newActiveBattery, True);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: enabled battery #%d (charge mode)\n", __FUNCTION__, index);

    return 0;
}

/**************************************************************************/
/*                   Bbu_GetEnabledBattery()                              */
/**************************************************************************/
/* DESCRIPTION: Returns the battery that is currently Enabled.            */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      -1 - no Enabled battery.                                  */
/*              0 - battery #1 is Enabled.                                */
/*              1 - battery #2 is Enabled (only in 2-battery designs).    */
/**************************************************************************/
Int32 Bbu_GetEnabledBattery(void)
{
    Uint32 bbuCrVal;

    bbuCrVal = Bbu_GetRegValue(BBU_CR);
    bbuCrVal &= BBU_EN_BAT1_MASK;
    if (bbuCrVal)
    {
        return BBU_BAT1_INDEX; 
    }

    bbuCrVal = Bbu_GetRegValue(BBU_CR);
    bbuCrVal &= BBU_EN_BAT2_MASK;
    if (bbuCrVal)
    {
        return BBU_BAT2_INDEX;
    }

    return BBU_NO_BATTERIES;
}

/**************************************************************************/
/*               Bbu_EmergencyBatteryShutDown()                           */
/**************************************************************************/
/* DESCRIPTION: Returns the battery that is currently Enabled.            */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      none.                                                     */
/**************************************************************************/
void Bbu_EmergencyBatteryShutDown(void)
{
    /* Close battery #1 */
    Bbu_EnableBattery(BBU_BAT1_INDEX, False);
    /* Close battery #2 */
    Bbu_EnableBattery(BBU_BAT2_INDEX, False);
}

/******************************************************************************
 *                    Calibration & Value Conversion                          *
 ******************************************************************************/

/**************************************************************************/
/*                       Bbu_AdcCalibrate()                               */
/**************************************************************************/
/* DESCRIPTION: Performs calibration and updates the calibration          */
/*              coefficients.                                             */
/*              Notes:                                                    */
/*                   1) This function must be called ONLY when the ADC is */
/*                      in IDLE mode (this forces the higher level to     */
/*                      stop any previous ADC task before performing      */
/*                      calibration).                                     */
/*                   2) This function assumes that the default            */
/*                      configuration of channel #7 is differential mode, */
/*                      and therefore at the end of this function channel */
/*                      #7 is always set to this mode.                    */
/*                   3) differential channel calibration is done for the  */
/*                      current amplification settings of the D2S.        */
/*                      Changing the D2S amp setting MUST be followed by  */
/*                      differential channel calibration.                 */
/*                                                                        */
/* INPUT:       diffMode - 1=differential channel calibration.            */
/*                         0=single-ended channel calibration.            */
/*                                                                        */
/* OUTPUT:      0 - success                                               */
/*              -1 - ADC is not in IDLE mode, or invalid input,           */
/*                    or general failure.                                 */
/**************************************************************************/
Int32 Bbu_AdcCalibrate(BbuCurrentMode_e diffMode)
{
    Uint32 regVal;
    Uint16 sampleLow;
    Uint16 sampleHigh;
    Uint16 clock_div;
    Int32  channel;
    bbu_float64  a_coeff;
    bbu_float64  b_coeff;

    Bbu_AveragingModeType_e keepAveragingMode;
    Bbu_AveragingModeType_e calAveragingMode = AVERAGING_MODE_16;
    Uint32 calAveragingModeShift = BBU_AVERAGING_TYPE_2_SHIFT(calAveragingMode);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: start calibration for diffMode %d \n", __FUNCTION__, diffMode);

    /* check input. */
    if ((diffMode != BBU_SingleEnded) && (diffMode != BBU_Differential))
    {
        printk(KERN_ERR "%s: calibration Failed: Unknown value for diffMode - %d \n", __FUNCTION__, diffMode);

        return -1;
    }

    /* Check if BBU ADC control HW is in IDLE mode. */
    if (Bbu_AdcSetToIdle() == -1)
    {
        BBU_KDEBUG_PRINT(KERN_ERR, "%s: Failed to set ADC to idle \n", __FUNCTION__);
        return -1;
    }

    /* setting the channel:                                       */
    /* if performing differential calibration - sample channel 7. */
    /* if performing single-ended calibration - sample channel 0. */
    if (diffMode == BBU_Differential)
    {
        channel = BBU_ADC_CHANNEL_BATT_CURRENT;
    }
    else
    {
        channel = BBU_ADC_CHANNEL_RESERVED1;
    }

    /* Save BBU Clock Settings, set clock to highest frequency */
    clock_div = Bbu_GetBbuClockDivider();
    if (clock_div != PCLKCR1_PUMA5_BBU_CLK_DIV_MIN)
    {
        Bbu_SetBbuClockDivider(PCLKCR1_PUMA5_BBU_CLK_DIV_MIN);
    }

    /* Save current averaging mode */
    keepAveragingMode = Bbu_GetAveragingMode();

    /* Set averaging mode to 16 */
    Bbu_SetAveragingMode(calAveragingMode);

    /* Analog Control Register settings */
    regVal = Bbu_GetRegValue(BBU_ACTRL_PUMA5);

    if (diffMode == BBU_Differential)
    {
        regVal |= BBU_ACTRL_PUMA5_DIFF_MASK;  /* set 'DIFF' to 1 */ 
    }
    else
    {
        regVal &= ~BBU_ACTRL_PUMA5_DIFF_MASK;         /* set 'DIFF' to 0 */
    }
    regVal |= BBU_ACTRL_PUMA5_CALMODE_MASK;            /* set 'CALMODE' to 1 */

    /* Perform low sample */

    regVal &= ~BBU_ACTRL_PUMA5_SELCALIN_MASK;   /* set SELCALIN to 0 - low input */

    if (revision2 && (diffMode == BBU_Differential))
    {
       /* TEST_MODES 2, 3 set to 0 steady*/
        regVal &= ~BBU_ACTRL_PUMA5_TESTMODE_2_MASK;
        regVal &= ~BBU_ACTRL_PUMA5_TESTMODE_3_MASK;
    
        /* for  all gains */
        regVal |= BBU_ACTRL_PUMA5_TESTMODE_0_MASK;  /*  TM 0, 1 to 1*/ 
        regVal |= BBU_ACTRL_PUMA5_TESTMODE_1_MASK;
        BBU_KDEBUG_PRINT(KERN_INFO, "%s:  rev 2 calibration! regval 0x%x\n", __FUNCTION__, regVal);
    }

    Bbu_SetRegValue(BBU_ACTRL_PUMA5, regVal);

    /* sampling */
    Bbu_SetUpSingleSampleAndPoll(channel, &sampleLow);

    /* compensate for averaging  */
    sampleLow = sampleLow >> calAveragingModeShift;

    /* Perform high sample */
    if (!(revision2) || (diffMode == BBU_SingleEnded) )
    { 
        regVal |= BBU_ACTRL_PUMA5_SELCALIN_MASK;   /* set SELCALIN to 1 - high input */
    }
    else /* diff mode in rev 2*/
    {
           /* for  all gains */
        regVal &= ~BBU_ACTRL_PUMA5_TESTMODE_0_MASK;  /*  TM 0, 1 to 0*/ 
        regVal &= ~BBU_ACTRL_PUMA5_TESTMODE_1_MASK;
    }

    Bbu_SetRegValue(BBU_ACTRL_PUMA5, regVal);

    /* sampling */
    Bbu_SetUpSingleSampleAndPoll(channel, &sampleHigh);

    /* compensate for averaging */
    sampleHigh = sampleHigh >> calAveragingModeShift;

    /* calculate the actual coeficients */
    Bbu_CalculateCalibrationCoefficients(sampleLow, sampleHigh, &a_coeff, &b_coeff, diffMode);
    BBU_KDEBUG_PRINT(KERN_INFO, "%s mode %d a.m 0x%x a.e %d b.m 0x%x b.e %d\n", __FUNCTION__, diffMode, a_coeff.mantissa, 
                     a_coeff.exponent,  b_coeff.mantissa, b_coeff.exponent);


    /* set the new coefficients */
    if (diffMode == BBU_Differential)
    {
        bbuWorkingParams.bbuCurrentCalValues.diff_a = a_coeff;
        bbuWorkingParams.bbuCurrentCalValues.diff_b = b_coeff;
    }
    else
    {
        bbuWorkingParams.bbuCurrentCalValues.single_a = a_coeff;
        bbuWorkingParams.bbuCurrentCalValues.single_b = b_coeff;
    }

    /* restore previous averaging mode */
    Bbu_SetAveragingMode(keepAveragingMode);

    /* restore Analog Control Register settings */
    regVal |= BBU_ACTRL_PUMA5_DIFF_MASK;           /* set 'DIFF' to 1 */ 
    regVal &= ~BBU_ACTRL_PUMA5_CALMODE_MASK;       /* set 'CALMODE' to 0 */
    Bbu_SetRegValue(BBU_ACTRL_PUMA5,regVal);

    /* restoring previous clock settings */
    if (clock_div != PCLKCR1_PUMA5_BBU_CLK_DIV_MIN)
    {
        Bbu_SetBbuClockDivider(clock_div);
    }

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: %s Calibration Performed:\n", __FUNCTION__, (diffMode == BBU_Differential) ? "diff" : "single");
    BBU_KDEBUG_PRINT(KERN_INFO, "%s: sampleLow = %u sampleHigh = %u\n", __FUNCTION__,  sampleLow, sampleHigh);

    return 0;
}

/**************************************************************************/
/*                    Bbu_AveragingCompensation()                      */
/**************************************************************************/
/* DESCRIPTION: Compensating for the averaging performed by the ADC logic.*/
/*              Can convert an averaged value to an equivalent 12bit (not */
/*              averaged) value, or the other way around, depending on    */
/*              required direction.                                       */
/*                                                                        */
/* INPUT:       value - the value to be converted.                        */
/*              dir - direction: 0 = from averaged to 'pure'.             */
/*                               1 = from 'pure' to averaged.             */
/*                                                                        */
/* OUTPUT:      converted value.                                          */
/**************************************************************************/
Uint16 Bbu_AveragingCompensation(Uint16 value, Int32 dir)
{
    Uint16 averagingShift;
    Bbu_AveragingModeType_e averagingMode;
    Uint16 retVal;
    Uint32 tempVal;

    averagingMode = Bbu_GetAveragingMode();

    averagingShift = BBU_AVERAGING_TYPE_2_SHIFT(averagingMode);

    if (dir == AVERAGED_TO_PURE_CONVERSION) /* from averaged sample to pure value */
    {
        retVal = value >> averagingShift;
    }
    else /* from pure sample to averaged value */
    {
        tempVal = value << averagingShift;
        retVal = (tempVal > MAX_ADC_VALUE) ? MAX_ADC_VALUE : tempVal;
    }

    return retVal;
}

/**************************************************************************/
/*                   Bbu_ConvertPureToHw_Single()                         */
/**************************************************************************/
/* DESCRIPTION: The SW considers all values to be "pure", meaning that    */
/*              they are not effected by calibration figures and          */
/*              averaging mode. However, in the HW logic, the values used */
/*              are effected by these factors.                            */
/*              This function translates a SW "pure" value to a HW value. */
/*                                                                        */
/* INPUT:       pureValue - the "pure" value to be converted.             */
/*                                                                        */
/* OUTPUT:      converted value.                                          */
/**************************************************************************/
Uint16 Bbu_ConvertPureToHw_Single(Uint16 pureValue)
{
    Uint16 retVal;
    bbu_float64 a_coeff;
    bbu_float64 b_coeff;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    Int32 tempVal;

    a_coeff = bbuWorkingParams.bbuCurrentCalValues.single_a; 
    b_coeff = bbuWorkingParams.bbuCurrentCalValues.single_b;

    /* compensate for calibration */

    /* tempResult = ((float)pureValue) * a_coeff; */
    Int32ToFloat64(pureValue, &tempFloat1);
    MulFloat64(a_coeff, tempFloat1, &tempFloat2);

    /* retVal = (Uint16)(tempResult + b_coeff); */
    AddFloat64(tempFloat2, b_coeff, &tempFloat1);
    tempVal = Float64ToInt(tempFloat1);
    /* the sum may be negative, in this case use 0 as  limit*/
    retVal = (tempVal >= 0) ? (Uint16)tempVal : 0;


    /* compensate for averaging */
    retVal = Bbu_AveragingCompensation(retVal, PURE_TO_AVERAGED_CONVERSION);
    
    BBU_KDEBUG_PRINT(KERN_INFO, "%s  a.m 0x%x a.e %d b.m 0x%x b.e %d pure 0x%x converted 0x%x \n", __FUNCTION__,  a_coeff.mantissa, 
                     a_coeff.exponent,  b_coeff.mantissa, b_coeff.exponent, pureValue, retVal);
    return retVal;       
}

/**************************************************************************/
/*                   Bbu_ConvertHwToPure_Single()                         */
/**************************************************************************/
/* DESCRIPTION: The SW considers all values to be "pure", meaning that    */
/*              they are not effected by calibration figures and          */
/*              averaging mode. However, in the HW logic, the values used */
/*              are effected by these factors.                            */
/*              This function translates a HW value to a "pure" value.    */
/*                                                                        */
/* INPUT:       hwValue - the HW value to be converted.                   */
/*                                                                        */
/* OUTPUT:      converted value.                                          */
/**************************************************************************/
Uint16 Bbu_ConvertHwToPure_Single(Uint16 hwValue)
{
    Uint16 retVal;
    bbu_float64 a_coeff;
    bbu_float64 b_coeff;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    bbu_float64 tempFloat3;

    a_coeff = bbuWorkingParams.bbuCurrentCalValues.single_a;
    b_coeff = bbuWorkingParams.bbuCurrentCalValues.single_b;

    BBU_KDEBUG_PRINT(KERN_INFO, "%s  a.m 0x%x a.e %d b.m 0x%x b.e %d\n", __FUNCTION__,  a_coeff.mantissa, 
                     a_coeff.exponent,  b_coeff.mantissa, b_coeff.exponent);
    /* compensate for averaging */
    retVal = Bbu_AveragingCompensation(hwValue, AVERAGED_TO_PURE_CONVERSION);

    /* compensate for calibration */

    /* temp_res = ((float)value - b_param) */
    Int32ToFloat64(retVal, &tempFloat1);
    SubFloat64(tempFloat1, b_coeff, &tempFloat2);
    if (!IsGreaterEqualZeroFloat64(tempFloat2))
    {
        Int32ToFloat64(0, &tempFloat2);
    }

    /* retVal = (Uint16)(temp_res / a_coeff); */
    DivideFloat64(tempFloat2, a_coeff, &tempFloat1);

   /* *corrected = (Uint16)(retVal+0.5); */
    InitFloat64(BBU_HALF_VALUE_MANTISSA, BBU_HALF_VALUE_EXPONENT, &tempFloat3); 
    AddFloat64(tempFloat1, tempFloat3, &tempFloat2);
    retVal = Float64ToUint16(tempFloat2);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: converted Single-Ended HW value 0x%X to 'pure' value 0x%X \n", __FUNCTION__, hwValue,  retVal);
    return retVal;
}

/**************************************************************************/
/*                    Bbu_ConvertPureToHw_Diff()                          */
/**************************************************************************/
/* DESCRIPTION: The SW considers all values to be "pure", meaning that    */
/*              they are not effected by calibration figures and          */
/*              averaging mode. However, in the HW logic, the values used */
/*              are effected by these factors.                            */
/*              This function translates a "pure" value to a HW value.    */
/*                                                                        */
/* INPUT:       pureValue - the "pure" value to be converted (in mAmp).   */
/*              diffMode - indicates if channel #7 is in differential mode*/
/*                         or not.                                        */
/*                                                                        */
/* OUTPUT:      converted value.                                          */
/**************************************************************************/
Uint16 Bbu_ConvertPureToHw_Diff(Int32 pureValue, BbuCurrentMode_e diffMode)
{
    Uint16 retVal;
    bbu_float64 a_coeff;
    bbu_float64 b_coeff;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    bbu_float64 tempFloat3;

    if (diffMode == BBU_Differential)
    {
        a_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_a;
        b_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_b;
    }
    else
    {
        a_coeff = bbuWorkingParams.bbuCurrentCalValues.single_a;
        b_coeff = bbuWorkingParams.bbuCurrentCalValues.single_b;
    }

    /* convert to voltage : temp = pureValue * BBU_SENSE_CURRENT_VALUE; */
    Int32ToFloat64(pureValue, &tempFloat1);
    BBU_KDEBUG_PRINT(KERN_INFO, "%s value %d value.m 0x%x value.e %d\n", __FUNCTION__, pureValue, tempFloat1.mantissa, tempFloat1.exponent);
    InitFloat64(BBU_SENSE_CURRENT_VALUE_MANTISSA, BBU_SENSE_CURRENT_VALUE_EXPONENT, &tempFloat2);
    MulFloat64(tempFloat1, tempFloat2, &tempFloat3);

    BBU_KDEBUG_PRINT(KERN_INFO, "sample*0.05 res.m 0x%x res.e %d\n", tempFloat3.mantissa, tempFloat3.exponent);

     /* divide by 1000  (mV)) */
    InitFloat64(THOUSAND_MANTISSA, THOUSAND_EXPONENT, &tempFloat1); 
    DivideFloat64(tempFloat3, tempFloat1, &tempFloat2);

    BBU_KDEBUG_PRINT(KERN_INFO, " dividby 1000 res.m 0x%x res.e %d\n", tempFloat2.mantissa, tempFloat2.exponent);

    /* transport to sample temp = (temp / BBU_ADC_CAL_VREF) *BBU_ADC_CAL_VREF_WORD; */
    InitFloat64(BBU_ADC_CAL_VREF_MANTISSA, BBU_ADC_CAL_VREF_EXPONENT, &tempFloat1);
    DivideFloat64(tempFloat2, tempFloat1, &tempFloat3);
    InitFloat64(BBU_ADC_CAL_VREF_WORD_MANTISSA, BBU_ADC_CAL_VREF_WORD_EXPONENT, &tempFloat1);
    MulFloat64(tempFloat3, tempFloat1, &tempFloat2);

    BBU_KDEBUG_PRINT(KERN_INFO, " /1.5*4095  res.m 0x%x res.e %d \n", tempFloat2.mantissa, tempFloat2.exponent);

    /* add calibration influence: temp = ((float)a_coeff * temp) + b_coeff; */
    MulFloat64(a_coeff, tempFloat2, &tempFloat1);
    AddFloat64(tempFloat1, b_coeff, &tempFloat2);

    BBU_KDEBUG_PRINT(KERN_INFO, " *a + b res.m 0x%x res.e %d \n", tempFloat2.mantissa, tempFloat2.exponent);

    if (Float64ToInt(tempFloat2) < 0) 
    {
		InitFloat64(BBU_SENSE_CURRENT_VALUE_MANTISSA, BBU_SENSE_CURRENT_VALUE_EXPONENT, &tempFloat2);
		Int32ToFloat64(0, &tempFloat2);
		BBU_KDEBUG_PRINT(KERN_INFO, " result is negative. initiate result res.m 0x%x res.e %d \n", tempFloat2.mantissa, tempFloat2.exponent);
    }
    /* retVal = (Uint16)(temp + 0.5); todo if needed */
    InitFloat64(BBU_HALF_VALUE_MANTISSA, BBU_HALF_VALUE_EXPONENT, &tempFloat1);
    AddFloat64(tempFloat2, tempFloat1, &tempFloat3);

    BBU_KDEBUG_PRINT(KERN_INFO, " +0.5 res.m 0x%x res.e %d\n", tempFloat3.mantissa, tempFloat3.exponent);

    retVal = Float64ToUint16(tempFloat3);
    BBU_KDEBUG_PRINT(KERN_INFO, " converted to int 0x%x \n", retVal);

    /* compensate for averaging */
    retVal = Bbu_AveragingCompensation(retVal, PURE_TO_AVERAGED_CONVERSION);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s  a.m 0x%x a.e %d b.m 0x%x b.e %d pure 0x%x converted 0x%x \n", __FUNCTION__,  a_coeff.mantissa, 
                     a_coeff.exponent,  b_coeff.mantissa, b_coeff.exponent, pureValue, retVal);

    return retVal;
}

/**************************************************************************/
/*                    Bbu_ConvertHwToPure_Diff()                          */
/**************************************************************************/
/* DESCRIPTION: The SW considers all values to be "pure", meaning that    */
/*              they are not effected by calibration figures and          */
/*              averaging mode. However, in the HW logic, the values used */
/*              are effected by these factors.                            */
/*              This function translates a HW value to a "pure" value.    */
/*              It is used for the ADC's differential channel (#7), and   */
/*              can return negative values.                               */
/*                                                                        */
/* INPUT:       hwValue - the HW value to be converted.                   */
/*              diffMode - indicates if channel #7 is in differential mode*/
/*                         or not.                                        */
/*                                                                        */
/* OUTPUT:      converted value (returned in samples*1000).               */
/**************************************************************************/
Int32 Bbu_ConvertHwToPure_Diff(Uint16 hwValue, BbuCurrentMode_e diffMode)
{
    Int32 retVal;
    bbu_float64 a_coeff;
    bbu_float64 b_coeff;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    bbu_float64 tempFloat3; 

    /* compensate for averaging */
    retVal = Bbu_AveragingCompensation(hwValue, AVERAGED_TO_PURE_CONVERSION);

    /* compensate for calibration */

    if (diffMode == BBU_Differential)
    {
        a_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_a;
        b_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_b;
    }
    else
    {
        a_coeff = bbuWorkingParams.bbuCurrentCalValues.single_a;
        b_coeff = bbuWorkingParams.bbuCurrentCalValues.single_b;
    }

    /* temp = (float)(retVal - b_coeff); */
    Int32ToFloat64(retVal, &tempFloat1);
    SubFloat64(tempFloat1, b_coeff, &tempFloat2);

    /* temp = (float)(temp / a_coeff); -> converted sample */
    DivideFloat64(tempFloat2, a_coeff, &tempFloat1);
    BBU_KDEBUG_PRINT(KERN_INFO, "%s: converted diff, sample 0x%X pure.m 0x%X pure e %d\n", __FUNCTION__, hwValue, tempFloat1.mantissa,
                     tempFloat1.exponent);

    /* multiply by 1000 so that small numbers not disappear. application needs to take this into account and not multiply for mA*/
    /*temp = (temp * 1000) */
    InitFloat64(THOUSAND_MANTISSA, THOUSAND_EXPONENT, &tempFloat2); 
    MulFloat64(tempFloat1, tempFloat2, &tempFloat3);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s:  diff mult 1000,  pure.m 0x%X pure e %d\n", __FUNCTION__,  tempFloat3.mantissa,
                     tempFloat3.exponent);
    /* retVal = (int)temp; */
    retVal = Float64ToInt(tempFloat3);

    BBU_KDEBUG_PRINT(KERN_INFO, "%s: converted differential HW value 0x%X to 'pure' value 0x%X\n", __FUNCTION__, hwValue, retVal);

    return retVal; 
}

/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/

/* returns true when running on P5 chip rev 2*/
/**************************************************************************/
/*! \fn BbuIsRev2 
 **************************************************************************
 *  \brief Checks the chip version
 *  needed for calibration purposes
 *  \param[in] none
 *  \return True if chip version 2 or False otherwise
 **************************************************************************/
static Bool BbuIsRev2(void)
{
    static Bool notifyP5Ver = False;/* for debug printing the version*/
    Bool rev2 = False;
    
     rev2 = (system_rev == P5_VERSION_REV2);

      /* print the rev id just once*/
       if (!notifyP5Ver)
       {
           printk(KERN_WARNING "%s: rev %d\n", __FUNCTION__, system_rev);
           notifyP5Ver = True;
       }
       return rev2;
}

/**************************************************************************/
/*                   Bbu_CalculateEqSample()                              */
/**************************************************************************/
/* DESCRIPTION: When changing gain settings, a sample that was taken in   */ 
/*              the previous gain needs to be translated to an equivalent */
/*              value in the new gain.                                    */
/*                                                                        */
/* INPUT:       samlpe - the sample to be translated.                     */
/*              newGain - the new gain setting.                           */
/*                                                                        */
/* OUTPUT:      equivalent sample.                                        */
/**************************************************************************/
static Uint16 Bbu_CalculateEqSample(Uint16 sample, Int32 newGain)
{
    bbu_float64 a_coeff;
    bbu_float64 b_coeff;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    bbu_float64 tempFloat3;
    Uint16 result;

    a_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_a;
    b_coeff = bbuWorkingParams.bbuCurrentCalValues.diff_b;

    /* ((float)(sample-b_coeff) */
    Int32ToFloat64(sample, &tempFloat1);
    SubFloat64(tempFloat1, b_coeff, &tempFloat2);

    /* temp = ((float)(sample-b_coeff))/a_coeff; */
    DivideFloat64(tempFloat2, a_coeff, &tempFloat3);

    newGain *= (-1);
    /* temp = temp*newGain + b_coeff; */
    Int32ToFloat64(newGain, &tempFloat1);
    MulFloat64(tempFloat3, tempFloat1, &tempFloat2);
    AddFloat64(tempFloat2, b_coeff, &tempFloat3);

    /* sample = (Uint16)temp;*/
    result = Float64ToUint16(tempFloat3);

    BBU_KDEBUG_PRINT(KERN_INFO, " %s 2 new gain: 0x%x sample 0x%x result 0x%x", __FUNCTION__, newGain, sample, result);

    return result;
}

/**************************************************************************/
/*                Bbu_CalculateCalibrationCoefficients                    */
/**************************************************************************/
/* DESCRIPTION: Calculates the calibration coefficients from a given set  */
/*              of samples. Performs a differrent calculation for         */
/*              differential and single-ended channels.                   */
/*                                                                        */
/* INPUT:       low_sample - the low value sampled in the calibration.    */
/*              high_sample - the high value sampled in the calibration.  */
/*              a_coeff - calculated value for coefficien A.              */
/*              b_coeff - calculated value for coefficien B.              */
/*              diffMode - 0 = calculate for single-ended channel.        */
/*                         1 = calculate for differencial channel.        */
/*                                                                        */
/* OUTPUT:      0 - success                                               */
/*              -1 - failure.                                             */
/**************************************************************************/
static Int32 Bbu_CalculateCalibrationCoefficients(Uint16 sampleLow,
                                              Uint16 sampleHigh,
                                              bbu_float64 *a_coeff,
                                              bbu_float64 *b_coeff,
                                              BbuCurrentMode_e    diffMode)
{

    bbu_float64 div_param;
    bbu_float64 tempFloat1;
    bbu_float64 tempFloat2;
    bbu_float64 tempFloat3;
    bbu_float64 diff_low_word;

    if (diffMode == BBU_Differential) /* calculate for differential channel */
    {
/*   flow with floating numbers */
/*   a_param = ((float)(sampleLow - sampleHigh))/((float)( BBU_CAL_DIFF_VLOW_WORD-BBU_CAL_DIFF_VHIGH_WORD )); 
     b_param = (float)sampleLow - (a_param*((float)BBU_CAL_DIFF_VLOW_WORD));
        *corrected =  ((float)value - b_param)/a_param;
*/
     
           /* a_coeff = ((float)(sampleLow - sampleHigh))/((float)( BBU_CAL_DIFF_VLOW_WORD-BBU_CAL_DIFF_VHIGH_WORD ));  */
         Int32ToFloat64((sampleLow - sampleHigh ), &tempFloat1);
         InitFloat64(diffMantissa, diffExponent, &div_param);
         DivideFloat64(tempFloat1, div_param, a_coeff);
 
        /* b_param = (float)sampleLow - (a_param*((float)BBU_CAL_DIFF_VLOW_WORD)); */
	InitFloat64(lowWordMantissa, lowWordExponent, &diff_low_word);
        MulFloat64(*a_coeff, diff_low_word, &tempFloat3);

        Int32ToFloat64(sampleLow, &tempFloat1);
        SubFloat64(tempFloat1, tempFloat3, b_coeff);

    }
    else /* calculate for single-ended channel */
    {
        /* *a_coeff = ((float)(sampleHigh - sampleLow))/((float)(BBU_CAL_VHIGH_WORD - BBU_CAL_VLOW_WORD)); */
        Int32ToFloat64((sampleHigh - sampleLow), &tempFloat1);
        InitFloat64(BBU_CAL_SINGLE_DIFFERENCE_MANTISSA, BBU_CAL_SINGLE_DIFFERENCE_EXPONENT, &tempFloat2);
        DivideFloat64(tempFloat1, tempFloat2, a_coeff);

        /* *b_coeff = ((float)sampleLow - (a_param*((float)BBU_CAL_VLOW_WORD)); */
        Int32ToFloat64(BBU_CAL_VLOW_WORD, &tempFloat1);
        MulFloat64(*a_coeff, tempFloat1, &tempFloat2);
        Int32ToFloat64(sampleLow, &tempFloat1);
        SubFloat64(tempFloat1, tempFloat2, b_coeff);
    }
    BBU_KDEBUG_PRINT(KERN_INFO, "%s calib type %d sampl0 0x%x sampl1 0x%x  a.m 0x%x a.e %d b.m 0x%x b.e %d\n", __FUNCTION__, diffMode, sampleLow, sampleHigh, a_coeff->mantissa, a_coeff->exponent, 
                     b_coeff->mantissa, b_coeff->exponent);

    return 0;
}

/**************************************************************************/
/*                   Bbu_DetermineCalibrationData()                       */
/**************************************************************************/
/* DESCRIPTION: on init, fill calibration data upon P5 chip version       */ 
/* INPUT:       None                                                      */
/*                                                                        */
/* OUTPUT:      None                                                      */
/**************************************************************************/
static void Bbu_DetermineCalibrationData(void)
{
    revision2 = BbuIsRev2();

    if (revision2)
    {
         diffExponent = BBU_CAL_DIFF_DIFFERENCE_VER2_EXPONENT;
         diffMantissa = BBU_CAL_DIFF_DIFFERENCE_VER2_MANTISSA;
         lowWordMantissa = BBU_CAL_DIFF_VLOW_WORD_VER2_MANTISSA;
         lowWordExponent = BBU_CAL_DIFF_VLOW_WORD_VER2_EXPONENT;
    }
    else
    {
         diffExponent = BBU_CAL_DIFF_DIFFERENCE_EXPONENT;
         diffMantissa = BBU_CAL_DIFF_DIFFERENCE_MANTISSA;
         lowWordMantissa = BBU_CAL_DIFF_VLOW_WORD_MANTISSA;
         lowWordExponent = BBU_CAL_DIFF_VLOW_WORD_EXPONENT;
    }
}

/**************************************************************************/
/*                   BbuChangeGainSettings()                              */
/**************************************************************************/
/* DESCRIPTION: change gain setting and calculate the sample              */
/*                  in the new gain                                       */ 
/* INPUT:       None                                                      */
/*                                                                        */
/* OUTPUT:      None                                                      */
/**************************************************************************/
#if 0 /* todo to be done - configurable from working params */
static void BbuChangeGainSettings(Uint16 sample, Uint16 *pSample, Bbu_D2sGainType_e newGain)
{
    /* Set ADC to Idle state. */
    Bbu_AdcSetToIdle();

    /* change gain setting */
    Bbu_SetD2SAmplification(newGain);  

    /* calculate an equivalent sample in the new gain */
    *pSample = Bbu_CalculateEqSample(sample, gainEquivalents[newGain]);
    *pSample = Bbu_AveragingCompensation(*pSample, PURE_TO_AVERAGED_CONVERSION);
}

#endif

