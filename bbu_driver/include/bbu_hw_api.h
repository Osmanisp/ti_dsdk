/*
 *
 * bbu_hw_api.h
 * Description:
 * Types/routines to inteface P5 BBU hardware
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

/*! \file bbu_hw_api.h
 *  /brief types/routines to inteface P5 BBU hardware
*/

#ifndef _BBU_HW_API_H_
#define _BBU_HW_API_H_

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/

#include <_tistdtypes.h>
#include <asm/atomic.h>
#include "bbu_regs.h"
#include "bbu_board_def.h"
#include "bbu_types.h"

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

#if 0 /* uncomment for debugging driver */

/*! \def BBU_HAL_DEBUG
 *  \brief enable printing kernel logs for bbu
 */
#define BBU_HAL_DEBUG

#endif

#ifdef BBU_HAL_DEBUG
#define BBU_KDEBUG_PRINT(level, fmt, args...) (printk(level fmt,##args))
#else
#define BBU_KDEBUG_PRINT(level, fmt, args...)
#endif

/*! \def BBU_NEED_CALIBRATE_... , BBU_CALIBRATION_DONE
 *  \brief setting of the 2 bit variable that signals Kernel thread whether single and/or diff calibration must be performed
 */
#define BBU_NEED_CALIBRATE_SINGLE (1)
#define BBU_NEED_CALIBRATE_DIFF (2)
#define BBU_NEED_CALIBRATE_BOTH (3)
#define BBU_CALIBRATION_DONE (0)

/*! \def BBU_SINGLE_CALIBRATION_MASK , BBU_DIFF_CALIBRATION_MASK
 *  \brief masks to enable setting of the 2 bit variable that signals Kernel about needed calibration
 */
#define BBU_SINGLE_CALIBRATION_MASK (0x1)
#define BBU_DIFF_CALIBRATION_MASK   (0x2)

/*! \def BBU_SINGLE_CALIBRATION_SET , BBU_DIFF_CALIBRATION_SET
 *  \brief macros to enable setting the 2 bit variable that signals Kernel about needed calibration
 */
#define BBU_SINGLE_CALIBRATION_SET(x) ((x) & BBU_SINGLE_CALIBRATION_MASK )
#define BBU_DIFF_CALIBRATION_SET(x) ((x) & BBU_DIFF_CALIBRATION_MASK )

/*! \def BBU_SET_SINGLE_CALIBRATION_NEEDED , BBU_SET_DIFF_CALIBRATION_NEEDED
 *  \brief macros to enable querying if calibration needed
 */
#define BBU_SET_SINGLE_CALIBRATION_NEEDED(x) ((x) |= BBU_SINGLE_CALIBRATION_MASK )
#define BBU_SET_DIFF_CALIBRATION_NEEDED(x) ((x) |= BBU_DIFF_CALIBRATION_MASK )

/*! \def AVERAGED_TO_PURE_CONVERSION , AVERAGED_TO_PURE_CONVERSION
 *  \brief macros to assist conversion of values from hardware (ADC samples) to pure (volts, mA) etc and vv
 */
#define AVERAGED_TO_PURE_CONVERSION 0
#define PURE_TO_AVERAGED_CONVERSION 1


/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/


/**************************************************************************/
/*      EXTERN definition block                                           */
/*                                                                        */
/**************************************************************************/

/*! \variable bbuWorkingParams
*  /brief   contains all working params needed by the driver and determined by the user -
*  timer values, security limits etc
*  download by bbu driver ioctl
*/
extern BbuWorkingParams_t bbuWorkingParams; 
extern atomic_t     bbuNeedCalibrate;

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/


/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/

/**************************************************************************/
/*                    Bbu_IsHwEnabled()                                   */
/**************************************************************************/
/* OUTPUT:      True - BBU supported on chip                              */
/*              False - BBU not supported on chip                         */
/*                                                                        */
/**************************************************************************/
Bool Bbu_IsHwEnabled(void);

/**************************************************************************/
/*                        Bbu_HalInit()                                */
/**************************************************************************/
/* DESCRIPTION: Performs the necessary initializations and settings in    */
/*              order for the BBU hardware to operate correctly.          */
/*                                                                        */
/* INPUT:       none                                                      */
/*                                                                        */
/* OUTPUT:     none                                                       */
/**************************************************************************/
void Bbu_HalInit(void);

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
Int32 Bbu_EnablePWM(Int32 enable);

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
Int32 Bbu_SetPwmDutyCycle(Uint32 val);

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
Uint32 Bbu_GetPwmDutyCycle(void);

/**************************************************************************/
/*                     Bbu_SetPwmRange()                                  */
/**************************************************************************/
/* DESCRIPTION: Sets the Range parameter in the PWM config register.      */
/*                                                                        */
/* INPUT:       val : requested value of range parameter.                 */
/*                                                                        */
/* OUTPUT:      0  - success.                                            */
/*              -1 - invalid input value.                                */
/**************************************************************************/
Int32 Bbu_SetPwmRange(Uint32 val);

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
Uint32 Bbu_GetPwmRange(void);

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
Bbu_AdcModeType_e Bbu_AdcGetMode(void);

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
Int32 Bbu_AdcSetToIdle(void);

/**************************************************************************/
/*                        Bbu_AdcReset()                                  */
/**************************************************************************/
/* DESCRIPTION: reset the ADC and BBU control.                           */
/*                                                                        */
/* INPUT:       none                                                      */
/*                                                                        */
/* OUTPUT:      0  - reset successfully                                   */
/*              -1 - Error                                                */
/**************************************************************************/
Int32 Bbu_AdcReset(Bool reset);

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
Int32 Bbu_SetAdcDftMode(void);

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
Int32 Bbu_SetUpSingleSample(Int32 channel);

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
Int32 Bbu_OneShotSamplePoll(Bbu_OneShotDataType_t * pData);

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
Int32 Bbu_SetUpSingleSampleAndPoll(Int32 channel, Uint16 * pData);

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
Int32 Bbu_GetSingleSampleValue(Uint16 * pData);

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
/*              -1 - invalid channel number or other failure.            */
/**************************************************************************/
Int32 Bbu_SetUpOneShotSample(void);

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
Int32 Bbu_SetUpOneShotSampleAndPoll(Bbu_OneShotDataType_t * pData);

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
Int32 Bbu_GetOneShotSampleValues(Bbu_OneShotDataType_t * pData);

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
Int32 Bbu_SetAveragingMode(Bbu_AveragingModeType_e mode);

/**************************************************************************/
/*                  Bbu_GetAveragingMode()                                */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU-ADC averaging mode                          */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Bbu_AveragingModeType_e Bbu_GetAveragingMode(void);

Uint16 Bbu_AveragingCompensation(Uint16 value, Int32 dir);

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
Int32 Bbu_SetBbuClockDivider(Uint16 divider);

/**************************************************************************/
/*                  Bbu_GetBbuClockDivider()                              */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU clock divider.                              */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Uint16 Bbu_GetBbuClockDivider(void);

/**************************************************************************/
/*                  Bbu_SetD2SAmplification()                          */
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
/* OUTPUT:      0  - success.                                            */
/*              -1 - undefined input value.                              */
/**************************************************************************/
Int32 Bbu_SetD2SAmplification(Bbu_D2sGainType_e level);

/**************************************************************************/
/*                 Bbu_GetD2SAmplification()                              */
/**************************************************************************/
/* DESCRIPTION: Reads the BBU D2 amplicifation.                           */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      mode                                                      */
/**************************************************************************/
Bbu_D2sGainType_e Bbu_GetD2SAmplification(void);

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
/* OUTPUT:      TRUE  - setting was changed.                              */
/*              FALSE - setting was NOT changed.                          */
/**************************************************************************/
Int32 Bbu_GainControl(Uint16 *pSample);

/******************************************************************************
 *                              Battery Interface                             *
 ******************************************************************************/

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
Int32 Bbu_EnableBattery(Uint16 index, Uint16 enable);

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
Int32 Bbu_SetActiveBattery_discharge(Uint16 index);

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
Int32 Bbu_SetActiveBattery(Uint16 index);

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
Int32 Bbu_GetEnabledBattery(void);

/**************************************************************************/
/*               Bbu_EmergencyBatteryShutDown()                           */
/**************************************************************************/
/* DESCRIPTION: Returns the battery that is currently Enabled.            */
/*                                                                        */
/* INPUT:       none.                                                     */
/*                                                                        */
/* OUTPUT:      none.                                                     */
/**************************************************************************/
void Bbu_EmergencyBatteryShutDown(void);

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
Int32 Bbu_AdcCalibrate(BbuCurrentMode_e diffMode);


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
Uint16 Bbu_ConvertPureToHw_Single(Uint16 pureValue);

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
Uint16 Bbu_ConvertHwToPure_Single(Uint16 hwValue);

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
Uint16 Bbu_ConvertPureToHw_Diff(Int32 pureValue, BbuCurrentMode_e diffMode);

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
Int32 Bbu_ConvertHwToPure_Diff(Uint16 hwValue, BbuCurrentMode_e diffMode);

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
void Bbu_SetRegValue(Uint32 regAddr, Uint32 val);

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
Uint32 Bbu_GetRegValue(Uint32 regAddr);

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
void Bbu_EnableBatteryDelay(Uint16 microSec);

#endif /*_BBU_API_H_*/
