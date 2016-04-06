/*
 *
 * bbu_types.h
 * Description:
 * Types and macros used both by driver and application
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

#ifndef _BBU_TYPES_H_
#define _BBU_TYPES_H_


/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/
#include <_tistdtypes.h>
#include "bbu_float_support.h"

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

/*! \def MAX_ADC_CHANNEL_INDEX
 *  \brief max ADC channel
 */
#define MAX_ADC_CHANNEL_INDEX               7

/*! \def BBU_GET_CURRENT_SAMPLE
 *  \brief Macro for getting the current sample value on a specific channel in adcdata
 */
#define BBU_GET_CURRENT_SAMPLE(channel, pAdcData) ((pAdcData->vals)[channel])

/*! \def BBU_PWM_RANGE_DEFAULT, BBU_PWMCR_MAX_VAL
 *  \brief PWM  frequency default and max values
 */
#define BBU_PWM_RANGE_DEFAULT         0x3E7 /* Sets PWM Freq to 200KHz */
#define BBU_PWMCR_MAX_VAL             0xFFFF

/*! \def BBU_SAFETY_VIOLATION_REASONS
 *  \brief list of possible BBU violations reasons
 */

        /*             violation type          violation type string    */
        /* -------------------------------------------------------------*/
#define BBU_SAFETY_VIOLATION_REASONS \
     BBU_SAFETY_VIOL(BBU_VIOLATION_NONE,        "BBU_VIOLATION_NONE")\
     BBU_SAFETY_VIOL(BBU_TEMPERATURE_HIGH,      "BBU_TEMPERATURE_HIGH")\
     BBU_SAFETY_VIOL(BBU_TEMPERATURE_LOW,       "BBU_TEMPERATURE_LOW")\
     BBU_SAFETY_VIOL(BBU_VOLTAGE_HIGH,          "BBU_VOLTAGE_HIGH")\
     BBU_SAFETY_VIOL(BBU_CURRENT_HIGH_CHARGE,   "BBU_CURRENT_HIGH_CHARGE")\
     BBU_SAFETY_VIOL(BBU_CURRENT_HIGH_DISCHARGE,"BBU_CURRENT_HIGH_DISCHARGE")\
     BBU_SAFETY_VIOL(BBU_CLOSED_BUCK_VOLTAGE,   "BBU_CLOSED_BUCK_VOLTAGE")\
     BBU_SAFETY_VIOL(BBU_TEMPERATURE_NORMAL,    "BBU_TEMPERATURE_NORMAL")

#define BBU_SAFETY_VIOL(cause, causeString) cause,

/*! \enum typedef enum BbuSafetyViolationCauses_e
 *  /brief bbu safety violation causes found by driver and reported to user
 */
typedef enum
{
    BBU_SAFETY_VIOLATION_REASONS
    BBU_SAFETY_VIOLATION_REASON_LAST
} BbuSafetyViolationCauses_e;

#undef BBU_SAFETY_VIOL

/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/

/*! \enum typedef enum Bbu_D2sGainType_e
 *  /brief Differential to single-ended amplifier gain levels.
 *  more details (not mandatory)
 */
typedef enum
{
    BBU_D2S_GAIN_1,
    BBU_D2S_GAIN_10,
    BBU_D2S_GAIN_15,
    BBU_D2S_GAIN_20
}Bbu_D2sGainType_e;

/*! \enum typedef enum Bbu_AveragingModeType_e
 *  /brief BBU Averaging modes
 */
typedef enum
{
    AVERAGING_MODE_2 = 0,
    AVERAGING_MODE_4,
    AVERAGING_MODE_8,
    AVERAGING_MODE_16,
    AVERAGING_MODE_1
}Bbu_AveragingModeType_e; 

/*! \enum typedef enum BbuCurrentMode_e
 *  /brief ADC sampling and calibration modes
 */
typedef enum
{
    BBU_SingleEnded,
    BBU_Differential
} BbuCurrentMode_e;

/*! \struct typedef struct BbuCalibrationValues_t
 *  /brief all calibration coefficients for singled_ended and diff modes
 */
typedef struct
{
    bbu_float64 single_a;
    bbu_float64 single_b;
    bbu_float64 diff_a;
    bbu_float64 diff_b;
} BbuCalibrationValues_t;


/*! \enum typedef enum BbuGainLimits_e
 *  /brief limits for moving into different gain
 * the values are battery type specific
 */
typedef enum
{
    BBU_GAIN_20_MAX_VALUE,          
    BBU_GAIN_15_MIN_VALUE,         
    BBU_GAIN_15_MAX_VALUE,           
    BBU_GAIN_10_MIN_VALUE,          
    BBU_GAIN_10_MAX_VALUE,           
    BBU_GAIN_1_MIN_VALUE, 
    BBU_GAIN_LIMIT_LAST
} BbuGainLimits_e;

/*! \enum typedef enum BbuCurrentDirection_e
 *  /brief current direction
 */
typedef enum
{
    BBU_POSITIVE_CURRENT,
    BBU_NEGATIVE_CURRENT
}BbuCurrentDirection_e;

/*! \struct typedef struct BbuWorkingParams_t
 *  /brief BBU working parameters:
 *  contains all working params needed by the driver and determined by the user -
 *  timer values, security limits etc.
 *  download by bbu driver ioctl, may be read by user too
 *  ----------------------------------------
 *  safety checking values have 2 forms: pure, and converted to hw form (averaging, calibration) to be used by driver 
 *  pure values are received once user donwloads working parameters, conversion to hw is done on each calibration according toe
 *  the calibration coefficients. driver uses the converted form for safety checks
 */
typedef struct
{
/* safety limits*/
    Uint32 bbuMaxVoltLimit;           /* value to be used by driver, in "hw" form, after conversion*/
    Uint32 bbuMaxVoltLimitPure;       /* pure value received from user - in samples*/
    Uint32 bbuMinVoltLimit;
    Uint32 bbuMinVoltLimitPure;       /* pure value received from user - in samples*/
    Uint32 bbuMaxBuckClosedVoltage;
    Uint32 bbuMaxBuckClosedVoltagePure; /* pure value received from user - in samples*/
    Uint32 bbuMaxTempLimit;
    Uint32 bbuMaxTempLimitPure;         /* pure value received from user - in samples*/
    Uint32 bbuMinTempLimit;
    Uint32 bbuMinTempLimitPure;         /* pure value received from user - in samples*/
    Uint32 bbuMaxCurrLimitPositive;
    Uint32 bbuMaxCurrLimitPositivePure; /* pure value received from user - in mA*/
    Int32  bbuMaxCurrLimitNegative;
    Int32  bbuMaxCurrLimitNegativePure; /* pure value received from user - in mA*/
/* end of safety limits*/
    BbuCurrentDirection_e bbuCurrentDirection; /* positive or negative (during discharge only) */
    Uint32 bbuMaxCurrentViolationCount;   /* this many consecutive over-current events are considered a safety violation */
    BbuCalibrationValues_t bbuCurrentCalValues; /* read only parameter for user*/
    Uint32 averagingMode;                       /* read only parameter for user*/
    Uint32 bbuScanningPeriod;                   /* Kernel thread frequency -  msec */
    Uint32 bbuCalibrationPeriod;                /* periodic calibration frequency -msec */
    Uint32 bbuMinCountTemperatureNormal;        /* this many normal temperature measuments enable remove temperature bad notification*/
    Uint32 bbuNeedToCheckTemperatureBackToNormal;  /* need to check temperature got to norm? Boolean */
    Uint32 bbuNeedToCheckBuckClosedVoltage;        /* need to perform buck voltage test? ( is not performed during charging) - Boolean*/
    Uint32 bbuNeedToAccumulateCurrent;          /* need to accumulate current? (accumulator is used only in charge, discharge states) - Boolean*/
    Uint32 bbuGainLimits[BBU_GAIN_LIMIT_LAST];  /* samples that serve as limits to change gain setting */
} BbuWorkingParams_t;

/*! \array typedef array Bbu_GainLimits
 *  /brief table containing current limits for changing gain
 *  there is a set for positive current and another one for negative current
 */
typedef Uint32 Bbu_GainLimits[2][BBU_GAIN_LIMIT_LAST];

/*! \struct typedef struct BbuAdcStateData_t
 *  /brief ADC data measured by driver and returned to user on request
 *  includes last measurements on each channel - converted to pure state,
 *  current measurement in pure state, may be positive or negative,
 * current accumulation buffer ( may be positive or negative) and the number of accumulated samples
 */
typedef struct 
{
    Uint16 vals[MAX_ADC_CHANNEL_INDEX + 1];
    Int32 currentVal;
    Int32 bbuCurrentChannelAcc;
    Uint32 samplesCounter; /* counter for number of accumulated samples */
}BbuAdcStateData_t;

/*! \struct typedef struct BbuFloatCalc_t
 *  /brief data for "floating" ( float64) calculations
 */
typedef struct
{
    Int32 operand1;
    Int32 operand2;
    bbu_float64 resultFloat;
    Int32 result;
    Int32 operation;
}BbuFloatCalc_t;

/*! \struct typedef struct BbuPwmParameters_t
 *  /brief PWM parameters - whether is active, duty cycle and gain used (read only)
 */
typedef struct
{
    Bool  active;
    Int32 pwmDutyCycle;
    Int32 pwmGain;  /* used ONLY for reading current value*/
}BbuPwmParameters_t;

/*! \struct typedef struct BbuSafetyViolationData_t
 *  /brief structure includes allsafety  violation data - 
 *   list of detected violation causes and the last violation found
 */
typedef struct
{
    BbuSafetyViolationCauses_e       newViolationCauseDetected;
    Bool          safetyViolationList[BBU_SAFETY_VIOLATION_REASON_LAST];
}BbuSafetyViolationData_t;

/**************************************************************************/
/*      EXTERN definition block                                           */                                                                 
/**************************************************************************/
extern const Char *BbuSafetyViolationCauses[];

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/


#endif /*_BBU_TYPES_H_*/


