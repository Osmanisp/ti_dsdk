/*
 *
 * bbo_board_def.h
 * Description:
 * Definitions specific to board
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

/*! \file bbo_board_def.h
 *  /brief definitions specific to board
*/

#ifndef _BBU_BOARD_DEF_H_
#define _BBU_BOARD_DEF_H_

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/


/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

/******************************************************************************
 *                    Defining 1 or 2 Battery Design                          *
 ******************************************************************************/

/*! \def BBU_NUM_OF_BATTERIES_SUPPORTED
 *  \brief how many batteries does the design support (Possible Values: 1 or 2 
 */
#define BBU_NUM_OF_BATTERIES_SUPPORTED 2

/*! \def BBU_BAT1_INDEX, BBU_BAT2_INDEX
 *  \brief bat 1 and bat2 indices (the way  used in BBU_CR reg) 
 */
#define BBU_BAT1_INDEX                0 
#define BBU_BAT2_INDEX                1 

/*! \def BBU_EN_BAT1_MASK, BBU_EN_BAT2_MASK
 *  \brief bat 1 and bat2 masking  in BBU_CR reg
 */
#define BBU_EN_BAT1_MASK                 (1<<BBU_BAT1_INDEX) /*0x00000001*/
#define BBU_EN_BAT2_MASK                 (1<<BBU_BAT2_INDEX) /*0x00000002*/

/*! \def BBU_NO_BATTERIES
 *  \brief means no batteries are inserted 
 */
#define BBU_NO_BATTERIES (-1)

/* NOTE:
   If the design is defined to support only 1 battery - that battery must be 
   defined as battery #1, and all the configuration definitions in this file 
   must follow this rule.                                                  
*/ 
                                                               
/******************************************************************************
 *                               Defining ADC Inputs                          *
 ******************************************************************************/

/*! \def BBU_ADC_CHANNEL_xxx
 *  \brief functionality of each ADC input line, according to the board design
 */
/*     Channel Name                    **    Channel #                 Functionality  */   
/**********************************************************/
#define BBU_ADC_CHANNEL_BUCK_VOLTAGE  /**/       6      /**/        /* Buck voltage         */
#define BBU_ADC_CHANNEL_BP2_ID        /**/       1      /**/        /* Battery Pack 2 ID    */
#define BBU_ADC_CHANNEL_BP1_ID        /**/       3      /**/        /* Battery Pack 1 ID    */
#define BBU_ADC_CHANNEL_TEMPERATURE   /**/       2      /**/        /* Temperature          */
#define BBU_ADC_CHANNEL_BATT_CURRENT  /**/       7      /**/        /* Battery current      */
#define BBU_ADC_CHANNEL_BATT_VOLTAGE  /**/       5      /**/        /* Battery voltage      */
#define BBU_ADC_CHANNEL_RESERVED1     /**/       0      /**/        /* reserved #2          */
#define BBU_ADC_CHANNEL_RESERVED2     /**/       4      /**/        /* reserved #3          */

#define MAX_ADC_CHANNEL_INDEX                    7
                                                               
/*! \def DEFAULT_ACTIVE_ADC_CHANNELS
 *  \brief defines which of the ADC inputs are active (i.e. connected  
 * and functional), and prevents sampling values on lines that are not       
 * connected.                                                              
 */
#define DEFAULT_ACTIVE_ADC_CHANNELS 0xFF /* all channels */  

/******************************************************************************
 *                          Battery interface & GPIOs                         *
 ******************************************************************************/

 /* In order to allow simple porting, the following convension is used:
	- For standard GPIOs, use the actual GPIO index. 
	- For auxilary GPIOs, add 32 (dec) to the auxilary-index of the GPIO. */
/* NOTE: adding/removing GPIOs must be followed by updating the "Bbu_HwInit" */
/*       function, in bbu_hw_api.c, for initialization and direction settings.      */

/*! \def BBU_GPIO_PWM
 *  \brief GPIO used for PWM. GPIO must be disabled so that PWM may be used 
 */
#define BBU_GPIO_PWM           28

/*! \def BBU_BAT_EN_DELAY_USEC
 *  \brief used for switching batteries 
 */
#define BBU_BAT_EN_DELAY_USEC  1/*1000*/ /* 1 millisecond */

/******************************************************************************
 *                             Working Parameters                             *
 ******************************************************************************/ 

/*! \def BBU_IS_TEMPERATURE_ABOVE_THRESHOLD, BBU_IS_TEMPERATURE_BELOW_THRESHOLD
 *  \brief macros for comparing temperature values.
 *   Note - logic is reversed
 */
#define BBU_IS_TEMPERATURE_ABOVE_THRESHOLD(sample, threshold) (sample < threshold) /*Note - logic is reversed */
#define BBU_IS_TEMPERATURE_BELOW_THRESHOLD(sample, threshold) (sample > threshold) /*Note - logic is reversed */
                                                               

/*! \def BBU_SENSE_CURRENT_VALUE
 *  \brief value of the current-sense resistor
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_SENSE_CURRENT_VALUE             0.05 /* ohm */
#define BBU_SENSE_CURRENT_VALUE_MANTISSA     0x66666666
#define BBU_SENSE_CURRENT_VALUE_EXPONENT    0xfffffffc

/*! \def BBU_SENSE_CURRENT_ZERO
 *  \brief value of the current considered zero
 */
#define BBU_SENSE_CURRENT_ZERO              0

/*! \def BBU_SENSE_CURRENT_THRESHOLD
 *  \brief threshold for determning current direction
 *  When sampling Sense Resistor in order to calculate the current, use this 
   threshold when deciding on current direction .
 */
#define BBU_SENSE_CURRENT_THRESHOLD         30 /* mA */ 

/*! \def BBU_ESTIMATED_CURRENT
 *  \brief Estimated device average current consumption
 */
#define BBU_ESTIMATED_CURRENT               360 /* 360mA */      
                                                      
/******************************************************************************
 *                      Calibration and Value Conversion                      *
 ******************************************************************************/

/*! \def BBU_ADC_CAL_VREF
 *  \brief value of ADC reference voltage
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_ADC_CAL_VREF              1.5 
#define BBU_ADC_CAL_VREF_MANTISSA     0x60000000 
#define BBU_ADC_CAL_VREF_EXPONENT     0x00000001 

/*! \def BBU_ADC_CAL_VREF_WORD
 *  \brief  === 4095, max value on ADC WORD
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_ADC_CAL_VREF_WORD         0xFFF 
#define BBU_ADC_CAL_VREF_WORD_MANTISSA  0x7ff80000
#define BBU_ADC_CAL_VREF_WORD_EXPONENT 0x0000000c

/*! \def BBU_BATT_VOLTAGE_DIVIDER
 *  \brief ratio between the actual battery voltage and
 *   the corresponding ADC input (due to the ratio of resistors).
 */
#define BBU_BATT_VOLTAGE_DIVIDER      9.248  

/*! \def BBU_BATT_VOLTAGE_TO_ADC_WORD
 *  \brief macro used to translate a given battery voltage to the corresponding ADC word
 */
#define BBU_BATT_VOLTAGE_TO_ADC_WORD(x)    ((((x)/BBU_BATT_VOLTAGE_DIVIDER)/BBU_ADC_CAL_VREF)*BBU_ADC_CAL_VREF_WORD)

/*! \def BBU_ADC_WORD_TO_BATT_VOLTAGE
 *  \brief macro used to translate an ADC word  to battery voltage
 */
#define BBU_ADC_WORD_TO_BATT_VOLTAGE(x)    (((x) /(BBU_ADC_CAL_VREF_WORD))*(BBU_ADC_CAL_VREF)*(BBU_BATT_VOLTAGE_DIVIDER))

/*! \def BBU_CAL_VHIGH_WORD
 *  \brief  high cal constant for single ended calibration
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_VHIGH_WORD                  (3067.0)

/*! \def BBU_CAL_VLOW_WORD
 *  \brief low cal constant for single ended calibration
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_VLOW_WORD                   (1027.0)
#define BBU_CAL_VLOW_WORD_EXPONENT          (0x40300000)
#define BBU_CAL_VLOW_WORD_MANTISSA          (11)

/*! \def BBU_CAL_SINGLE_DIFFERENCE
 *  \brief  (high - low) for single ended calibration
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_SINGLE_DIFFERENCE           (BBU_CAL_VHIGH_WORD - BBU_CAL_VLOW_WORD)
#define BBU_CAL_SINGLE_DIFFERENCE_MANTISSA   (0x7f800000)
#define BBU_CAL_SINGLE_DIFFERENCE_EXPONENT   (11)

/*! \def BBU_CAL_DIFF_VLOW_WORD
 *  \brief  (low, X2) for diff calibration for ver 1
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_VLOW_WORD              (0.0)
#define BBU_CAL_DIFF_VLOW_WORD_EXPONENT     (ZERO_EXPONENT)
#define BBU_CAL_DIFF_VLOW_WORD_MANTISSA     (ZERO_MANTISSA)

/*! \def BBU_CAL_DIFF_VHIGH_WORD
 *  \brief  (high, X1) for diff calibration for ver 1
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_VHIGH_WORD             (10.92)
#define BBU_CAL_DIFF_VHIGH_WORD_EXPONENT    (4)
#define BBU_CAL_DIFF_VHIGH_WORD_MANTISSA    (0x575C28F5)

/*! \def BBU_CAL_DIFF_VLOW_WORD_VER2
 *  \brief  (low, X2) for diff calibration for ver 2
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_VLOW_WORD_VER2          (-53.89)
#define BBU_CAL_DIFF_VLOW_WORD_VER2_EXPONENT (11)
#define BBU_CAL_DIFF_VLOW_WORD_VER2_MANTISSA (0xFCA1C290)

/*! \def BBU_CAL_DIFF_VHIGH_WORD_VER2
 *  \brief  (high, X1) for diff calibration for ver 2
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_VHIGH_WORD_VER2         (0.0)

/*! \def BBU_CAL_DIFF_DIFFERENCE
 *  \brief  (low - high) for diff ended calibration ver 1
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_DIFFERENCE             (BBU_CAL_DIFF_VLOW_WORD - BBU_CAL_DIFF_VHIGH_WORD) /* -10.92 */
#define BBU_CAL_DIFF_DIFFERENCE_EXPONENT    (4)
#define BBU_CAL_DIFF_DIFFERENCE_MANTISSA    (0xA8A3D70B) 

/*! \def BBU_CAL_DIFF_DIFFERENCE
 *  \brief  (low - high) for diff ended calibration ver 2
 *   Note - mantissa and exponent in float_64 precalculated here.
 *   If value changed  - those need be changed as well!!!!!!!!!!!!!!!!!
 */
#define BBU_CAL_DIFF_DIFFERENCE_VER2          (BBU_CAL_DIFF_VLOW_WORD_VER2 - BBU_CAL_DIFF_VHIGH_WORD_VER2)
#define BBU_CAL_DIFF_DIFFERENCE_VER2_EXPONENT ( 11)   
#define BBU_CAL_DIFF_DIFFERENCE_VER2_MANTISSA  (0xFCA1C290) /*(-35E3D70)*/ 


/* todo per battery !!!!!!!!!!!!!!!!! need to add to working params */
/* definitions of the threshold-values for changing gain settings TODO!!! verify correct values check with Ilan directions, */
/* these are for discharge*/
#define BBU_GAIN_20_MAX_VAL           0xDC2 /* equals  750 mA in gain setting x20 */ /*  need to be 520 mA */
#define BBU_GAIN_15_MIN_VAL           0xC57 /* equals  700 mA in gain setting x15 */ /*  need to be 470 mA */
#define BBU_GAIN_15_MAX_VAL           0xE81 /* equals 1150 mA in gain setting x15 */  /* need to be 850 mA*/
#define BBU_GAIN_10_MIN_VAL           0xC80 /* equals 1100 mA in gain setting x10 */  /* need to be 800 mA*/
#define BBU_GAIN_10_MAX_VAL           0xEBE /* equals 1850 mA in gain setting x10 */ /* need to be 1450 mA*/
#define BBU_GAIN_1_MIN_VAL            0x98D /* equals 1800 mA in gain setting x1  */ /* need to be 1400 mA*/

#if 0 /* todo these are for charge*/
#define BBU_GAIN_20_MAX_VAL           0xDC2 /* equals  750 mA in gain setting x20 */ /*  need to be 110 mA */
#define BBU_GAIN_15_MIN_VAL           0xC57 /* equals  700 mA in gain setting x15 */ /*  need to be 80 mA */
#define BBU_GAIN_15_MAX_VAL           0xE81 /* equals 1150 mA in gain setting x15 */  /* need to be 170 mA*/
#define BBU_GAIN_10_MIN_VAL           0xC80 /* equals 1100 mA in gain setting x10 */  /* need to be 140 mA*/
#define BBU_GAIN_10_MAX_VAL           0xEBE /* equals 1850 mA in gain setting x10 */ /* need to be 300 mA*/
#define BBU_GAIN_1_MIN_VAL            0x98D /* equals 1800 mA in gain setting x1  */ /* need to be 270 mA*/
#endif

/*! \def BBU_BUCK_VOLTAGE_DIVIDER
 *  \brief  ratio between actual buck voltage 
 *   the corresponding ADC input (due to the ratio of resistors)                                     
 */
#define BBU_BUCK_VOLTAGE_DIVIDER      10.76   

/*! \def BBU_SAFETY_BUCK_CLOSED_VOLTAGE
 *  \brief  When the buck is supposed to be closed (all states except charge),
 *       this threshold is used to determine that it is infact at low voltage.                                     
 */
#define BBU_SAFETY_BUCK_CLOSED_VOLTAGE      ((4/BBU_BUCK_VOLTAGE_DIVIDER)/BBU_ADC_CAL_VREF)*BBU_ADC_CAL_VREF_WORD   /* 4 Volt */

/*! \def BBU_BUCK_SAFETY_DELAY_MSEC
 *  \brief  time after charging, not to test BBU_SAFETY_BUCK_CLOSED_VOLTAGE
 *       to let PWM stop charging                                     
 */
#define BBU_BUCK_SAFETY_DELAY_MSEC          15000 /* 15 seconds */ 

/*! \def BBU_NORMALIZING_VOLTAGE
 *  \brief  estimated average voltage thru the battery
 */
 /* todo per battery */
#define BBU_NORMALIZING_VOLTAGE(volt)       ((volt/BBU_BATT_VOLTAGE_DIVIDER)/BBU_ADC_CAL_VREF)*BBU_ADC_CAL_VREF_WORD


/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/


/**************************************************************************/
/*      EXTERN definition block                                           */                                                                 
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/


#endif /*_BBU_BOARD_DEF_H_*/
