/*
 *
 * bbu_regs.h
 * Description:
 * P5 board addresses and types for use by driver
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

/*! \file bbu_regs.h
 *  /brief P5 board addresses and types for use by driver
 *
*/

#ifndef _BBUREGS_H_
#define _BBUREGS_H_

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/

#include "puma5_hardware.h" 
#include "bbu_types.h"

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

/****************************************************************************************************************/
/*          Note:  IO_PHY2VIRT macro is used to convert physical memory addresses to virtual ones accessible    */
/*               by software                                                                                    */

/******************************************************************************
 *                                  BBU - for P5 chip                             *
 ******************************************************************************/

/*! \def BBU_REGISTER_BASE_PUMA5
 *  \brief BBU control register
 *  
 */
#define BBU_REGISTER_BASE_PUMA5  IO_PHY2VIRT(0x08611C00)

/*! \def BBU_ENABLED_REG_PUMA5
 *  \brief BBU Enabled register
 *  \ if bit 3 is  “1” ,  BBU is disabled
 *  
 */
#define BBU_ENABLED_REG_PUMA5 IO_PHY2VIRT(0x08611a24) 
#define BBU_ENABLED_MASK 0x8   /*bit 3 – “1” means that the BBU is disabled*/

/*! \def BBU_P5_STORAGE
 *  \brief BBU Persistent data storage in chip internal memory
 *  \ This is BBU data kept over resets except full power reset
 *  
 */
#define BBU_P5_STORAGE IO_PHY2VIRT(0x0)

/******************************************************************************
 *                                  PWM Control                               *
 ******************************************************************************/

/*! \def BBU_PWMCR_PUMA5
 *  \brief PWM control register 
 *  \ controls pwm disabled/enabled and frequency
 *  
 */
#define BBU_PWMCR_PUMA5                     BBU_REGISTER_BASE_PUMA5+0x0

#define BBU_PWMCR_PUMA5_DC_MASK             0xFFFF0000
#define BBU_PWMCR_PUMA5_RANGE_MASK          0x0000FFFF
#define BBU_PWMCR_PUMA5_MAX_VAL             BBU_PWMCR_MAX_VAL

#define BBU_PWMCR_RANGE_DEFAULT             BBU_PWM_RANGE_DEFAULT

#define BBU_PWMCR_PUMA5_DC_SHIFT            16
    
#define BBU_PWMEN_PUMA5                     BBU_REGISTER_BASE_PUMA5+0xC

#define BBU_PWMEN_PUMA5_ENABLE_VAL          0x1
#define BBU_PWMEN_PUMA5_DISABLE_VAL         0x0
 
/******************************************************************************
 *                                  BBU ADC Mode                              *
 ******************************************************************************/

/*! \typedef enum Bbu_AdcModeType_e
 *  /brief ADC Operational modes
 */
typedef enum {
    ADC_MODE_UNKNOWN = -1,
    ADC_MODE_IDLE = 0,
    ADC_MODE_DFT,
    ADC_MODE_CONT,
    ADC_MODE_ONE_SHOT,
    ADC_MODE_SINGLE
}Bbu_AdcModeType_e; 

/*! \def BBU_AVERAGING_TYPE_2_SHIFT
 *  \brief convert from the defined type "BBU_AVERAGING_MODE_TYPE" to the shift
 *  \ needed in order to compensate for averaging.
 *    e.g.:
 *  BBU_AVERAGING_TYPE_2_SHIFT(AVERAGING_MODE_8) = 3
 *  8 averaged samples must be shifted by 3  
 *  AVERAGING_MODE_1 means no averaging and no shift to be done  
 */
#define BBU_AVERAGING_TYPE_2_SHIFT(x) ((x == AVERAGING_MODE_1) ? 0 :  (((x)+1)%5) )

/*! \def P5_VERSION_REV1, P5_VERSION_REV2
 *  \brief Puma5 version ID 
 *  \ the version influences the way BBU calibration is done
 *  
 */
#define P5_VERSION_REV1      0
#define P5_VERSION_REV2      1

/*! \def MAX_ADC_IDLE_POLL
 *  \brief max number of times to poll ADC register to verify idle bit has been set- when trying to set IDLE explicitely 
 *  \if after this number of times ADC is not IDLE yet, there is a problem and BBU subsystem should be reset 
 *  
 */
#define MAX_ADC_IDLE_POLL                   10000 

/*! \def MAX_ADC_SAMPLE_POLL
 *  \brief max number of times to poll the valid bit indicating data is ready
 *    if after this number of times data is not ready, there is a problem and BBU subsystem should be reset
 *  
 */
#define MAX_ADC_SAMPLE_POLL                 10000 

/*! \def BBU_MODECR_PUMA5
 *  \brief controlling ADC - setting mode, controlling averaging 
 *  
 */
#define BBU_MODECR_PUMA5                    BBU_REGISTER_BASE_PUMA5+0x4

#define BBU_MODECR_PUMA5_IDLE_SHIFT         0
#define BBU_MODECR_PUMA5_STOP_SHIFT         3
#define BBU_MODECR_PUMA5_DFT_SHIFT          4
#define BBU_MODECR_PUMA5_CONT_SHIFT         5
#define BBU_MODECR_PUMA5_ONE_SHOT_SHIFT     6
#define BBU_MODECR_PUMA5_SINGLE_SHIFT       7
#define BBU_MODECR_PUMA5_AVERAGE_SHIFT      8
#define BBU_MODECR_PUMA5_AVGNUM_SHIFT       9

#define BBU_MODECR_PUMA5_IDLE_MASK          (0x1<<BBU_MODECR_PUMA5_IDLE_SHIFT)
#define BBU_MODECR_PUMA5_STOP_MASK          (0x1<<BBU_MODECR_PUMA5_STOP_SHIFT)
#define BBU_MODECR_PUMA5_DFT_MASK           (0x1<<BBU_MODECR_PUMA5_DFT_SHIFT)
#define BBU_MODECR_PUMA5_CONT_MASK          (0x1<<BBU_MODECR_PUMA5_CONT_SHIFT)
#define BBU_MODECR_PUMA5_ONE_SHOT_MASK      (0x1<<BBU_MODECR_PUMA5_ONE_SHOT_SHIFT)
#define BBU_MODECR_PUMA5_SINGLE_MASK        (0x1<<BBU_MODECR_PUMA5_SINGLE_SHIFT)
#define BBU_MODECR_PUMA5_AVERAGE_MASK       (0x1<<BBU_MODECR_PUMA5_AVERAGE_SHIFT)
#define BBU_MODECR_PUMA5_AVGNUM_MASK        (0x3<<BBU_MODECR_PUMA5_AVGNUM_SHIFT)

                                              
/******************************************************************************
 *                                  ADC Analog Control                        *
 ******************************************************************************/

/*! \def BBU_MODECR_PUMA5
 *  \brief controlling ADC for purposes of calibration, gain control, differential measurements enabling
 *  
 */
#define BBU_ACTRL_PUMA5                     BBU_REGISTER_BASE_PUMA5+0x8     
        
#define BBU_ACTRL_PUMA5_MODE_SHIFT          0
#define BBU_ACTRL_PUMA5_BPASS_IBUF_SHIFT    1
#define BBU_ACTRL_PUMA5_PWDN_DBUF_SHIFT     2
#define BBU_ACTRL_PUMA5_DIFF_SHIFT          3
#define BBU_ACTRL_PUMA5_SELCALIN_SHIFT      4
#define BBU_ACTRL_PUMA5_CALMODE_SHIFT       5
#define BBU_ACTRL_PUMA5_GAINSEL_SHIFT       12

/* for rev 2*/
#define BBU_ACTRL_PUMA5_TESTMODE_0_SHIFT      6
#define BBU_ACTRL_PUMA5_TESTMODE_1_SHIFT      7
#define BBU_ACTRL_PUMA5_TESTMODE_2_SHIFT      8
#define BBU_ACTRL_PUMA5_TESTMODE_3_SHIFT      9

#define BBU_ACTRL_PUMA5_MODE_MASK           (0x1<<BBU_ACTRL_PUMA5_MODE_SHIFT)
#define BBU_ACTRL_PUMA5_BPASS_IBUF_MASK     (0x1<<BBU_ACTRL_PUMA5_BPASS_IBUF_SHIFT)                                                                
#define BBU_ACTRL_PUMA5_PWDN_DBUF_MASK      (0x1<<BBU_ACTRL_PUMA5_PWDN_DBUF_SHIFT)
#define BBU_ACTRL_PUMA5_DIFF_MASK           (0x1<<BBU_ACTRL_PUMA5_DIFF_SHIFT)
#define BBU_ACTRL_PUMA5_SELCALIN_MASK       (0x1<<BBU_ACTRL_PUMA5_SELCALIN_SHIFT)
#define BBU_ACTRL_PUMA5_CALMODE_MASK        (0x1<<BBU_ACTRL_PUMA5_CALMODE_SHIFT)
#define BBU_ACTRL_PUMA5_GAINSEL_MASK        (0x3<<BBU_ACTRL_PUMA5_GAINSEL_SHIFT)

/* for rev 2*/
#define BBU_ACTRL_PUMA5_TESTMODE_0_MASK     (0x1<<BBU_ACTRL_PUMA5_TESTMODE_0_SHIFT)
#define BBU_ACTRL_PUMA5_TESTMODE_1_MASK     (0x1<<BBU_ACTRL_PUMA5_TESTMODE_1_SHIFT)
#define BBU_ACTRL_PUMA5_TESTMODE_2_MASK     (0x1<<BBU_ACTRL_PUMA5_TESTMODE_2_SHIFT)
#define BBU_ACTRL_PUMA5_TESTMODE_3_MASK     (0x1<<BBU_ACTRL_PUMA5_TESTMODE_3_SHIFT)

/******************************************************************************
 *                             ADC Single and DFT Mode                        *
 ******************************************************************************/

/*! \def BBU_ADCCHNL_PUMA5
 *  \brief settings for working in ADC single and DFT mode
 *  
 */
#define BBU_ADCCHNL_PUMA5                   BBU_REGISTER_BASE_PUMA5+0x60     

#define BBU_ADCCHNL_PUMA5_MASK              0x7
                   
#define BBU_ADCDATA_PUMA5                   BBU_REGISTER_BASE_PUMA5+0x14

#define BBU_ADCDATA_PUMA5_DATA_SHIFT        0
#define BBU_ADCDATA_PUMA5_VALID_SHIFT       16
#define BBU_ADCDATA_PUMA5_DFT_VIO_SHIFT     17

#define BBU_ADCDATA_PUMA5_DATA_MASK         (0xFFFF << BBU_ADCDATA_PUMA5_DATA_SHIFT)
#define BBU_ADCDATA_PUMA5_VALID_MASK        (0x1 << BBU_ADCDATA_PUMA5_VALID_SHIFT)
#define BBU_ADCDATA_PUMA5_DFT_VIO_MASK      (0x1 << BBU_ADCDATA_PUMA5_DFT_VIO_SHIFT)

/*! \def BBU_SINGLE_DATA_VALID_PUMA5
 *  \brief macro for quick access to check single-mode data validity
 *  
 */
#define BBU_SINGLE_DATA_VALID_PUMA5         ( (*(volatile Uint32*)((Uint32)BBU_ADCDATA_PUMA5) & BBU_ADCDATA_PUMA5_VALID_MASK )


/******************************************************************************
 *                                 ADC One-Shot Mode                          *
 ******************************************************************************/

/*! \def BBU_ONE_SHOT_VALID_REG_PUMA5
 *  \brief settings for working in ADC one shot mode
 *  
 */
#define BBU_ONE_SHOT_VALID_REG_PUMA5        BBU_REGISTER_BASE_PUMA5+0x20   

#define BBU_ONE_SHOT_VALID_PUMA5_SHIFT      16  
#define BBU_ONE_SHOT_VALID_PUMA5_MASK       (0x1 << BBU_ONE_SHOT_VALID_PUMA5_SHIFT)

/*! \def BBU_ONE_SHOT_DATA_VALID_PUMA5
 *  \brief macro for quick access to check one-shot-mode data validity
 *  
 */
#define BBU_ONE_SHOT_DATA_VALID_PUMA5       ( (*(volatile Uint32*)((Uint32)BBU_ONE_SHOT_VALID_REG_PUMA5) & BBU_ONE_SHOT_VALID_PUMA5_MASK )

/*! \def BBU_ONE_SHOT_CHANNEL_REG_PUMA5
 *  \brief macro for access to one-shot value registers. note that that the channel number is assumed to be between 0-7
 *  
 */
#define BBU_ONE_SHOT_CHANNEL_REG_PUMA5(x)   BBU_REGISTER_BASE_PUMA5+0x20+(0x8*(x&0x7))

/*! \def BBU_ONE_SHOT_PUMA5_DATA_MASK
 *  \brief mask assisting to get a value of a channel 
 *  
 */
#define BBU_ONE_SHOT_PUMA5_DATA_MASK        0xFFFF

/*! \typedef struct Bbu_OneShotDataType_t
 *  /brief container for holding sampled data from all 8 channels
 */
typedef struct{
	Uint16 vals[MAX_ADC_CHANNEL_INDEX + 1];
}Bbu_OneShotDataType_t;


/******************************************************************************
 *                                ADC Continuous Mode                         *
 ******************************************************************************/

/*! \def BBU_CONT_VIO_CH_PUMA5, BBU_CONT_VIO_DATA_PUMA5
 *  \brief settings for working in ADC continuous mode
 *  
 */
#define BBU_CONT_VIO_CH_PUMA5               BBU_REGISTER_BASE_PUMA5+0x10   

#define BBU_CONT_VIO_CH_PUMA5_MASK          0x7
 
#define BBU_CONT_VIO_DATA_PUMA5             BBU_REGISTER_BASE_PUMA5+0x14   

#define BBU_CONT_VIO_DATA_PUMA5_MASK        0xFFFF

/*! \def BBU_CONT_MIN_LIMIT_CH_PUMA5, BBU_CONT_MAX_LIMIT_CH_PUMA5
 *  \brief macro for access to continuous-mode limit registers. note that that the channel number is assumed to be between 0-7
 *  
 */                                                    
#define BBU_CONT_MIN_LIMIT_CH_PUMA5(x)      BBU_REGISTER_BASE_PUMA5+0x20+(0x8*(x&0x7))
#define BBU_CONT_MAX_LIMIT_CH_PUMA5(x)      BBU_REGISTER_BASE_PUMA5+0x24+(0x8*(x&0x7))

#define BBU_CONT_LIMIT_MASK_PUMA5           0xFFFF

  
  
/******************************************************************************
 *                                Additional Chip Registers                       *
 ******************************************************************************/

/*! \def BBU_CR 
 *  \brief settings for BBU Clock Control - in Pripheral Clock Control Register #1
 *  
 */
#define BBU_CR                              IO_PHY2VIRT(0x08611b70) /*control register*/
#define PCLKCR1_PUMA5_BBU_CLK_DIV_MASK      0x00000FF0
#define PCLKCR1_PUMA5_BBU_CLK_DIV_MAX       0xFF
#define PCLKCR1_PUMA5_BBU_CLK_DIV_MIN       0xD
#define BBU_CR_DIV_SHIFT                    4

/*! \def GPIO_PUMA5_BASE 
 *  \brief settings for access to P5 GPIOs (one of GPIOs is used for PWM)
 *  
 */
#define GPIO_PUMA5_BASE                     IO_PHY2VIRT(0x08610900)

/* Standard GPIOs *//*0-31*/
#define GPIO_PUMA5_IN                       GPIO_PUMA5_BASE+0x0
#define GPIO_PUMA5_OUT                      GPIO_PUMA5_BASE+0x4
#define GPIO_PUMA5_DIR                      GPIO_PUMA5_BASE+0x8
#define GPIO_PUMA5_EN                       GPIO_PUMA5_BASE+0xC

/* Auxiliary GPIOs */ /*32-63*/       
#define GPIO_PUMA5_AUX_IN                   GPIO_PUMA5_BASE+0x28
#define GPIO_PUMA5_AUX_OUT                  GPIO_PUMA5_BASE+0x2C
#define GPIO_PUMA5_AUX_DIR                  GPIO_PUMA5_BASE+0x30
#define GPIO_PUMA5_AUX_EN                   GPIO_PUMA5_BASE+0x34

/*registers for perfoming BBU reset*/
#define BBU_ADC_CNTL            IO_PHY2VIRT(0x08621A24)
#define BBU_ADC_ACT             IO_PHY2VIRT(0x08621120)
#define BBU_ADC_STS             IO_PHY2VIRT(0x08621128)

#endif /*_BBUREGS_H_*/

