/*
 *
 * bbu_driver.h
 * Description:
 * BBU driver related defines and types
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

/*! \file bbu_driver.h
 *  /brief BBU driver related defines and types
*/

#ifndef _BBU_DRIVER_H_
#define _BBU_DRIVER_H_

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/

#include <_tistdtypes.h>

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

/*! \def BAT_ENABLE_DATA
 *  \brief 
 *  use int32 for enabling/disabling battery, LSB 16 bits are status - 0/1 disable/enable, MSB are index - 0 or 1
 */
#define BAT_ENABLE_DATA(index, status)			\
	( (Uint32) (  ((index) << 16) | (status) ) )

/*! \def BAT_ENABLE_INDEX
 *  \brief 
 *  use int32 for enabling/disabling battery, LSB 16 bits are status - 0/1 disable/enable, MSB are index - 0 or 1
 */
#define BAT_ENABLE_INDEX(param) ( (Uint16) (param >> 16))

/*! \def BAT_ENABLE_STATUS
 *  \brief 
 *  use param1 for enabling/disabling battery, LSB 16 bits are status - 0/1 disable/enable, MSB are index - 0 or 1
 */
#define BAT_ENABLE_STATUS(param) ( (Uint16) param)

/*! \def BAT_CHARGE_STATUS
 *  \brief 
 *  use param1 for determining charge/discharge, LSB 16 bits are status - 0/1 disable/enable, MSB are index - 0 or 1
 */
#define BAT_CHARGE_STATUS(param) ( (Uint16) param)

/*! \def BBU_IOCTL_MAGIC
 *  \brief part of BBU IOCTL command - identifier
 */
#define BBU_IOCTL_MAGIC			0xF9 /*249 - driver major number */


/***************************BBU Ioctl commands**************************/

/*! \def BBUIOC_xxx
 *  \brief BBU ioctls, for each defined cmd number ( pay attention first part is always the same),
 *   data transferred and read/write indication
 */
#define BBUIOC_CALIB	_IO(BBU_IOCTL_MAGIC, BBUIOC_CALIB_CMD) /* calibrate */
#define BBUIOC_ENABLE_BAT		_IOW(BBU_IOCTL_MAGIC, BBUIOC_ENABLE_BAT_CMD, BbuIoctlDataS_t) /* enable battery, battery index parameter expected */
#define BBUIOC_SWITCH_BAT		_IOW(BBU_IOCTL_MAGIC, BBUIOC_SWITCH_BAT_CMD, BbuIoctlDataS_t) /* switch batteries, active bat index parameter expected*/
#define BBUIOC_UPDATE_WORKING_PARAMS _IOW(BBU_IOCTL_MAGIC, BBUIOC_UPDATE_WORKING_PARAMS_CMD, BbuIoctlDataS_t)
#define BBUIOC_GET_WORKING_DATA _IOR(BBU_IOCTL_MAGIC, BBUIOC_UPDATE_WORKING_PARAMS_CMD, BbuIoctlDataS_t)
#define BBUIOC_GET_ADC_DATA _IOR(BBU_IOCTL_MAGIC, BBUIOC_GET_ADC_DATA_CMD, BbuIoctlDataS_t)
#define BBUIOC_UPDATE_PWM_DATA _IOW(BBU_IOCTL_MAGIC, BBUIOC_UPDATE_PWM_DATA_CMD, BbuIoctlDataS_t)
#define BBUIOC_GET_PWM_DATA _IOR(BBU_IOCTL_MAGIC, BBUIOC_GET_PWM_DATA_CMD, BbuIoctlDataS_t)
#define BBUIOC_READ_BBU_ADDR _IOR(BBU_IOCTL_MAGIC, BBUIOC_GET_READ_REG_CMD, BbuIoctlDataS_t) 
#define BBUIOC_WRITE_BBU_ADDR _IOW(BBU_IOCTL_MAGIC, BBUIOC_GET_WRITE_REG_CMD, BbuIoctlDataS_t)
#define BBUIOC_TEST	_IOWR(BBU_IOCTL_MAGIC, BBUIOC_TEST_CMD, BbuIoctlDataS_t)
#define BBUIOC_ONE_SHOT	_IOR(BBU_IOCTL_MAGIC, BBUIOC_ONE_SHOT_CMD, BbuIoctlDataS_t)
#define BBUIOC_READ_VIOLATION	_IOR(BBU_IOCTL_MAGIC, BBUIOC_READ_VIOLATION_CMD, BbuIoctlDataS_t)
#define BBUIOC_GET_ENABLED_BAT	_IOR(BBU_IOCTL_MAGIC, BBUIOC_GET_ENABLED_BAT_CMD, BbuIoctlDataS_t)
/**/

/*Start ioctl*/
#define BBUIOC_START		_IOW(BBU_IOCTL_MAGIC, BBUIOC_START_CMD, BbuIoctlDataS_t) /* start*/
#define BBUIOC_STP		    _IOW(BBU_IOCTL_MAGIC, BBUIOC_STOP_CMD, BbuIoctlDataS_t) /* stop*/

#define BBU_IOC_MAX_NUM          BBUIOC_STP


/*! \def BBU_IOCTL_CMDS
 *  \brief BBU ioctl cmds
 * when adding  ioctls - make sure 'stop' is the last 
 */
#define  BBU_IOCTL_CMDS \
    BBU_IOCTL_CMD (BBUIOC_CALIB_CMD,               "BBUIOC_CALIB")\
    BBU_IOCTL_CMD (BBUIOC_ENABLE_BAT_CMD,          "BBUIOC_ENABLE_BAT")\
    BBU_IOCTL_CMD (BBUIOC_SWITCH_BAT_CMD,          "BBUIOC_SWITCH_BAT")\
    BBU_IOCTL_CMD (BBUIOC_UPDATE_WORKING_PARAMS_CMD,  "BBUIOC_UPDATE_WORKING_PARAMS")\
    BBU_IOCTL_CMD (BBUIOC_GET_WORKING_DATA_CMD,    "BBUIOC_GET_WORKING_DATA")\
    BBU_IOCTL_CMD (BBUIOC_GET_ADC_DATA_CMD,        "BBUIOC_GET_ADC_DATA")\
    BBU_IOCTL_CMD (BBUIOC_UPDATE_PWM_DATA_CMD,     "BBUIOC_UPDATE_PWM_DATA")\
    BBU_IOCTL_CMD (BBUIOC_GET_PWM_DATA_CMD,        "BBUIOC_GET_PWM_DATA")\
    BBU_IOCTL_CMD (BBUIOC_GET_READ_REG_CMD,        "BBUIOC_GET_READ_REG")\
    BBU_IOCTL_CMD (BBUIOC_GET_WRITE_REG_CMD,       "BBUIOC_GET_WRITE_REG")\
    BBU_IOCTL_CMD (BBUIOC_TEST_CMD,                "BBUIOC_TEST")\
    BBU_IOCTL_CMD (BBUIOC_ONE_SHOT_CMD,            "BBUIOC_ONE_SHOT")\
    BBU_IOCTL_CMD (BBUIOC_READ_VIOLATION_CMD,      "BBUIOC_READ_VIOLATION")\
    BBU_IOCTL_CMD (BBUIOC_GET_ENABLED_BAT_CMD,     "BBUIOC_GET_ENABLED_BAT")\
    BBU_IOCTL_CMD (BBUIOC_START_CMD,               "BBUIOC_START")\
    BBU_IOCTL_CMD (BBUIOC_STOP_CMD,                "BBUIOC_STOP")

/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/

#define BBU_IOCTL_CMD(cmd, cmdstring) cmd,

typedef enum
{
    BBU_IOCTL_CMDS
    BBU_IOCTL_CMDS_LAST_CMD 
} BbuIoctlCmd_e;

#undef BBU_IOCTL_CMD

/*! \struct typedef struct BbuIoctlDataS_t
 *  /brief BBU ioctl data structure for all ioctl commands
 *  ioctl may have no parameters, one or two parameters, or a buffer with data
 */
typedef struct bbuIoctlDataS
{
    Uint32 param1;      /* additional integer parameter*/
    Uint32 param2;      /* additional integer parameter*/
    Int32 bufLen;      /* buffer length */
    void  *buf;   /* data buffer   */
}BbuIoctlDataS_t;

/**************************************************************************/
/*      EXTERN definition block                                           */
/**************************************************************************/

extern const Char *BbuIoctlCmdDisplay[]; 

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/

#endif /*_BBU_DRIVER_H_*/
