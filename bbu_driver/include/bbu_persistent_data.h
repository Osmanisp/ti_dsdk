/*
 *
 * bbu_persistent_data.h
 * Description:
 * BBU data persistent over restarts except full power reset
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

/***************************************************************************/
/*! \file bbu_persistent_data.h
 *  /brief BBU data persistent over restarts except full power reset
****************************************************************************/

#ifndef _BBU_PERSISTENT_DATA_H_
#define _BBU_PERSISTENT_DATA_H_

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/
#include <_tistdtypes.h>
#include "bbu_board_def.h"

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/


/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/

/*! \struct typedef struct BbuSingleBatStatus_t
 *  /brief per battery persistent data
 */
typedef struct
{
    Bool             batStatus;         /* Is battery  inserted.                             */
    Uint32           batId;            /* votage ID one to one related to type               */
    Uint32           batSoc;           /* Battery  State of Charge.                          */
    Uint16           batType;          /* The type (overall capacity) of battery             */
    Uint32           batCycles;        /* Number of charging cycles performed on battery     */
    Bool             batBad;           /* Indicates if battery  is considered "bad".         */
    Bool             batShutDown;      /* The battery is shutdown due to failure.            */
    Uint32    batLastChargeTime;       /* The last time the battery was fully-charged.       */
}BbuSingleBatStatus_t;


/* todo some data does not need to be persistent, move it to other struct in shared db*/ 


/*! \struct typedef struct Bbu_BatteryPersistentData_t
 *  /brief all persistent data kept in P5 on -chip memory
 *  more details (not mandatory)
 */
typedef struct 
{
    BbuSingleBatStatus_t batteriesStatus[BBU_NUM_OF_BATTERIES_SUPPORTED];

    Int32       currentDirection;    /* Direction of the current through the batteries.         */
#if 0 /* not supported yet*/
    Uint16  psmThreshold;           /* Low-Battery Threshold defined by Power Save Mechanism.  */
#endif
    Uint32    mibThreshold;        /* Low-Battery Threshold defined by MIB.                   */
    Int32     mibStatus;           /* Battery status as defined by MIB                        */
    Int32     batteryOutputEn;     /* A flag which indicates if the battery output is enabled.*/
    Uint32    dischargeStartTime;  /* Time that the current discharge process started.        */
    Uint16    chargePercentage;    /* Percentage of charge left in the batteries.             */ /* todo maybe not needed here*/
    Uint16    chargeMinutes;       /* Number of minutes left on batteries (at current load).  */ /* todo maybe not needed here*/
    Uint32    maintainTime;        /* Time that the system has been in maintain state.        */ /* maybe not needed here*/
    Int32     temperatureShutdown; /* Temperature is out-of-range, the batteries are shutdown.*/
    Int32     buckVoltageShutdown; /* Buck voltage is out-of-range, the system is shutdown.   */
    Uint32    bbuBuckSafetyTimer;  /* A counter used for delaying the buck-safety mechanism.  */ /* maybe not needed here*/
    Uint32    bbuVoltSafetyTimer;  /* A counter used for delaying the voltage-safety mechanism*/ /* maybe not needed here*/
    volatile Uint32    validityIndicator;  /* Used to check the validity of the DB after reset.       */
} Bbu_BatteryPersistentData_t;

/**************************************************************************/
/*      EXTERN definition block                                           */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/



#endif /*_BBU_PERSISTENT_DATA_H_*/


