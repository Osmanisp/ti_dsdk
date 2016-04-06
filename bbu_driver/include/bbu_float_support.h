/*
 *
 * bbu_float_support.h
 * Description:
 * Support for float64 arithmetic
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

/*! \file bbu_float_support.h
 *  /brief support for float64 arithmetic
 *  needed in kernel as no real floating number arithmetic is present
*/

#ifndef BBU_FLOAT_SUPPORT_H
#define BBU_FLOAT_SUPPORT_H

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/
#include "_tistdtypes.h"

/**************************************************************************/
/*      INTERFACE MACRO Definitions                                       */
/**************************************************************************/

/*! \def BBU_HALF_VALUE_MANTISSA, BBU_HALF_VALUE_EXPONENT
 *  \brief '0.5' constant used for rounding - precalculated mantissa and exponent in float_64 notation
 */
#define BBU_HALF_VALUE_MANTISSA     (0x40000000)
#define BBU_HALF_VALUE_EXPONENT     (0x00000000)

/*! \def ONE_MANTISSA, ONE_EXPONENT
 *  \brief '1' constant used for calculations - precalculated mantissa and exponent  in float_64 notation
 */
#define ONE_MANTISSA (0x40000000)
#define ONE_EXPONENT (0x00000001)

/*! \def ZERO_MANTISSA, ZERO_EXPONENT
 *  \brief '0' constant used for calculations - precalculated mantissa and exponent  in float_64 notation
 */
#define ZERO_MANTISSA (0x00000000)
#define ZERO_EXPONENT (0x00000001)

/*! \def THOUSAND_MANTISSA, THOUSAND_EXPONENT
 *  \brief '1000' constant used for calculations - precalculated mantissa and exponent  in float_64 notation
 */
#define THOUSAND_MANTISSA (0x7d000000)
#define THOUSAND_EXPONENT (0x0000000a)

/**************************************************************************/
/*      INTERFACE TYPES and STRUCT Definitions                            */
/**************************************************************************/

/*! \struct typedef struct bbu_float64
 *  /brief float 64 representation of integers
 */
typedef struct
{
    Int32 mantissa;
    Int32 exponent;
} bbu_float64;

/*! \struct typedef struct bbu_int64
 *  /brief int 64 representation of integers
 */
typedef struct
{
    Int32 MSB;
    Int32 LSB;
} bbu_int64;

/**************************************************************************/
/*      EXTERN definition block                                           */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/

/*! \fn AddFloat64
 *  /brief adds 2 bbu_float64 integers
 *  \param[in]  a, b:integers to add
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void AddFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c);

/*! \fn SubFloat64
 *  /brief subtract 2 bbu_float64 integers
 *  \param[in]  a, b:integers to subtract b from a
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void SubFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c);

/*! \fn MulFloat64
 *  /brief multiply 2 bbu_float64 integers
 *  \param[in]  a, b:integers to multiply
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void MulFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c);

/*! \fn DivideFloat64
 *  /brief divide 2 bbu_float64 integers
 *  \param[in]  a, b:integers to divide a by b 
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void DivideFloat64(bbu_float64 dividend, bbu_float64 divider, bbu_float64 *result);

/*! \fn IsGreaterEqualZeroFloat64
 *  /brief check if bbu_float64 integer is greater/equal than zero
 *  \param[in]  num:integers to check
 *  \return 1 if num >=0, 0 otherwise
 */
int IsGreaterEqualZeroFloat64(bbu_float64 num);

/*! \fn InitFloat64
 *  /brief fill bbu_float64 integer with values
 *  \param[in]  mantissa, exponent
 *  \param[out] dst: bbu_float64 result.
 *  \return none
 */
void InitFloat64(Int32 mantissa, Int32 exponent, bbu_float64 *dst);

/*! \fn Int32ToFloat64
 *  /brief convert integet to bbu_float64 form
 *  \param[in]  src: integer
 *  \param[out] dst: bbu_float64.
 *  \return none
 */
void Int32ToFloat64(Int32 src, bbu_float64 *dst);

/*! \fn Float64ToUint16
 *  /brief convert bbu_float64 to Uint16
 *  \param[in]  argument: bbu_float64
 *  \return Uint16 result
 */
Uint16 Float64ToUint16(bbu_float64 argument);

/*! \fn Float64ToInt
 *  /brief convert bbu_float64 to int
 *  \param[in]  argument: bbu_float64
 *  \return int result
 */
Int32 Float64ToInt(bbu_float64 argument);

#endif /* BBU_FLOAT_SUPPORT_H */
