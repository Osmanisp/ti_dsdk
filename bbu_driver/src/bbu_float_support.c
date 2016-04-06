/*
 *
 * bbu_float_support.c
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

/*! \file bbu_float_support.c
 *  /brief support for float64 arithmetic
 *  needed in kernel as no real floating number arithmetic is present
*/

/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/

#include "bbu_float_support.h"
#include "_tistdtypes.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/

static void RightShift64(Int32 *numHigh, Int32 *numLow, Int32 shift,Int32 signExtend);
static void LeftShift64(Int32 *numHigh, Int32 *numLow, Int32 shift);
static Int32 Divide(Int32 *resFracBits, Int32 dividend, Int32 divider);
static void AlignFloat64(bbu_float64 *num);
static Int32 FindMSBitInt64(bbu_int64 num);
static void NegFloat64(bbu_float64 num, bbu_float64 *negatedNum);
static void NegInt64(bbu_int64 *num);
static void Int64ToFloat64(bbu_int64 src, bbu_float64 *dst);
/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/

/*! \fn AddFloat64
 *  /brief adds 2 bbu_float64 integers
 *  \param[in]  a, b:integers to add
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void AddFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c)
{
    if (a.exponent >= b.exponent)
    {
        b.mantissa = (b.mantissa) >> (a.exponent - b.exponent);
        c->exponent = a.exponent;
    }
    else
    {
        a.mantissa = (a.mantissa) >> (b.exponent - a.exponent);
        c->exponent = b.exponent;
    }
    c->mantissa = a.mantissa + b.mantissa;
    if ((b.mantissa >= 0) && (a.mantissa >= 0))
    {
        if ((c->mantissa) < 0)
        {
            c->mantissa = ((c->mantissa) >> 1) & 0x7FFFFFFF;
            c->exponent = (c->exponent) + 1;        
        }
        AlignFloat64(c);
        return;
    }
    if ((b.mantissa < 0) && (a.mantissa < 0))
    {
        if ((c->mantissa) >= 0)
        {       
            c->mantissa = ((c->mantissa) >> 1) | 0x80000000;
            c->exponent = (c->exponent) + 1;
        }
        AlignFloat64(c);
        return;
    }
    AlignFloat64(c);
}

/*! \fn IsGreaterEqualZeroFloat64
 *  /brief check if bbu_float64 integer is greater/equal than zero
 *  \param[in]  num:integers to check
 *  \return 1 if num >=0, 0 otherwise
 */
Int32 IsGreaterEqualZeroFloat64(bbu_float64 num)
{
    return (num.mantissa >= 0);
}

/*! \fn SubFloat64
 *  /brief subtract 2 bbu_float64 integers
 *  \param[in]  a, b:integers to subtract b from a
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void SubFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c)
{
    NegFloat64(b, &b);
    AddFloat64(a, b, c);
}   

/*! \fn MulFloat64
 *  /brief multiply 2 bbu_float64 integers
 *  \param[in]  a, b:integers to multiply
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void MulFloat64(bbu_float64 a, bbu_float64 b, bbu_float64 *c)
{
    Int32 tmpHigh;
    Int32 tmpLow;
    volatile register Int aMant = a.mantissa;
    volatile register Int bMant = b.mantissa;

   __asm__( "SMULL  %[inp1], %[inp2], %[res1], %[res2]" 
     : [res1] "=r" ( tmpLow), [res2] "=r" (tmpHigh) : [inp1] "r" ( aMant), [inp2] "r" (bMant) : "cc") ; 

    RightShift64(&tmpHigh, &tmpLow, 31, 1);
    c->mantissa = tmpLow;
    c->exponent = a.exponent + b.exponent;
    AlignFloat64(c);

}

/*! \fn DivideFloat64
 *  /brief divide 2 bbu_float64 integers
 *  \param[in]  a, b:integers to divide a by b 
 *  \param[out] c: bbu_float64 result.
 *  \return none
 */
void DivideFloat64(bbu_float64 dividend, bbu_float64 divider, bbu_float64 *result)
{
    if (divider.mantissa == 0)
    {
        if (dividend.mantissa >=0)
        {
            result->mantissa = 0x7FFFFFFF;
            result->exponent = 0x7FFFFFFF;  
        }
        else
        {
            result->mantissa = 0x80000000;
            result->exponent = 0x7FFFFFFF;
        }
        return;
    }
    result->mantissa = Divide(&(result->exponent),dividend.mantissa,divider.mantissa);
    result->exponent = (31 - (result->exponent)) + dividend.exponent - divider.exponent;
    AlignFloat64(result);
}


/*! \fn Float64ToUint16
 *  /brief convert bbu_float64 to Uint16
 *  \param[in]  argument: bbu_float64
 *  \return Uint16 result
 */
Uint16 Float64ToUint16(bbu_float64 argument)
{
    Int32 shift;  
    Uint16 returnVal;

    /* Note - this function assumes an argument that satisfies: 0 <= argument < 2^(16) */

    if (argument.exponent > 0)
    {
        shift = (31 - argument.exponent);
        returnVal = (argument.mantissa >> shift) & 0xFFFF;
    }
    else
    {
        returnVal = 0;
    }

    return returnVal;
}

/*! \fn Float64ToInt
 *  /brief convert bbu_float64 to Int32
 *  \param[in]  argument: bbu_float64
 *  \return Int32 result
 */
Int32 Float64ToInt(bbu_float64 argument)
{
    Int32 shift;  
    Int32 returnVal;

    /* Note - this function assumes an argument that satisfies the range of an 32 bit */

    if (argument.exponent > 0)
    {
        shift = (31 - argument.exponent);
        returnVal = (argument.mantissa >> shift);
    }
    else
    {
        returnVal = 0;
    }

    return returnVal;
}


/*! \fn Int32ToFloat64
 *  /brief convert integet to bbu_float64 form
 *  \param[in]  src: integer
 *  \param[out] dst: bbu_float64.
 *  \return 1 if num >=0, 0 otherwise
 */
void Int32ToInt64(Int32 src, bbu_int64 *dst)
{
    dst->LSB = src;
    if (src < 0)
    {
        dst->MSB = 0xFFFFFFFF;
    }
    else
    {
        dst->MSB = 0;
    }
}

/*! \fn InitFloat64
 *  /brief fill bbu_float64 integer with values
 *  \param[in]  mantissa, exponent
 *  \param[out] dst: bbu_float64 result.
 *  \return none
 */
void InitFloat64(Int32 mantissa, Int32 exponent, bbu_float64 *dst)
{
    dst->mantissa = mantissa;
    dst->exponent = exponent;
}

/*! \fn Int32ToFloat64
 *  /brief convert integet to bbu_float64 form
 *  \param[in]  src: integer
 *  \param[out] dst: bbu_float64.
 *  \return none
 */
void Int32ToFloat64(Int32 src, bbu_float64 *dst)
{
    bbu_int64 tempInt64;

    Int32ToInt64(src, &tempInt64);

    Int64ToFloat64(tempInt64, dst);
}

/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/

static void RightShift64(Int32 *numHigh, Int32 *numLow, Int32 shift,Int32 signExtend)
{
    Int32 tmp1;
    Int32 tmp2;
    tmp1 = (1 << (32-shift));
    tmp1 = tmp1 - 1;
    (*numLow) = ((*numLow) >> shift) & tmp1;
    tmp2 = ((*numHigh) << (32 - shift));
    (*numLow) += tmp2;
    if (signExtend)
    {
        (*numHigh) = ((*numHigh) >> shift);
    }
    else
    {
        (*numHigh) = ((*numHigh) >> shift) & tmp1;
    }
}       


static void LeftShift64(Int32 *numHigh, Int32 *numLow, Int32 shift)
{
    Int32 tmp;    
    if (shift >=32)
    {
        *numHigh = (*numLow) << (shift - 32);
        *numLow = 0;
    }
    else
    {
        tmp = (1 << shift) - 1;
        *numHigh = ((*numHigh) << shift) + (((*numLow) >> (32 - shift)) & tmp);
        *numLow = (*numLow) << shift;
    }
}

static void NegFloat64(bbu_float64 num, bbu_float64 *negatedNum)
{
    negatedNum->mantissa = -(num.mantissa);
    negatedNum->exponent = num.exponent;
}


static void NegInt64(bbu_int64 *num)
{
    num->LSB = -(num->LSB);
    num->MSB = 0xFFFFFFFF - (num->MSB);
}

/* This function performs a division of two integers which have the same fractional size
   and returns a result in a fixed point format                                         */
static Int32 Divide(Int32 *resFracBits, Int32 dividend, Int32 divider)
{
    Int32 sign;
    Int32 nextBit;
    Uint32 tmpDivider;
    Uint32 tmpDividend;
    Int32 intPart;
    Int32 fracPart;
    Int32 result;
    Int32 i;

    sign = 0;
    intPart = 0;
    fracPart = 0;

    if (dividend & 0x80000000)
    {
        dividend = -dividend;
        sign = 1;       
    }
    if (divider & 0x80000000)
    {
        divider = -divider;
        sign = 1 - sign;
    }
    tmpDividend = dividend;
    tmpDivider = divider;
    (*resFracBits) = 31;

    while (tmpDividend >= tmpDivider)
    {
        nextBit = 1;
        while (tmpDividend >= (tmpDivider << 1))
        {
            nextBit = nextBit << 1;
            tmpDivider = tmpDivider << 1;
        }
        intPart += nextBit;
        tmpDividend -= tmpDivider;
        tmpDivider = divider;
    }
    if (intPart > 0)
    {
        nextBit = 1;
        while (intPart >= nextBit)
        {
            nextBit = nextBit << 1;             
            (*resFracBits) = (*resFracBits) - 1;
        }
    }
    nextBit = 1 << ((*resFracBits) - 1);
    tmpDividend = tmpDividend << 1;
    for (i=0;i < (*resFracBits);i++)
    {
        if (tmpDividend >= divider)
        {
            fracPart += nextBit;    
            tmpDividend -= divider;             
        }
        tmpDividend = tmpDividend << 1;
        nextBit = nextBit >> 1;
    }
    result = ((intPart << (*resFracBits)) + fracPart);
    if (sign)
    {
        result = -result;
    }
    return result; 
}


static void AlignFloat64(bbu_float64 *num)
{
    Int32 i;
    Int32 sign;

    if (num->mantissa == 0)
    {
        num->exponent = 0;  
        return;
    }
    if (num->mantissa == 0x80000000)
    {
        return;
    }
    if ((num->mantissa) >= 0)
    {
        sign = 0;
    }
    else
    {
        sign = 1;
        num->mantissa = -(num->mantissa);
    }
    i = 0;
    while (((num->mantissa) & 0x40000000) == 0)
    {
        num->mantissa = (num->mantissa) << 1;
        i--;
    }
    num->exponent = (num->exponent) + i;
    if (sign == 1)
    {
        num->mantissa = -(num->mantissa);
    }
}

static void Int64ToFloat64(bbu_int64 src, bbu_float64 *dst)
{
    Int32 MSBit;
    Int32 sign;

    sign = 0;
    if (src.MSB < 0)
    {
        sign = 1;
        NegInt64(&src);
    }
    MSBit = FindMSBitInt64(src);    
    LeftShift64(&(src.MSB), &(src.LSB), 62 - MSBit);
    if (sign == 1)
    {
        NegInt64(&src);
    }
    dst->mantissa = src.MSB;
    dst->exponent = MSBit + 1;
}

static Int32 FindMSBitInt64(bbu_int64 num)
{
    Int32 MSBit;
    Int32 mask;
        
    if ((num.MSB & 0x80000000) && (num.LSB == 0))
    {
        return 63;
    }
    mask = 0x80000000;
    if (num.MSB & mask)
    {
        NegInt64(&num);
    }
    mask = 0x40000000;  
    MSBit = 62;
    while (MSBit >= 32)
    {
        if (num.MSB & mask)
        {
            return MSBit;
        }
        MSBit--;
        mask = mask >> 1;
    }
    mask = 0x80000000;
    while (MSBit > 0)
    {
        if (num.LSB & mask)
        {
            return MSBit;
        }
        MSBit--;
        mask = mask >> 1;
    }
    return MSBit; 
}





