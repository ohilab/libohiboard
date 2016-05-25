/******************************************************************************
 * Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Francesco Piunti <francesco.piunti89@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale    <matteo.civale@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

/**
 * @file libohiboard/include/xbar.h
 * @author Matteo Civale    <matteo.civale@gmail.com>
 */
#ifdef LIBOHIBOARD_XBAR



#if defined (LIBOHIBOARD_KV46F)     || \
    defined (LIBOHIBOARD_TRWKV46F)

#include "xbar.h"
#include "errors.h"


#define XBAR_A_MAX_IN   51
#define XBAR_A_MAX_OUT  58

#define XBAR_B_MAX_IN   27
#define XBAR_B_MAX_OUT  15

#define XBAR_SEL_SHIFT(X)   8*(X%2)

typedef struct Xbar_Device {

    XBARA_MemMapPtr  regMapA;
    XBARB_MemMapPtr  regMapB;

    volatile uint32_t* simScgPtr;
    uint32_t maskClockEn;
    Xbar_Input input[XBAR_A_MAX_IN];
    Xbar_Output output[XBAR_A_MAX_OUT];


    bool devinitialized;

    void (*isr) (void);

}Xbar_Device;





static Xbar_Device xbarA={
        .regMapA     = XBARA,
        .simScgPtr   = &SIM_SCGC5,
        .maskClockEn = SIM_SCGC5_XBARA_MASK,

        /* Set input */
        .input[42]   = XBAR_IN_PIT_CH0,

        /* Set output */

        .output[15]  = XBAR_OUT_DAC0_12B_SYNC,
        .isr         = 0,
 };

Xbar_DeviceHandle OB_XBARA = &xbarA;

static Xbar_Device xbarB={

};

uint8_t Xbara_findCannelNum(uint8_t* channelList, uint8_t channel, uint8_t len);


/*
 * Function implementation
 * */
System_Errors Xbar_init(Xbar_DeviceHandle dev)
{

    if(dev->devinitialized)
        return ERRORS_XBAR_JUST_INIT;


    *dev->simScgPtr |= dev->maskClockEn;

    return ERRORS_NO_ERROR;
}

System_Errors Xbar_ruteInToOut(Xbar_DeviceHandle dev, Xbar_Input in, Xbar_Output out)
{
    uint8_t inNum, outNum;
    uint32_t mask;
    volatile uint32_t* ptr;
    uint32_t shift;

    if(!dev->devinitialized)
        return ERRORS_XBAR_NOT_INIT;

    /* Find in index */
    if((inNum = Xbara_findCannelNum(dev->input, in, XBAR_A_MAX_IN))<0)
        return ERRORS_XBAR_IN_WRONG;
    /* Find out index */
    if((outNum = Xbara_findCannelNum(dev->output, out, XBAR_A_MAX_OUT))<0)
        return ERRORS_XBAR_OUT_WRONG;


    if(dev==OB_XBARA)
    {
        ptr= (volatile uint32_t*)(&XBARA_SEL0_REG(dev->regMapA) + (uint8_t)(outNum>>1));
        shift =XBAR_SEL_SHIFT(outNum);
        mask = 0x3F<<shift;

        *ptr &= ~mask;
        *ptr |= (in<<shift)&mask;
    }
    else
    {

    }
}
#endif

uint8_t Xbara_findCannelNum(uint8_t* channelList, uint8_t channel, uint8_t len)
{
    uint8_t i;

    i=0;
    while((i<len)&&(channelList[i]!=channel)){i++;}

    if (i==len)
        return -1;
    else
        return i;
}

#endif /* LIBOHIBOARD_XBAR*/
