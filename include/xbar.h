/******************************************************************************
 * Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Francesco Piunti <francesco.piunti89@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <m.civale@gmail.com>
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

#ifndef __XBAR_H
#define __XBAR_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum
{
    XBAR_IN_PIT_CH0 = 0x1,
}Xbar_Input;

typedef enum
{
    XBAR_OUT_DAC0_12B_SYNC = 0x1,
}Xbar_Output;

typedef struct Xbar_Device* Xbar_DeviceHandle ;


extern Xbar_DeviceHandle OB_XBARA;
extern Xbar_DeviceHandle OB_XBARB;

/**
 * This function initialize the XBAR device
 *
 * @param[in] dev Xbar device handle to initialize
 */

System_Errors Xbar_init(Xbar_DeviceHandle dev);

/**
 * This function route the  XBAR in input to XBAR out output
 *
 * @param[in] dev Xbar device handle to initialize
 * @param[in] in channel input selected
 * @param[out] out channel out selected
 */


System_Errors Xbar_ruteInToOut(Xbar_DeviceHandle dev, Xbar_Input in, Xbar_Output out);


#endif
#endif /* LIBOHIBOARD_XBAR */
