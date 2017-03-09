/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Pirro
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

#ifndef __FILTER_H
#define __FILTER_H

#include "types.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef enum
{
	_MOVING_AVERAGE,
	_RMS,
}PtbyPtFilterMode;

typedef struct {

	float* buffer;
	uint8_t length;
	float in;
	float out;

	float* lastSample;

	uint8_t transient;

	PtbyPtFilterMode mode;

}PtbyPtFilter_t;

typedef PtbyPtFilter_t* PtbyPtFilterHandle;

void PtbyPtFilterInit(PtbyPtFilterHandle dev, uint8_t len, PtbyPtFilterMode mode);
void PtbyPtFilterDispose(PtbyPtFilterHandle dev);
void PtbyPtFilterReset(PtbyPtFilterHandle dev);
void PtbyPtFilterRun(PtbyPtFilterHandle dev);
void PtbyPtFilterAddSample(PtbyPtFilterHandle dev, float sample);
void PtbyPtFilterGetOut(PtbyPtFilterHandle dev, float* out);

#endif /* __FILTER_H */
