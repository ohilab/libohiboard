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


#ifdef LIBOHIBOARD_FILTER

#include "filter.h"

void PtbyPtFilterInit(PtbyPtFilterHandle dev, uint8_t len, PtbyPtFilterMode mode)
{

	dev->length = len;
	free(dev->buffer);
	dev->buffer = (float*)malloc(dev->length * sizeof(float));
	dev->lastSample = dev->buffer;

	dev->transient = 1;

	dev->mode = mode;

}

void PtbyPtFilterDispose(PtbyPtFilterHandle dev)
{
	dev->lastSample = dev->buffer;
	free(dev->buffer);
	dev->transient = 0;
}

void PtbyPtFilterAddSample(PtbyPtFilterHandle dev, float sample)
{
	dev->in = sample;
}

void PtbyPtFilterGetOut(PtbyPtFilterHandle dev, float* out)
{
	*out = dev->out;
}

void PtbyPtFilterReset(PtbyPtFilterHandle dev)
{
	dev->transient = 1;
	dev->lastSample = dev->buffer;
	memset(dev->buffer,0,dev->length);
}

void PtbyPtFilterRun(PtbyPtFilterHandle dev)
{
	*(dev->lastSample) = dev->in;

	float sum = 0;

	if(dev->transient == 0)
	{
		/* Sum */
		for(int i=0;i<dev->length;i++)
		{
			switch(dev->mode)
			{
			case _MOVING_AVERAGE:
				sum += dev->buffer[i];
				break;
			case _RMS:
				sum += (dev->buffer[i]*dev->buffer[i]);
				break;
			}
		}

		/* Averaging */
		switch(dev->mode)
		{
		case _MOVING_AVERAGE:
			dev->out = sum / (float)dev->length;
			break;
		case _RMS:
			/* TODO: valutare se � troppo pesante... in caso calcolare la radice solo al get dell'output */
			dev->out = sqrt(sum / (float)dev->length);
			break;
		}

	}
	else	/* If buffer is not full */
	{
		if((dev->lastSample - dev->buffer) != 0 )	/* More than one sample */
		{
			for(int i=0;i<(dev->lastSample-dev->buffer);i++)
			{
				switch(dev->mode)
				{
				case _MOVING_AVERAGE:
					sum += dev->buffer[i];
					break;
				case _RMS:
					sum += (dev->buffer[i]*dev->buffer[i]);
					break;
				}
			}

			/* Averaging */
			switch(dev->mode)
			{
			case _MOVING_AVERAGE:

				break;
			case _RMS:
				/* TODO: valutare se � troppo pesante... in caso calcolare la radice solo al get dell'output */
				dev->out = sqrt(sum / (float)(dev->lastSample - dev->buffer));
				break;
			}

		}
		else /* If there is only one sample */
		{
			switch(dev->mode)
			{
			case _MOVING_AVERAGE:
				dev->out = dev->in;
				break;
			case _RMS:
				dev->out = abs(dev->in);
				break;
			}
		}
	}

	if(dev->lastSample == &(dev->buffer[dev->length-1]))
	{
		dev->lastSample = dev->buffer;
		dev->transient = 0;
	}
	else
	{
		dev->lastSample++;
	}

}

#endif
