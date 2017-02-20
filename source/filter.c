/*
 * filter.c
 *
 *  Created on: 20/feb/2017
 *      Author: develop
 */

#include "filter.h"

void MovAvgInit(MovAvgFilterHandle dev, uint8_t len)
{

	dev->buffer = (float*)malloc(dev->length * sizeof(float));
	dev->lastSample = dev->buffer;

	dev->transient = 1;

}

void MovAvgReset(MovAvgFilterHandle dev)
{
	dev->transient = 1;
	dev->lastSample = dev->buffer;
	memset(dev->buffer,0,dev->length);
}

void MovAvgRun(MovAvgFilterHandle dev)
{
	*(dev->lastSample) = dev->in;

	float sum = 0;

	if(dev->transient == 0)
	{
		for(int i=0;i<dev->length;i++)
		{
			sum += dev->buffer[i];
		}
		dev->out = sum / (float)dev->length;
	}
	else
	{
		for(int i=0;i<(dev->lastSample-dev->buffer);i++)
		{
			sum += dev->buffer[i];
		}

		//serve??
		if((dev->lastSample - dev->buffer) != 0 )
		{
			dev->out = sum / (float)(dev->lastSample - dev->buffer);
		}
		else
		{
			dev->out = dev->in;
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
