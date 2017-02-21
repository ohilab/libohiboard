/*
 * filter.h
 *
 *  Created on: 20/feb/2017
 *      Author: develop
 */

#ifndef SOURCES_LIBOHIBOARD_INCLUDE_FILTER_H_
#define SOURCES_LIBOHIBOARD_INCLUDE_FILTER_H_

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





#endif /* SOURCES_LIBOHIBOARD_INCLUDE_FILTER_H_ */
