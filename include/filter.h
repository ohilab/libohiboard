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

typedef struct {

	float* buffer;
	uint8_t length;
	float in;
	float out;

	float* lastSample;

	uint8_t transient;

}MovAvgFilter_t;

typedef MovAvgFilter_t* MovAvgFilterHandle;

void MovAvgInit(MovAvgFilterHandle dev, uint8_t len);
void MovAvgDispose(MovAvgFilterHandle dev);
void MovAvgReset(MovAvgFilterHandle dev);
void MovAvgRun(MovAvgFilterHandle dev);



#endif /* SOURCES_LIBOHIBOARD_INCLUDE_FILTER_H_ */
