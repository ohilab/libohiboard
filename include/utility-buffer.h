/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 */

/**
 * @file libohiboard/include/utility-buffer.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful buffer functions and definitions.
 */

#ifndef __UTILITY_BUFFER_H
#define __UTILITY_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

#include "platforms.h"
#include "errors.h"
#include "types.h"

/**
 * 
 * 
 */
typedef struct _UtilityBuffer_Descriptor 
{
    uint8_t* buffer;        //!< Pointer to unsigned-8 bits location

    volatile uint16_t head; //!< The index to the next empty location
    volatile uint16_t tail; //!< The index to the next readable location

    uint16_t size;          //!< Size of the buffer
    uint16_t mask;          //!< Mask used to manage circular buffer
} UtilityBuffer_Descriptor, *UtilityBuffer_DescriptorHandle;

/**
 * 
 * 
 */
System_Errors UtilityBuffer_init (UtilityBuffer_DescriptorHandle b, uint8_t* buffer, uint16_t size);

/**
 * This function return the number of free locations into the buffer.
 * 
 * @param[in] buffer:
 * @return The number of free location into the buffer. 
 */
static inline uint16_t UtilityBuffer_getFreeSize (UtilityBuffer_DescriptorHandle buffer)
{
    return buffer->size - ((buffer->tail - buffer->head) & buffer->mask);
}

/**
 * This function return the number of used locations into the buffer.
 * 
 * @param[in] buffer:
 * @return The number of used location into the buffer. 
 */
static inline uint16_t UtilityBuffer_getUsedSize (UtilityBuffer_DescriptorHandle buffer)
{
    return ((buffer->head - buffer->tail) & buffer->mask);
}

/**
 * 
 * @return The buffer status
 *   @retval TRUE when the buffer is empty.
 *   @retval FALSE when the buffer is not empty. 
 */
static inline bool UtilityBuffer_isEmpty (UtilityBuffer_DescriptorHandle buffer)
{
    return (buffer->tail == buffer->head) ? TRUE : FALSE;
}

/**
 * 
 * @return The buffer status
 *   @retval TRUE when the buffer is full.
 *   @retval FALSE when the buffer is not full. 
 */
static inline bool UtilityBuffer_isFull (UtilityBuffer_DescriptorHandle buffer)
{
    return (UtilityBuffer_getUsedSize(buffer) == (buffer->size - 1)) ? TRUE : FALSE;
}

/**
 * Flushes the buffer.
 *
 * @param[in] fifo_desc  The FIFO descriptor.
 */
static inline void UtilityBuffer_flush (UtilityBuffer_DescriptorHandle buffer)
{
    buffer->tail = buffer->head = 0;
}

/**
 * 
 * @param[in] buffer:
 * @param[out] value:
 * 
 * @return The result of the operation
 *   @retval ERRORS_UTILITYBUFFER_UNDERFLOW when the buffer is empty
 *   @retval ERRORS_NO_ERROR when the pull works
 */
static inline System_Errors UtilityBuffer_pull (UtilityBuffer_DescriptorHandle b, uint8_t* value)
{
    if (UtilityBuffer_isEmpty(b)) 
    {
        return ERRORS_UTILITYBUFFER_UNDERFLOW;
    }

    // Read the next value
    *value = b->buffer[b->tail];

    // Increment the tail index, and check with the mask
    b->tail++;
    b->tail &= b->mask;

    return ERRORS_NO_ERROR;
}

/**
 * 
 * @param[in] buffer:
 * @param[in]  value:
 * 
 * @return The result of the operation
 *   @retval ERRORS_UTILITYBUFFER_OVERFLOW when the buffer is full
 *   @retval ERRORS_NO_ERROR when the push works
 */
static inline System_Errors UtilityBuffer_push (UtilityBuffer_DescriptorHandle b, uint8_t value)
{
    if (UtilityBuffer_isFull(b)) 
    {
        return ERRORS_UTILITYBUFFER_OVERFLOW;
    }

    // Add new value into the buffer...
    b->buffer[b->head] = value;

    // Increment the head index, and check with the mask
    b->head++;
    b->head &= b->mask;

    return ERRORS_NO_ERROR;
}

#ifdef __cplusplus
}
#endif

#endif // __UTILITY_BUFFER_H
