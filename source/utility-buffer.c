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
 * @file libohiboard/source/utility-buffer.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful buffer functions inplementations.
 */

#include "utility-buffer.h"

System_Errors UtilityBuffer_init (UtilityBuffer_DescriptorHandle b, 
                                  uint8_t* buffer, 
                                  uint16_t size)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check the size
    err = ohiassert(size>0);
    // Size must be a 2-power
    err |= ohiassert(!(size & (size-1)));

    if (err != ERRORS_NO_ERROR) return err;

    // Clear buffer
    memset((void *) buffer, 0, size);

    // clear index
    b->tail = 0;
    b->head = 0;

    // Save the buffer size and mask
    b->size = size;
    b->mask = size - 1;

    // Save the buffer pointer.
    b->buffer = buffer;

    return err;
}
