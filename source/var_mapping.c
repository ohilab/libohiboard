/* Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/include/var_mapping.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief memory location  for modbus protocol.
 */


#include "libohiboard.h"
#include "var_mapping.h"




uint16_t A0;
uint16_t A1;
uint16_t A2;
uint16_t A3;
uint16_t A4;
uint16_t A5;
uint16_t A6;
uint16_t A7;
uint16_t A8;
uint16_t A9;
uint16_t A10;

uint8_t STATUS;


uint16_t *Map[MEM_MAP_LEN]={&A0,&A1,&A2,&A3,&A4,&A5,&A6,&A7,&A8,&A9,&A10};

