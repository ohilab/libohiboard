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

#ifndef VAR_MAPPING_H_
#define VAR_MAPPING_H_
#include "libohiboard.h"

#define MEM_MAP_LEN 11

#define MAX_MAP_ADDRESS  MEM_MAP_LEN
#define MIN_MAP_ADDRESS 	0

//define Analog Variable Here

#define STATUS Status_Macchine_Register

#define A0  RED    //Red
#define A1  GREEN  //Green
#define A2  BLUE   //Blue
#define A3  A3
#define A4 	A4
#define A5  A5
#define A6  A6
#define A7  A7
#define A8  A8
#define A9  A9
#define A10 A10




extern uint16_t *Map[MEM_MAP_LEN];
extern uint8_t STATUS;
extern uint16_t A0;
extern uint16_t A1;
extern uint16_t A2;
extern uint16_t A3;
extern uint16_t A4;
extern uint16_t A5;
extern uint16_t A6;
extern uint16_t A7;
extern uint16_t A8;
extern uint16_t A9;
extern uint16_t A10;
//---------------------------Insert here additional Var-----------------------------------------------
//Note:Remember to modify the new Map dimension so MEM_MAP_LEN

//----------------------------------------------------------------------------------------------------



#endif /* VAR_MAPPING_H_ */
