/*
 * var_mapping.h
 *
 *  Created on: 13/mag/2015
 *      Author: Matt
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
extern uint8_t STATUS;


#endif /* VAR_MAPPING_H_ */
