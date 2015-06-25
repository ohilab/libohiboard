/*
 * var_mapping.c
 *
 *  Created on: 13/mag/2015
 *      Author: Matt
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

