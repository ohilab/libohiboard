/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/hardware/STM32WB/gpio_STM32WB.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO pins and device definitions for STM32WB series
 */

#ifndef __GPIO_STM32WB_H
#define __GPIO_STM32WB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h" //File da modificare (va aggiunto STM32WB)

#if defined(LIBOHIBOARD_STM32L4) //Si trova all'interno di platform.h e andrà aggiunto LIBOHIBOARD_STM32WB
								 //Se definisco il processore STM32L4 (LIBOHIBOARD_STM32WB)
typedef enum
{
    GPIO_PINS_NONE = 0, //Se non definisco nessun processore non ho definito nessun pin

#if defined (LIBOHIBOARD_STM32L476) //Si trova all'interno di platform.h e andrà aggiunto LIBOHIBOARD_STM32WB
									//Se definisco il processore STM32L476 (LIBOHIBOARD_STM32WB55)
// WLCSP72 ballout
// LQFP64 (VFQFPN68)
#if defined (LIBOHIBOARD_STM32L476Rx) //|| \
	defined (LIBOHIBOARD_STM32L476Jx) (LIBOHIBOARD_STM32WB55Rx)

    GPIO_PINS_PA0,
    GPIO_PINS_PA1,
    GPIO_PINS_PA2,
    GPIO_PINS_PA3,
    GPIO_PINS_PA4,
    GPIO_PINS_PA5,
    GPIO_PINS_PA6,
    GPIO_PINS_PA7,
    GPIO_PINS_PA8,
    GPIO_PINS_PA9,
    GPIO_PINS_PA10,
    GPIO_PINS_PA11,
    GPIO_PINS_PA12,
    GPIO_PINS_PA13,
    GPIO_PINS_PA14,
    GPIO_PINS_PA15,

    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
    GPIO_PINS_PB2,
    GPIO_PINS_PB3,
    GPIO_PINS_PB4,
    GPIO_PINS_PB5,
    GPIO_PINS_PB6,
    GPIO_PINS_PB7,
    GPIO_PINS_PB8,
    GPIO_PINS_PB9,
    GPIO_PINS_PB10,
    GPIO_PINS_PB11,
    GPIO_PINS_PB12,
    GPIO_PINS_PB13,
    GPIO_PINS_PB14,
    GPIO_PINS_PB15,

    GPIO_PINS_PC0,
    GPIO_PINS_PC1,
    GPIO_PINS_PC2,
    GPIO_PINS_PC3,
    GPIO_PINS_PC4,
    GPIO_PINS_PC5,
    GPIO_PINS_PC6,
	//GPIO_PINS_PC7, Non ci sono sull'STM32WB55Rx
	//GPIO_PINS_PC8,
	//GPIO_PINS_PC9,
    GPIO_PINS_PC10,
    GPIO_PINS_PC11,
    GPIO_PINS_PC12,
    GPIO_PINS_PC13,
    GPIO_PINS_PC14, //OSC32_IN
    GPIO_PINS_PC15, //OSC32_OUT

    GPIO_PINS_PD0, //Aggiunto PD0 e PD1 e rimosso PD2
	GPIO_PINS_PD1,
	//GPIO_PINS_PD2,

//#if defined (LIBOHIBOARD_STM32L476Jx)
    //GPIO_PINS_PG9,
    //GPIO_PINS_PG10,
    //GPIO_PINS_PG11,
    //GPIO_PINS_PG12,
    //GPIO_PINS_PG13,
    //GPIO_PINS_PG14,
//#endif

    //GPIO_PINS_PH0,
	//GPIO_PINS_PH1,
	GPIO_PINS_PH3, //Aggiunto PH3 e rimosso PH0 e PH1

#endif

#endif // LIBOHIBOARD_STM32L476

} Gpio_Pins; //I gpio_pins possono assumere tutti i volaori sopra descritti

/**
 * List of possible GPIO ports.
 */
typedef enum
{
    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,
    GPIO_PORTS_F,
    GPIO_PORTS_G,
    GPIO_PORTS_H,

	GPIO_PORTS_NUMBER,
} Gpio_Ports;

#define GPIO_MAX_PINS_NUMBER_FOR_PORT	16
#define GPIO_MAX_PORTS_NUMBER			GPIO_PORTS_NUMBER

#define GPIO_PORTS_BASE               ((GPIO_TypeDef *) GPIOA_BASE) //Dove si trova GPIOA_BASE

#endif // LIBOHIBOARD_STM32L4 (STM32WB)

#ifdef __cplusplus
}
#endif

#endif // __GPIO_STM32L4_H (STM32WB_H)
