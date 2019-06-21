/*
 * This file is part of the libohiboard project.
 * 
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/STM32L4/critical_STM32L4.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief CRITICAL Function for implementing Critical Section on STM32L4
 */

#if defined(LIBOHIBOARD_CRITICAL)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "critical.h"
#include "utility.h"

#if defined (LIBOHIBOARD_STM32L4)


inline void Critical_sectionBegin( uint32_t *mask )
{
    *mask = __get_BASEPRI( );  __set_BASEPRI( CRITICAL_DEFAULT_TICK_PRIO + 1 );
    //*mask = __get_PRIMASK( ); __disable_irq( );
}

// from [1], section 2.1.3: Core Register
// Base Priority Mask Register
//The BASEPRI register defines the minimum priority for exception processing.
// When BASEPRI is set to a nonzero value, it prevents the activation of all exceptions
// with the same or lower priority level as the BASEPRI.
//    Priority mask bits:
//          0x00 = no effect
//          Nonzero = defines the base priority for exception processing.
//                The processor does not process any exception with a priority
//                value greater than or equal to BASEPRI
// // ... Remember that higher priority field values correspond to lower exception priorities.
//
// [1]:: http://infocenter.arm.com/help/topic/com.arm.doc.dui0553b/DUI0553.pdf
// [2]:: https://www.keil.com/pack/doc/cmsis/Core/html/index.html#ref_man_sec
//

// lascio vivo il valore zero di priorita'
//    *mask = __get_BASEPRI( );  __set_BASEPRI( 0x00000001 );
// __set_BASEPRI( *mask );


inline void Critical_sectionEnd( uint32_t *mask )
{
    __set_BASEPRI( *mask );
    //__set_PRIMASK( *mask );
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CRITICAL
