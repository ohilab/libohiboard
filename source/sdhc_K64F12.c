/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/sdhc_K64F12.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SDHC implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_SDHC

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "platforms.h"
#include "sdhc.h"

#define SDHC_MAX_PINS 1

typedef struct Sdhc_Device
{
    SDHC_Type* regMap;                         /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    volatile uint32_t* simSoptPtr;

    Sdhc_ClockPins clockPins[SDHC_MAX_PINS];
    Sdhc_CmdPins cmdPins[SDHC_MAX_PINS];
    Sdhc_D0Pins d0Pins[SDHC_MAX_PINS];
    Sdhc_D0Pins d1Pins[SDHC_MAX_PINS];
    Sdhc_D1Pins d2Pins[SDHC_MAX_PINS];
    Sdhc_D2Pins d3Pins[SDHC_MAX_PINS];
    Sdhc_D3Pins d4Pins[SDHC_MAX_PINS];
    Sdhc_D4Pins d5Pins[SDHC_MAX_PINS];
    Sdhc_D5Pins d6Pins[SDHC_MAX_PINS];
    Sdhc_D6Pins d7Pins[SDHC_MAX_PINS];
    Sdhc_ClockInPins clkinPins[SDHC_MAX_PINS];

    volatile uint32_t* clockPinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* cmdPinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d0PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d1PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d2PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d3PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d4PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d5PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d6PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* d7PinsPtr[SDHC_MAX_PINS];
    volatile uint32_t* clkinPinsPtr[SDHC_MAX_PINS];
    uint8_t clockPinsMux[SDHC_MAX_PINS];
    uint8_t cmdPinsMux[SDHC_MAX_PINS];
    uint8_t d0PinsMux[SDHC_MAX_PINS];
    uint8_t d1PinsMux[SDHC_MAX_PINS];
    uint8_t d2PinsMux[SDHC_MAX_PINS];
    uint8_t d3PinsMux[SDHC_MAX_PINS];
    uint8_t d4PinsMux[SDHC_MAX_PINS];
    uint8_t d5PinsMux[SDHC_MAX_PINS];
    uint8_t d6PinsMux[SDHC_MAX_PINS];
    uint8_t d7PinsMux[SDHC_MAX_PINS];
    uint8_t clkinPinsMux[SDHC_MAX_PINS];

    void (*isr)(void);                     /**< The function pointer for ISR. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */
    void (*cardRemoved)(void);
    void (*cardInserted)(void);

} Sdhc_Device;

static Sdhc_Device sdhc0 = {
    .regMap           = SDHC_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC3,
    .simScgcBitEnable = SIM_SCGC3_SDHC_MASK,

    .simSoptPtr       = &SIM_SOPT2,

    .clockPins        = {SDHC_PINS_PTE2},
    .cmdPins          = {SDHC_PINS_PTE3},
    .clkinPins        = {SDHC_PINS_PTD11},
    .d0Pins           = {SDHC_PINS_PTE1},
    .d1Pins           = {SDHC_PINS_PTE0},
    .d2Pins           = {SDHC_PINS_PTE5},
    .d3Pins           = {SDHC_PINS_PTE4},
    .d4Pins           = {SDHC_PINS_PTD12},
    .d5Pins           = {SDHC_PINS_PTD13},
    .d6Pins           = {SDHC_PINS_PTD14},
    .d7Pins           = {SDHC_PINS_PTD15},

    .clockPinsPtr     = {&PORTE_PCR2},
    .cmdPinsPtr       = {&PORTE_PCR3},
    .clkinPinsPtr     = {&PORTD_PCR11},
    .d0PinsPtr        = {&PORTE_PCR1},
    .d1PinsPtr        = {&PORTE_PCR0},
    .d2PinsPtr        = {&PORTE_PCR5},
    .d3PinsPtr        = {&PORTE_PCR4},
    .d4PinsPtr        = {&PORTD_PCR12},
    .d5PinsPtr        = {&PORTD_PCR13},
    .d6PinsPtr        = {&PORTD_PCR14},
    .d7PinsPtr        = {&PORTD_PCR15},

    .clockPinsMux     = {4},
    .cmdPinsMux       = {4},
    .clkinPinsMux     = {4},
    .d0PinsMux        = {4},
    .d1PinsMux        = {4},
    .d2PinsMux        = {4},
    .d3PinsMux        = {4},
    .d4PinsMux        = {4},
    .d5PinsMux        = {4},
    .d6PinsMux        = {4},
    .d7PinsMux        = {4},

    .isr              = SDHC_IRQHandler,
    .isrNumber        = INTERRUPT_SDHC,
    .cardRemoved      = 0,
    .cardInserted     = 0,

};
Flash_DeviceHandle OB_SDHC0 = &sdhc0;

System_Errors Sdhc_init (Sdhc_DeviceHandle dev)
{


    return ERRORS_NO_ERROR;
}

System_Errors Sdhc_deinit (Sdhc_DeviceHandle dev)
{

}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_SDHC */
