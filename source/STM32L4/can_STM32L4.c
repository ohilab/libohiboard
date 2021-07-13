/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L4/can_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief CAN implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_CAN

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "can.h"


typedef struct _Can_Device
{
    CAN_TypeDef* regmap;                           /**< Device memory pointer */

//    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
//    uint32_t rccRegisterEnable;         /**< Register mask for current device */
//
//    volatile uint32_t* rccTypeRegisterPtr;   /**< Register for clock enabling */
//    uint32_t rccTypeRegisterMask;       /**< Register mask for user selection */
//    uint32_t rccTypeRegisterPos;        /**< Mask position for user selection */

//    Adc_Pins pins[ADC_MAX_PINS];      /**< List of the pin for the peripheral */
//    Adc_Channels pinsChannel[ADC_MAX_PINS];
//    Gpio_Pins pinsGpio[ADC_MAX_PINS];

//    void (* eocCallback)(struct _Adc_Device *dev);
//    void (* eosCallback)(struct _Adc_Device *dev);
//    void (* overrunCallback)(struct _Adc_Device *dev);

    Interrupt_Vector isrTxNumber;                   /**< ISR TX vector number */
    Interrupt_Vector isrRx0Number;                 /**< ISR RX0 vector number */
    Interrupt_Vector isrRx1Number;                 /**< ISR RX1 vector number */
    Interrupt_Vector isrSceNumber;                 /**< ISR SCE vector number */

    Can_DeviceState state;                      /**< Current peripheral state */

    Can_Config config;                                /**< User configuration */

} Can_Device;

#define CAN_IS_DEVICE(DEVICE) (((DEVICE) == OB_CAN1))

static Can_Device can1 =
{
    .regmap              = CAN1,

//    .rccRegisterPtr      = &RCC->AHB2ENR,
//    .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,
//
//    .rccTypeRegisterPtr  = &RCC->CCIPR,
//    .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
//    .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

    .isrTxNumber         = INTERRUPT_CAN1_TX,
    .isrRx0Number        = INTERRUPT_CAN1_RX0,
    .isrRx1Number        = INTERRUPT_CAN1_RX1,
    .isrSceNumber        = INTERRUPT_CAN1_SCE,

    .state               = CAN_DEVICESTATE_RESET,
};
Can_DeviceHandle OB_CAN1 = &can1;


System_Errors Can_init (Can_DeviceHandle dev, Can_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }
}

System_Errors Can_deInit (Can_DeviceHandle dev)
{

}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CAN
