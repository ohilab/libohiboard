/*
** ###################################################################
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    KV4XP100M168RM Rev 3, 08/2015
**     Version:             rev. 1.3, 2015-07-29
**     Build:               b150911
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MKV46F16
**
**     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 0.1 (2014-01-27)
**         Initial version.
**     - rev. 1.0 (2014-07-29)
**         FPU initialization and VLLSx wake-up recovery added to the system initialization
**     - rev. 1.1 (2014-10-14)
**         Renamed interrupt vector I2C to I2C0 and LPTimer to LPTMR0
**     - rev. 1.2 (2015-02-11)
**         AIPS registers have been added.
**     - rev. 1.3 (2015-07-29)
**         Correction of backward compatibility.
**
** ###################################################################
*/

/*!
 * @file MKV46F16.h
 * @version 1.3
 * @date 2015-07-29
 * @brief CMSIS Peripheral Access Layer for MKV46F16
 *
 * CMSIS Peripheral Access Layer for MKV46F16
 */


/* ----------------------------------------------------------------------------
   -- MCU activation
   ---------------------------------------------------------------------------- */

/* Prevention from multiple including the same memory map */
#if !defined(MKV46F16_H_)  /* Check if memory map has not been already included */
#define MKV46F16_H_
#define MCU_MKV46F16

/* Check if another memory map has not been also included */
#if (defined(MCU_ACTIVE))
  #error MKV46F16 memory map: There is already included another memory map. Only one memory map can be included.
#endif /* (defined(MCU_ACTIVE)) */
#define MCU_ACTIVE

#include <stdint.h>

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0100u
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0003u

/**
 * @brief Macro to calculate address of an aliased word in the peripheral
 *        bitband area for a peripheral register and bit (bit band region 0x40000000 to
 *        0x400FFFFF).
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return  Address of the aliased word in the peripheral bitband area.
 */
#define BITBAND_REGADDR(Reg,Bit) (0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 32bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG32(Reg,Bit) (*((uint32_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
#define BITBAND_REG(Reg,Bit) (BITBAND_REG32(Reg,Bit))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 16bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG16(Reg,Bit) (*((uint16_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 8bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG8(Reg,Bit) (*((uint8_t volatile*)(BITBAND_REGADDR(Reg,Bit))))

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 116                /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_IRQn                    = 0,                /**< DMA channel 0, 16 transfer complete */
  DMA1_IRQn                    = 1,                /**< DMA channel 1, 17 transfer complete */
  DMA2_IRQn                    = 2,                /**< DMA channel 2, 18 transfer complete */
  DMA3_IRQn                    = 3,                /**< DMA channel 3, 19 transfer complete */
  DMA4_IRQn                    = 4,                /**< DMA channel 4, 20 transfer complete */
  DMA5_IRQn                    = 5,                /**< DMA channel 5, 21 transfer complete */
  DMA6_IRQn                    = 6,                /**< DMA channel 6, 22 transfer complete */
  DMA7_IRQn                    = 7,                /**< DMA channel 7, 23 transfer complete */
  DMA8_IRQn                    = 8,                /**< DMA channel 8, 24 transfer complete */
  DMA9_IRQn                    = 9,                /**< DMA channel 9, 25 transfer complete */
  DMA10_IRQn                   = 10,               /**< DMA channel 10, 26 transfer complete */
  DMA11_IRQn                   = 11,               /**< DMA channel 11, 27 transfer complete */
  DMA12_IRQn                   = 12,               /**< DMA channel 12, 28 transfer complete */
  DMA13_IRQn                   = 13,               /**< DMA channel 13, 29 transfer complete */
  DMA14_IRQn                   = 14,               /**< DMA channel 14, 30 transfer complete */
  DMA15_IRQn                   = 15,               /**< DMA channel 15, 31 transfer complete */
  DMA_Error_IRQn               = 16,               /**< DMA error interrupt channels 0-1531 */
  MCM_IRQn                     = 17,               /**< MCM interrupt */
  FTFA_IRQn                    = 18,               /**< Command complete */
  FTFA_Collision_IRQn          = 19,               /**< Read collision */
  PMC_IRQn                     = 20,               /**< Low-voltage detect, low-voltage warning */
  LLWU_IRQn                    = 21,               /**< Low Leakage Wakeup */
  WDOG_EWM_IRQn                = 22,               /**< Both watchdog modules share this interrupt */
  Reserved39_IRQn              = 23,               /**< Reserved interrupt */
  I2C0_IRQn                    = 24,               /**< I2C0 */
  Reserved41_IRQn              = 25,               /**< Reserved interrupt */
  SPI0_IRQn                    = 26,               /**< SPI0 */
  Reserved43_IRQn              = 27,               /**< Reserved interrupt */
  Reserved44_IRQn              = 28,               /**< Reserved interrupt */
  Reserved45_IRQn              = 29,               /**< Reserved interrupt */
  Reserved46_IRQn              = 30,               /**< Reserved interrupt */
  UART0_RX_TX_IRQn             = 31,               /**< UART0 status sources */
  UART0_ERR_IRQn               = 32,               /**< UART0 error sources */
  UART1_RX_TX_IRQn             = 33,               /**< UART1 status sources */
  UART1_ERR_IRQn               = 34,               /**< UART1 error sources */
  Reserved51_IRQn              = 35,               /**< Reserved interrupt */
  Reserved52_IRQn              = 36,               /**< Reserved interrupt */
  Reserved53_IRQn              = 37,               /**< Reserved interrupt */
  ADC_ERR_IRQn                 = 38,               /**< ADC_ERR A and B ( zero cross, high/low limit) */
  ADCA_IRQn                    = 39,               /**< ADCA Scan complete */
  CMP0_IRQn                    = 40,               /**< CMP0 */
  CMP1_IRQn                    = 41,               /**< CMP1 */
  FTM0_IRQn                    = 42,               /**< FTM0 8 channels */
  FTM1_IRQn                    = 43,               /**< FTM1 2 channels */
  Reserved60_IRQn              = 44,               /**< Reserved interrupt */
  Reserved61_IRQn              = 45,               /**< Reserved interrupt */
  Reserved62_IRQn              = 46,               /**< Reserved interrupt */
  Reserved63_IRQn              = 47,               /**< Reserved interrupt */
  PIT0_IRQn                    = 48,               /**< PIT Channel 0 */
  PIT1_IRQn                    = 49,               /**< PIT Channel 1 */
  PIT2_IRQn                    = 50,               /**< PIT Channel 2 */
  PIT3_IRQn                    = 51,               /**< PIT Channel 3 */
  PDB0_IRQn                    = 52,               /**< PDB0 */
  Reserved69_IRQn              = 53,               /**< Reserved interrupt */
  XBARA_IRQn                   = 54,               /**< XBARA */
  PDB1_IRQn                    = 55,               /**< PDB1 */
  DAC0_IRQn                    = 56,               /**< DAC0 */
  MCG_IRQn                     = 57,               /**< MCG */
  LPTMR0_IRQn                  = 58,               /**< LPTMR0 */
  PORTA_IRQn                   = 59,               /**< Pin detect (Port A) */
  PORTB_IRQn                   = 60,               /**< Pin detect (Port B) */
  PORTC_IRQn                   = 61,               /**< Pin detect (Port C) */
  PORTD_IRQn                   = 62,               /**< Pin detect (Port D) */
  PORTE_IRQn                   = 63,               /**< Pin detect (Port E) */
  SWI_IRQn                     = 64,               /**< Software */
  Reserved81_IRQn              = 65,               /**< Reserved interrupt */
  ENC0_COMPARE_IRQn            = 66,               /**< ENC0 Compare */
  ENC0_HOME_IRQn               = 67,               /**< ENC0 Home */
  ENC0_WDOG_SAB_IRQn           = 68,               /**< ENC0 Watchdog/Simultaneous A and B change */
  ENC0_INDEX_IRQn              = 69,               /**< ENC0 Index/Roll over/Roll Under */
  CMP2_IRQn                    = 70,               /**< CMP2 */
  FTM3_IRQn                    = 71,               /**< FTM3 8 channels */
  Reserved88_IRQn              = 72,               /**< Reserved interrupt */
  ADCB_IRQn                    = 73,               /**< ADCB Scan complete */
  Reserved90_IRQn              = 74,               /**< Reserved interrupt */
  CAN0_ORed_Message_buffer_IRQn = 75,              /**< FLexCAN0 OR'ed Message buffer (0-15) */
  CAN0_Bus_Off_IRQn            = 76,               /**< FLexCAN0 Bus Off */
  CAN0_Error_IRQn              = 77,               /**< FLexCAN0 Error */
  CAN0_Tx_Warning_IRQn         = 78,               /**< FLexCAN0 Transmit Warning */
  CAN0_Rx_Warning_IRQn         = 79,               /**< FLexCAN0 Receive Warning */
  CAN0_Wake_Up_IRQn            = 80,               /**< FLexCAN0 Wake Up */
  PWMA_CMP0_IRQn               = 81,               /**< eFlexPWM submodule 0 Compare */
  PWMA_RELOAD0_IRQn            = 82,               /**< eFlexPWM submodule 0 Reload */
  PWMA_CMP1_IRQn               = 83,               /**< eFlexPWM submodule 1 Compare */
  PWMA_RELOAD1_IRQn            = 84,               /**< eFlexPWM submodule 1 Reload */
  PWMA_CMP2_IRQn               = 85,               /**< eFlexPWM submodule 2 Compare */
  PWMA_RELOAD2_IRQn            = 86,               /**< eFlexPWM submodule 2 Reload */
  PWMA_CMP3_IRQn               = 87,               /**< eFlexPWM submodule 3 Compare */
  PWMA_RELOAD3_IRQn            = 88,               /**< eFlexPWM submodule 3 Reload */
  PWMA_CAP_IRQn                = 89,               /**< eFlexPWM all input captures */
  PWMA_RERR_IRQn               = 90,               /**< eFlexPWM reload error */
  PWMA_FAULT_IRQn              = 91,               /**< eFlexPWM Fault */
  CMP3_IRQn                    = 92,               /**< CMP3 */
  Reserved109_IRQn             = 93,               /**< Reserved interrupt */
  CAN1_ORed_Message_buffer_IRQn = 94,              /**< FLexCAN1 OR'ed Message buffer (0-15) */
  CAN1_Bus_Off_IRQn            = 95,               /**< FLexCAN1 Bus Off */
  CAN1_Error_IRQn              = 96,               /**< FLexCAN1 Error */
  CAN1_Tx_Warning_IRQn         = 97,               /**< FLexCAN1 Transmit Warning */
  CAN1_Rx_Warning_IRQn         = 98,               /**< FLexCAN1 Receive Warning */
  CAN1_Wake_Up_IRQn            = 99                /**< FLexCAN1 Wake Up */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M4 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M4 Core Configuration
 * @{
 */

#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __NVIC_PRIO_BITS               4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT                  1         /**< Defines if an FPU is present or not */

#include "core_cm4.h"                  /* Core Peripheral Access Layer */
#include "system_MKV46F16.h"           /* Device specific configuration file */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */


/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL1;                             /**< ADC Control Register 1, offset: 0x0 */
  __IO uint16_t CTRL2;                             /**< ADC Control Register 2, offset: 0x2 */
  __IO uint16_t ZXCTRL1;                           /**< ADC Zero Crossing Control 1 Register, offset: 0x4 */
  __IO uint16_t ZXCTRL2;                           /**< ADC Zero Crossing Control 2 Register, offset: 0x6 */
  __IO uint16_t CLIST1;                            /**< ADC Channel List Register 1, offset: 0x8 */
  __IO uint16_t CLIST2;                            /**< ADC Channel List Register 2, offset: 0xA */
  __IO uint16_t CLIST3;                            /**< ADC Channel List Register 3, offset: 0xC */
  __IO uint16_t CLIST4;                            /**< ADC Channel List Register 4, offset: 0xE */
  __IO uint16_t SDIS;                              /**< ADC Sample Disable Register, offset: 0x10 */
  __IO uint16_t STAT;                              /**< ADC Status Register, offset: 0x12 */
  __I  uint16_t RDY;                               /**< ADC Ready Register, offset: 0x14 */
  __IO uint16_t LOLIMSTAT;                         /**< ADC Low Limit Status Register, offset: 0x16 */
  __IO uint16_t HILIMSTAT;                         /**< ADC High Limit Status Register, offset: 0x18 */
  __IO uint16_t ZXSTAT;                            /**< ADC Zero Crossing Status Register, offset: 0x1A */
  __IO uint16_t RSLT[16];                          /**< ADC Result Registers with sign extension, array offset: 0x1C, array step: 0x2 */
  __IO uint16_t LOLIM[16];                         /**< ADC Low Limit Registers, array offset: 0x3C, array step: 0x2 */
  __IO uint16_t HILIM[16];                         /**< ADC High Limit Registers, array offset: 0x5C, array step: 0x2 */
  __IO uint16_t OFFST[16];                         /**< ADC Offset Registers, array offset: 0x7C, array step: 0x2 */
  __IO uint16_t PWR;                               /**< ADC Power Control Register, offset: 0x9C */
  __IO uint16_t CAL;                               /**< ADC Calibration Register, offset: 0x9E */
  __IO uint16_t GC1;                               /**< Gain Control 1 Register, offset: 0xA0 */
  __IO uint16_t GC2;                               /**< Gain Control 2 Register, offset: 0xA2 */
  __IO uint16_t SCTRL;                             /**< ADC Scan Control Register, offset: 0xA4 */
  __IO uint16_t PWR2;                              /**< ADC Power Control Register, offset: 0xA6 */
  __IO uint16_t CTRL3;                             /**< ADC Control Register 3, offset: 0xA8 */
  __IO uint16_t SCHLTEN;                           /**< ADC Scan Interrupt Enable Register, offset: 0xAA */
} ADC_Type, *ADC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register accessors */
#define ADC_CTRL1_REG(base)                      ((base)->CTRL1)
#define ADC_CTRL2_REG(base)                      ((base)->CTRL2)
#define ADC_ZXCTRL1_REG(base)                    ((base)->ZXCTRL1)
#define ADC_ZXCTRL2_REG(base)                    ((base)->ZXCTRL2)
#define ADC_CLIST1_REG(base)                     ((base)->CLIST1)
#define ADC_CLIST2_REG(base)                     ((base)->CLIST2)
#define ADC_CLIST3_REG(base)                     ((base)->CLIST3)
#define ADC_CLIST4_REG(base)                     ((base)->CLIST4)
#define ADC_SDIS_REG(base)                       ((base)->SDIS)
#define ADC_STAT_REG(base)                       ((base)->STAT)
#define ADC_RDY_REG(base)                        ((base)->RDY)
#define ADC_LOLIMSTAT_REG(base)                  ((base)->LOLIMSTAT)
#define ADC_HILIMSTAT_REG(base)                  ((base)->HILIMSTAT)
#define ADC_ZXSTAT_REG(base)                     ((base)->ZXSTAT)
#define ADC_RSLT_REG(base,index)                 ((base)->RSLT[index])
#define ADC_RSLT_COUNT                           16
#define ADC_LOLIM_REG(base,index)                ((base)->LOLIM[index])
#define ADC_LOLIM_COUNT                          16
#define ADC_HILIM_REG(base,index)                ((base)->HILIM[index])
#define ADC_HILIM_COUNT                          16
#define ADC_OFFST_REG(base,index)                ((base)->OFFST[index])
#define ADC_OFFST_COUNT                          16
#define ADC_PWR_REG(base)                        ((base)->PWR)
#define ADC_CAL_REG(base)                        ((base)->CAL)
#define ADC_GC1_REG(base)                        ((base)->GC1)
#define ADC_GC2_REG(base)                        ((base)->GC2)
#define ADC_SCTRL_REG(base)                      ((base)->SCTRL)
#define ADC_PWR2_REG(base)                       ((base)->PWR2)
#define ADC_CTRL3_REG(base)                      ((base)->CTRL3)
#define ADC_SCHLTEN_REG(base)                    ((base)->SCHLTEN)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/* CTRL1 Bit Fields */
#define ADC_CTRL1_SMODE_MASK                     0x7u
#define ADC_CTRL1_SMODE_SHIFT                    0
#define ADC_CTRL1_SMODE_WIDTH                    3
#define ADC_CTRL1_SMODE(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_SMODE_SHIFT))&ADC_CTRL1_SMODE_MASK)
#define ADC_CTRL1_CHNCFG_L_MASK                  0xF0u
#define ADC_CTRL1_CHNCFG_L_SHIFT                 4
#define ADC_CTRL1_CHNCFG_L_WIDTH                 4
#define ADC_CTRL1_CHNCFG_L(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_CHNCFG_L_SHIFT))&ADC_CTRL1_CHNCFG_L_MASK)
#define ADC_CTRL1_HLMTIE_MASK                    0x100u
#define ADC_CTRL1_HLMTIE_SHIFT                   8
#define ADC_CTRL1_HLMTIE_WIDTH                   1
#define ADC_CTRL1_HLMTIE(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_HLMTIE_SHIFT))&ADC_CTRL1_HLMTIE_MASK)
#define ADC_CTRL1_LLMTIE_MASK                    0x200u
#define ADC_CTRL1_LLMTIE_SHIFT                   9
#define ADC_CTRL1_LLMTIE_WIDTH                   1
#define ADC_CTRL1_LLMTIE(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_LLMTIE_SHIFT))&ADC_CTRL1_LLMTIE_MASK)
#define ADC_CTRL1_ZCIE_MASK                      0x400u
#define ADC_CTRL1_ZCIE_SHIFT                     10
#define ADC_CTRL1_ZCIE_WIDTH                     1
#define ADC_CTRL1_ZCIE(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_ZCIE_SHIFT))&ADC_CTRL1_ZCIE_MASK)
#define ADC_CTRL1_EOSIE0_MASK                    0x800u
#define ADC_CTRL1_EOSIE0_SHIFT                   11
#define ADC_CTRL1_EOSIE0_WIDTH                   1
#define ADC_CTRL1_EOSIE0(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_EOSIE0_SHIFT))&ADC_CTRL1_EOSIE0_MASK)
#define ADC_CTRL1_SYNC0_MASK                     0x1000u
#define ADC_CTRL1_SYNC0_SHIFT                    12
#define ADC_CTRL1_SYNC0_WIDTH                    1
#define ADC_CTRL1_SYNC0(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_SYNC0_SHIFT))&ADC_CTRL1_SYNC0_MASK)
#define ADC_CTRL1_START0_MASK                    0x2000u
#define ADC_CTRL1_START0_SHIFT                   13
#define ADC_CTRL1_START0_WIDTH                   1
#define ADC_CTRL1_START0(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_START0_SHIFT))&ADC_CTRL1_START0_MASK)
#define ADC_CTRL1_STOP0_MASK                     0x4000u
#define ADC_CTRL1_STOP0_SHIFT                    14
#define ADC_CTRL1_STOP0_WIDTH                    1
#define ADC_CTRL1_STOP0(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_STOP0_SHIFT))&ADC_CTRL1_STOP0_MASK)
#define ADC_CTRL1_DMAEN0_MASK                    0x8000u
#define ADC_CTRL1_DMAEN0_SHIFT                   15
#define ADC_CTRL1_DMAEN0_WIDTH                   1
#define ADC_CTRL1_DMAEN0(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_DMAEN0_SHIFT))&ADC_CTRL1_DMAEN0_MASK)
/* CTRL2 Bit Fields */
#define ADC_CTRL2_DIV0_MASK                      0x3Fu
#define ADC_CTRL2_DIV0_SHIFT                     0
#define ADC_CTRL2_DIV0_WIDTH                     6
#define ADC_CTRL2_DIV0(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_DIV0_SHIFT))&ADC_CTRL2_DIV0_MASK)
#define ADC_CTRL2_SIMULT_MASK                    0x40u
#define ADC_CTRL2_SIMULT_SHIFT                   6
#define ADC_CTRL2_SIMULT_WIDTH                   1
#define ADC_CTRL2_SIMULT(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_SIMULT_SHIFT))&ADC_CTRL2_SIMULT_MASK)
#define ADC_CTRL2_CHNCFG_H_MASK                  0x780u
#define ADC_CTRL2_CHNCFG_H_SHIFT                 7
#define ADC_CTRL2_CHNCFG_H_WIDTH                 4
#define ADC_CTRL2_CHNCFG_H(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_CHNCFG_H_SHIFT))&ADC_CTRL2_CHNCFG_H_MASK)
#define ADC_CTRL2_EOSIE1_MASK                    0x800u
#define ADC_CTRL2_EOSIE1_SHIFT                   11
#define ADC_CTRL2_EOSIE1_WIDTH                   1
#define ADC_CTRL2_EOSIE1(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_EOSIE1_SHIFT))&ADC_CTRL2_EOSIE1_MASK)
#define ADC_CTRL2_SYNC1_MASK                     0x1000u
#define ADC_CTRL2_SYNC1_SHIFT                    12
#define ADC_CTRL2_SYNC1_WIDTH                    1
#define ADC_CTRL2_SYNC1(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_SYNC1_SHIFT))&ADC_CTRL2_SYNC1_MASK)
#define ADC_CTRL2_START1_MASK                    0x2000u
#define ADC_CTRL2_START1_SHIFT                   13
#define ADC_CTRL2_START1_WIDTH                   1
#define ADC_CTRL2_START1(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_START1_SHIFT))&ADC_CTRL2_START1_MASK)
#define ADC_CTRL2_STOP1_MASK                     0x4000u
#define ADC_CTRL2_STOP1_SHIFT                    14
#define ADC_CTRL2_STOP1_WIDTH                    1
#define ADC_CTRL2_STOP1(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_STOP1_SHIFT))&ADC_CTRL2_STOP1_MASK)
#define ADC_CTRL2_DMAEN1_MASK                    0x8000u
#define ADC_CTRL2_DMAEN1_SHIFT                   15
#define ADC_CTRL2_DMAEN1_WIDTH                   1
#define ADC_CTRL2_DMAEN1(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_DMAEN1_SHIFT))&ADC_CTRL2_DMAEN1_MASK)
/* ZXCTRL1 Bit Fields */
#define ADC_ZXCTRL1_ZCE0_MASK                    0x3u
#define ADC_ZXCTRL1_ZCE0_SHIFT                   0
#define ADC_ZXCTRL1_ZCE0_WIDTH                   2
#define ADC_ZXCTRL1_ZCE0(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE0_SHIFT))&ADC_ZXCTRL1_ZCE0_MASK)
#define ADC_ZXCTRL1_ZCE1_MASK                    0xCu
#define ADC_ZXCTRL1_ZCE1_SHIFT                   2
#define ADC_ZXCTRL1_ZCE1_WIDTH                   2
#define ADC_ZXCTRL1_ZCE1(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE1_SHIFT))&ADC_ZXCTRL1_ZCE1_MASK)
#define ADC_ZXCTRL1_ZCE2_MASK                    0x30u
#define ADC_ZXCTRL1_ZCE2_SHIFT                   4
#define ADC_ZXCTRL1_ZCE2_WIDTH                   2
#define ADC_ZXCTRL1_ZCE2(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE2_SHIFT))&ADC_ZXCTRL1_ZCE2_MASK)
#define ADC_ZXCTRL1_ZCE3_MASK                    0xC0u
#define ADC_ZXCTRL1_ZCE3_SHIFT                   6
#define ADC_ZXCTRL1_ZCE3_WIDTH                   2
#define ADC_ZXCTRL1_ZCE3(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE3_SHIFT))&ADC_ZXCTRL1_ZCE3_MASK)
#define ADC_ZXCTRL1_ZCE4_MASK                    0x300u
#define ADC_ZXCTRL1_ZCE4_SHIFT                   8
#define ADC_ZXCTRL1_ZCE4_WIDTH                   2
#define ADC_ZXCTRL1_ZCE4(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE4_SHIFT))&ADC_ZXCTRL1_ZCE4_MASK)
#define ADC_ZXCTRL1_ZCE5_MASK                    0xC00u
#define ADC_ZXCTRL1_ZCE5_SHIFT                   10
#define ADC_ZXCTRL1_ZCE5_WIDTH                   2
#define ADC_ZXCTRL1_ZCE5(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE5_SHIFT))&ADC_ZXCTRL1_ZCE5_MASK)
#define ADC_ZXCTRL1_ZCE6_MASK                    0x3000u
#define ADC_ZXCTRL1_ZCE6_SHIFT                   12
#define ADC_ZXCTRL1_ZCE6_WIDTH                   2
#define ADC_ZXCTRL1_ZCE6(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE6_SHIFT))&ADC_ZXCTRL1_ZCE6_MASK)
#define ADC_ZXCTRL1_ZCE7_MASK                    0xC000u
#define ADC_ZXCTRL1_ZCE7_SHIFT                   14
#define ADC_ZXCTRL1_ZCE7_WIDTH                   2
#define ADC_ZXCTRL1_ZCE7(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE7_SHIFT))&ADC_ZXCTRL1_ZCE7_MASK)
/* ZXCTRL2 Bit Fields */
#define ADC_ZXCTRL2_ZCE8_MASK                    0x3u
#define ADC_ZXCTRL2_ZCE8_SHIFT                   0
#define ADC_ZXCTRL2_ZCE8_WIDTH                   2
#define ADC_ZXCTRL2_ZCE8(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE8_SHIFT))&ADC_ZXCTRL2_ZCE8_MASK)
#define ADC_ZXCTRL2_ZCE9_MASK                    0xCu
#define ADC_ZXCTRL2_ZCE9_SHIFT                   2
#define ADC_ZXCTRL2_ZCE9_WIDTH                   2
#define ADC_ZXCTRL2_ZCE9(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE9_SHIFT))&ADC_ZXCTRL2_ZCE9_MASK)
#define ADC_ZXCTRL2_ZCE10_MASK                   0x30u
#define ADC_ZXCTRL2_ZCE10_SHIFT                  4
#define ADC_ZXCTRL2_ZCE10_WIDTH                  2
#define ADC_ZXCTRL2_ZCE10(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE10_SHIFT))&ADC_ZXCTRL2_ZCE10_MASK)
#define ADC_ZXCTRL2_ZCE11_MASK                   0xC0u
#define ADC_ZXCTRL2_ZCE11_SHIFT                  6
#define ADC_ZXCTRL2_ZCE11_WIDTH                  2
#define ADC_ZXCTRL2_ZCE11(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE11_SHIFT))&ADC_ZXCTRL2_ZCE11_MASK)
#define ADC_ZXCTRL2_ZCE12_MASK                   0x300u
#define ADC_ZXCTRL2_ZCE12_SHIFT                  8
#define ADC_ZXCTRL2_ZCE12_WIDTH                  2
#define ADC_ZXCTRL2_ZCE12(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE12_SHIFT))&ADC_ZXCTRL2_ZCE12_MASK)
#define ADC_ZXCTRL2_ZCE13_MASK                   0xC00u
#define ADC_ZXCTRL2_ZCE13_SHIFT                  10
#define ADC_ZXCTRL2_ZCE13_WIDTH                  2
#define ADC_ZXCTRL2_ZCE13(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE13_SHIFT))&ADC_ZXCTRL2_ZCE13_MASK)
#define ADC_ZXCTRL2_ZCE14_MASK                   0x3000u
#define ADC_ZXCTRL2_ZCE14_SHIFT                  12
#define ADC_ZXCTRL2_ZCE14_WIDTH                  2
#define ADC_ZXCTRL2_ZCE14(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE14_SHIFT))&ADC_ZXCTRL2_ZCE14_MASK)
#define ADC_ZXCTRL2_ZCE15_MASK                   0xC000u
#define ADC_ZXCTRL2_ZCE15_SHIFT                  14
#define ADC_ZXCTRL2_ZCE15_WIDTH                  2
#define ADC_ZXCTRL2_ZCE15(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE15_SHIFT))&ADC_ZXCTRL2_ZCE15_MASK)
/* CLIST1 Bit Fields */
#define ADC_CLIST1_SAMPLE0_MASK                  0xFu
#define ADC_CLIST1_SAMPLE0_SHIFT                 0
#define ADC_CLIST1_SAMPLE0_WIDTH                 4
#define ADC_CLIST1_SAMPLE0(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE0_SHIFT))&ADC_CLIST1_SAMPLE0_MASK)
#define ADC_CLIST1_SAMPLE1_MASK                  0xF0u
#define ADC_CLIST1_SAMPLE1_SHIFT                 4
#define ADC_CLIST1_SAMPLE1_WIDTH                 4
#define ADC_CLIST1_SAMPLE1(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE1_SHIFT))&ADC_CLIST1_SAMPLE1_MASK)
#define ADC_CLIST1_SAMPLE2_MASK                  0xF00u
#define ADC_CLIST1_SAMPLE2_SHIFT                 8
#define ADC_CLIST1_SAMPLE2_WIDTH                 4
#define ADC_CLIST1_SAMPLE2(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE2_SHIFT))&ADC_CLIST1_SAMPLE2_MASK)
#define ADC_CLIST1_SAMPLE3_MASK                  0xF000u
#define ADC_CLIST1_SAMPLE3_SHIFT                 12
#define ADC_CLIST1_SAMPLE3_WIDTH                 4
#define ADC_CLIST1_SAMPLE3(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE3_SHIFT))&ADC_CLIST1_SAMPLE3_MASK)
/* CLIST2 Bit Fields */
#define ADC_CLIST2_SAMPLE4_MASK                  0xFu
#define ADC_CLIST2_SAMPLE4_SHIFT                 0
#define ADC_CLIST2_SAMPLE4_WIDTH                 4
#define ADC_CLIST2_SAMPLE4(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE4_SHIFT))&ADC_CLIST2_SAMPLE4_MASK)
#define ADC_CLIST2_SAMPLE5_MASK                  0xF0u
#define ADC_CLIST2_SAMPLE5_SHIFT                 4
#define ADC_CLIST2_SAMPLE5_WIDTH                 4
#define ADC_CLIST2_SAMPLE5(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE5_SHIFT))&ADC_CLIST2_SAMPLE5_MASK)
#define ADC_CLIST2_SAMPLE6_MASK                  0xF00u
#define ADC_CLIST2_SAMPLE6_SHIFT                 8
#define ADC_CLIST2_SAMPLE6_WIDTH                 4
#define ADC_CLIST2_SAMPLE6(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE6_SHIFT))&ADC_CLIST2_SAMPLE6_MASK)
#define ADC_CLIST2_SAMPLE7_MASK                  0xF000u
#define ADC_CLIST2_SAMPLE7_SHIFT                 12
#define ADC_CLIST2_SAMPLE7_WIDTH                 4
#define ADC_CLIST2_SAMPLE7(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE7_SHIFT))&ADC_CLIST2_SAMPLE7_MASK)
/* CLIST3 Bit Fields */
#define ADC_CLIST3_SAMPLE8_MASK                  0xFu
#define ADC_CLIST3_SAMPLE8_SHIFT                 0
#define ADC_CLIST3_SAMPLE8_WIDTH                 4
#define ADC_CLIST3_SAMPLE8(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE8_SHIFT))&ADC_CLIST3_SAMPLE8_MASK)
#define ADC_CLIST3_SAMPLE9_MASK                  0xF0u
#define ADC_CLIST3_SAMPLE9_SHIFT                 4
#define ADC_CLIST3_SAMPLE9_WIDTH                 4
#define ADC_CLIST3_SAMPLE9(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE9_SHIFT))&ADC_CLIST3_SAMPLE9_MASK)
#define ADC_CLIST3_SAMPLE10_MASK                 0xF00u
#define ADC_CLIST3_SAMPLE10_SHIFT                8
#define ADC_CLIST3_SAMPLE10_WIDTH                4
#define ADC_CLIST3_SAMPLE10(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE10_SHIFT))&ADC_CLIST3_SAMPLE10_MASK)
#define ADC_CLIST3_SAMPLE11_MASK                 0xF000u
#define ADC_CLIST3_SAMPLE11_SHIFT                12
#define ADC_CLIST3_SAMPLE11_WIDTH                4
#define ADC_CLIST3_SAMPLE11(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE11_SHIFT))&ADC_CLIST3_SAMPLE11_MASK)
/* CLIST4 Bit Fields */
#define ADC_CLIST4_SAMPLE12_MASK                 0xFu
#define ADC_CLIST4_SAMPLE12_SHIFT                0
#define ADC_CLIST4_SAMPLE12_WIDTH                4
#define ADC_CLIST4_SAMPLE12(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE12_SHIFT))&ADC_CLIST4_SAMPLE12_MASK)
#define ADC_CLIST4_SAMPLE13_MASK                 0xF0u
#define ADC_CLIST4_SAMPLE13_SHIFT                4
#define ADC_CLIST4_SAMPLE13_WIDTH                4
#define ADC_CLIST4_SAMPLE13(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE13_SHIFT))&ADC_CLIST4_SAMPLE13_MASK)
#define ADC_CLIST4_SAMPLE14_MASK                 0xF00u
#define ADC_CLIST4_SAMPLE14_SHIFT                8
#define ADC_CLIST4_SAMPLE14_WIDTH                4
#define ADC_CLIST4_SAMPLE14(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE14_SHIFT))&ADC_CLIST4_SAMPLE14_MASK)
#define ADC_CLIST4_SAMPLE15_MASK                 0xF000u
#define ADC_CLIST4_SAMPLE15_SHIFT                12
#define ADC_CLIST4_SAMPLE15_WIDTH                4
#define ADC_CLIST4_SAMPLE15(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE15_SHIFT))&ADC_CLIST4_SAMPLE15_MASK)
/* SDIS Bit Fields */
#define ADC_SDIS_DS_MASK                         0xFFFFu
#define ADC_SDIS_DS_SHIFT                        0
#define ADC_SDIS_DS_WIDTH                        16
#define ADC_SDIS_DS(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_SDIS_DS_SHIFT))&ADC_SDIS_DS_MASK)
/* STAT Bit Fields */
#define ADC_STAT_UNDEFINED_MASK                  0xFFu
#define ADC_STAT_UNDEFINED_SHIFT                 0
#define ADC_STAT_UNDEFINED_WIDTH                 8
#define ADC_STAT_UNDEFINED(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_STAT_UNDEFINED_SHIFT))&ADC_STAT_UNDEFINED_MASK)
#define ADC_STAT_HLMTI_MASK                      0x100u
#define ADC_STAT_HLMTI_SHIFT                     8
#define ADC_STAT_HLMTI_WIDTH                     1
#define ADC_STAT_HLMTI(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_STAT_HLMTI_SHIFT))&ADC_STAT_HLMTI_MASK)
#define ADC_STAT_LLMTI_MASK                      0x200u
#define ADC_STAT_LLMTI_SHIFT                     9
#define ADC_STAT_LLMTI_WIDTH                     1
#define ADC_STAT_LLMTI(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_STAT_LLMTI_SHIFT))&ADC_STAT_LLMTI_MASK)
#define ADC_STAT_ZCI_MASK                        0x400u
#define ADC_STAT_ZCI_SHIFT                       10
#define ADC_STAT_ZCI_WIDTH                       1
#define ADC_STAT_ZCI(x)                          (((uint16_t)(((uint16_t)(x))<<ADC_STAT_ZCI_SHIFT))&ADC_STAT_ZCI_MASK)
#define ADC_STAT_EOSI0_MASK                      0x800u
#define ADC_STAT_EOSI0_SHIFT                     11
#define ADC_STAT_EOSI0_WIDTH                     1
#define ADC_STAT_EOSI0(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_STAT_EOSI0_SHIFT))&ADC_STAT_EOSI0_MASK)
#define ADC_STAT_EOSI1_MASK                      0x1000u
#define ADC_STAT_EOSI1_SHIFT                     12
#define ADC_STAT_EOSI1_WIDTH                     1
#define ADC_STAT_EOSI1(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_STAT_EOSI1_SHIFT))&ADC_STAT_EOSI1_MASK)
#define ADC_STAT_CIP1_MASK                       0x4000u
#define ADC_STAT_CIP1_SHIFT                      14
#define ADC_STAT_CIP1_WIDTH                      1
#define ADC_STAT_CIP1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_STAT_CIP1_SHIFT))&ADC_STAT_CIP1_MASK)
#define ADC_STAT_CIP0_MASK                       0x8000u
#define ADC_STAT_CIP0_SHIFT                      15
#define ADC_STAT_CIP0_WIDTH                      1
#define ADC_STAT_CIP0(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_STAT_CIP0_SHIFT))&ADC_STAT_CIP0_MASK)
/* RDY Bit Fields */
#define ADC_RDY_RDY_MASK                         0xFFFFu
#define ADC_RDY_RDY_SHIFT                        0
#define ADC_RDY_RDY_WIDTH                        16
#define ADC_RDY_RDY(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_RDY_RDY_SHIFT))&ADC_RDY_RDY_MASK)
/* LOLIMSTAT Bit Fields */
#define ADC_LOLIMSTAT_LLS_MASK                   0xFFFFu
#define ADC_LOLIMSTAT_LLS_SHIFT                  0
#define ADC_LOLIMSTAT_LLS_WIDTH                  16
#define ADC_LOLIMSTAT_LLS(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_LOLIMSTAT_LLS_SHIFT))&ADC_LOLIMSTAT_LLS_MASK)
/* HILIMSTAT Bit Fields */
#define ADC_HILIMSTAT_HLS_MASK                   0xFFFFu
#define ADC_HILIMSTAT_HLS_SHIFT                  0
#define ADC_HILIMSTAT_HLS_WIDTH                  16
#define ADC_HILIMSTAT_HLS(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_HILIMSTAT_HLS_SHIFT))&ADC_HILIMSTAT_HLS_MASK)
/* ZXSTAT Bit Fields */
#define ADC_ZXSTAT_ZCS_MASK                      0xFFFFu
#define ADC_ZXSTAT_ZCS_SHIFT                     0
#define ADC_ZXSTAT_ZCS_WIDTH                     16
#define ADC_ZXSTAT_ZCS(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_ZXSTAT_ZCS_SHIFT))&ADC_ZXSTAT_ZCS_MASK)
/* RSLT Bit Fields */
#define ADC_RSLT_RSLT_MASK                       0x7FF8u
#define ADC_RSLT_RSLT_SHIFT                      3
#define ADC_RSLT_RSLT_WIDTH                      12
#define ADC_RSLT_RSLT(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_RSLT_RSLT_SHIFT))&ADC_RSLT_RSLT_MASK)
#define ADC_RSLT_SEXT_MASK                       0x8000u
#define ADC_RSLT_SEXT_SHIFT                      15
#define ADC_RSLT_SEXT_WIDTH                      1
#define ADC_RSLT_SEXT(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_RSLT_SEXT_SHIFT))&ADC_RSLT_SEXT_MASK)
/* LOLIM Bit Fields */
#define ADC_LOLIM_LLMT_MASK                      0x7FF8u
#define ADC_LOLIM_LLMT_SHIFT                     3
#define ADC_LOLIM_LLMT_WIDTH                     12
#define ADC_LOLIM_LLMT(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_LOLIM_LLMT_SHIFT))&ADC_LOLIM_LLMT_MASK)
/* HILIM Bit Fields */
#define ADC_HILIM_HLMT_MASK                      0x7FF8u
#define ADC_HILIM_HLMT_SHIFT                     3
#define ADC_HILIM_HLMT_WIDTH                     12
#define ADC_HILIM_HLMT(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_HILIM_HLMT_SHIFT))&ADC_HILIM_HLMT_MASK)
/* OFFST Bit Fields */
#define ADC_OFFST_OFFSET_MASK                    0x7FF8u
#define ADC_OFFST_OFFSET_SHIFT                   3
#define ADC_OFFST_OFFSET_WIDTH                   12
#define ADC_OFFST_OFFSET(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_OFFST_OFFSET_SHIFT))&ADC_OFFST_OFFSET_MASK)
/* PWR Bit Fields */
#define ADC_PWR_PD0_MASK                         0x1u
#define ADC_PWR_PD0_SHIFT                        0
#define ADC_PWR_PD0_WIDTH                        1
#define ADC_PWR_PD0(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PD0_SHIFT))&ADC_PWR_PD0_MASK)
#define ADC_PWR_PD1_MASK                         0x2u
#define ADC_PWR_PD1_SHIFT                        1
#define ADC_PWR_PD1_WIDTH                        1
#define ADC_PWR_PD1(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PD1_SHIFT))&ADC_PWR_PD1_MASK)
#define ADC_PWR_APD_MASK                         0x8u
#define ADC_PWR_APD_SHIFT                        3
#define ADC_PWR_APD_WIDTH                        1
#define ADC_PWR_APD(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_PWR_APD_SHIFT))&ADC_PWR_APD_MASK)
#define ADC_PWR_PUDELAY_MASK                     0x3F0u
#define ADC_PWR_PUDELAY_SHIFT                    4
#define ADC_PWR_PUDELAY_WIDTH                    6
#define ADC_PWR_PUDELAY(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PUDELAY_SHIFT))&ADC_PWR_PUDELAY_MASK)
#define ADC_PWR_PSTS0_MASK                       0x400u
#define ADC_PWR_PSTS0_SHIFT                      10
#define ADC_PWR_PSTS0_WIDTH                      1
#define ADC_PWR_PSTS0(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PSTS0_SHIFT))&ADC_PWR_PSTS0_MASK)
#define ADC_PWR_PSTS1_MASK                       0x800u
#define ADC_PWR_PSTS1_SHIFT                      11
#define ADC_PWR_PSTS1_WIDTH                      1
#define ADC_PWR_PSTS1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PSTS1_SHIFT))&ADC_PWR_PSTS1_MASK)
#define ADC_PWR_ASB_MASK                         0x8000u
#define ADC_PWR_ASB_SHIFT                        15
#define ADC_PWR_ASB_WIDTH                        1
#define ADC_PWR_ASB(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_PWR_ASB_SHIFT))&ADC_PWR_ASB_MASK)
/* CAL Bit Fields */
#define ADC_CAL_SEL_VREFLO_A_MASK                0x1000u
#define ADC_CAL_SEL_VREFLO_A_SHIFT               12
#define ADC_CAL_SEL_VREFLO_A_WIDTH               1
#define ADC_CAL_SEL_VREFLO_A(x)                  (((uint16_t)(((uint16_t)(x))<<ADC_CAL_SEL_VREFLO_A_SHIFT))&ADC_CAL_SEL_VREFLO_A_MASK)
#define ADC_CAL_SEL_VREFH_A_MASK                 0x2000u
#define ADC_CAL_SEL_VREFH_A_SHIFT                13
#define ADC_CAL_SEL_VREFH_A_WIDTH                1
#define ADC_CAL_SEL_VREFH_A(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CAL_SEL_VREFH_A_SHIFT))&ADC_CAL_SEL_VREFH_A_MASK)
#define ADC_CAL_SEL_VREFLO_B_MASK                0x4000u
#define ADC_CAL_SEL_VREFLO_B_SHIFT               14
#define ADC_CAL_SEL_VREFLO_B_WIDTH               1
#define ADC_CAL_SEL_VREFLO_B(x)                  (((uint16_t)(((uint16_t)(x))<<ADC_CAL_SEL_VREFLO_B_SHIFT))&ADC_CAL_SEL_VREFLO_B_MASK)
#define ADC_CAL_SEL_VREFH_B_MASK                 0x8000u
#define ADC_CAL_SEL_VREFH_B_SHIFT                15
#define ADC_CAL_SEL_VREFH_B_WIDTH                1
#define ADC_CAL_SEL_VREFH_B(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CAL_SEL_VREFH_B_SHIFT))&ADC_CAL_SEL_VREFH_B_MASK)
/* GC1 Bit Fields */
#define ADC_GC1_GAIN0_MASK                       0x3u
#define ADC_GC1_GAIN0_SHIFT                      0
#define ADC_GC1_GAIN0_WIDTH                      2
#define ADC_GC1_GAIN0(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN0_SHIFT))&ADC_GC1_GAIN0_MASK)
#define ADC_GC1_GAIN1_MASK                       0xCu
#define ADC_GC1_GAIN1_SHIFT                      2
#define ADC_GC1_GAIN1_WIDTH                      2
#define ADC_GC1_GAIN1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN1_SHIFT))&ADC_GC1_GAIN1_MASK)
#define ADC_GC1_GAIN2_MASK                       0x30u
#define ADC_GC1_GAIN2_SHIFT                      4
#define ADC_GC1_GAIN2_WIDTH                      2
#define ADC_GC1_GAIN2(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN2_SHIFT))&ADC_GC1_GAIN2_MASK)
#define ADC_GC1_GAIN3_MASK                       0xC0u
#define ADC_GC1_GAIN3_SHIFT                      6
#define ADC_GC1_GAIN3_WIDTH                      2
#define ADC_GC1_GAIN3(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN3_SHIFT))&ADC_GC1_GAIN3_MASK)
#define ADC_GC1_GAIN4_MASK                       0x300u
#define ADC_GC1_GAIN4_SHIFT                      8
#define ADC_GC1_GAIN4_WIDTH                      2
#define ADC_GC1_GAIN4(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN4_SHIFT))&ADC_GC1_GAIN4_MASK)
#define ADC_GC1_GAIN5_MASK                       0xC00u
#define ADC_GC1_GAIN5_SHIFT                      10
#define ADC_GC1_GAIN5_WIDTH                      2
#define ADC_GC1_GAIN5(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN5_SHIFT))&ADC_GC1_GAIN5_MASK)
#define ADC_GC1_GAIN6_MASK                       0x3000u
#define ADC_GC1_GAIN6_SHIFT                      12
#define ADC_GC1_GAIN6_WIDTH                      2
#define ADC_GC1_GAIN6(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN6_SHIFT))&ADC_GC1_GAIN6_MASK)
#define ADC_GC1_GAIN7_MASK                       0xC000u
#define ADC_GC1_GAIN7_SHIFT                      14
#define ADC_GC1_GAIN7_WIDTH                      2
#define ADC_GC1_GAIN7(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN7_SHIFT))&ADC_GC1_GAIN7_MASK)
/* GC2 Bit Fields */
#define ADC_GC2_GAIN8_MASK                       0x3u
#define ADC_GC2_GAIN8_SHIFT                      0
#define ADC_GC2_GAIN8_WIDTH                      2
#define ADC_GC2_GAIN8(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN8_SHIFT))&ADC_GC2_GAIN8_MASK)
#define ADC_GC2_GAIN9_MASK                       0xCu
#define ADC_GC2_GAIN9_SHIFT                      2
#define ADC_GC2_GAIN9_WIDTH                      2
#define ADC_GC2_GAIN9(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN9_SHIFT))&ADC_GC2_GAIN9_MASK)
#define ADC_GC2_GAIN10_MASK                      0x30u
#define ADC_GC2_GAIN10_SHIFT                     4
#define ADC_GC2_GAIN10_WIDTH                     2
#define ADC_GC2_GAIN10(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN10_SHIFT))&ADC_GC2_GAIN10_MASK)
#define ADC_GC2_GAIN11_MASK                      0xC0u
#define ADC_GC2_GAIN11_SHIFT                     6
#define ADC_GC2_GAIN11_WIDTH                     2
#define ADC_GC2_GAIN11(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN11_SHIFT))&ADC_GC2_GAIN11_MASK)
#define ADC_GC2_GAIN12_MASK                      0x300u
#define ADC_GC2_GAIN12_SHIFT                     8
#define ADC_GC2_GAIN12_WIDTH                     2
#define ADC_GC2_GAIN12(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN12_SHIFT))&ADC_GC2_GAIN12_MASK)
#define ADC_GC2_GAIN13_MASK                      0xC00u
#define ADC_GC2_GAIN13_SHIFT                     10
#define ADC_GC2_GAIN13_WIDTH                     2
#define ADC_GC2_GAIN13(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN13_SHIFT))&ADC_GC2_GAIN13_MASK)
#define ADC_GC2_GAIN14_MASK                      0x3000u
#define ADC_GC2_GAIN14_SHIFT                     12
#define ADC_GC2_GAIN14_WIDTH                     2
#define ADC_GC2_GAIN14(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN14_SHIFT))&ADC_GC2_GAIN14_MASK)
#define ADC_GC2_GAIN15_MASK                      0xC000u
#define ADC_GC2_GAIN15_SHIFT                     14
#define ADC_GC2_GAIN15_WIDTH                     2
#define ADC_GC2_GAIN15(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN15_SHIFT))&ADC_GC2_GAIN15_MASK)
/* SCTRL Bit Fields */
#define ADC_SCTRL_SC_MASK                        0xFFFFu
#define ADC_SCTRL_SC_SHIFT                       0
#define ADC_SCTRL_SC_WIDTH                       16
#define ADC_SCTRL_SC(x)                          (((uint16_t)(((uint16_t)(x))<<ADC_SCTRL_SC_SHIFT))&ADC_SCTRL_SC_MASK)
/* PWR2 Bit Fields */
#define ADC_PWR2_SPEEDA_MASK                     0x3u
#define ADC_PWR2_SPEEDA_SHIFT                    0
#define ADC_PWR2_SPEEDA_WIDTH                    2
#define ADC_PWR2_SPEEDA(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_PWR2_SPEEDA_SHIFT))&ADC_PWR2_SPEEDA_MASK)
#define ADC_PWR2_SPEEDB_MASK                     0xCu
#define ADC_PWR2_SPEEDB_SHIFT                    2
#define ADC_PWR2_SPEEDB_WIDTH                    2
#define ADC_PWR2_SPEEDB(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_PWR2_SPEEDB_SHIFT))&ADC_PWR2_SPEEDB_MASK)
#define ADC_PWR2_DIV1_MASK                       0x3F00u
#define ADC_PWR2_DIV1_SHIFT                      8
#define ADC_PWR2_DIV1_WIDTH                      6
#define ADC_PWR2_DIV1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_PWR2_DIV1_SHIFT))&ADC_PWR2_DIV1_MASK)
/* CTRL3 Bit Fields */
#define ADC_CTRL3_SCNT0_MASK                     0x7u
#define ADC_CTRL3_SCNT0_SHIFT                    0
#define ADC_CTRL3_SCNT0_WIDTH                    3
#define ADC_CTRL3_SCNT0(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL3_SCNT0_SHIFT))&ADC_CTRL3_SCNT0_MASK)
#define ADC_CTRL3_SCNT1_MASK                     0x38u
#define ADC_CTRL3_SCNT1_SHIFT                    3
#define ADC_CTRL3_SCNT1_WIDTH                    3
#define ADC_CTRL3_SCNT1(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL3_SCNT1_SHIFT))&ADC_CTRL3_SCNT1_MASK)
#define ADC_CTRL3_DMASRC_MASK                    0x40u
#define ADC_CTRL3_DMASRC_SHIFT                   6
#define ADC_CTRL3_DMASRC_WIDTH                   1
#define ADC_CTRL3_DMASRC(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_CTRL3_DMASRC_SHIFT))&ADC_CTRL3_DMASRC_MASK)
/* SCHLTEN Bit Fields */
#define ADC_SCHLTEN_SCHLTEN_MASK                 0xFFFFu
#define ADC_SCHLTEN_SCHLTEN_SHIFT                0
#define ADC_SCHLTEN_SCHLTEN_WIDTH                16
#define ADC_SCHLTEN_SCHLTEN(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_SCHLTEN_SCHLTEN_SHIFT))&ADC_SCHLTEN_SCHLTEN_MASK)

/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC base address */
#define ADC_BASE                                 (0x4005C000u)
/** Peripheral ADC base pointer */
#define ADC                                      ((ADC_Type *)ADC_BASE)
#define ADC_BASE_PTR                             (ADC)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC }

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register instance definitions */
/* ADC */
#define ADC_CTRL1                                ADC_CTRL1_REG(ADC)
#define ADC_CTRL2                                ADC_CTRL2_REG(ADC)
#define ADC_ZXCTRL1                              ADC_ZXCTRL1_REG(ADC)
#define ADC_ZXCTRL2                              ADC_ZXCTRL2_REG(ADC)
#define ADC_CLIST1                               ADC_CLIST1_REG(ADC)
#define ADC_CLIST2                               ADC_CLIST2_REG(ADC)
#define ADC_CLIST3                               ADC_CLIST3_REG(ADC)
#define ADC_CLIST4                               ADC_CLIST4_REG(ADC)
#define ADC_SDIS                                 ADC_SDIS_REG(ADC)
#define ADC_STAT                                 ADC_STAT_REG(ADC)
#define ADC_RDY                                  ADC_RDY_REG(ADC)
#define ADC_LOLIMSTAT                            ADC_LOLIMSTAT_REG(ADC)
#define ADC_HILIMSTAT                            ADC_HILIMSTAT_REG(ADC)
#define ADC_ZXSTAT                               ADC_ZXSTAT_REG(ADC)
#define ADC_RSLT0                                ADC_RSLT_REG(ADC,0)
#define ADC_RSLT1                                ADC_RSLT_REG(ADC,1)
#define ADC_RSLT2                                ADC_RSLT_REG(ADC,2)
#define ADC_RSLT3                                ADC_RSLT_REG(ADC,3)
#define ADC_RSLT4                                ADC_RSLT_REG(ADC,4)
#define ADC_RSLT5                                ADC_RSLT_REG(ADC,5)
#define ADC_RSLT6                                ADC_RSLT_REG(ADC,6)
#define ADC_RSLT7                                ADC_RSLT_REG(ADC,7)
#define ADC_RSLT8                                ADC_RSLT_REG(ADC,8)
#define ADC_RSLT9                                ADC_RSLT_REG(ADC,9)
#define ADC_RSLT10                               ADC_RSLT_REG(ADC,10)
#define ADC_RSLT11                               ADC_RSLT_REG(ADC,11)
#define ADC_RSLT12                               ADC_RSLT_REG(ADC,12)
#define ADC_RSLT13                               ADC_RSLT_REG(ADC,13)
#define ADC_RSLT14                               ADC_RSLT_REG(ADC,14)
#define ADC_RSLT15                               ADC_RSLT_REG(ADC,15)
#define ADC_LOLIM0                               ADC_LOLIM_REG(ADC,0)
#define ADC_LOLIM1                               ADC_LOLIM_REG(ADC,1)
#define ADC_LOLIM2                               ADC_LOLIM_REG(ADC,2)
#define ADC_LOLIM3                               ADC_LOLIM_REG(ADC,3)
#define ADC_LOLIM4                               ADC_LOLIM_REG(ADC,4)
#define ADC_LOLIM5                               ADC_LOLIM_REG(ADC,5)
#define ADC_LOLIM6                               ADC_LOLIM_REG(ADC,6)
#define ADC_LOLIM7                               ADC_LOLIM_REG(ADC,7)
#define ADC_LOLIM8                               ADC_LOLIM_REG(ADC,8)
#define ADC_LOLIM9                               ADC_LOLIM_REG(ADC,9)
#define ADC_LOLIM10                              ADC_LOLIM_REG(ADC,10)
#define ADC_LOLIM11                              ADC_LOLIM_REG(ADC,11)
#define ADC_LOLIM12                              ADC_LOLIM_REG(ADC,12)
#define ADC_LOLIM13                              ADC_LOLIM_REG(ADC,13)
#define ADC_LOLIM14                              ADC_LOLIM_REG(ADC,14)
#define ADC_LOLIM15                              ADC_LOLIM_REG(ADC,15)
#define ADC_HILIM0                               ADC_HILIM_REG(ADC,0)
#define ADC_HILIM1                               ADC_HILIM_REG(ADC,1)
#define ADC_HILIM2                               ADC_HILIM_REG(ADC,2)
#define ADC_HILIM3                               ADC_HILIM_REG(ADC,3)
#define ADC_HILIM4                               ADC_HILIM_REG(ADC,4)
#define ADC_HILIM5                               ADC_HILIM_REG(ADC,5)
#define ADC_HILIM6                               ADC_HILIM_REG(ADC,6)
#define ADC_HILIM7                               ADC_HILIM_REG(ADC,7)
#define ADC_HILIM8                               ADC_HILIM_REG(ADC,8)
#define ADC_HILIM9                               ADC_HILIM_REG(ADC,9)
#define ADC_HILIM10                              ADC_HILIM_REG(ADC,10)
#define ADC_HILIM11                              ADC_HILIM_REG(ADC,11)
#define ADC_HILIM12                              ADC_HILIM_REG(ADC,12)
#define ADC_HILIM13                              ADC_HILIM_REG(ADC,13)
#define ADC_HILIM14                              ADC_HILIM_REG(ADC,14)
#define ADC_HILIM15                              ADC_HILIM_REG(ADC,15)
#define ADC_OFFST0                               ADC_OFFST_REG(ADC,0)
#define ADC_OFFST1                               ADC_OFFST_REG(ADC,1)
#define ADC_OFFST2                               ADC_OFFST_REG(ADC,2)
#define ADC_OFFST3                               ADC_OFFST_REG(ADC,3)
#define ADC_OFFST4                               ADC_OFFST_REG(ADC,4)
#define ADC_OFFST5                               ADC_OFFST_REG(ADC,5)
#define ADC_OFFST6                               ADC_OFFST_REG(ADC,6)
#define ADC_OFFST7                               ADC_OFFST_REG(ADC,7)
#define ADC_OFFST8                               ADC_OFFST_REG(ADC,8)
#define ADC_OFFST9                               ADC_OFFST_REG(ADC,9)
#define ADC_OFFST10                              ADC_OFFST_REG(ADC,10)
#define ADC_OFFST11                              ADC_OFFST_REG(ADC,11)
#define ADC_OFFST12                              ADC_OFFST_REG(ADC,12)
#define ADC_OFFST13                              ADC_OFFST_REG(ADC,13)
#define ADC_OFFST14                              ADC_OFFST_REG(ADC,14)
#define ADC_OFFST15                              ADC_OFFST_REG(ADC,15)
#define ADC_PWR                                  ADC_PWR_REG(ADC)
#define ADC_CAL                                  ADC_CAL_REG(ADC)
#define ADC_GC1                                  ADC_GC1_REG(ADC)
#define ADC_GC2                                  ADC_GC2_REG(ADC)
#define ADC_SCTRL                                ADC_SCTRL_REG(ADC)
#define ADC_PWR2                                 ADC_PWR2_REG(ADC)
#define ADC_CTRL3                                ADC_CTRL3_REG(ADC)
#define ADC_SCHLTEN                              ADC_SCHLTEN_REG(ADC)

/* ADC - Register array accessors */
#define ADC_RSLT(index)                          ADC_RSLT_REG(ADC,index)
#define ADC_LOLIM(index)                         ADC_LOLIM_REG(ADC,index)
#define ADC_HILIM(index)                         ADC_HILIM_REG(ADC,index)
#define ADC_OFFST(index)                         ADC_OFFST_REG(ADC,index)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- AIPS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Peripheral_Access_Layer AIPS Peripheral Access Layer
 * @{
 */

/** AIPS - Register Layout Typedef */
typedef struct {
  __IO uint32_t MPRA;                              /**< Master Privilege Register A, offset: 0x0 */
       uint8_t RESERVED_0[28];
  __IO uint32_t PACRA;                             /**< Peripheral Access Control Register, offset: 0x20 */
  __IO uint32_t PACRB;                             /**< Peripheral Access Control Register, offset: 0x24 */
  __IO uint32_t PACRC;                             /**< Peripheral Access Control Register, offset: 0x28 */
  __IO uint32_t PACRD;                             /**< Peripheral Access Control Register, offset: 0x2C */
       uint8_t RESERVED_1[16];
  __IO uint32_t PACRE;                             /**< Peripheral Access Control Register, offset: 0x40 */
  __IO uint32_t PACRF;                             /**< Peripheral Access Control Register, offset: 0x44 */
  __IO uint32_t PACRG;                             /**< Peripheral Access Control Register, offset: 0x48 */
  __IO uint32_t PACRH;                             /**< Peripheral Access Control Register, offset: 0x4C */
  __IO uint32_t PACRI;                             /**< Peripheral Access Control Register, offset: 0x50 */
  __IO uint32_t PACRJ;                             /**< Peripheral Access Control Register, offset: 0x54 */
  __IO uint32_t PACRK;                             /**< Peripheral Access Control Register, offset: 0x58 */
  __IO uint32_t PACRL;                             /**< Peripheral Access Control Register, offset: 0x5C */
  __IO uint32_t PACRM;                             /**< Peripheral Access Control Register, offset: 0x60 */
  __IO uint32_t PACRN;                             /**< Peripheral Access Control Register, offset: 0x64 */
  __IO uint32_t PACRO;                             /**< Peripheral Access Control Register, offset: 0x68 */
  __IO uint32_t PACRP;                             /**< Peripheral Access Control Register, offset: 0x6C */
} AIPS_Type, *AIPS_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- AIPS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Accessor_Macros AIPS - Register accessor macros
 * @{
 */


/* AIPS - Register accessors */
#define AIPS_MPRA_REG(base)                      ((base)->MPRA)
#define AIPS_PACRA_REG(base)                     ((base)->PACRA)
#define AIPS_PACRB_REG(base)                     ((base)->PACRB)
#define AIPS_PACRC_REG(base)                     ((base)->PACRC)
#define AIPS_PACRD_REG(base)                     ((base)->PACRD)
#define AIPS_PACRE_REG(base)                     ((base)->PACRE)
#define AIPS_PACRF_REG(base)                     ((base)->PACRF)
#define AIPS_PACRG_REG(base)                     ((base)->PACRG)
#define AIPS_PACRH_REG(base)                     ((base)->PACRH)
#define AIPS_PACRI_REG(base)                     ((base)->PACRI)
#define AIPS_PACRJ_REG(base)                     ((base)->PACRJ)
#define AIPS_PACRK_REG(base)                     ((base)->PACRK)
#define AIPS_PACRL_REG(base)                     ((base)->PACRL)
#define AIPS_PACRM_REG(base)                     ((base)->PACRM)
#define AIPS_PACRN_REG(base)                     ((base)->PACRN)
#define AIPS_PACRO_REG(base)                     ((base)->PACRO)
#define AIPS_PACRP_REG(base)                     ((base)->PACRP)

/*!
 * @}
 */ /* end of group AIPS_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- AIPS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Masks AIPS Register Masks
 * @{
 */

/* MPRA Bit Fields */
#define AIPS_MPRA_MPL2_MASK                      0x100000u
#define AIPS_MPRA_MPL2_SHIFT                     20
#define AIPS_MPRA_MPL2_WIDTH                     1
#define AIPS_MPRA_MPL2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MPL2_SHIFT))&AIPS_MPRA_MPL2_MASK)
#define AIPS_MPRA_MTW2_MASK                      0x200000u
#define AIPS_MPRA_MTW2_SHIFT                     21
#define AIPS_MPRA_MTW2_WIDTH                     1
#define AIPS_MPRA_MTW2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTW2_SHIFT))&AIPS_MPRA_MTW2_MASK)
#define AIPS_MPRA_MTR2_MASK                      0x400000u
#define AIPS_MPRA_MTR2_SHIFT                     22
#define AIPS_MPRA_MTR2_WIDTH                     1
#define AIPS_MPRA_MTR2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTR2_SHIFT))&AIPS_MPRA_MTR2_MASK)
#define AIPS_MPRA_MPL1_MASK                      0x1000000u
#define AIPS_MPRA_MPL1_SHIFT                     24
#define AIPS_MPRA_MPL1_WIDTH                     1
#define AIPS_MPRA_MPL1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MPL1_SHIFT))&AIPS_MPRA_MPL1_MASK)
#define AIPS_MPRA_MTW1_MASK                      0x2000000u
#define AIPS_MPRA_MTW1_SHIFT                     25
#define AIPS_MPRA_MTW1_WIDTH                     1
#define AIPS_MPRA_MTW1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTW1_SHIFT))&AIPS_MPRA_MTW1_MASK)
#define AIPS_MPRA_MTR1_MASK                      0x4000000u
#define AIPS_MPRA_MTR1_SHIFT                     26
#define AIPS_MPRA_MTR1_WIDTH                     1
#define AIPS_MPRA_MTR1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTR1_SHIFT))&AIPS_MPRA_MTR1_MASK)
#define AIPS_MPRA_MPL0_MASK                      0x10000000u
#define AIPS_MPRA_MPL0_SHIFT                     28
#define AIPS_MPRA_MPL0_WIDTH                     1
#define AIPS_MPRA_MPL0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MPL0_SHIFT))&AIPS_MPRA_MPL0_MASK)
#define AIPS_MPRA_MTW0_MASK                      0x20000000u
#define AIPS_MPRA_MTW0_SHIFT                     29
#define AIPS_MPRA_MTW0_WIDTH                     1
#define AIPS_MPRA_MTW0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTW0_SHIFT))&AIPS_MPRA_MTW0_MASK)
#define AIPS_MPRA_MTR0_MASK                      0x40000000u
#define AIPS_MPRA_MTR0_SHIFT                     30
#define AIPS_MPRA_MTR0_WIDTH                     1
#define AIPS_MPRA_MTR0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_MPRA_MTR0_SHIFT))&AIPS_MPRA_MTR0_MASK)
/* PACRA Bit Fields */
#define AIPS_PACRA_TP7_MASK                      0x1u
#define AIPS_PACRA_TP7_SHIFT                     0
#define AIPS_PACRA_TP7_WIDTH                     1
#define AIPS_PACRA_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP7_SHIFT))&AIPS_PACRA_TP7_MASK)
#define AIPS_PACRA_WP7_MASK                      0x2u
#define AIPS_PACRA_WP7_SHIFT                     1
#define AIPS_PACRA_WP7_WIDTH                     1
#define AIPS_PACRA_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP7_SHIFT))&AIPS_PACRA_WP7_MASK)
#define AIPS_PACRA_SP7_MASK                      0x4u
#define AIPS_PACRA_SP7_SHIFT                     2
#define AIPS_PACRA_SP7_WIDTH                     1
#define AIPS_PACRA_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP7_SHIFT))&AIPS_PACRA_SP7_MASK)
#define AIPS_PACRA_TP6_MASK                      0x10u
#define AIPS_PACRA_TP6_SHIFT                     4
#define AIPS_PACRA_TP6_WIDTH                     1
#define AIPS_PACRA_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP6_SHIFT))&AIPS_PACRA_TP6_MASK)
#define AIPS_PACRA_WP6_MASK                      0x20u
#define AIPS_PACRA_WP6_SHIFT                     5
#define AIPS_PACRA_WP6_WIDTH                     1
#define AIPS_PACRA_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP6_SHIFT))&AIPS_PACRA_WP6_MASK)
#define AIPS_PACRA_SP6_MASK                      0x40u
#define AIPS_PACRA_SP6_SHIFT                     6
#define AIPS_PACRA_SP6_WIDTH                     1
#define AIPS_PACRA_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP6_SHIFT))&AIPS_PACRA_SP6_MASK)
#define AIPS_PACRA_TP5_MASK                      0x100u
#define AIPS_PACRA_TP5_SHIFT                     8
#define AIPS_PACRA_TP5_WIDTH                     1
#define AIPS_PACRA_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP5_SHIFT))&AIPS_PACRA_TP5_MASK)
#define AIPS_PACRA_WP5_MASK                      0x200u
#define AIPS_PACRA_WP5_SHIFT                     9
#define AIPS_PACRA_WP5_WIDTH                     1
#define AIPS_PACRA_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP5_SHIFT))&AIPS_PACRA_WP5_MASK)
#define AIPS_PACRA_SP5_MASK                      0x400u
#define AIPS_PACRA_SP5_SHIFT                     10
#define AIPS_PACRA_SP5_WIDTH                     1
#define AIPS_PACRA_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP5_SHIFT))&AIPS_PACRA_SP5_MASK)
#define AIPS_PACRA_TP4_MASK                      0x1000u
#define AIPS_PACRA_TP4_SHIFT                     12
#define AIPS_PACRA_TP4_WIDTH                     1
#define AIPS_PACRA_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP4_SHIFT))&AIPS_PACRA_TP4_MASK)
#define AIPS_PACRA_WP4_MASK                      0x2000u
#define AIPS_PACRA_WP4_SHIFT                     13
#define AIPS_PACRA_WP4_WIDTH                     1
#define AIPS_PACRA_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP4_SHIFT))&AIPS_PACRA_WP4_MASK)
#define AIPS_PACRA_SP4_MASK                      0x4000u
#define AIPS_PACRA_SP4_SHIFT                     14
#define AIPS_PACRA_SP4_WIDTH                     1
#define AIPS_PACRA_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP4_SHIFT))&AIPS_PACRA_SP4_MASK)
#define AIPS_PACRA_TP3_MASK                      0x10000u
#define AIPS_PACRA_TP3_SHIFT                     16
#define AIPS_PACRA_TP3_WIDTH                     1
#define AIPS_PACRA_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP3_SHIFT))&AIPS_PACRA_TP3_MASK)
#define AIPS_PACRA_WP3_MASK                      0x20000u
#define AIPS_PACRA_WP3_SHIFT                     17
#define AIPS_PACRA_WP3_WIDTH                     1
#define AIPS_PACRA_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP3_SHIFT))&AIPS_PACRA_WP3_MASK)
#define AIPS_PACRA_SP3_MASK                      0x40000u
#define AIPS_PACRA_SP3_SHIFT                     18
#define AIPS_PACRA_SP3_WIDTH                     1
#define AIPS_PACRA_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP3_SHIFT))&AIPS_PACRA_SP3_MASK)
#define AIPS_PACRA_TP2_MASK                      0x100000u
#define AIPS_PACRA_TP2_SHIFT                     20
#define AIPS_PACRA_TP2_WIDTH                     1
#define AIPS_PACRA_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP2_SHIFT))&AIPS_PACRA_TP2_MASK)
#define AIPS_PACRA_WP2_MASK                      0x200000u
#define AIPS_PACRA_WP2_SHIFT                     21
#define AIPS_PACRA_WP2_WIDTH                     1
#define AIPS_PACRA_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP2_SHIFT))&AIPS_PACRA_WP2_MASK)
#define AIPS_PACRA_SP2_MASK                      0x400000u
#define AIPS_PACRA_SP2_SHIFT                     22
#define AIPS_PACRA_SP2_WIDTH                     1
#define AIPS_PACRA_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP2_SHIFT))&AIPS_PACRA_SP2_MASK)
#define AIPS_PACRA_TP1_MASK                      0x1000000u
#define AIPS_PACRA_TP1_SHIFT                     24
#define AIPS_PACRA_TP1_WIDTH                     1
#define AIPS_PACRA_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP1_SHIFT))&AIPS_PACRA_TP1_MASK)
#define AIPS_PACRA_WP1_MASK                      0x2000000u
#define AIPS_PACRA_WP1_SHIFT                     25
#define AIPS_PACRA_WP1_WIDTH                     1
#define AIPS_PACRA_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP1_SHIFT))&AIPS_PACRA_WP1_MASK)
#define AIPS_PACRA_SP1_MASK                      0x4000000u
#define AIPS_PACRA_SP1_SHIFT                     26
#define AIPS_PACRA_SP1_WIDTH                     1
#define AIPS_PACRA_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP1_SHIFT))&AIPS_PACRA_SP1_MASK)
#define AIPS_PACRA_TP0_MASK                      0x10000000u
#define AIPS_PACRA_TP0_SHIFT                     28
#define AIPS_PACRA_TP0_WIDTH                     1
#define AIPS_PACRA_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_TP0_SHIFT))&AIPS_PACRA_TP0_MASK)
#define AIPS_PACRA_WP0_MASK                      0x20000000u
#define AIPS_PACRA_WP0_SHIFT                     29
#define AIPS_PACRA_WP0_WIDTH                     1
#define AIPS_PACRA_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_WP0_SHIFT))&AIPS_PACRA_WP0_MASK)
#define AIPS_PACRA_SP0_MASK                      0x40000000u
#define AIPS_PACRA_SP0_SHIFT                     30
#define AIPS_PACRA_SP0_WIDTH                     1
#define AIPS_PACRA_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRA_SP0_SHIFT))&AIPS_PACRA_SP0_MASK)
/* PACRB Bit Fields */
#define AIPS_PACRB_TP7_MASK                      0x1u
#define AIPS_PACRB_TP7_SHIFT                     0
#define AIPS_PACRB_TP7_WIDTH                     1
#define AIPS_PACRB_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP7_SHIFT))&AIPS_PACRB_TP7_MASK)
#define AIPS_PACRB_WP7_MASK                      0x2u
#define AIPS_PACRB_WP7_SHIFT                     1
#define AIPS_PACRB_WP7_WIDTH                     1
#define AIPS_PACRB_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP7_SHIFT))&AIPS_PACRB_WP7_MASK)
#define AIPS_PACRB_SP7_MASK                      0x4u
#define AIPS_PACRB_SP7_SHIFT                     2
#define AIPS_PACRB_SP7_WIDTH                     1
#define AIPS_PACRB_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP7_SHIFT))&AIPS_PACRB_SP7_MASK)
#define AIPS_PACRB_TP6_MASK                      0x10u
#define AIPS_PACRB_TP6_SHIFT                     4
#define AIPS_PACRB_TP6_WIDTH                     1
#define AIPS_PACRB_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP6_SHIFT))&AIPS_PACRB_TP6_MASK)
#define AIPS_PACRB_WP6_MASK                      0x20u
#define AIPS_PACRB_WP6_SHIFT                     5
#define AIPS_PACRB_WP6_WIDTH                     1
#define AIPS_PACRB_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP6_SHIFT))&AIPS_PACRB_WP6_MASK)
#define AIPS_PACRB_SP6_MASK                      0x40u
#define AIPS_PACRB_SP6_SHIFT                     6
#define AIPS_PACRB_SP6_WIDTH                     1
#define AIPS_PACRB_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP6_SHIFT))&AIPS_PACRB_SP6_MASK)
#define AIPS_PACRB_TP5_MASK                      0x100u
#define AIPS_PACRB_TP5_SHIFT                     8
#define AIPS_PACRB_TP5_WIDTH                     1
#define AIPS_PACRB_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP5_SHIFT))&AIPS_PACRB_TP5_MASK)
#define AIPS_PACRB_WP5_MASK                      0x200u
#define AIPS_PACRB_WP5_SHIFT                     9
#define AIPS_PACRB_WP5_WIDTH                     1
#define AIPS_PACRB_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP5_SHIFT))&AIPS_PACRB_WP5_MASK)
#define AIPS_PACRB_SP5_MASK                      0x400u
#define AIPS_PACRB_SP5_SHIFT                     10
#define AIPS_PACRB_SP5_WIDTH                     1
#define AIPS_PACRB_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP5_SHIFT))&AIPS_PACRB_SP5_MASK)
#define AIPS_PACRB_TP4_MASK                      0x1000u
#define AIPS_PACRB_TP4_SHIFT                     12
#define AIPS_PACRB_TP4_WIDTH                     1
#define AIPS_PACRB_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP4_SHIFT))&AIPS_PACRB_TP4_MASK)
#define AIPS_PACRB_WP4_MASK                      0x2000u
#define AIPS_PACRB_WP4_SHIFT                     13
#define AIPS_PACRB_WP4_WIDTH                     1
#define AIPS_PACRB_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP4_SHIFT))&AIPS_PACRB_WP4_MASK)
#define AIPS_PACRB_SP4_MASK                      0x4000u
#define AIPS_PACRB_SP4_SHIFT                     14
#define AIPS_PACRB_SP4_WIDTH                     1
#define AIPS_PACRB_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP4_SHIFT))&AIPS_PACRB_SP4_MASK)
#define AIPS_PACRB_TP3_MASK                      0x10000u
#define AIPS_PACRB_TP3_SHIFT                     16
#define AIPS_PACRB_TP3_WIDTH                     1
#define AIPS_PACRB_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP3_SHIFT))&AIPS_PACRB_TP3_MASK)
#define AIPS_PACRB_WP3_MASK                      0x20000u
#define AIPS_PACRB_WP3_SHIFT                     17
#define AIPS_PACRB_WP3_WIDTH                     1
#define AIPS_PACRB_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP3_SHIFT))&AIPS_PACRB_WP3_MASK)
#define AIPS_PACRB_SP3_MASK                      0x40000u
#define AIPS_PACRB_SP3_SHIFT                     18
#define AIPS_PACRB_SP3_WIDTH                     1
#define AIPS_PACRB_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP3_SHIFT))&AIPS_PACRB_SP3_MASK)
#define AIPS_PACRB_TP2_MASK                      0x100000u
#define AIPS_PACRB_TP2_SHIFT                     20
#define AIPS_PACRB_TP2_WIDTH                     1
#define AIPS_PACRB_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP2_SHIFT))&AIPS_PACRB_TP2_MASK)
#define AIPS_PACRB_WP2_MASK                      0x200000u
#define AIPS_PACRB_WP2_SHIFT                     21
#define AIPS_PACRB_WP2_WIDTH                     1
#define AIPS_PACRB_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP2_SHIFT))&AIPS_PACRB_WP2_MASK)
#define AIPS_PACRB_SP2_MASK                      0x400000u
#define AIPS_PACRB_SP2_SHIFT                     22
#define AIPS_PACRB_SP2_WIDTH                     1
#define AIPS_PACRB_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP2_SHIFT))&AIPS_PACRB_SP2_MASK)
#define AIPS_PACRB_TP1_MASK                      0x1000000u
#define AIPS_PACRB_TP1_SHIFT                     24
#define AIPS_PACRB_TP1_WIDTH                     1
#define AIPS_PACRB_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP1_SHIFT))&AIPS_PACRB_TP1_MASK)
#define AIPS_PACRB_WP1_MASK                      0x2000000u
#define AIPS_PACRB_WP1_SHIFT                     25
#define AIPS_PACRB_WP1_WIDTH                     1
#define AIPS_PACRB_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP1_SHIFT))&AIPS_PACRB_WP1_MASK)
#define AIPS_PACRB_SP1_MASK                      0x4000000u
#define AIPS_PACRB_SP1_SHIFT                     26
#define AIPS_PACRB_SP1_WIDTH                     1
#define AIPS_PACRB_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP1_SHIFT))&AIPS_PACRB_SP1_MASK)
#define AIPS_PACRB_TP0_MASK                      0x10000000u
#define AIPS_PACRB_TP0_SHIFT                     28
#define AIPS_PACRB_TP0_WIDTH                     1
#define AIPS_PACRB_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_TP0_SHIFT))&AIPS_PACRB_TP0_MASK)
#define AIPS_PACRB_WP0_MASK                      0x20000000u
#define AIPS_PACRB_WP0_SHIFT                     29
#define AIPS_PACRB_WP0_WIDTH                     1
#define AIPS_PACRB_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_WP0_SHIFT))&AIPS_PACRB_WP0_MASK)
#define AIPS_PACRB_SP0_MASK                      0x40000000u
#define AIPS_PACRB_SP0_SHIFT                     30
#define AIPS_PACRB_SP0_WIDTH                     1
#define AIPS_PACRB_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRB_SP0_SHIFT))&AIPS_PACRB_SP0_MASK)
/* PACRC Bit Fields */
#define AIPS_PACRC_TP7_MASK                      0x1u
#define AIPS_PACRC_TP7_SHIFT                     0
#define AIPS_PACRC_TP7_WIDTH                     1
#define AIPS_PACRC_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP7_SHIFT))&AIPS_PACRC_TP7_MASK)
#define AIPS_PACRC_WP7_MASK                      0x2u
#define AIPS_PACRC_WP7_SHIFT                     1
#define AIPS_PACRC_WP7_WIDTH                     1
#define AIPS_PACRC_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP7_SHIFT))&AIPS_PACRC_WP7_MASK)
#define AIPS_PACRC_SP7_MASK                      0x4u
#define AIPS_PACRC_SP7_SHIFT                     2
#define AIPS_PACRC_SP7_WIDTH                     1
#define AIPS_PACRC_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP7_SHIFT))&AIPS_PACRC_SP7_MASK)
#define AIPS_PACRC_TP6_MASK                      0x10u
#define AIPS_PACRC_TP6_SHIFT                     4
#define AIPS_PACRC_TP6_WIDTH                     1
#define AIPS_PACRC_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP6_SHIFT))&AIPS_PACRC_TP6_MASK)
#define AIPS_PACRC_WP6_MASK                      0x20u
#define AIPS_PACRC_WP6_SHIFT                     5
#define AIPS_PACRC_WP6_WIDTH                     1
#define AIPS_PACRC_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP6_SHIFT))&AIPS_PACRC_WP6_MASK)
#define AIPS_PACRC_SP6_MASK                      0x40u
#define AIPS_PACRC_SP6_SHIFT                     6
#define AIPS_PACRC_SP6_WIDTH                     1
#define AIPS_PACRC_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP6_SHIFT))&AIPS_PACRC_SP6_MASK)
#define AIPS_PACRC_TP5_MASK                      0x100u
#define AIPS_PACRC_TP5_SHIFT                     8
#define AIPS_PACRC_TP5_WIDTH                     1
#define AIPS_PACRC_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP5_SHIFT))&AIPS_PACRC_TP5_MASK)
#define AIPS_PACRC_WP5_MASK                      0x200u
#define AIPS_PACRC_WP5_SHIFT                     9
#define AIPS_PACRC_WP5_WIDTH                     1
#define AIPS_PACRC_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP5_SHIFT))&AIPS_PACRC_WP5_MASK)
#define AIPS_PACRC_SP5_MASK                      0x400u
#define AIPS_PACRC_SP5_SHIFT                     10
#define AIPS_PACRC_SP5_WIDTH                     1
#define AIPS_PACRC_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP5_SHIFT))&AIPS_PACRC_SP5_MASK)
#define AIPS_PACRC_TP4_MASK                      0x1000u
#define AIPS_PACRC_TP4_SHIFT                     12
#define AIPS_PACRC_TP4_WIDTH                     1
#define AIPS_PACRC_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP4_SHIFT))&AIPS_PACRC_TP4_MASK)
#define AIPS_PACRC_WP4_MASK                      0x2000u
#define AIPS_PACRC_WP4_SHIFT                     13
#define AIPS_PACRC_WP4_WIDTH                     1
#define AIPS_PACRC_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP4_SHIFT))&AIPS_PACRC_WP4_MASK)
#define AIPS_PACRC_SP4_MASK                      0x4000u
#define AIPS_PACRC_SP4_SHIFT                     14
#define AIPS_PACRC_SP4_WIDTH                     1
#define AIPS_PACRC_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP4_SHIFT))&AIPS_PACRC_SP4_MASK)
#define AIPS_PACRC_TP3_MASK                      0x10000u
#define AIPS_PACRC_TP3_SHIFT                     16
#define AIPS_PACRC_TP3_WIDTH                     1
#define AIPS_PACRC_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP3_SHIFT))&AIPS_PACRC_TP3_MASK)
#define AIPS_PACRC_WP3_MASK                      0x20000u
#define AIPS_PACRC_WP3_SHIFT                     17
#define AIPS_PACRC_WP3_WIDTH                     1
#define AIPS_PACRC_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP3_SHIFT))&AIPS_PACRC_WP3_MASK)
#define AIPS_PACRC_SP3_MASK                      0x40000u
#define AIPS_PACRC_SP3_SHIFT                     18
#define AIPS_PACRC_SP3_WIDTH                     1
#define AIPS_PACRC_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP3_SHIFT))&AIPS_PACRC_SP3_MASK)
#define AIPS_PACRC_TP2_MASK                      0x100000u
#define AIPS_PACRC_TP2_SHIFT                     20
#define AIPS_PACRC_TP2_WIDTH                     1
#define AIPS_PACRC_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP2_SHIFT))&AIPS_PACRC_TP2_MASK)
#define AIPS_PACRC_WP2_MASK                      0x200000u
#define AIPS_PACRC_WP2_SHIFT                     21
#define AIPS_PACRC_WP2_WIDTH                     1
#define AIPS_PACRC_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP2_SHIFT))&AIPS_PACRC_WP2_MASK)
#define AIPS_PACRC_SP2_MASK                      0x400000u
#define AIPS_PACRC_SP2_SHIFT                     22
#define AIPS_PACRC_SP2_WIDTH                     1
#define AIPS_PACRC_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP2_SHIFT))&AIPS_PACRC_SP2_MASK)
#define AIPS_PACRC_TP1_MASK                      0x1000000u
#define AIPS_PACRC_TP1_SHIFT                     24
#define AIPS_PACRC_TP1_WIDTH                     1
#define AIPS_PACRC_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP1_SHIFT))&AIPS_PACRC_TP1_MASK)
#define AIPS_PACRC_WP1_MASK                      0x2000000u
#define AIPS_PACRC_WP1_SHIFT                     25
#define AIPS_PACRC_WP1_WIDTH                     1
#define AIPS_PACRC_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP1_SHIFT))&AIPS_PACRC_WP1_MASK)
#define AIPS_PACRC_SP1_MASK                      0x4000000u
#define AIPS_PACRC_SP1_SHIFT                     26
#define AIPS_PACRC_SP1_WIDTH                     1
#define AIPS_PACRC_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP1_SHIFT))&AIPS_PACRC_SP1_MASK)
#define AIPS_PACRC_TP0_MASK                      0x10000000u
#define AIPS_PACRC_TP0_SHIFT                     28
#define AIPS_PACRC_TP0_WIDTH                     1
#define AIPS_PACRC_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_TP0_SHIFT))&AIPS_PACRC_TP0_MASK)
#define AIPS_PACRC_WP0_MASK                      0x20000000u
#define AIPS_PACRC_WP0_SHIFT                     29
#define AIPS_PACRC_WP0_WIDTH                     1
#define AIPS_PACRC_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_WP0_SHIFT))&AIPS_PACRC_WP0_MASK)
#define AIPS_PACRC_SP0_MASK                      0x40000000u
#define AIPS_PACRC_SP0_SHIFT                     30
#define AIPS_PACRC_SP0_WIDTH                     1
#define AIPS_PACRC_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRC_SP0_SHIFT))&AIPS_PACRC_SP0_MASK)
/* PACRD Bit Fields */
#define AIPS_PACRD_TP7_MASK                      0x1u
#define AIPS_PACRD_TP7_SHIFT                     0
#define AIPS_PACRD_TP7_WIDTH                     1
#define AIPS_PACRD_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP7_SHIFT))&AIPS_PACRD_TP7_MASK)
#define AIPS_PACRD_WP7_MASK                      0x2u
#define AIPS_PACRD_WP7_SHIFT                     1
#define AIPS_PACRD_WP7_WIDTH                     1
#define AIPS_PACRD_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP7_SHIFT))&AIPS_PACRD_WP7_MASK)
#define AIPS_PACRD_SP7_MASK                      0x4u
#define AIPS_PACRD_SP7_SHIFT                     2
#define AIPS_PACRD_SP7_WIDTH                     1
#define AIPS_PACRD_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP7_SHIFT))&AIPS_PACRD_SP7_MASK)
#define AIPS_PACRD_TP6_MASK                      0x10u
#define AIPS_PACRD_TP6_SHIFT                     4
#define AIPS_PACRD_TP6_WIDTH                     1
#define AIPS_PACRD_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP6_SHIFT))&AIPS_PACRD_TP6_MASK)
#define AIPS_PACRD_WP6_MASK                      0x20u
#define AIPS_PACRD_WP6_SHIFT                     5
#define AIPS_PACRD_WP6_WIDTH                     1
#define AIPS_PACRD_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP6_SHIFT))&AIPS_PACRD_WP6_MASK)
#define AIPS_PACRD_SP6_MASK                      0x40u
#define AIPS_PACRD_SP6_SHIFT                     6
#define AIPS_PACRD_SP6_WIDTH                     1
#define AIPS_PACRD_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP6_SHIFT))&AIPS_PACRD_SP6_MASK)
#define AIPS_PACRD_TP5_MASK                      0x100u
#define AIPS_PACRD_TP5_SHIFT                     8
#define AIPS_PACRD_TP5_WIDTH                     1
#define AIPS_PACRD_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP5_SHIFT))&AIPS_PACRD_TP5_MASK)
#define AIPS_PACRD_WP5_MASK                      0x200u
#define AIPS_PACRD_WP5_SHIFT                     9
#define AIPS_PACRD_WP5_WIDTH                     1
#define AIPS_PACRD_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP5_SHIFT))&AIPS_PACRD_WP5_MASK)
#define AIPS_PACRD_SP5_MASK                      0x400u
#define AIPS_PACRD_SP5_SHIFT                     10
#define AIPS_PACRD_SP5_WIDTH                     1
#define AIPS_PACRD_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP5_SHIFT))&AIPS_PACRD_SP5_MASK)
#define AIPS_PACRD_TP4_MASK                      0x1000u
#define AIPS_PACRD_TP4_SHIFT                     12
#define AIPS_PACRD_TP4_WIDTH                     1
#define AIPS_PACRD_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP4_SHIFT))&AIPS_PACRD_TP4_MASK)
#define AIPS_PACRD_WP4_MASK                      0x2000u
#define AIPS_PACRD_WP4_SHIFT                     13
#define AIPS_PACRD_WP4_WIDTH                     1
#define AIPS_PACRD_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP4_SHIFT))&AIPS_PACRD_WP4_MASK)
#define AIPS_PACRD_SP4_MASK                      0x4000u
#define AIPS_PACRD_SP4_SHIFT                     14
#define AIPS_PACRD_SP4_WIDTH                     1
#define AIPS_PACRD_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP4_SHIFT))&AIPS_PACRD_SP4_MASK)
#define AIPS_PACRD_TP3_MASK                      0x10000u
#define AIPS_PACRD_TP3_SHIFT                     16
#define AIPS_PACRD_TP3_WIDTH                     1
#define AIPS_PACRD_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP3_SHIFT))&AIPS_PACRD_TP3_MASK)
#define AIPS_PACRD_WP3_MASK                      0x20000u
#define AIPS_PACRD_WP3_SHIFT                     17
#define AIPS_PACRD_WP3_WIDTH                     1
#define AIPS_PACRD_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP3_SHIFT))&AIPS_PACRD_WP3_MASK)
#define AIPS_PACRD_SP3_MASK                      0x40000u
#define AIPS_PACRD_SP3_SHIFT                     18
#define AIPS_PACRD_SP3_WIDTH                     1
#define AIPS_PACRD_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP3_SHIFT))&AIPS_PACRD_SP3_MASK)
#define AIPS_PACRD_TP2_MASK                      0x100000u
#define AIPS_PACRD_TP2_SHIFT                     20
#define AIPS_PACRD_TP2_WIDTH                     1
#define AIPS_PACRD_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP2_SHIFT))&AIPS_PACRD_TP2_MASK)
#define AIPS_PACRD_WP2_MASK                      0x200000u
#define AIPS_PACRD_WP2_SHIFT                     21
#define AIPS_PACRD_WP2_WIDTH                     1
#define AIPS_PACRD_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP2_SHIFT))&AIPS_PACRD_WP2_MASK)
#define AIPS_PACRD_SP2_MASK                      0x400000u
#define AIPS_PACRD_SP2_SHIFT                     22
#define AIPS_PACRD_SP2_WIDTH                     1
#define AIPS_PACRD_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP2_SHIFT))&AIPS_PACRD_SP2_MASK)
#define AIPS_PACRD_TP1_MASK                      0x1000000u
#define AIPS_PACRD_TP1_SHIFT                     24
#define AIPS_PACRD_TP1_WIDTH                     1
#define AIPS_PACRD_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP1_SHIFT))&AIPS_PACRD_TP1_MASK)
#define AIPS_PACRD_WP1_MASK                      0x2000000u
#define AIPS_PACRD_WP1_SHIFT                     25
#define AIPS_PACRD_WP1_WIDTH                     1
#define AIPS_PACRD_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP1_SHIFT))&AIPS_PACRD_WP1_MASK)
#define AIPS_PACRD_SP1_MASK                      0x4000000u
#define AIPS_PACRD_SP1_SHIFT                     26
#define AIPS_PACRD_SP1_WIDTH                     1
#define AIPS_PACRD_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP1_SHIFT))&AIPS_PACRD_SP1_MASK)
#define AIPS_PACRD_TP0_MASK                      0x10000000u
#define AIPS_PACRD_TP0_SHIFT                     28
#define AIPS_PACRD_TP0_WIDTH                     1
#define AIPS_PACRD_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_TP0_SHIFT))&AIPS_PACRD_TP0_MASK)
#define AIPS_PACRD_WP0_MASK                      0x20000000u
#define AIPS_PACRD_WP0_SHIFT                     29
#define AIPS_PACRD_WP0_WIDTH                     1
#define AIPS_PACRD_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_WP0_SHIFT))&AIPS_PACRD_WP0_MASK)
#define AIPS_PACRD_SP0_MASK                      0x40000000u
#define AIPS_PACRD_SP0_SHIFT                     30
#define AIPS_PACRD_SP0_WIDTH                     1
#define AIPS_PACRD_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRD_SP0_SHIFT))&AIPS_PACRD_SP0_MASK)
/* PACRE Bit Fields */
#define AIPS_PACRE_TP7_MASK                      0x1u
#define AIPS_PACRE_TP7_SHIFT                     0
#define AIPS_PACRE_TP7_WIDTH                     1
#define AIPS_PACRE_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP7_SHIFT))&AIPS_PACRE_TP7_MASK)
#define AIPS_PACRE_WP7_MASK                      0x2u
#define AIPS_PACRE_WP7_SHIFT                     1
#define AIPS_PACRE_WP7_WIDTH                     1
#define AIPS_PACRE_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP7_SHIFT))&AIPS_PACRE_WP7_MASK)
#define AIPS_PACRE_SP7_MASK                      0x4u
#define AIPS_PACRE_SP7_SHIFT                     2
#define AIPS_PACRE_SP7_WIDTH                     1
#define AIPS_PACRE_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP7_SHIFT))&AIPS_PACRE_SP7_MASK)
#define AIPS_PACRE_TP6_MASK                      0x10u
#define AIPS_PACRE_TP6_SHIFT                     4
#define AIPS_PACRE_TP6_WIDTH                     1
#define AIPS_PACRE_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP6_SHIFT))&AIPS_PACRE_TP6_MASK)
#define AIPS_PACRE_WP6_MASK                      0x20u
#define AIPS_PACRE_WP6_SHIFT                     5
#define AIPS_PACRE_WP6_WIDTH                     1
#define AIPS_PACRE_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP6_SHIFT))&AIPS_PACRE_WP6_MASK)
#define AIPS_PACRE_SP6_MASK                      0x40u
#define AIPS_PACRE_SP6_SHIFT                     6
#define AIPS_PACRE_SP6_WIDTH                     1
#define AIPS_PACRE_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP6_SHIFT))&AIPS_PACRE_SP6_MASK)
#define AIPS_PACRE_TP5_MASK                      0x100u
#define AIPS_PACRE_TP5_SHIFT                     8
#define AIPS_PACRE_TP5_WIDTH                     1
#define AIPS_PACRE_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP5_SHIFT))&AIPS_PACRE_TP5_MASK)
#define AIPS_PACRE_WP5_MASK                      0x200u
#define AIPS_PACRE_WP5_SHIFT                     9
#define AIPS_PACRE_WP5_WIDTH                     1
#define AIPS_PACRE_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP5_SHIFT))&AIPS_PACRE_WP5_MASK)
#define AIPS_PACRE_SP5_MASK                      0x400u
#define AIPS_PACRE_SP5_SHIFT                     10
#define AIPS_PACRE_SP5_WIDTH                     1
#define AIPS_PACRE_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP5_SHIFT))&AIPS_PACRE_SP5_MASK)
#define AIPS_PACRE_TP4_MASK                      0x1000u
#define AIPS_PACRE_TP4_SHIFT                     12
#define AIPS_PACRE_TP4_WIDTH                     1
#define AIPS_PACRE_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP4_SHIFT))&AIPS_PACRE_TP4_MASK)
#define AIPS_PACRE_WP4_MASK                      0x2000u
#define AIPS_PACRE_WP4_SHIFT                     13
#define AIPS_PACRE_WP4_WIDTH                     1
#define AIPS_PACRE_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP4_SHIFT))&AIPS_PACRE_WP4_MASK)
#define AIPS_PACRE_SP4_MASK                      0x4000u
#define AIPS_PACRE_SP4_SHIFT                     14
#define AIPS_PACRE_SP4_WIDTH                     1
#define AIPS_PACRE_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP4_SHIFT))&AIPS_PACRE_SP4_MASK)
#define AIPS_PACRE_TP3_MASK                      0x10000u
#define AIPS_PACRE_TP3_SHIFT                     16
#define AIPS_PACRE_TP3_WIDTH                     1
#define AIPS_PACRE_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP3_SHIFT))&AIPS_PACRE_TP3_MASK)
#define AIPS_PACRE_WP3_MASK                      0x20000u
#define AIPS_PACRE_WP3_SHIFT                     17
#define AIPS_PACRE_WP3_WIDTH                     1
#define AIPS_PACRE_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP3_SHIFT))&AIPS_PACRE_WP3_MASK)
#define AIPS_PACRE_SP3_MASK                      0x40000u
#define AIPS_PACRE_SP3_SHIFT                     18
#define AIPS_PACRE_SP3_WIDTH                     1
#define AIPS_PACRE_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP3_SHIFT))&AIPS_PACRE_SP3_MASK)
#define AIPS_PACRE_TP2_MASK                      0x100000u
#define AIPS_PACRE_TP2_SHIFT                     20
#define AIPS_PACRE_TP2_WIDTH                     1
#define AIPS_PACRE_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP2_SHIFT))&AIPS_PACRE_TP2_MASK)
#define AIPS_PACRE_WP2_MASK                      0x200000u
#define AIPS_PACRE_WP2_SHIFT                     21
#define AIPS_PACRE_WP2_WIDTH                     1
#define AIPS_PACRE_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP2_SHIFT))&AIPS_PACRE_WP2_MASK)
#define AIPS_PACRE_SP2_MASK                      0x400000u
#define AIPS_PACRE_SP2_SHIFT                     22
#define AIPS_PACRE_SP2_WIDTH                     1
#define AIPS_PACRE_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP2_SHIFT))&AIPS_PACRE_SP2_MASK)
#define AIPS_PACRE_TP1_MASK                      0x1000000u
#define AIPS_PACRE_TP1_SHIFT                     24
#define AIPS_PACRE_TP1_WIDTH                     1
#define AIPS_PACRE_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP1_SHIFT))&AIPS_PACRE_TP1_MASK)
#define AIPS_PACRE_WP1_MASK                      0x2000000u
#define AIPS_PACRE_WP1_SHIFT                     25
#define AIPS_PACRE_WP1_WIDTH                     1
#define AIPS_PACRE_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP1_SHIFT))&AIPS_PACRE_WP1_MASK)
#define AIPS_PACRE_SP1_MASK                      0x4000000u
#define AIPS_PACRE_SP1_SHIFT                     26
#define AIPS_PACRE_SP1_WIDTH                     1
#define AIPS_PACRE_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP1_SHIFT))&AIPS_PACRE_SP1_MASK)
#define AIPS_PACRE_TP0_MASK                      0x10000000u
#define AIPS_PACRE_TP0_SHIFT                     28
#define AIPS_PACRE_TP0_WIDTH                     1
#define AIPS_PACRE_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_TP0_SHIFT))&AIPS_PACRE_TP0_MASK)
#define AIPS_PACRE_WP0_MASK                      0x20000000u
#define AIPS_PACRE_WP0_SHIFT                     29
#define AIPS_PACRE_WP0_WIDTH                     1
#define AIPS_PACRE_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_WP0_SHIFT))&AIPS_PACRE_WP0_MASK)
#define AIPS_PACRE_SP0_MASK                      0x40000000u
#define AIPS_PACRE_SP0_SHIFT                     30
#define AIPS_PACRE_SP0_WIDTH                     1
#define AIPS_PACRE_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRE_SP0_SHIFT))&AIPS_PACRE_SP0_MASK)
/* PACRF Bit Fields */
#define AIPS_PACRF_TP7_MASK                      0x1u
#define AIPS_PACRF_TP7_SHIFT                     0
#define AIPS_PACRF_TP7_WIDTH                     1
#define AIPS_PACRF_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP7_SHIFT))&AIPS_PACRF_TP7_MASK)
#define AIPS_PACRF_WP7_MASK                      0x2u
#define AIPS_PACRF_WP7_SHIFT                     1
#define AIPS_PACRF_WP7_WIDTH                     1
#define AIPS_PACRF_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP7_SHIFT))&AIPS_PACRF_WP7_MASK)
#define AIPS_PACRF_SP7_MASK                      0x4u
#define AIPS_PACRF_SP7_SHIFT                     2
#define AIPS_PACRF_SP7_WIDTH                     1
#define AIPS_PACRF_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP7_SHIFT))&AIPS_PACRF_SP7_MASK)
#define AIPS_PACRF_TP6_MASK                      0x10u
#define AIPS_PACRF_TP6_SHIFT                     4
#define AIPS_PACRF_TP6_WIDTH                     1
#define AIPS_PACRF_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP6_SHIFT))&AIPS_PACRF_TP6_MASK)
#define AIPS_PACRF_WP6_MASK                      0x20u
#define AIPS_PACRF_WP6_SHIFT                     5
#define AIPS_PACRF_WP6_WIDTH                     1
#define AIPS_PACRF_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP6_SHIFT))&AIPS_PACRF_WP6_MASK)
#define AIPS_PACRF_SP6_MASK                      0x40u
#define AIPS_PACRF_SP6_SHIFT                     6
#define AIPS_PACRF_SP6_WIDTH                     1
#define AIPS_PACRF_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP6_SHIFT))&AIPS_PACRF_SP6_MASK)
#define AIPS_PACRF_TP5_MASK                      0x100u
#define AIPS_PACRF_TP5_SHIFT                     8
#define AIPS_PACRF_TP5_WIDTH                     1
#define AIPS_PACRF_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP5_SHIFT))&AIPS_PACRF_TP5_MASK)
#define AIPS_PACRF_WP5_MASK                      0x200u
#define AIPS_PACRF_WP5_SHIFT                     9
#define AIPS_PACRF_WP5_WIDTH                     1
#define AIPS_PACRF_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP5_SHIFT))&AIPS_PACRF_WP5_MASK)
#define AIPS_PACRF_SP5_MASK                      0x400u
#define AIPS_PACRF_SP5_SHIFT                     10
#define AIPS_PACRF_SP5_WIDTH                     1
#define AIPS_PACRF_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP5_SHIFT))&AIPS_PACRF_SP5_MASK)
#define AIPS_PACRF_TP4_MASK                      0x1000u
#define AIPS_PACRF_TP4_SHIFT                     12
#define AIPS_PACRF_TP4_WIDTH                     1
#define AIPS_PACRF_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP4_SHIFT))&AIPS_PACRF_TP4_MASK)
#define AIPS_PACRF_WP4_MASK                      0x2000u
#define AIPS_PACRF_WP4_SHIFT                     13
#define AIPS_PACRF_WP4_WIDTH                     1
#define AIPS_PACRF_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP4_SHIFT))&AIPS_PACRF_WP4_MASK)
#define AIPS_PACRF_SP4_MASK                      0x4000u
#define AIPS_PACRF_SP4_SHIFT                     14
#define AIPS_PACRF_SP4_WIDTH                     1
#define AIPS_PACRF_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP4_SHIFT))&AIPS_PACRF_SP4_MASK)
#define AIPS_PACRF_TP3_MASK                      0x10000u
#define AIPS_PACRF_TP3_SHIFT                     16
#define AIPS_PACRF_TP3_WIDTH                     1
#define AIPS_PACRF_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP3_SHIFT))&AIPS_PACRF_TP3_MASK)
#define AIPS_PACRF_WP3_MASK                      0x20000u
#define AIPS_PACRF_WP3_SHIFT                     17
#define AIPS_PACRF_WP3_WIDTH                     1
#define AIPS_PACRF_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP3_SHIFT))&AIPS_PACRF_WP3_MASK)
#define AIPS_PACRF_SP3_MASK                      0x40000u
#define AIPS_PACRF_SP3_SHIFT                     18
#define AIPS_PACRF_SP3_WIDTH                     1
#define AIPS_PACRF_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP3_SHIFT))&AIPS_PACRF_SP3_MASK)
#define AIPS_PACRF_TP2_MASK                      0x100000u
#define AIPS_PACRF_TP2_SHIFT                     20
#define AIPS_PACRF_TP2_WIDTH                     1
#define AIPS_PACRF_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP2_SHIFT))&AIPS_PACRF_TP2_MASK)
#define AIPS_PACRF_WP2_MASK                      0x200000u
#define AIPS_PACRF_WP2_SHIFT                     21
#define AIPS_PACRF_WP2_WIDTH                     1
#define AIPS_PACRF_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP2_SHIFT))&AIPS_PACRF_WP2_MASK)
#define AIPS_PACRF_SP2_MASK                      0x400000u
#define AIPS_PACRF_SP2_SHIFT                     22
#define AIPS_PACRF_SP2_WIDTH                     1
#define AIPS_PACRF_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP2_SHIFT))&AIPS_PACRF_SP2_MASK)
#define AIPS_PACRF_TP1_MASK                      0x1000000u
#define AIPS_PACRF_TP1_SHIFT                     24
#define AIPS_PACRF_TP1_WIDTH                     1
#define AIPS_PACRF_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP1_SHIFT))&AIPS_PACRF_TP1_MASK)
#define AIPS_PACRF_WP1_MASK                      0x2000000u
#define AIPS_PACRF_WP1_SHIFT                     25
#define AIPS_PACRF_WP1_WIDTH                     1
#define AIPS_PACRF_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP1_SHIFT))&AIPS_PACRF_WP1_MASK)
#define AIPS_PACRF_SP1_MASK                      0x4000000u
#define AIPS_PACRF_SP1_SHIFT                     26
#define AIPS_PACRF_SP1_WIDTH                     1
#define AIPS_PACRF_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP1_SHIFT))&AIPS_PACRF_SP1_MASK)
#define AIPS_PACRF_TP0_MASK                      0x10000000u
#define AIPS_PACRF_TP0_SHIFT                     28
#define AIPS_PACRF_TP0_WIDTH                     1
#define AIPS_PACRF_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_TP0_SHIFT))&AIPS_PACRF_TP0_MASK)
#define AIPS_PACRF_WP0_MASK                      0x20000000u
#define AIPS_PACRF_WP0_SHIFT                     29
#define AIPS_PACRF_WP0_WIDTH                     1
#define AIPS_PACRF_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_WP0_SHIFT))&AIPS_PACRF_WP0_MASK)
#define AIPS_PACRF_SP0_MASK                      0x40000000u
#define AIPS_PACRF_SP0_SHIFT                     30
#define AIPS_PACRF_SP0_WIDTH                     1
#define AIPS_PACRF_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRF_SP0_SHIFT))&AIPS_PACRF_SP0_MASK)
/* PACRG Bit Fields */
#define AIPS_PACRG_TP7_MASK                      0x1u
#define AIPS_PACRG_TP7_SHIFT                     0
#define AIPS_PACRG_TP7_WIDTH                     1
#define AIPS_PACRG_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP7_SHIFT))&AIPS_PACRG_TP7_MASK)
#define AIPS_PACRG_WP7_MASK                      0x2u
#define AIPS_PACRG_WP7_SHIFT                     1
#define AIPS_PACRG_WP7_WIDTH                     1
#define AIPS_PACRG_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP7_SHIFT))&AIPS_PACRG_WP7_MASK)
#define AIPS_PACRG_SP7_MASK                      0x4u
#define AIPS_PACRG_SP7_SHIFT                     2
#define AIPS_PACRG_SP7_WIDTH                     1
#define AIPS_PACRG_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP7_SHIFT))&AIPS_PACRG_SP7_MASK)
#define AIPS_PACRG_TP6_MASK                      0x10u
#define AIPS_PACRG_TP6_SHIFT                     4
#define AIPS_PACRG_TP6_WIDTH                     1
#define AIPS_PACRG_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP6_SHIFT))&AIPS_PACRG_TP6_MASK)
#define AIPS_PACRG_WP6_MASK                      0x20u
#define AIPS_PACRG_WP6_SHIFT                     5
#define AIPS_PACRG_WP6_WIDTH                     1
#define AIPS_PACRG_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP6_SHIFT))&AIPS_PACRG_WP6_MASK)
#define AIPS_PACRG_SP6_MASK                      0x40u
#define AIPS_PACRG_SP6_SHIFT                     6
#define AIPS_PACRG_SP6_WIDTH                     1
#define AIPS_PACRG_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP6_SHIFT))&AIPS_PACRG_SP6_MASK)
#define AIPS_PACRG_TP5_MASK                      0x100u
#define AIPS_PACRG_TP5_SHIFT                     8
#define AIPS_PACRG_TP5_WIDTH                     1
#define AIPS_PACRG_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP5_SHIFT))&AIPS_PACRG_TP5_MASK)
#define AIPS_PACRG_WP5_MASK                      0x200u
#define AIPS_PACRG_WP5_SHIFT                     9
#define AIPS_PACRG_WP5_WIDTH                     1
#define AIPS_PACRG_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP5_SHIFT))&AIPS_PACRG_WP5_MASK)
#define AIPS_PACRG_SP5_MASK                      0x400u
#define AIPS_PACRG_SP5_SHIFT                     10
#define AIPS_PACRG_SP5_WIDTH                     1
#define AIPS_PACRG_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP5_SHIFT))&AIPS_PACRG_SP5_MASK)
#define AIPS_PACRG_TP4_MASK                      0x1000u
#define AIPS_PACRG_TP4_SHIFT                     12
#define AIPS_PACRG_TP4_WIDTH                     1
#define AIPS_PACRG_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP4_SHIFT))&AIPS_PACRG_TP4_MASK)
#define AIPS_PACRG_WP4_MASK                      0x2000u
#define AIPS_PACRG_WP4_SHIFT                     13
#define AIPS_PACRG_WP4_WIDTH                     1
#define AIPS_PACRG_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP4_SHIFT))&AIPS_PACRG_WP4_MASK)
#define AIPS_PACRG_SP4_MASK                      0x4000u
#define AIPS_PACRG_SP4_SHIFT                     14
#define AIPS_PACRG_SP4_WIDTH                     1
#define AIPS_PACRG_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP4_SHIFT))&AIPS_PACRG_SP4_MASK)
#define AIPS_PACRG_TP3_MASK                      0x10000u
#define AIPS_PACRG_TP3_SHIFT                     16
#define AIPS_PACRG_TP3_WIDTH                     1
#define AIPS_PACRG_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP3_SHIFT))&AIPS_PACRG_TP3_MASK)
#define AIPS_PACRG_WP3_MASK                      0x20000u
#define AIPS_PACRG_WP3_SHIFT                     17
#define AIPS_PACRG_WP3_WIDTH                     1
#define AIPS_PACRG_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP3_SHIFT))&AIPS_PACRG_WP3_MASK)
#define AIPS_PACRG_SP3_MASK                      0x40000u
#define AIPS_PACRG_SP3_SHIFT                     18
#define AIPS_PACRG_SP3_WIDTH                     1
#define AIPS_PACRG_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP3_SHIFT))&AIPS_PACRG_SP3_MASK)
#define AIPS_PACRG_TP2_MASK                      0x100000u
#define AIPS_PACRG_TP2_SHIFT                     20
#define AIPS_PACRG_TP2_WIDTH                     1
#define AIPS_PACRG_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP2_SHIFT))&AIPS_PACRG_TP2_MASK)
#define AIPS_PACRG_WP2_MASK                      0x200000u
#define AIPS_PACRG_WP2_SHIFT                     21
#define AIPS_PACRG_WP2_WIDTH                     1
#define AIPS_PACRG_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP2_SHIFT))&AIPS_PACRG_WP2_MASK)
#define AIPS_PACRG_SP2_MASK                      0x400000u
#define AIPS_PACRG_SP2_SHIFT                     22
#define AIPS_PACRG_SP2_WIDTH                     1
#define AIPS_PACRG_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP2_SHIFT))&AIPS_PACRG_SP2_MASK)
#define AIPS_PACRG_TP1_MASK                      0x1000000u
#define AIPS_PACRG_TP1_SHIFT                     24
#define AIPS_PACRG_TP1_WIDTH                     1
#define AIPS_PACRG_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP1_SHIFT))&AIPS_PACRG_TP1_MASK)
#define AIPS_PACRG_WP1_MASK                      0x2000000u
#define AIPS_PACRG_WP1_SHIFT                     25
#define AIPS_PACRG_WP1_WIDTH                     1
#define AIPS_PACRG_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP1_SHIFT))&AIPS_PACRG_WP1_MASK)
#define AIPS_PACRG_SP1_MASK                      0x4000000u
#define AIPS_PACRG_SP1_SHIFT                     26
#define AIPS_PACRG_SP1_WIDTH                     1
#define AIPS_PACRG_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP1_SHIFT))&AIPS_PACRG_SP1_MASK)
#define AIPS_PACRG_TP0_MASK                      0x10000000u
#define AIPS_PACRG_TP0_SHIFT                     28
#define AIPS_PACRG_TP0_WIDTH                     1
#define AIPS_PACRG_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_TP0_SHIFT))&AIPS_PACRG_TP0_MASK)
#define AIPS_PACRG_WP0_MASK                      0x20000000u
#define AIPS_PACRG_WP0_SHIFT                     29
#define AIPS_PACRG_WP0_WIDTH                     1
#define AIPS_PACRG_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_WP0_SHIFT))&AIPS_PACRG_WP0_MASK)
#define AIPS_PACRG_SP0_MASK                      0x40000000u
#define AIPS_PACRG_SP0_SHIFT                     30
#define AIPS_PACRG_SP0_WIDTH                     1
#define AIPS_PACRG_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRG_SP0_SHIFT))&AIPS_PACRG_SP0_MASK)
/* PACRH Bit Fields */
#define AIPS_PACRH_TP7_MASK                      0x1u
#define AIPS_PACRH_TP7_SHIFT                     0
#define AIPS_PACRH_TP7_WIDTH                     1
#define AIPS_PACRH_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP7_SHIFT))&AIPS_PACRH_TP7_MASK)
#define AIPS_PACRH_WP7_MASK                      0x2u
#define AIPS_PACRH_WP7_SHIFT                     1
#define AIPS_PACRH_WP7_WIDTH                     1
#define AIPS_PACRH_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP7_SHIFT))&AIPS_PACRH_WP7_MASK)
#define AIPS_PACRH_SP7_MASK                      0x4u
#define AIPS_PACRH_SP7_SHIFT                     2
#define AIPS_PACRH_SP7_WIDTH                     1
#define AIPS_PACRH_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP7_SHIFT))&AIPS_PACRH_SP7_MASK)
#define AIPS_PACRH_TP6_MASK                      0x10u
#define AIPS_PACRH_TP6_SHIFT                     4
#define AIPS_PACRH_TP6_WIDTH                     1
#define AIPS_PACRH_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP6_SHIFT))&AIPS_PACRH_TP6_MASK)
#define AIPS_PACRH_WP6_MASK                      0x20u
#define AIPS_PACRH_WP6_SHIFT                     5
#define AIPS_PACRH_WP6_WIDTH                     1
#define AIPS_PACRH_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP6_SHIFT))&AIPS_PACRH_WP6_MASK)
#define AIPS_PACRH_SP6_MASK                      0x40u
#define AIPS_PACRH_SP6_SHIFT                     6
#define AIPS_PACRH_SP6_WIDTH                     1
#define AIPS_PACRH_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP6_SHIFT))&AIPS_PACRH_SP6_MASK)
#define AIPS_PACRH_TP5_MASK                      0x100u
#define AIPS_PACRH_TP5_SHIFT                     8
#define AIPS_PACRH_TP5_WIDTH                     1
#define AIPS_PACRH_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP5_SHIFT))&AIPS_PACRH_TP5_MASK)
#define AIPS_PACRH_WP5_MASK                      0x200u
#define AIPS_PACRH_WP5_SHIFT                     9
#define AIPS_PACRH_WP5_WIDTH                     1
#define AIPS_PACRH_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP5_SHIFT))&AIPS_PACRH_WP5_MASK)
#define AIPS_PACRH_SP5_MASK                      0x400u
#define AIPS_PACRH_SP5_SHIFT                     10
#define AIPS_PACRH_SP5_WIDTH                     1
#define AIPS_PACRH_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP5_SHIFT))&AIPS_PACRH_SP5_MASK)
#define AIPS_PACRH_TP4_MASK                      0x1000u
#define AIPS_PACRH_TP4_SHIFT                     12
#define AIPS_PACRH_TP4_WIDTH                     1
#define AIPS_PACRH_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP4_SHIFT))&AIPS_PACRH_TP4_MASK)
#define AIPS_PACRH_WP4_MASK                      0x2000u
#define AIPS_PACRH_WP4_SHIFT                     13
#define AIPS_PACRH_WP4_WIDTH                     1
#define AIPS_PACRH_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP4_SHIFT))&AIPS_PACRH_WP4_MASK)
#define AIPS_PACRH_SP4_MASK                      0x4000u
#define AIPS_PACRH_SP4_SHIFT                     14
#define AIPS_PACRH_SP4_WIDTH                     1
#define AIPS_PACRH_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP4_SHIFT))&AIPS_PACRH_SP4_MASK)
#define AIPS_PACRH_TP3_MASK                      0x10000u
#define AIPS_PACRH_TP3_SHIFT                     16
#define AIPS_PACRH_TP3_WIDTH                     1
#define AIPS_PACRH_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP3_SHIFT))&AIPS_PACRH_TP3_MASK)
#define AIPS_PACRH_WP3_MASK                      0x20000u
#define AIPS_PACRH_WP3_SHIFT                     17
#define AIPS_PACRH_WP3_WIDTH                     1
#define AIPS_PACRH_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP3_SHIFT))&AIPS_PACRH_WP3_MASK)
#define AIPS_PACRH_SP3_MASK                      0x40000u
#define AIPS_PACRH_SP3_SHIFT                     18
#define AIPS_PACRH_SP3_WIDTH                     1
#define AIPS_PACRH_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP3_SHIFT))&AIPS_PACRH_SP3_MASK)
#define AIPS_PACRH_TP2_MASK                      0x100000u
#define AIPS_PACRH_TP2_SHIFT                     20
#define AIPS_PACRH_TP2_WIDTH                     1
#define AIPS_PACRH_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP2_SHIFT))&AIPS_PACRH_TP2_MASK)
#define AIPS_PACRH_WP2_MASK                      0x200000u
#define AIPS_PACRH_WP2_SHIFT                     21
#define AIPS_PACRH_WP2_WIDTH                     1
#define AIPS_PACRH_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP2_SHIFT))&AIPS_PACRH_WP2_MASK)
#define AIPS_PACRH_SP2_MASK                      0x400000u
#define AIPS_PACRH_SP2_SHIFT                     22
#define AIPS_PACRH_SP2_WIDTH                     1
#define AIPS_PACRH_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP2_SHIFT))&AIPS_PACRH_SP2_MASK)
#define AIPS_PACRH_TP1_MASK                      0x1000000u
#define AIPS_PACRH_TP1_SHIFT                     24
#define AIPS_PACRH_TP1_WIDTH                     1
#define AIPS_PACRH_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP1_SHIFT))&AIPS_PACRH_TP1_MASK)
#define AIPS_PACRH_WP1_MASK                      0x2000000u
#define AIPS_PACRH_WP1_SHIFT                     25
#define AIPS_PACRH_WP1_WIDTH                     1
#define AIPS_PACRH_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP1_SHIFT))&AIPS_PACRH_WP1_MASK)
#define AIPS_PACRH_SP1_MASK                      0x4000000u
#define AIPS_PACRH_SP1_SHIFT                     26
#define AIPS_PACRH_SP1_WIDTH                     1
#define AIPS_PACRH_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP1_SHIFT))&AIPS_PACRH_SP1_MASK)
#define AIPS_PACRH_TP0_MASK                      0x10000000u
#define AIPS_PACRH_TP0_SHIFT                     28
#define AIPS_PACRH_TP0_WIDTH                     1
#define AIPS_PACRH_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_TP0_SHIFT))&AIPS_PACRH_TP0_MASK)
#define AIPS_PACRH_WP0_MASK                      0x20000000u
#define AIPS_PACRH_WP0_SHIFT                     29
#define AIPS_PACRH_WP0_WIDTH                     1
#define AIPS_PACRH_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_WP0_SHIFT))&AIPS_PACRH_WP0_MASK)
#define AIPS_PACRH_SP0_MASK                      0x40000000u
#define AIPS_PACRH_SP0_SHIFT                     30
#define AIPS_PACRH_SP0_WIDTH                     1
#define AIPS_PACRH_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRH_SP0_SHIFT))&AIPS_PACRH_SP0_MASK)
/* PACRI Bit Fields */
#define AIPS_PACRI_TP7_MASK                      0x1u
#define AIPS_PACRI_TP7_SHIFT                     0
#define AIPS_PACRI_TP7_WIDTH                     1
#define AIPS_PACRI_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP7_SHIFT))&AIPS_PACRI_TP7_MASK)
#define AIPS_PACRI_WP7_MASK                      0x2u
#define AIPS_PACRI_WP7_SHIFT                     1
#define AIPS_PACRI_WP7_WIDTH                     1
#define AIPS_PACRI_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP7_SHIFT))&AIPS_PACRI_WP7_MASK)
#define AIPS_PACRI_SP7_MASK                      0x4u
#define AIPS_PACRI_SP7_SHIFT                     2
#define AIPS_PACRI_SP7_WIDTH                     1
#define AIPS_PACRI_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP7_SHIFT))&AIPS_PACRI_SP7_MASK)
#define AIPS_PACRI_TP6_MASK                      0x10u
#define AIPS_PACRI_TP6_SHIFT                     4
#define AIPS_PACRI_TP6_WIDTH                     1
#define AIPS_PACRI_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP6_SHIFT))&AIPS_PACRI_TP6_MASK)
#define AIPS_PACRI_WP6_MASK                      0x20u
#define AIPS_PACRI_WP6_SHIFT                     5
#define AIPS_PACRI_WP6_WIDTH                     1
#define AIPS_PACRI_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP6_SHIFT))&AIPS_PACRI_WP6_MASK)
#define AIPS_PACRI_SP6_MASK                      0x40u
#define AIPS_PACRI_SP6_SHIFT                     6
#define AIPS_PACRI_SP6_WIDTH                     1
#define AIPS_PACRI_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP6_SHIFT))&AIPS_PACRI_SP6_MASK)
#define AIPS_PACRI_TP5_MASK                      0x100u
#define AIPS_PACRI_TP5_SHIFT                     8
#define AIPS_PACRI_TP5_WIDTH                     1
#define AIPS_PACRI_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP5_SHIFT))&AIPS_PACRI_TP5_MASK)
#define AIPS_PACRI_WP5_MASK                      0x200u
#define AIPS_PACRI_WP5_SHIFT                     9
#define AIPS_PACRI_WP5_WIDTH                     1
#define AIPS_PACRI_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP5_SHIFT))&AIPS_PACRI_WP5_MASK)
#define AIPS_PACRI_SP5_MASK                      0x400u
#define AIPS_PACRI_SP5_SHIFT                     10
#define AIPS_PACRI_SP5_WIDTH                     1
#define AIPS_PACRI_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP5_SHIFT))&AIPS_PACRI_SP5_MASK)
#define AIPS_PACRI_TP4_MASK                      0x1000u
#define AIPS_PACRI_TP4_SHIFT                     12
#define AIPS_PACRI_TP4_WIDTH                     1
#define AIPS_PACRI_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP4_SHIFT))&AIPS_PACRI_TP4_MASK)
#define AIPS_PACRI_WP4_MASK                      0x2000u
#define AIPS_PACRI_WP4_SHIFT                     13
#define AIPS_PACRI_WP4_WIDTH                     1
#define AIPS_PACRI_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP4_SHIFT))&AIPS_PACRI_WP4_MASK)
#define AIPS_PACRI_SP4_MASK                      0x4000u
#define AIPS_PACRI_SP4_SHIFT                     14
#define AIPS_PACRI_SP4_WIDTH                     1
#define AIPS_PACRI_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP4_SHIFT))&AIPS_PACRI_SP4_MASK)
#define AIPS_PACRI_TP3_MASK                      0x10000u
#define AIPS_PACRI_TP3_SHIFT                     16
#define AIPS_PACRI_TP3_WIDTH                     1
#define AIPS_PACRI_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP3_SHIFT))&AIPS_PACRI_TP3_MASK)
#define AIPS_PACRI_WP3_MASK                      0x20000u
#define AIPS_PACRI_WP3_SHIFT                     17
#define AIPS_PACRI_WP3_WIDTH                     1
#define AIPS_PACRI_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP3_SHIFT))&AIPS_PACRI_WP3_MASK)
#define AIPS_PACRI_SP3_MASK                      0x40000u
#define AIPS_PACRI_SP3_SHIFT                     18
#define AIPS_PACRI_SP3_WIDTH                     1
#define AIPS_PACRI_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP3_SHIFT))&AIPS_PACRI_SP3_MASK)
#define AIPS_PACRI_TP2_MASK                      0x100000u
#define AIPS_PACRI_TP2_SHIFT                     20
#define AIPS_PACRI_TP2_WIDTH                     1
#define AIPS_PACRI_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP2_SHIFT))&AIPS_PACRI_TP2_MASK)
#define AIPS_PACRI_WP2_MASK                      0x200000u
#define AIPS_PACRI_WP2_SHIFT                     21
#define AIPS_PACRI_WP2_WIDTH                     1
#define AIPS_PACRI_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP2_SHIFT))&AIPS_PACRI_WP2_MASK)
#define AIPS_PACRI_SP2_MASK                      0x400000u
#define AIPS_PACRI_SP2_SHIFT                     22
#define AIPS_PACRI_SP2_WIDTH                     1
#define AIPS_PACRI_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP2_SHIFT))&AIPS_PACRI_SP2_MASK)
#define AIPS_PACRI_TP1_MASK                      0x1000000u
#define AIPS_PACRI_TP1_SHIFT                     24
#define AIPS_PACRI_TP1_WIDTH                     1
#define AIPS_PACRI_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP1_SHIFT))&AIPS_PACRI_TP1_MASK)
#define AIPS_PACRI_WP1_MASK                      0x2000000u
#define AIPS_PACRI_WP1_SHIFT                     25
#define AIPS_PACRI_WP1_WIDTH                     1
#define AIPS_PACRI_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP1_SHIFT))&AIPS_PACRI_WP1_MASK)
#define AIPS_PACRI_SP1_MASK                      0x4000000u
#define AIPS_PACRI_SP1_SHIFT                     26
#define AIPS_PACRI_SP1_WIDTH                     1
#define AIPS_PACRI_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP1_SHIFT))&AIPS_PACRI_SP1_MASK)
#define AIPS_PACRI_TP0_MASK                      0x10000000u
#define AIPS_PACRI_TP0_SHIFT                     28
#define AIPS_PACRI_TP0_WIDTH                     1
#define AIPS_PACRI_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_TP0_SHIFT))&AIPS_PACRI_TP0_MASK)
#define AIPS_PACRI_WP0_MASK                      0x20000000u
#define AIPS_PACRI_WP0_SHIFT                     29
#define AIPS_PACRI_WP0_WIDTH                     1
#define AIPS_PACRI_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_WP0_SHIFT))&AIPS_PACRI_WP0_MASK)
#define AIPS_PACRI_SP0_MASK                      0x40000000u
#define AIPS_PACRI_SP0_SHIFT                     30
#define AIPS_PACRI_SP0_WIDTH                     1
#define AIPS_PACRI_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRI_SP0_SHIFT))&AIPS_PACRI_SP0_MASK)
/* PACRJ Bit Fields */
#define AIPS_PACRJ_TP7_MASK                      0x1u
#define AIPS_PACRJ_TP7_SHIFT                     0
#define AIPS_PACRJ_TP7_WIDTH                     1
#define AIPS_PACRJ_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP7_SHIFT))&AIPS_PACRJ_TP7_MASK)
#define AIPS_PACRJ_WP7_MASK                      0x2u
#define AIPS_PACRJ_WP7_SHIFT                     1
#define AIPS_PACRJ_WP7_WIDTH                     1
#define AIPS_PACRJ_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP7_SHIFT))&AIPS_PACRJ_WP7_MASK)
#define AIPS_PACRJ_SP7_MASK                      0x4u
#define AIPS_PACRJ_SP7_SHIFT                     2
#define AIPS_PACRJ_SP7_WIDTH                     1
#define AIPS_PACRJ_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP7_SHIFT))&AIPS_PACRJ_SP7_MASK)
#define AIPS_PACRJ_TP6_MASK                      0x10u
#define AIPS_PACRJ_TP6_SHIFT                     4
#define AIPS_PACRJ_TP6_WIDTH                     1
#define AIPS_PACRJ_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP6_SHIFT))&AIPS_PACRJ_TP6_MASK)
#define AIPS_PACRJ_WP6_MASK                      0x20u
#define AIPS_PACRJ_WP6_SHIFT                     5
#define AIPS_PACRJ_WP6_WIDTH                     1
#define AIPS_PACRJ_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP6_SHIFT))&AIPS_PACRJ_WP6_MASK)
#define AIPS_PACRJ_SP6_MASK                      0x40u
#define AIPS_PACRJ_SP6_SHIFT                     6
#define AIPS_PACRJ_SP6_WIDTH                     1
#define AIPS_PACRJ_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP6_SHIFT))&AIPS_PACRJ_SP6_MASK)
#define AIPS_PACRJ_TP5_MASK                      0x100u
#define AIPS_PACRJ_TP5_SHIFT                     8
#define AIPS_PACRJ_TP5_WIDTH                     1
#define AIPS_PACRJ_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP5_SHIFT))&AIPS_PACRJ_TP5_MASK)
#define AIPS_PACRJ_WP5_MASK                      0x200u
#define AIPS_PACRJ_WP5_SHIFT                     9
#define AIPS_PACRJ_WP5_WIDTH                     1
#define AIPS_PACRJ_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP5_SHIFT))&AIPS_PACRJ_WP5_MASK)
#define AIPS_PACRJ_SP5_MASK                      0x400u
#define AIPS_PACRJ_SP5_SHIFT                     10
#define AIPS_PACRJ_SP5_WIDTH                     1
#define AIPS_PACRJ_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP5_SHIFT))&AIPS_PACRJ_SP5_MASK)
#define AIPS_PACRJ_TP4_MASK                      0x1000u
#define AIPS_PACRJ_TP4_SHIFT                     12
#define AIPS_PACRJ_TP4_WIDTH                     1
#define AIPS_PACRJ_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP4_SHIFT))&AIPS_PACRJ_TP4_MASK)
#define AIPS_PACRJ_WP4_MASK                      0x2000u
#define AIPS_PACRJ_WP4_SHIFT                     13
#define AIPS_PACRJ_WP4_WIDTH                     1
#define AIPS_PACRJ_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP4_SHIFT))&AIPS_PACRJ_WP4_MASK)
#define AIPS_PACRJ_SP4_MASK                      0x4000u
#define AIPS_PACRJ_SP4_SHIFT                     14
#define AIPS_PACRJ_SP4_WIDTH                     1
#define AIPS_PACRJ_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP4_SHIFT))&AIPS_PACRJ_SP4_MASK)
#define AIPS_PACRJ_TP3_MASK                      0x10000u
#define AIPS_PACRJ_TP3_SHIFT                     16
#define AIPS_PACRJ_TP3_WIDTH                     1
#define AIPS_PACRJ_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP3_SHIFT))&AIPS_PACRJ_TP3_MASK)
#define AIPS_PACRJ_WP3_MASK                      0x20000u
#define AIPS_PACRJ_WP3_SHIFT                     17
#define AIPS_PACRJ_WP3_WIDTH                     1
#define AIPS_PACRJ_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP3_SHIFT))&AIPS_PACRJ_WP3_MASK)
#define AIPS_PACRJ_SP3_MASK                      0x40000u
#define AIPS_PACRJ_SP3_SHIFT                     18
#define AIPS_PACRJ_SP3_WIDTH                     1
#define AIPS_PACRJ_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP3_SHIFT))&AIPS_PACRJ_SP3_MASK)
#define AIPS_PACRJ_TP2_MASK                      0x100000u
#define AIPS_PACRJ_TP2_SHIFT                     20
#define AIPS_PACRJ_TP2_WIDTH                     1
#define AIPS_PACRJ_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP2_SHIFT))&AIPS_PACRJ_TP2_MASK)
#define AIPS_PACRJ_WP2_MASK                      0x200000u
#define AIPS_PACRJ_WP2_SHIFT                     21
#define AIPS_PACRJ_WP2_WIDTH                     1
#define AIPS_PACRJ_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP2_SHIFT))&AIPS_PACRJ_WP2_MASK)
#define AIPS_PACRJ_SP2_MASK                      0x400000u
#define AIPS_PACRJ_SP2_SHIFT                     22
#define AIPS_PACRJ_SP2_WIDTH                     1
#define AIPS_PACRJ_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP2_SHIFT))&AIPS_PACRJ_SP2_MASK)
#define AIPS_PACRJ_TP1_MASK                      0x1000000u
#define AIPS_PACRJ_TP1_SHIFT                     24
#define AIPS_PACRJ_TP1_WIDTH                     1
#define AIPS_PACRJ_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP1_SHIFT))&AIPS_PACRJ_TP1_MASK)
#define AIPS_PACRJ_WP1_MASK                      0x2000000u
#define AIPS_PACRJ_WP1_SHIFT                     25
#define AIPS_PACRJ_WP1_WIDTH                     1
#define AIPS_PACRJ_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP1_SHIFT))&AIPS_PACRJ_WP1_MASK)
#define AIPS_PACRJ_SP1_MASK                      0x4000000u
#define AIPS_PACRJ_SP1_SHIFT                     26
#define AIPS_PACRJ_SP1_WIDTH                     1
#define AIPS_PACRJ_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP1_SHIFT))&AIPS_PACRJ_SP1_MASK)
#define AIPS_PACRJ_TP0_MASK                      0x10000000u
#define AIPS_PACRJ_TP0_SHIFT                     28
#define AIPS_PACRJ_TP0_WIDTH                     1
#define AIPS_PACRJ_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_TP0_SHIFT))&AIPS_PACRJ_TP0_MASK)
#define AIPS_PACRJ_WP0_MASK                      0x20000000u
#define AIPS_PACRJ_WP0_SHIFT                     29
#define AIPS_PACRJ_WP0_WIDTH                     1
#define AIPS_PACRJ_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_WP0_SHIFT))&AIPS_PACRJ_WP0_MASK)
#define AIPS_PACRJ_SP0_MASK                      0x40000000u
#define AIPS_PACRJ_SP0_SHIFT                     30
#define AIPS_PACRJ_SP0_WIDTH                     1
#define AIPS_PACRJ_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRJ_SP0_SHIFT))&AIPS_PACRJ_SP0_MASK)
/* PACRK Bit Fields */
#define AIPS_PACRK_TP7_MASK                      0x1u
#define AIPS_PACRK_TP7_SHIFT                     0
#define AIPS_PACRK_TP7_WIDTH                     1
#define AIPS_PACRK_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP7_SHIFT))&AIPS_PACRK_TP7_MASK)
#define AIPS_PACRK_WP7_MASK                      0x2u
#define AIPS_PACRK_WP7_SHIFT                     1
#define AIPS_PACRK_WP7_WIDTH                     1
#define AIPS_PACRK_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP7_SHIFT))&AIPS_PACRK_WP7_MASK)
#define AIPS_PACRK_SP7_MASK                      0x4u
#define AIPS_PACRK_SP7_SHIFT                     2
#define AIPS_PACRK_SP7_WIDTH                     1
#define AIPS_PACRK_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP7_SHIFT))&AIPS_PACRK_SP7_MASK)
#define AIPS_PACRK_TP6_MASK                      0x10u
#define AIPS_PACRK_TP6_SHIFT                     4
#define AIPS_PACRK_TP6_WIDTH                     1
#define AIPS_PACRK_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP6_SHIFT))&AIPS_PACRK_TP6_MASK)
#define AIPS_PACRK_WP6_MASK                      0x20u
#define AIPS_PACRK_WP6_SHIFT                     5
#define AIPS_PACRK_WP6_WIDTH                     1
#define AIPS_PACRK_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP6_SHIFT))&AIPS_PACRK_WP6_MASK)
#define AIPS_PACRK_SP6_MASK                      0x40u
#define AIPS_PACRK_SP6_SHIFT                     6
#define AIPS_PACRK_SP6_WIDTH                     1
#define AIPS_PACRK_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP6_SHIFT))&AIPS_PACRK_SP6_MASK)
#define AIPS_PACRK_TP5_MASK                      0x100u
#define AIPS_PACRK_TP5_SHIFT                     8
#define AIPS_PACRK_TP5_WIDTH                     1
#define AIPS_PACRK_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP5_SHIFT))&AIPS_PACRK_TP5_MASK)
#define AIPS_PACRK_WP5_MASK                      0x200u
#define AIPS_PACRK_WP5_SHIFT                     9
#define AIPS_PACRK_WP5_WIDTH                     1
#define AIPS_PACRK_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP5_SHIFT))&AIPS_PACRK_WP5_MASK)
#define AIPS_PACRK_SP5_MASK                      0x400u
#define AIPS_PACRK_SP5_SHIFT                     10
#define AIPS_PACRK_SP5_WIDTH                     1
#define AIPS_PACRK_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP5_SHIFT))&AIPS_PACRK_SP5_MASK)
#define AIPS_PACRK_TP4_MASK                      0x1000u
#define AIPS_PACRK_TP4_SHIFT                     12
#define AIPS_PACRK_TP4_WIDTH                     1
#define AIPS_PACRK_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP4_SHIFT))&AIPS_PACRK_TP4_MASK)
#define AIPS_PACRK_WP4_MASK                      0x2000u
#define AIPS_PACRK_WP4_SHIFT                     13
#define AIPS_PACRK_WP4_WIDTH                     1
#define AIPS_PACRK_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP4_SHIFT))&AIPS_PACRK_WP4_MASK)
#define AIPS_PACRK_SP4_MASK                      0x4000u
#define AIPS_PACRK_SP4_SHIFT                     14
#define AIPS_PACRK_SP4_WIDTH                     1
#define AIPS_PACRK_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP4_SHIFT))&AIPS_PACRK_SP4_MASK)
#define AIPS_PACRK_TP3_MASK                      0x10000u
#define AIPS_PACRK_TP3_SHIFT                     16
#define AIPS_PACRK_TP3_WIDTH                     1
#define AIPS_PACRK_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP3_SHIFT))&AIPS_PACRK_TP3_MASK)
#define AIPS_PACRK_WP3_MASK                      0x20000u
#define AIPS_PACRK_WP3_SHIFT                     17
#define AIPS_PACRK_WP3_WIDTH                     1
#define AIPS_PACRK_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP3_SHIFT))&AIPS_PACRK_WP3_MASK)
#define AIPS_PACRK_SP3_MASK                      0x40000u
#define AIPS_PACRK_SP3_SHIFT                     18
#define AIPS_PACRK_SP3_WIDTH                     1
#define AIPS_PACRK_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP3_SHIFT))&AIPS_PACRK_SP3_MASK)
#define AIPS_PACRK_TP2_MASK                      0x100000u
#define AIPS_PACRK_TP2_SHIFT                     20
#define AIPS_PACRK_TP2_WIDTH                     1
#define AIPS_PACRK_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP2_SHIFT))&AIPS_PACRK_TP2_MASK)
#define AIPS_PACRK_WP2_MASK                      0x200000u
#define AIPS_PACRK_WP2_SHIFT                     21
#define AIPS_PACRK_WP2_WIDTH                     1
#define AIPS_PACRK_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP2_SHIFT))&AIPS_PACRK_WP2_MASK)
#define AIPS_PACRK_SP2_MASK                      0x400000u
#define AIPS_PACRK_SP2_SHIFT                     22
#define AIPS_PACRK_SP2_WIDTH                     1
#define AIPS_PACRK_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP2_SHIFT))&AIPS_PACRK_SP2_MASK)
#define AIPS_PACRK_TP1_MASK                      0x1000000u
#define AIPS_PACRK_TP1_SHIFT                     24
#define AIPS_PACRK_TP1_WIDTH                     1
#define AIPS_PACRK_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP1_SHIFT))&AIPS_PACRK_TP1_MASK)
#define AIPS_PACRK_WP1_MASK                      0x2000000u
#define AIPS_PACRK_WP1_SHIFT                     25
#define AIPS_PACRK_WP1_WIDTH                     1
#define AIPS_PACRK_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP1_SHIFT))&AIPS_PACRK_WP1_MASK)
#define AIPS_PACRK_SP1_MASK                      0x4000000u
#define AIPS_PACRK_SP1_SHIFT                     26
#define AIPS_PACRK_SP1_WIDTH                     1
#define AIPS_PACRK_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP1_SHIFT))&AIPS_PACRK_SP1_MASK)
#define AIPS_PACRK_TP0_MASK                      0x10000000u
#define AIPS_PACRK_TP0_SHIFT                     28
#define AIPS_PACRK_TP0_WIDTH                     1
#define AIPS_PACRK_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_TP0_SHIFT))&AIPS_PACRK_TP0_MASK)
#define AIPS_PACRK_WP0_MASK                      0x20000000u
#define AIPS_PACRK_WP0_SHIFT                     29
#define AIPS_PACRK_WP0_WIDTH                     1
#define AIPS_PACRK_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_WP0_SHIFT))&AIPS_PACRK_WP0_MASK)
#define AIPS_PACRK_SP0_MASK                      0x40000000u
#define AIPS_PACRK_SP0_SHIFT                     30
#define AIPS_PACRK_SP0_WIDTH                     1
#define AIPS_PACRK_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRK_SP0_SHIFT))&AIPS_PACRK_SP0_MASK)
/* PACRL Bit Fields */
#define AIPS_PACRL_TP7_MASK                      0x1u
#define AIPS_PACRL_TP7_SHIFT                     0
#define AIPS_PACRL_TP7_WIDTH                     1
#define AIPS_PACRL_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP7_SHIFT))&AIPS_PACRL_TP7_MASK)
#define AIPS_PACRL_WP7_MASK                      0x2u
#define AIPS_PACRL_WP7_SHIFT                     1
#define AIPS_PACRL_WP7_WIDTH                     1
#define AIPS_PACRL_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP7_SHIFT))&AIPS_PACRL_WP7_MASK)
#define AIPS_PACRL_SP7_MASK                      0x4u
#define AIPS_PACRL_SP7_SHIFT                     2
#define AIPS_PACRL_SP7_WIDTH                     1
#define AIPS_PACRL_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP7_SHIFT))&AIPS_PACRL_SP7_MASK)
#define AIPS_PACRL_TP6_MASK                      0x10u
#define AIPS_PACRL_TP6_SHIFT                     4
#define AIPS_PACRL_TP6_WIDTH                     1
#define AIPS_PACRL_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP6_SHIFT))&AIPS_PACRL_TP6_MASK)
#define AIPS_PACRL_WP6_MASK                      0x20u
#define AIPS_PACRL_WP6_SHIFT                     5
#define AIPS_PACRL_WP6_WIDTH                     1
#define AIPS_PACRL_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP6_SHIFT))&AIPS_PACRL_WP6_MASK)
#define AIPS_PACRL_SP6_MASK                      0x40u
#define AIPS_PACRL_SP6_SHIFT                     6
#define AIPS_PACRL_SP6_WIDTH                     1
#define AIPS_PACRL_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP6_SHIFT))&AIPS_PACRL_SP6_MASK)
#define AIPS_PACRL_TP5_MASK                      0x100u
#define AIPS_PACRL_TP5_SHIFT                     8
#define AIPS_PACRL_TP5_WIDTH                     1
#define AIPS_PACRL_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP5_SHIFT))&AIPS_PACRL_TP5_MASK)
#define AIPS_PACRL_WP5_MASK                      0x200u
#define AIPS_PACRL_WP5_SHIFT                     9
#define AIPS_PACRL_WP5_WIDTH                     1
#define AIPS_PACRL_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP5_SHIFT))&AIPS_PACRL_WP5_MASK)
#define AIPS_PACRL_SP5_MASK                      0x400u
#define AIPS_PACRL_SP5_SHIFT                     10
#define AIPS_PACRL_SP5_WIDTH                     1
#define AIPS_PACRL_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP5_SHIFT))&AIPS_PACRL_SP5_MASK)
#define AIPS_PACRL_TP4_MASK                      0x1000u
#define AIPS_PACRL_TP4_SHIFT                     12
#define AIPS_PACRL_TP4_WIDTH                     1
#define AIPS_PACRL_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP4_SHIFT))&AIPS_PACRL_TP4_MASK)
#define AIPS_PACRL_WP4_MASK                      0x2000u
#define AIPS_PACRL_WP4_SHIFT                     13
#define AIPS_PACRL_WP4_WIDTH                     1
#define AIPS_PACRL_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP4_SHIFT))&AIPS_PACRL_WP4_MASK)
#define AIPS_PACRL_SP4_MASK                      0x4000u
#define AIPS_PACRL_SP4_SHIFT                     14
#define AIPS_PACRL_SP4_WIDTH                     1
#define AIPS_PACRL_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP4_SHIFT))&AIPS_PACRL_SP4_MASK)
#define AIPS_PACRL_TP3_MASK                      0x10000u
#define AIPS_PACRL_TP3_SHIFT                     16
#define AIPS_PACRL_TP3_WIDTH                     1
#define AIPS_PACRL_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP3_SHIFT))&AIPS_PACRL_TP3_MASK)
#define AIPS_PACRL_WP3_MASK                      0x20000u
#define AIPS_PACRL_WP3_SHIFT                     17
#define AIPS_PACRL_WP3_WIDTH                     1
#define AIPS_PACRL_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP3_SHIFT))&AIPS_PACRL_WP3_MASK)
#define AIPS_PACRL_SP3_MASK                      0x40000u
#define AIPS_PACRL_SP3_SHIFT                     18
#define AIPS_PACRL_SP3_WIDTH                     1
#define AIPS_PACRL_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP3_SHIFT))&AIPS_PACRL_SP3_MASK)
#define AIPS_PACRL_TP2_MASK                      0x100000u
#define AIPS_PACRL_TP2_SHIFT                     20
#define AIPS_PACRL_TP2_WIDTH                     1
#define AIPS_PACRL_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP2_SHIFT))&AIPS_PACRL_TP2_MASK)
#define AIPS_PACRL_WP2_MASK                      0x200000u
#define AIPS_PACRL_WP2_SHIFT                     21
#define AIPS_PACRL_WP2_WIDTH                     1
#define AIPS_PACRL_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP2_SHIFT))&AIPS_PACRL_WP2_MASK)
#define AIPS_PACRL_SP2_MASK                      0x400000u
#define AIPS_PACRL_SP2_SHIFT                     22
#define AIPS_PACRL_SP2_WIDTH                     1
#define AIPS_PACRL_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP2_SHIFT))&AIPS_PACRL_SP2_MASK)
#define AIPS_PACRL_TP1_MASK                      0x1000000u
#define AIPS_PACRL_TP1_SHIFT                     24
#define AIPS_PACRL_TP1_WIDTH                     1
#define AIPS_PACRL_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP1_SHIFT))&AIPS_PACRL_TP1_MASK)
#define AIPS_PACRL_WP1_MASK                      0x2000000u
#define AIPS_PACRL_WP1_SHIFT                     25
#define AIPS_PACRL_WP1_WIDTH                     1
#define AIPS_PACRL_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP1_SHIFT))&AIPS_PACRL_WP1_MASK)
#define AIPS_PACRL_SP1_MASK                      0x4000000u
#define AIPS_PACRL_SP1_SHIFT                     26
#define AIPS_PACRL_SP1_WIDTH                     1
#define AIPS_PACRL_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP1_SHIFT))&AIPS_PACRL_SP1_MASK)
#define AIPS_PACRL_TP0_MASK                      0x10000000u
#define AIPS_PACRL_TP0_SHIFT                     28
#define AIPS_PACRL_TP0_WIDTH                     1
#define AIPS_PACRL_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_TP0_SHIFT))&AIPS_PACRL_TP0_MASK)
#define AIPS_PACRL_WP0_MASK                      0x20000000u
#define AIPS_PACRL_WP0_SHIFT                     29
#define AIPS_PACRL_WP0_WIDTH                     1
#define AIPS_PACRL_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_WP0_SHIFT))&AIPS_PACRL_WP0_MASK)
#define AIPS_PACRL_SP0_MASK                      0x40000000u
#define AIPS_PACRL_SP0_SHIFT                     30
#define AIPS_PACRL_SP0_WIDTH                     1
#define AIPS_PACRL_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRL_SP0_SHIFT))&AIPS_PACRL_SP0_MASK)
/* PACRM Bit Fields */
#define AIPS_PACRM_TP7_MASK                      0x1u
#define AIPS_PACRM_TP7_SHIFT                     0
#define AIPS_PACRM_TP7_WIDTH                     1
#define AIPS_PACRM_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP7_SHIFT))&AIPS_PACRM_TP7_MASK)
#define AIPS_PACRM_WP7_MASK                      0x2u
#define AIPS_PACRM_WP7_SHIFT                     1
#define AIPS_PACRM_WP7_WIDTH                     1
#define AIPS_PACRM_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP7_SHIFT))&AIPS_PACRM_WP7_MASK)
#define AIPS_PACRM_SP7_MASK                      0x4u
#define AIPS_PACRM_SP7_SHIFT                     2
#define AIPS_PACRM_SP7_WIDTH                     1
#define AIPS_PACRM_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP7_SHIFT))&AIPS_PACRM_SP7_MASK)
#define AIPS_PACRM_TP6_MASK                      0x10u
#define AIPS_PACRM_TP6_SHIFT                     4
#define AIPS_PACRM_TP6_WIDTH                     1
#define AIPS_PACRM_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP6_SHIFT))&AIPS_PACRM_TP6_MASK)
#define AIPS_PACRM_WP6_MASK                      0x20u
#define AIPS_PACRM_WP6_SHIFT                     5
#define AIPS_PACRM_WP6_WIDTH                     1
#define AIPS_PACRM_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP6_SHIFT))&AIPS_PACRM_WP6_MASK)
#define AIPS_PACRM_SP6_MASK                      0x40u
#define AIPS_PACRM_SP6_SHIFT                     6
#define AIPS_PACRM_SP6_WIDTH                     1
#define AIPS_PACRM_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP6_SHIFT))&AIPS_PACRM_SP6_MASK)
#define AIPS_PACRM_TP5_MASK                      0x100u
#define AIPS_PACRM_TP5_SHIFT                     8
#define AIPS_PACRM_TP5_WIDTH                     1
#define AIPS_PACRM_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP5_SHIFT))&AIPS_PACRM_TP5_MASK)
#define AIPS_PACRM_WP5_MASK                      0x200u
#define AIPS_PACRM_WP5_SHIFT                     9
#define AIPS_PACRM_WP5_WIDTH                     1
#define AIPS_PACRM_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP5_SHIFT))&AIPS_PACRM_WP5_MASK)
#define AIPS_PACRM_SP5_MASK                      0x400u
#define AIPS_PACRM_SP5_SHIFT                     10
#define AIPS_PACRM_SP5_WIDTH                     1
#define AIPS_PACRM_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP5_SHIFT))&AIPS_PACRM_SP5_MASK)
#define AIPS_PACRM_TP4_MASK                      0x1000u
#define AIPS_PACRM_TP4_SHIFT                     12
#define AIPS_PACRM_TP4_WIDTH                     1
#define AIPS_PACRM_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP4_SHIFT))&AIPS_PACRM_TP4_MASK)
#define AIPS_PACRM_WP4_MASK                      0x2000u
#define AIPS_PACRM_WP4_SHIFT                     13
#define AIPS_PACRM_WP4_WIDTH                     1
#define AIPS_PACRM_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP4_SHIFT))&AIPS_PACRM_WP4_MASK)
#define AIPS_PACRM_SP4_MASK                      0x4000u
#define AIPS_PACRM_SP4_SHIFT                     14
#define AIPS_PACRM_SP4_WIDTH                     1
#define AIPS_PACRM_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP4_SHIFT))&AIPS_PACRM_SP4_MASK)
#define AIPS_PACRM_TP3_MASK                      0x10000u
#define AIPS_PACRM_TP3_SHIFT                     16
#define AIPS_PACRM_TP3_WIDTH                     1
#define AIPS_PACRM_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP3_SHIFT))&AIPS_PACRM_TP3_MASK)
#define AIPS_PACRM_WP3_MASK                      0x20000u
#define AIPS_PACRM_WP3_SHIFT                     17
#define AIPS_PACRM_WP3_WIDTH                     1
#define AIPS_PACRM_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP3_SHIFT))&AIPS_PACRM_WP3_MASK)
#define AIPS_PACRM_SP3_MASK                      0x40000u
#define AIPS_PACRM_SP3_SHIFT                     18
#define AIPS_PACRM_SP3_WIDTH                     1
#define AIPS_PACRM_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP3_SHIFT))&AIPS_PACRM_SP3_MASK)
#define AIPS_PACRM_TP2_MASK                      0x100000u
#define AIPS_PACRM_TP2_SHIFT                     20
#define AIPS_PACRM_TP2_WIDTH                     1
#define AIPS_PACRM_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP2_SHIFT))&AIPS_PACRM_TP2_MASK)
#define AIPS_PACRM_WP2_MASK                      0x200000u
#define AIPS_PACRM_WP2_SHIFT                     21
#define AIPS_PACRM_WP2_WIDTH                     1
#define AIPS_PACRM_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP2_SHIFT))&AIPS_PACRM_WP2_MASK)
#define AIPS_PACRM_SP2_MASK                      0x400000u
#define AIPS_PACRM_SP2_SHIFT                     22
#define AIPS_PACRM_SP2_WIDTH                     1
#define AIPS_PACRM_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP2_SHIFT))&AIPS_PACRM_SP2_MASK)
#define AIPS_PACRM_TP1_MASK                      0x1000000u
#define AIPS_PACRM_TP1_SHIFT                     24
#define AIPS_PACRM_TP1_WIDTH                     1
#define AIPS_PACRM_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP1_SHIFT))&AIPS_PACRM_TP1_MASK)
#define AIPS_PACRM_WP1_MASK                      0x2000000u
#define AIPS_PACRM_WP1_SHIFT                     25
#define AIPS_PACRM_WP1_WIDTH                     1
#define AIPS_PACRM_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP1_SHIFT))&AIPS_PACRM_WP1_MASK)
#define AIPS_PACRM_SP1_MASK                      0x4000000u
#define AIPS_PACRM_SP1_SHIFT                     26
#define AIPS_PACRM_SP1_WIDTH                     1
#define AIPS_PACRM_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP1_SHIFT))&AIPS_PACRM_SP1_MASK)
#define AIPS_PACRM_TP0_MASK                      0x10000000u
#define AIPS_PACRM_TP0_SHIFT                     28
#define AIPS_PACRM_TP0_WIDTH                     1
#define AIPS_PACRM_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_TP0_SHIFT))&AIPS_PACRM_TP0_MASK)
#define AIPS_PACRM_WP0_MASK                      0x20000000u
#define AIPS_PACRM_WP0_SHIFT                     29
#define AIPS_PACRM_WP0_WIDTH                     1
#define AIPS_PACRM_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_WP0_SHIFT))&AIPS_PACRM_WP0_MASK)
#define AIPS_PACRM_SP0_MASK                      0x40000000u
#define AIPS_PACRM_SP0_SHIFT                     30
#define AIPS_PACRM_SP0_WIDTH                     1
#define AIPS_PACRM_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRM_SP0_SHIFT))&AIPS_PACRM_SP0_MASK)
/* PACRN Bit Fields */
#define AIPS_PACRN_TP7_MASK                      0x1u
#define AIPS_PACRN_TP7_SHIFT                     0
#define AIPS_PACRN_TP7_WIDTH                     1
#define AIPS_PACRN_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP7_SHIFT))&AIPS_PACRN_TP7_MASK)
#define AIPS_PACRN_WP7_MASK                      0x2u
#define AIPS_PACRN_WP7_SHIFT                     1
#define AIPS_PACRN_WP7_WIDTH                     1
#define AIPS_PACRN_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP7_SHIFT))&AIPS_PACRN_WP7_MASK)
#define AIPS_PACRN_SP7_MASK                      0x4u
#define AIPS_PACRN_SP7_SHIFT                     2
#define AIPS_PACRN_SP7_WIDTH                     1
#define AIPS_PACRN_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP7_SHIFT))&AIPS_PACRN_SP7_MASK)
#define AIPS_PACRN_TP6_MASK                      0x10u
#define AIPS_PACRN_TP6_SHIFT                     4
#define AIPS_PACRN_TP6_WIDTH                     1
#define AIPS_PACRN_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP6_SHIFT))&AIPS_PACRN_TP6_MASK)
#define AIPS_PACRN_WP6_MASK                      0x20u
#define AIPS_PACRN_WP6_SHIFT                     5
#define AIPS_PACRN_WP6_WIDTH                     1
#define AIPS_PACRN_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP6_SHIFT))&AIPS_PACRN_WP6_MASK)
#define AIPS_PACRN_SP6_MASK                      0x40u
#define AIPS_PACRN_SP6_SHIFT                     6
#define AIPS_PACRN_SP6_WIDTH                     1
#define AIPS_PACRN_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP6_SHIFT))&AIPS_PACRN_SP6_MASK)
#define AIPS_PACRN_TP5_MASK                      0x100u
#define AIPS_PACRN_TP5_SHIFT                     8
#define AIPS_PACRN_TP5_WIDTH                     1
#define AIPS_PACRN_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP5_SHIFT))&AIPS_PACRN_TP5_MASK)
#define AIPS_PACRN_WP5_MASK                      0x200u
#define AIPS_PACRN_WP5_SHIFT                     9
#define AIPS_PACRN_WP5_WIDTH                     1
#define AIPS_PACRN_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP5_SHIFT))&AIPS_PACRN_WP5_MASK)
#define AIPS_PACRN_SP5_MASK                      0x400u
#define AIPS_PACRN_SP5_SHIFT                     10
#define AIPS_PACRN_SP5_WIDTH                     1
#define AIPS_PACRN_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP5_SHIFT))&AIPS_PACRN_SP5_MASK)
#define AIPS_PACRN_TP4_MASK                      0x1000u
#define AIPS_PACRN_TP4_SHIFT                     12
#define AIPS_PACRN_TP4_WIDTH                     1
#define AIPS_PACRN_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP4_SHIFT))&AIPS_PACRN_TP4_MASK)
#define AIPS_PACRN_WP4_MASK                      0x2000u
#define AIPS_PACRN_WP4_SHIFT                     13
#define AIPS_PACRN_WP4_WIDTH                     1
#define AIPS_PACRN_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP4_SHIFT))&AIPS_PACRN_WP4_MASK)
#define AIPS_PACRN_SP4_MASK                      0x4000u
#define AIPS_PACRN_SP4_SHIFT                     14
#define AIPS_PACRN_SP4_WIDTH                     1
#define AIPS_PACRN_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP4_SHIFT))&AIPS_PACRN_SP4_MASK)
#define AIPS_PACRN_TP3_MASK                      0x10000u
#define AIPS_PACRN_TP3_SHIFT                     16
#define AIPS_PACRN_TP3_WIDTH                     1
#define AIPS_PACRN_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP3_SHIFT))&AIPS_PACRN_TP3_MASK)
#define AIPS_PACRN_WP3_MASK                      0x20000u
#define AIPS_PACRN_WP3_SHIFT                     17
#define AIPS_PACRN_WP3_WIDTH                     1
#define AIPS_PACRN_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP3_SHIFT))&AIPS_PACRN_WP3_MASK)
#define AIPS_PACRN_SP3_MASK                      0x40000u
#define AIPS_PACRN_SP3_SHIFT                     18
#define AIPS_PACRN_SP3_WIDTH                     1
#define AIPS_PACRN_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP3_SHIFT))&AIPS_PACRN_SP3_MASK)
#define AIPS_PACRN_TP2_MASK                      0x100000u
#define AIPS_PACRN_TP2_SHIFT                     20
#define AIPS_PACRN_TP2_WIDTH                     1
#define AIPS_PACRN_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP2_SHIFT))&AIPS_PACRN_TP2_MASK)
#define AIPS_PACRN_WP2_MASK                      0x200000u
#define AIPS_PACRN_WP2_SHIFT                     21
#define AIPS_PACRN_WP2_WIDTH                     1
#define AIPS_PACRN_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP2_SHIFT))&AIPS_PACRN_WP2_MASK)
#define AIPS_PACRN_SP2_MASK                      0x400000u
#define AIPS_PACRN_SP2_SHIFT                     22
#define AIPS_PACRN_SP2_WIDTH                     1
#define AIPS_PACRN_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP2_SHIFT))&AIPS_PACRN_SP2_MASK)
#define AIPS_PACRN_TP1_MASK                      0x1000000u
#define AIPS_PACRN_TP1_SHIFT                     24
#define AIPS_PACRN_TP1_WIDTH                     1
#define AIPS_PACRN_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP1_SHIFT))&AIPS_PACRN_TP1_MASK)
#define AIPS_PACRN_WP1_MASK                      0x2000000u
#define AIPS_PACRN_WP1_SHIFT                     25
#define AIPS_PACRN_WP1_WIDTH                     1
#define AIPS_PACRN_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP1_SHIFT))&AIPS_PACRN_WP1_MASK)
#define AIPS_PACRN_SP1_MASK                      0x4000000u
#define AIPS_PACRN_SP1_SHIFT                     26
#define AIPS_PACRN_SP1_WIDTH                     1
#define AIPS_PACRN_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP1_SHIFT))&AIPS_PACRN_SP1_MASK)
#define AIPS_PACRN_TP0_MASK                      0x10000000u
#define AIPS_PACRN_TP0_SHIFT                     28
#define AIPS_PACRN_TP0_WIDTH                     1
#define AIPS_PACRN_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_TP0_SHIFT))&AIPS_PACRN_TP0_MASK)
#define AIPS_PACRN_WP0_MASK                      0x20000000u
#define AIPS_PACRN_WP0_SHIFT                     29
#define AIPS_PACRN_WP0_WIDTH                     1
#define AIPS_PACRN_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_WP0_SHIFT))&AIPS_PACRN_WP0_MASK)
#define AIPS_PACRN_SP0_MASK                      0x40000000u
#define AIPS_PACRN_SP0_SHIFT                     30
#define AIPS_PACRN_SP0_WIDTH                     1
#define AIPS_PACRN_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRN_SP0_SHIFT))&AIPS_PACRN_SP0_MASK)
/* PACRO Bit Fields */
#define AIPS_PACRO_TP7_MASK                      0x1u
#define AIPS_PACRO_TP7_SHIFT                     0
#define AIPS_PACRO_TP7_WIDTH                     1
#define AIPS_PACRO_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP7_SHIFT))&AIPS_PACRO_TP7_MASK)
#define AIPS_PACRO_WP7_MASK                      0x2u
#define AIPS_PACRO_WP7_SHIFT                     1
#define AIPS_PACRO_WP7_WIDTH                     1
#define AIPS_PACRO_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP7_SHIFT))&AIPS_PACRO_WP7_MASK)
#define AIPS_PACRO_SP7_MASK                      0x4u
#define AIPS_PACRO_SP7_SHIFT                     2
#define AIPS_PACRO_SP7_WIDTH                     1
#define AIPS_PACRO_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP7_SHIFT))&AIPS_PACRO_SP7_MASK)
#define AIPS_PACRO_TP6_MASK                      0x10u
#define AIPS_PACRO_TP6_SHIFT                     4
#define AIPS_PACRO_TP6_WIDTH                     1
#define AIPS_PACRO_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP6_SHIFT))&AIPS_PACRO_TP6_MASK)
#define AIPS_PACRO_WP6_MASK                      0x20u
#define AIPS_PACRO_WP6_SHIFT                     5
#define AIPS_PACRO_WP6_WIDTH                     1
#define AIPS_PACRO_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP6_SHIFT))&AIPS_PACRO_WP6_MASK)
#define AIPS_PACRO_SP6_MASK                      0x40u
#define AIPS_PACRO_SP6_SHIFT                     6
#define AIPS_PACRO_SP6_WIDTH                     1
#define AIPS_PACRO_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP6_SHIFT))&AIPS_PACRO_SP6_MASK)
#define AIPS_PACRO_TP5_MASK                      0x100u
#define AIPS_PACRO_TP5_SHIFT                     8
#define AIPS_PACRO_TP5_WIDTH                     1
#define AIPS_PACRO_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP5_SHIFT))&AIPS_PACRO_TP5_MASK)
#define AIPS_PACRO_WP5_MASK                      0x200u
#define AIPS_PACRO_WP5_SHIFT                     9
#define AIPS_PACRO_WP5_WIDTH                     1
#define AIPS_PACRO_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP5_SHIFT))&AIPS_PACRO_WP5_MASK)
#define AIPS_PACRO_SP5_MASK                      0x400u
#define AIPS_PACRO_SP5_SHIFT                     10
#define AIPS_PACRO_SP5_WIDTH                     1
#define AIPS_PACRO_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP5_SHIFT))&AIPS_PACRO_SP5_MASK)
#define AIPS_PACRO_TP4_MASK                      0x1000u
#define AIPS_PACRO_TP4_SHIFT                     12
#define AIPS_PACRO_TP4_WIDTH                     1
#define AIPS_PACRO_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP4_SHIFT))&AIPS_PACRO_TP4_MASK)
#define AIPS_PACRO_WP4_MASK                      0x2000u
#define AIPS_PACRO_WP4_SHIFT                     13
#define AIPS_PACRO_WP4_WIDTH                     1
#define AIPS_PACRO_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP4_SHIFT))&AIPS_PACRO_WP4_MASK)
#define AIPS_PACRO_SP4_MASK                      0x4000u
#define AIPS_PACRO_SP4_SHIFT                     14
#define AIPS_PACRO_SP4_WIDTH                     1
#define AIPS_PACRO_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP4_SHIFT))&AIPS_PACRO_SP4_MASK)
#define AIPS_PACRO_TP3_MASK                      0x10000u
#define AIPS_PACRO_TP3_SHIFT                     16
#define AIPS_PACRO_TP3_WIDTH                     1
#define AIPS_PACRO_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP3_SHIFT))&AIPS_PACRO_TP3_MASK)
#define AIPS_PACRO_WP3_MASK                      0x20000u
#define AIPS_PACRO_WP3_SHIFT                     17
#define AIPS_PACRO_WP3_WIDTH                     1
#define AIPS_PACRO_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP3_SHIFT))&AIPS_PACRO_WP3_MASK)
#define AIPS_PACRO_SP3_MASK                      0x40000u
#define AIPS_PACRO_SP3_SHIFT                     18
#define AIPS_PACRO_SP3_WIDTH                     1
#define AIPS_PACRO_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP3_SHIFT))&AIPS_PACRO_SP3_MASK)
#define AIPS_PACRO_TP2_MASK                      0x100000u
#define AIPS_PACRO_TP2_SHIFT                     20
#define AIPS_PACRO_TP2_WIDTH                     1
#define AIPS_PACRO_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP2_SHIFT))&AIPS_PACRO_TP2_MASK)
#define AIPS_PACRO_WP2_MASK                      0x200000u
#define AIPS_PACRO_WP2_SHIFT                     21
#define AIPS_PACRO_WP2_WIDTH                     1
#define AIPS_PACRO_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP2_SHIFT))&AIPS_PACRO_WP2_MASK)
#define AIPS_PACRO_SP2_MASK                      0x400000u
#define AIPS_PACRO_SP2_SHIFT                     22
#define AIPS_PACRO_SP2_WIDTH                     1
#define AIPS_PACRO_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP2_SHIFT))&AIPS_PACRO_SP2_MASK)
#define AIPS_PACRO_TP1_MASK                      0x1000000u
#define AIPS_PACRO_TP1_SHIFT                     24
#define AIPS_PACRO_TP1_WIDTH                     1
#define AIPS_PACRO_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP1_SHIFT))&AIPS_PACRO_TP1_MASK)
#define AIPS_PACRO_WP1_MASK                      0x2000000u
#define AIPS_PACRO_WP1_SHIFT                     25
#define AIPS_PACRO_WP1_WIDTH                     1
#define AIPS_PACRO_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP1_SHIFT))&AIPS_PACRO_WP1_MASK)
#define AIPS_PACRO_SP1_MASK                      0x4000000u
#define AIPS_PACRO_SP1_SHIFT                     26
#define AIPS_PACRO_SP1_WIDTH                     1
#define AIPS_PACRO_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP1_SHIFT))&AIPS_PACRO_SP1_MASK)
#define AIPS_PACRO_TP0_MASK                      0x10000000u
#define AIPS_PACRO_TP0_SHIFT                     28
#define AIPS_PACRO_TP0_WIDTH                     1
#define AIPS_PACRO_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_TP0_SHIFT))&AIPS_PACRO_TP0_MASK)
#define AIPS_PACRO_WP0_MASK                      0x20000000u
#define AIPS_PACRO_WP0_SHIFT                     29
#define AIPS_PACRO_WP0_WIDTH                     1
#define AIPS_PACRO_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_WP0_SHIFT))&AIPS_PACRO_WP0_MASK)
#define AIPS_PACRO_SP0_MASK                      0x40000000u
#define AIPS_PACRO_SP0_SHIFT                     30
#define AIPS_PACRO_SP0_WIDTH                     1
#define AIPS_PACRO_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRO_SP0_SHIFT))&AIPS_PACRO_SP0_MASK)
/* PACRP Bit Fields */
#define AIPS_PACRP_TP7_MASK                      0x1u
#define AIPS_PACRP_TP7_SHIFT                     0
#define AIPS_PACRP_TP7_WIDTH                     1
#define AIPS_PACRP_TP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP7_SHIFT))&AIPS_PACRP_TP7_MASK)
#define AIPS_PACRP_WP7_MASK                      0x2u
#define AIPS_PACRP_WP7_SHIFT                     1
#define AIPS_PACRP_WP7_WIDTH                     1
#define AIPS_PACRP_WP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP7_SHIFT))&AIPS_PACRP_WP7_MASK)
#define AIPS_PACRP_SP7_MASK                      0x4u
#define AIPS_PACRP_SP7_SHIFT                     2
#define AIPS_PACRP_SP7_WIDTH                     1
#define AIPS_PACRP_SP7(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP7_SHIFT))&AIPS_PACRP_SP7_MASK)
#define AIPS_PACRP_TP6_MASK                      0x10u
#define AIPS_PACRP_TP6_SHIFT                     4
#define AIPS_PACRP_TP6_WIDTH                     1
#define AIPS_PACRP_TP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP6_SHIFT))&AIPS_PACRP_TP6_MASK)
#define AIPS_PACRP_WP6_MASK                      0x20u
#define AIPS_PACRP_WP6_SHIFT                     5
#define AIPS_PACRP_WP6_WIDTH                     1
#define AIPS_PACRP_WP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP6_SHIFT))&AIPS_PACRP_WP6_MASK)
#define AIPS_PACRP_SP6_MASK                      0x40u
#define AIPS_PACRP_SP6_SHIFT                     6
#define AIPS_PACRP_SP6_WIDTH                     1
#define AIPS_PACRP_SP6(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP6_SHIFT))&AIPS_PACRP_SP6_MASK)
#define AIPS_PACRP_TP5_MASK                      0x100u
#define AIPS_PACRP_TP5_SHIFT                     8
#define AIPS_PACRP_TP5_WIDTH                     1
#define AIPS_PACRP_TP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP5_SHIFT))&AIPS_PACRP_TP5_MASK)
#define AIPS_PACRP_WP5_MASK                      0x200u
#define AIPS_PACRP_WP5_SHIFT                     9
#define AIPS_PACRP_WP5_WIDTH                     1
#define AIPS_PACRP_WP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP5_SHIFT))&AIPS_PACRP_WP5_MASK)
#define AIPS_PACRP_SP5_MASK                      0x400u
#define AIPS_PACRP_SP5_SHIFT                     10
#define AIPS_PACRP_SP5_WIDTH                     1
#define AIPS_PACRP_SP5(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP5_SHIFT))&AIPS_PACRP_SP5_MASK)
#define AIPS_PACRP_TP4_MASK                      0x1000u
#define AIPS_PACRP_TP4_SHIFT                     12
#define AIPS_PACRP_TP4_WIDTH                     1
#define AIPS_PACRP_TP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP4_SHIFT))&AIPS_PACRP_TP4_MASK)
#define AIPS_PACRP_WP4_MASK                      0x2000u
#define AIPS_PACRP_WP4_SHIFT                     13
#define AIPS_PACRP_WP4_WIDTH                     1
#define AIPS_PACRP_WP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP4_SHIFT))&AIPS_PACRP_WP4_MASK)
#define AIPS_PACRP_SP4_MASK                      0x4000u
#define AIPS_PACRP_SP4_SHIFT                     14
#define AIPS_PACRP_SP4_WIDTH                     1
#define AIPS_PACRP_SP4(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP4_SHIFT))&AIPS_PACRP_SP4_MASK)
#define AIPS_PACRP_TP3_MASK                      0x10000u
#define AIPS_PACRP_TP3_SHIFT                     16
#define AIPS_PACRP_TP3_WIDTH                     1
#define AIPS_PACRP_TP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP3_SHIFT))&AIPS_PACRP_TP3_MASK)
#define AIPS_PACRP_WP3_MASK                      0x20000u
#define AIPS_PACRP_WP3_SHIFT                     17
#define AIPS_PACRP_WP3_WIDTH                     1
#define AIPS_PACRP_WP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP3_SHIFT))&AIPS_PACRP_WP3_MASK)
#define AIPS_PACRP_SP3_MASK                      0x40000u
#define AIPS_PACRP_SP3_SHIFT                     18
#define AIPS_PACRP_SP3_WIDTH                     1
#define AIPS_PACRP_SP3(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP3_SHIFT))&AIPS_PACRP_SP3_MASK)
#define AIPS_PACRP_TP2_MASK                      0x100000u
#define AIPS_PACRP_TP2_SHIFT                     20
#define AIPS_PACRP_TP2_WIDTH                     1
#define AIPS_PACRP_TP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP2_SHIFT))&AIPS_PACRP_TP2_MASK)
#define AIPS_PACRP_WP2_MASK                      0x200000u
#define AIPS_PACRP_WP2_SHIFT                     21
#define AIPS_PACRP_WP2_WIDTH                     1
#define AIPS_PACRP_WP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP2_SHIFT))&AIPS_PACRP_WP2_MASK)
#define AIPS_PACRP_SP2_MASK                      0x400000u
#define AIPS_PACRP_SP2_SHIFT                     22
#define AIPS_PACRP_SP2_WIDTH                     1
#define AIPS_PACRP_SP2(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP2_SHIFT))&AIPS_PACRP_SP2_MASK)
#define AIPS_PACRP_TP1_MASK                      0x1000000u
#define AIPS_PACRP_TP1_SHIFT                     24
#define AIPS_PACRP_TP1_WIDTH                     1
#define AIPS_PACRP_TP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP1_SHIFT))&AIPS_PACRP_TP1_MASK)
#define AIPS_PACRP_WP1_MASK                      0x2000000u
#define AIPS_PACRP_WP1_SHIFT                     25
#define AIPS_PACRP_WP1_WIDTH                     1
#define AIPS_PACRP_WP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP1_SHIFT))&AIPS_PACRP_WP1_MASK)
#define AIPS_PACRP_SP1_MASK                      0x4000000u
#define AIPS_PACRP_SP1_SHIFT                     26
#define AIPS_PACRP_SP1_WIDTH                     1
#define AIPS_PACRP_SP1(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP1_SHIFT))&AIPS_PACRP_SP1_MASK)
#define AIPS_PACRP_TP0_MASK                      0x10000000u
#define AIPS_PACRP_TP0_SHIFT                     28
#define AIPS_PACRP_TP0_WIDTH                     1
#define AIPS_PACRP_TP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_TP0_SHIFT))&AIPS_PACRP_TP0_MASK)
#define AIPS_PACRP_WP0_MASK                      0x20000000u
#define AIPS_PACRP_WP0_SHIFT                     29
#define AIPS_PACRP_WP0_WIDTH                     1
#define AIPS_PACRP_WP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_WP0_SHIFT))&AIPS_PACRP_WP0_MASK)
#define AIPS_PACRP_SP0_MASK                      0x40000000u
#define AIPS_PACRP_SP0_SHIFT                     30
#define AIPS_PACRP_SP0_WIDTH                     1
#define AIPS_PACRP_SP0(x)                        (((uint32_t)(((uint32_t)(x))<<AIPS_PACRP_SP0_SHIFT))&AIPS_PACRP_SP0_MASK)

/*!
 * @}
 */ /* end of group AIPS_Register_Masks */


/* AIPS - Peripheral instance base addresses */
/** Peripheral AIPS base address */
#define AIPS_BASE                                (0x40000000u)
/** Peripheral AIPS base pointer */
#define AIPS                                     ((AIPS_Type *)AIPS_BASE)
#define AIPS_BASE_PTR                            (AIPS)
/** Array initializer of AIPS peripheral base addresses */
#define AIPS_BASE_ADDRS                          { AIPS_BASE }
/** Array initializer of AIPS peripheral base pointers */
#define AIPS_BASE_PTRS                           { AIPS }

/* ----------------------------------------------------------------------------
   -- AIPS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Accessor_Macros AIPS - Register accessor macros
 * @{
 */


/* AIPS - Register instance definitions */
/* AIPS */
#define AIPS_MPRA                                AIPS_MPRA_REG(AIPS)
#define AIPS_PACRA                               AIPS_PACRA_REG(AIPS)
#define AIPS_PACRB                               AIPS_PACRB_REG(AIPS)
#define AIPS_PACRC                               AIPS_PACRC_REG(AIPS)
#define AIPS_PACRD                               AIPS_PACRD_REG(AIPS)
#define AIPS_PACRE                               AIPS_PACRE_REG(AIPS)
#define AIPS_PACRF                               AIPS_PACRF_REG(AIPS)
#define AIPS_PACRG                               AIPS_PACRG_REG(AIPS)
#define AIPS_PACRH                               AIPS_PACRH_REG(AIPS)
#define AIPS_PACRI                               AIPS_PACRI_REG(AIPS)
#define AIPS_PACRJ                               AIPS_PACRJ_REG(AIPS)
#define AIPS_PACRK                               AIPS_PACRK_REG(AIPS)
#define AIPS_PACRL                               AIPS_PACRL_REG(AIPS)
#define AIPS_PACRM                               AIPS_PACRM_REG(AIPS)
#define AIPS_PACRN                               AIPS_PACRN_REG(AIPS)
#define AIPS_PACRO                               AIPS_PACRO_REG(AIPS)
#define AIPS_PACRP                               AIPS_PACRP_REG(AIPS)

/*!
 * @}
 */ /* end of group AIPS_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group AIPS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- AOI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Peripheral_Access_Layer AOI Peripheral Access Layer
 * @{
 */

/** AOI - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x4 */
    __IO uint16_t BFCRT01;                           /**< Boolean Function Term 0 and 1 Configuration Register for EVENTn, array offset: 0x0, array step: 0x4 */
    __IO uint16_t BFCRT23;                           /**< Boolean Function Term 2 and 3 Configuration Register for EVENTn, array offset: 0x2, array step: 0x4 */
  } BFCRT[4];
} AOI_Type, *AOI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- AOI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Accessor_Macros AOI - Register accessor macros
 * @{
 */


/* AOI - Register accessors */
#define AOI_BFCRT01_REG(base,index)              ((base)->BFCRT[index].BFCRT01)
#define AOI_BFCRT01_COUNT                        4
#define AOI_BFCRT23_REG(base,index)              ((base)->BFCRT[index].BFCRT23)
#define AOI_BFCRT23_COUNT                        4

/*!
 * @}
 */ /* end of group AOI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- AOI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Masks AOI Register Masks
 * @{
 */

/* BFCRT01 Bit Fields */
#define AOI_BFCRT01_PT1_DC_MASK                  0x3u
#define AOI_BFCRT01_PT1_DC_SHIFT                 0
#define AOI_BFCRT01_PT1_DC_WIDTH                 2
#define AOI_BFCRT01_PT1_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_DC_SHIFT))&AOI_BFCRT01_PT1_DC_MASK)
#define AOI_BFCRT01_PT1_CC_MASK                  0xCu
#define AOI_BFCRT01_PT1_CC_SHIFT                 2
#define AOI_BFCRT01_PT1_CC_WIDTH                 2
#define AOI_BFCRT01_PT1_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_CC_SHIFT))&AOI_BFCRT01_PT1_CC_MASK)
#define AOI_BFCRT01_PT1_BC_MASK                  0x30u
#define AOI_BFCRT01_PT1_BC_SHIFT                 4
#define AOI_BFCRT01_PT1_BC_WIDTH                 2
#define AOI_BFCRT01_PT1_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_BC_SHIFT))&AOI_BFCRT01_PT1_BC_MASK)
#define AOI_BFCRT01_PT1_AC_MASK                  0xC0u
#define AOI_BFCRT01_PT1_AC_SHIFT                 6
#define AOI_BFCRT01_PT1_AC_WIDTH                 2
#define AOI_BFCRT01_PT1_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_AC_SHIFT))&AOI_BFCRT01_PT1_AC_MASK)
#define AOI_BFCRT01_PT0_DC_MASK                  0x300u
#define AOI_BFCRT01_PT0_DC_SHIFT                 8
#define AOI_BFCRT01_PT0_DC_WIDTH                 2
#define AOI_BFCRT01_PT0_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_DC_SHIFT))&AOI_BFCRT01_PT0_DC_MASK)
#define AOI_BFCRT01_PT0_CC_MASK                  0xC00u
#define AOI_BFCRT01_PT0_CC_SHIFT                 10
#define AOI_BFCRT01_PT0_CC_WIDTH                 2
#define AOI_BFCRT01_PT0_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_CC_SHIFT))&AOI_BFCRT01_PT0_CC_MASK)
#define AOI_BFCRT01_PT0_BC_MASK                  0x3000u
#define AOI_BFCRT01_PT0_BC_SHIFT                 12
#define AOI_BFCRT01_PT0_BC_WIDTH                 2
#define AOI_BFCRT01_PT0_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_BC_SHIFT))&AOI_BFCRT01_PT0_BC_MASK)
#define AOI_BFCRT01_PT0_AC_MASK                  0xC000u
#define AOI_BFCRT01_PT0_AC_SHIFT                 14
#define AOI_BFCRT01_PT0_AC_WIDTH                 2
#define AOI_BFCRT01_PT0_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_AC_SHIFT))&AOI_BFCRT01_PT0_AC_MASK)
/* BFCRT23 Bit Fields */
#define AOI_BFCRT23_PT3_DC_MASK                  0x3u
#define AOI_BFCRT23_PT3_DC_SHIFT                 0
#define AOI_BFCRT23_PT3_DC_WIDTH                 2
#define AOI_BFCRT23_PT3_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_DC_SHIFT))&AOI_BFCRT23_PT3_DC_MASK)
#define AOI_BFCRT23_PT3_CC_MASK                  0xCu
#define AOI_BFCRT23_PT3_CC_SHIFT                 2
#define AOI_BFCRT23_PT3_CC_WIDTH                 2
#define AOI_BFCRT23_PT3_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_CC_SHIFT))&AOI_BFCRT23_PT3_CC_MASK)
#define AOI_BFCRT23_PT3_BC_MASK                  0x30u
#define AOI_BFCRT23_PT3_BC_SHIFT                 4
#define AOI_BFCRT23_PT3_BC_WIDTH                 2
#define AOI_BFCRT23_PT3_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_BC_SHIFT))&AOI_BFCRT23_PT3_BC_MASK)
#define AOI_BFCRT23_PT3_AC_MASK                  0xC0u
#define AOI_BFCRT23_PT3_AC_SHIFT                 6
#define AOI_BFCRT23_PT3_AC_WIDTH                 2
#define AOI_BFCRT23_PT3_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_AC_SHIFT))&AOI_BFCRT23_PT3_AC_MASK)
#define AOI_BFCRT23_PT2_DC_MASK                  0x300u
#define AOI_BFCRT23_PT2_DC_SHIFT                 8
#define AOI_BFCRT23_PT2_DC_WIDTH                 2
#define AOI_BFCRT23_PT2_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_DC_SHIFT))&AOI_BFCRT23_PT2_DC_MASK)
#define AOI_BFCRT23_PT2_CC_MASK                  0xC00u
#define AOI_BFCRT23_PT2_CC_SHIFT                 10
#define AOI_BFCRT23_PT2_CC_WIDTH                 2
#define AOI_BFCRT23_PT2_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_CC_SHIFT))&AOI_BFCRT23_PT2_CC_MASK)
#define AOI_BFCRT23_PT2_BC_MASK                  0x3000u
#define AOI_BFCRT23_PT2_BC_SHIFT                 12
#define AOI_BFCRT23_PT2_BC_WIDTH                 2
#define AOI_BFCRT23_PT2_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_BC_SHIFT))&AOI_BFCRT23_PT2_BC_MASK)
#define AOI_BFCRT23_PT2_AC_MASK                  0xC000u
#define AOI_BFCRT23_PT2_AC_SHIFT                 14
#define AOI_BFCRT23_PT2_AC_WIDTH                 2
#define AOI_BFCRT23_PT2_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_AC_SHIFT))&AOI_BFCRT23_PT2_AC_MASK)

/*!
 * @}
 */ /* end of group AOI_Register_Masks */


/* AOI - Peripheral instance base addresses */
/** Peripheral AOI base address */
#define AOI_BASE                                 (0x4005B000u)
/** Peripheral AOI base pointer */
#define AOI                                      ((AOI_Type *)AOI_BASE)
#define AOI_BASE_PTR                             (AOI)
/** Array initializer of AOI peripheral base addresses */
#define AOI_BASE_ADDRS                           { AOI_BASE }
/** Array initializer of AOI peripheral base pointers */
#define AOI_BASE_PTRS                            { AOI }

/* ----------------------------------------------------------------------------
   -- AOI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Accessor_Macros AOI - Register accessor macros
 * @{
 */


/* AOI - Register instance definitions */
/* AOI */
#define AOI_BFCRT010                             AOI_BFCRT01_REG(AOI,0)
#define AOI_BFCRT230                             AOI_BFCRT23_REG(AOI,0)
#define AOI_BFCRT011                             AOI_BFCRT01_REG(AOI,1)
#define AOI_BFCRT231                             AOI_BFCRT23_REG(AOI,1)
#define AOI_BFCRT012                             AOI_BFCRT01_REG(AOI,2)
#define AOI_BFCRT232                             AOI_BFCRT23_REG(AOI,2)
#define AOI_BFCRT013                             AOI_BFCRT01_REG(AOI,3)
#define AOI_BFCRT233                             AOI_BFCRT23_REG(AOI,3)

/* AOI - Register array accessors */
#define AOI_BFCRT01(index)                       AOI_BFCRT01_REG(AOI,index)
#define AOI_BFCRT23(index)                       AOI_BFCRT23_REG(AOI,index)

/*!
 * @}
 */ /* end of group AOI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group AOI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CAN Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral_Access_Layer CAN Peripheral Access Layer
 * @{
 */

/** CAN - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
  __IO uint32_t CTRL1;                             /**< Control 1 register, offset: 0x4 */
  __IO uint32_t TIMER;                             /**< Free Running Timer, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t RXMGMASK;                          /**< Rx Mailboxes Global Mask Register, offset: 0x10 */
  __IO uint32_t RX14MASK;                          /**< Rx 14 Mask register, offset: 0x14 */
  __IO uint32_t RX15MASK;                          /**< Rx 15 Mask register, offset: 0x18 */
  __IO uint32_t ECR;                               /**< Error Counter, offset: 0x1C */
  __IO uint32_t ESR1;                              /**< Error and Status 1 register, offset: 0x20 */
       uint8_t RESERVED_1[4];
  __IO uint32_t IMASK1;                            /**< Interrupt Masks 1 register, offset: 0x28 */
       uint8_t RESERVED_2[4];
  __IO uint32_t IFLAG1;                            /**< Interrupt Flags 1 register, offset: 0x30 */
  __IO uint32_t CTRL2;                             /**< Control 2 register, offset: 0x34 */
  __I  uint32_t ESR2;                              /**< Error and Status 2 register, offset: 0x38 */
       uint8_t RESERVED_3[8];
  __I  uint32_t CRCR;                              /**< CRC Register, offset: 0x44 */
  __IO uint32_t RXFGMASK;                          /**< Rx FIFO Global Mask register, offset: 0x48 */
  __I  uint32_t RXFIR;                             /**< Rx FIFO Information Register, offset: 0x4C */
  __IO uint32_t CBT;                               /**< CAN Bit Timing Register, offset: 0x50 */
       uint8_t RESERVED_4[44];
  struct {                                         /* offset: 0x80, array step: 0x10 */
    __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 15 CS Register, array offset: 0x80, array step: 0x10 */
    __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 15 ID Register, array offset: 0x84, array step: 0x10 */
    __IO uint32_t WORD0;                             /**< Message Buffer 0 WORD0 Register..Message Buffer 15 WORD0 Register, array offset: 0x88, array step: 0x10 */
    __IO uint32_t WORD1;                             /**< Message Buffer 0 WORD1 Register..Message Buffer 15 WORD1 Register, array offset: 0x8C, array step: 0x10 */
  } MB[16];
       uint8_t RESERVED_5[1792];
  __IO uint32_t RXIMR[16];                         /**< Rx Individual Mask Registers, array offset: 0x880, array step: 0x4 */
} CAN_Type, *CAN_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register accessors */
#define CAN_MCR_REG(base)                        ((base)->MCR)
#define CAN_CTRL1_REG(base)                      ((base)->CTRL1)
#define CAN_TIMER_REG(base)                      ((base)->TIMER)
#define CAN_RXMGMASK_REG(base)                   ((base)->RXMGMASK)
#define CAN_RX14MASK_REG(base)                   ((base)->RX14MASK)
#define CAN_RX15MASK_REG(base)                   ((base)->RX15MASK)
#define CAN_ECR_REG(base)                        ((base)->ECR)
#define CAN_ESR1_REG(base)                       ((base)->ESR1)
#define CAN_IMASK1_REG(base)                     ((base)->IMASK1)
#define CAN_IFLAG1_REG(base)                     ((base)->IFLAG1)
#define CAN_CTRL2_REG(base)                      ((base)->CTRL2)
#define CAN_ESR2_REG(base)                       ((base)->ESR2)
#define CAN_CRCR_REG(base)                       ((base)->CRCR)
#define CAN_RXFGMASK_REG(base)                   ((base)->RXFGMASK)
#define CAN_RXFIR_REG(base)                      ((base)->RXFIR)
#define CAN_CBT_REG(base)                        ((base)->CBT)
#define CAN_CS_REG(base,index)                   ((base)->MB[index].CS)
#define CAN_CS_COUNT                             16
#define CAN_ID_REG(base,index)                   ((base)->MB[index].ID)
#define CAN_ID_COUNT                             16
#define CAN_WORD0_REG(base,index)                ((base)->MB[index].WORD0)
#define CAN_WORD0_COUNT                          16
#define CAN_WORD1_REG(base,index)                ((base)->MB[index].WORD1)
#define CAN_WORD1_COUNT                          16
#define CAN_RXIMR_REG(base,index)                ((base)->RXIMR[index])
#define CAN_RXIMR_COUNT                          16

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/* MCR Bit Fields */
#define CAN_MCR_MAXMB_MASK                       0x7Fu
#define CAN_MCR_MAXMB_SHIFT                      0
#define CAN_MCR_MAXMB_WIDTH                      7
#define CAN_MCR_MAXMB(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_MCR_MAXMB_SHIFT))&CAN_MCR_MAXMB_MASK)
#define CAN_MCR_IDAM_MASK                        0x300u
#define CAN_MCR_IDAM_SHIFT                       8
#define CAN_MCR_IDAM_WIDTH                       2
#define CAN_MCR_IDAM(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)
#define CAN_MCR_AEN_MASK                         0x1000u
#define CAN_MCR_AEN_SHIFT                        12
#define CAN_MCR_AEN_WIDTH                        1
#define CAN_MCR_AEN(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_MCR_AEN_SHIFT))&CAN_MCR_AEN_MASK)
#define CAN_MCR_LPRIOEN_MASK                     0x2000u
#define CAN_MCR_LPRIOEN_SHIFT                    13
#define CAN_MCR_LPRIOEN_WIDTH                    1
#define CAN_MCR_LPRIOEN(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_MCR_LPRIOEN_SHIFT))&CAN_MCR_LPRIOEN_MASK)
#define CAN_MCR_DMA_MASK                         0x8000u
#define CAN_MCR_DMA_SHIFT                        15
#define CAN_MCR_DMA_WIDTH                        1
#define CAN_MCR_DMA(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_MCR_DMA_SHIFT))&CAN_MCR_DMA_MASK)
#define CAN_MCR_IRMQ_MASK                        0x10000u
#define CAN_MCR_IRMQ_SHIFT                       16
#define CAN_MCR_IRMQ_WIDTH                       1
#define CAN_MCR_IRMQ(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_IRMQ_SHIFT))&CAN_MCR_IRMQ_MASK)
#define CAN_MCR_SRXDIS_MASK                      0x20000u
#define CAN_MCR_SRXDIS_SHIFT                     17
#define CAN_MCR_SRXDIS_WIDTH                     1
#define CAN_MCR_SRXDIS(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_SRXDIS_SHIFT))&CAN_MCR_SRXDIS_MASK)
#define CAN_MCR_DOZE_MASK                        0x40000u
#define CAN_MCR_DOZE_SHIFT                       18
#define CAN_MCR_DOZE_WIDTH                       1
#define CAN_MCR_DOZE(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_DOZE_SHIFT))&CAN_MCR_DOZE_MASK)
#define CAN_MCR_WAKSRC_MASK                      0x80000u
#define CAN_MCR_WAKSRC_SHIFT                     19
#define CAN_MCR_WAKSRC_WIDTH                     1
#define CAN_MCR_WAKSRC(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_WAKSRC_SHIFT))&CAN_MCR_WAKSRC_MASK)
#define CAN_MCR_LPMACK_MASK                      0x100000u
#define CAN_MCR_LPMACK_SHIFT                     20
#define CAN_MCR_LPMACK_WIDTH                     1
#define CAN_MCR_LPMACK(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_LPMACK_SHIFT))&CAN_MCR_LPMACK_MASK)
#define CAN_MCR_WRNEN_MASK                       0x200000u
#define CAN_MCR_WRNEN_SHIFT                      21
#define CAN_MCR_WRNEN_WIDTH                      1
#define CAN_MCR_WRNEN(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_MCR_WRNEN_SHIFT))&CAN_MCR_WRNEN_MASK)
#define CAN_MCR_SLFWAK_MASK                      0x400000u
#define CAN_MCR_SLFWAK_SHIFT                     22
#define CAN_MCR_SLFWAK_WIDTH                     1
#define CAN_MCR_SLFWAK(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_SLFWAK_SHIFT))&CAN_MCR_SLFWAK_MASK)
#define CAN_MCR_SUPV_MASK                        0x800000u
#define CAN_MCR_SUPV_SHIFT                       23
#define CAN_MCR_SUPV_WIDTH                       1
#define CAN_MCR_SUPV(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_SUPV_SHIFT))&CAN_MCR_SUPV_MASK)
#define CAN_MCR_FRZACK_MASK                      0x1000000u
#define CAN_MCR_FRZACK_SHIFT                     24
#define CAN_MCR_FRZACK_WIDTH                     1
#define CAN_MCR_FRZACK(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_FRZACK_SHIFT))&CAN_MCR_FRZACK_MASK)
#define CAN_MCR_SOFTRST_MASK                     0x2000000u
#define CAN_MCR_SOFTRST_SHIFT                    25
#define CAN_MCR_SOFTRST_WIDTH                    1
#define CAN_MCR_SOFTRST(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_MCR_SOFTRST_SHIFT))&CAN_MCR_SOFTRST_MASK)
#define CAN_MCR_WAKMSK_MASK                      0x4000000u
#define CAN_MCR_WAKMSK_SHIFT                     26
#define CAN_MCR_WAKMSK_WIDTH                     1
#define CAN_MCR_WAKMSK(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_WAKMSK_SHIFT))&CAN_MCR_WAKMSK_MASK)
#define CAN_MCR_NOTRDY_MASK                      0x8000000u
#define CAN_MCR_NOTRDY_SHIFT                     27
#define CAN_MCR_NOTRDY_WIDTH                     1
#define CAN_MCR_NOTRDY(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_MCR_NOTRDY_SHIFT))&CAN_MCR_NOTRDY_MASK)
#define CAN_MCR_HALT_MASK                        0x10000000u
#define CAN_MCR_HALT_SHIFT                       28
#define CAN_MCR_HALT_WIDTH                       1
#define CAN_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_HALT_SHIFT))&CAN_MCR_HALT_MASK)
#define CAN_MCR_RFEN_MASK                        0x20000000u
#define CAN_MCR_RFEN_SHIFT                       29
#define CAN_MCR_RFEN_WIDTH                       1
#define CAN_MCR_RFEN(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_RFEN_SHIFT))&CAN_MCR_RFEN_MASK)
#define CAN_MCR_FRZ_MASK                         0x40000000u
#define CAN_MCR_FRZ_SHIFT                        30
#define CAN_MCR_FRZ_WIDTH                        1
#define CAN_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_MCR_FRZ_SHIFT))&CAN_MCR_FRZ_MASK)
#define CAN_MCR_MDIS_MASK                        0x80000000u
#define CAN_MCR_MDIS_SHIFT                       31
#define CAN_MCR_MDIS_WIDTH                       1
#define CAN_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_MDIS_SHIFT))&CAN_MCR_MDIS_MASK)
/* CTRL1 Bit Fields */
#define CAN_CTRL1_PROPSEG_MASK                   0x7u
#define CAN_CTRL1_PROPSEG_SHIFT                  0
#define CAN_CTRL1_PROPSEG_WIDTH                  3
#define CAN_CTRL1_PROPSEG(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PROPSEG_SHIFT))&CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM_MASK                       0x8u
#define CAN_CTRL1_LOM_SHIFT                      3
#define CAN_CTRL1_LOM_WIDTH                      1
#define CAN_CTRL1_LOM(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_LOM_SHIFT))&CAN_CTRL1_LOM_MASK)
#define CAN_CTRL1_LBUF_MASK                      0x10u
#define CAN_CTRL1_LBUF_SHIFT                     4
#define CAN_CTRL1_LBUF_WIDTH                     1
#define CAN_CTRL1_LBUF(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_LBUF_SHIFT))&CAN_CTRL1_LBUF_MASK)
#define CAN_CTRL1_TSYN_MASK                      0x20u
#define CAN_CTRL1_TSYN_SHIFT                     5
#define CAN_CTRL1_TSYN_WIDTH                     1
#define CAN_CTRL1_TSYN(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_TSYN_SHIFT))&CAN_CTRL1_TSYN_MASK)
#define CAN_CTRL1_BOFFREC_MASK                   0x40u
#define CAN_CTRL1_BOFFREC_SHIFT                  6
#define CAN_CTRL1_BOFFREC_WIDTH                  1
#define CAN_CTRL1_BOFFREC(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_BOFFREC_SHIFT))&CAN_CTRL1_BOFFREC_MASK)
#define CAN_CTRL1_SMP_MASK                       0x80u
#define CAN_CTRL1_SMP_SHIFT                      7
#define CAN_CTRL1_SMP_WIDTH                      1
#define CAN_CTRL1_SMP(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_SMP_SHIFT))&CAN_CTRL1_SMP_MASK)
#define CAN_CTRL1_RWRNMSK_MASK                   0x400u
#define CAN_CTRL1_RWRNMSK_SHIFT                  10
#define CAN_CTRL1_RWRNMSK_WIDTH                  1
#define CAN_CTRL1_RWRNMSK(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_RWRNMSK_SHIFT))&CAN_CTRL1_RWRNMSK_MASK)
#define CAN_CTRL1_TWRNMSK_MASK                   0x800u
#define CAN_CTRL1_TWRNMSK_SHIFT                  11
#define CAN_CTRL1_TWRNMSK_WIDTH                  1
#define CAN_CTRL1_TWRNMSK(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_TWRNMSK_SHIFT))&CAN_CTRL1_TWRNMSK_MASK)
#define CAN_CTRL1_LPB_MASK                       0x1000u
#define CAN_CTRL1_LPB_SHIFT                      12
#define CAN_CTRL1_LPB_WIDTH                      1
#define CAN_CTRL1_LPB(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_LPB_SHIFT))&CAN_CTRL1_LPB_MASK)
#define CAN_CTRL1_CLKSRC_MASK                    0x2000u
#define CAN_CTRL1_CLKSRC_SHIFT                   13
#define CAN_CTRL1_CLKSRC_WIDTH                   1
#define CAN_CTRL1_CLKSRC(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_CLKSRC_SHIFT))&CAN_CTRL1_CLKSRC_MASK)
#define CAN_CTRL1_ERRMSK_MASK                    0x4000u
#define CAN_CTRL1_ERRMSK_SHIFT                   14
#define CAN_CTRL1_ERRMSK_WIDTH                   1
#define CAN_CTRL1_ERRMSK(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_ERRMSK_SHIFT))&CAN_CTRL1_ERRMSK_MASK)
#define CAN_CTRL1_BOFFMSK_MASK                   0x8000u
#define CAN_CTRL1_BOFFMSK_SHIFT                  15
#define CAN_CTRL1_BOFFMSK_WIDTH                  1
#define CAN_CTRL1_BOFFMSK(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_BOFFMSK_SHIFT))&CAN_CTRL1_BOFFMSK_MASK)
#define CAN_CTRL1_PSEG2_MASK                     0x70000u
#define CAN_CTRL1_PSEG2_SHIFT                    16
#define CAN_CTRL1_PSEG2_WIDTH                    3
#define CAN_CTRL1_PSEG2(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PSEG2_SHIFT))&CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_MASK                     0x380000u
#define CAN_CTRL1_PSEG1_SHIFT                    19
#define CAN_CTRL1_PSEG1_WIDTH                    3
#define CAN_CTRL1_PSEG1(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PSEG1_SHIFT))&CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_MASK                       0xC00000u
#define CAN_CTRL1_RJW_SHIFT                      22
#define CAN_CTRL1_RJW_WIDTH                      2
#define CAN_CTRL1_RJW(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_RJW_SHIFT))&CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_MASK                   0xFF000000u
#define CAN_CTRL1_PRESDIV_SHIFT                  24
#define CAN_CTRL1_PRESDIV_WIDTH                  8
#define CAN_CTRL1_PRESDIV(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PRESDIV_SHIFT))&CAN_CTRL1_PRESDIV_MASK)
/* TIMER Bit Fields */
#define CAN_TIMER_TIMER_MASK                     0xFFFFu
#define CAN_TIMER_TIMER_SHIFT                    0
#define CAN_TIMER_TIMER_WIDTH                    16
#define CAN_TIMER_TIMER(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_TIMER_TIMER_SHIFT))&CAN_TIMER_TIMER_MASK)
/* RXMGMASK Bit Fields */
#define CAN_RXMGMASK_MG_MASK                     0xFFFFFFFFu
#define CAN_RXMGMASK_MG_SHIFT                    0
#define CAN_RXMGMASK_MG_WIDTH                    32
#define CAN_RXMGMASK_MG(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_RXMGMASK_MG_SHIFT))&CAN_RXMGMASK_MG_MASK)
/* RX14MASK Bit Fields */
#define CAN_RX14MASK_RX14M_MASK                  0xFFFFFFFFu
#define CAN_RX14MASK_RX14M_SHIFT                 0
#define CAN_RX14MASK_RX14M_WIDTH                 32
#define CAN_RX14MASK_RX14M(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RX14MASK_RX14M_SHIFT))&CAN_RX14MASK_RX14M_MASK)
/* RX15MASK Bit Fields */
#define CAN_RX15MASK_RX15M_MASK                  0xFFFFFFFFu
#define CAN_RX15MASK_RX15M_SHIFT                 0
#define CAN_RX15MASK_RX15M_WIDTH                 32
#define CAN_RX15MASK_RX15M(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RX15MASK_RX15M_SHIFT))&CAN_RX15MASK_RX15M_MASK)
/* ECR Bit Fields */
#define CAN_ECR_TXERRCNT_MASK                    0xFFu
#define CAN_ECR_TXERRCNT_SHIFT                   0
#define CAN_ECR_TXERRCNT_WIDTH                   8
#define CAN_ECR_TXERRCNT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ECR_TXERRCNT_SHIFT))&CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_MASK                    0xFF00u
#define CAN_ECR_RXERRCNT_SHIFT                   8
#define CAN_ECR_RXERRCNT_WIDTH                   8
#define CAN_ECR_RXERRCNT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ECR_RXERRCNT_SHIFT))&CAN_ECR_RXERRCNT_MASK)
/* ESR1 Bit Fields */
#define CAN_ESR1_WAKINT_MASK                     0x1u
#define CAN_ESR1_WAKINT_SHIFT                    0
#define CAN_ESR1_WAKINT_WIDTH                    1
#define CAN_ESR1_WAKINT(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_WAKINT_SHIFT))&CAN_ESR1_WAKINT_MASK)
#define CAN_ESR1_ERRINT_MASK                     0x2u
#define CAN_ESR1_ERRINT_SHIFT                    1
#define CAN_ESR1_ERRINT_WIDTH                    1
#define CAN_ESR1_ERRINT(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_ERRINT_SHIFT))&CAN_ESR1_ERRINT_MASK)
#define CAN_ESR1_BOFFINT_MASK                    0x4u
#define CAN_ESR1_BOFFINT_SHIFT                   2
#define CAN_ESR1_BOFFINT_WIDTH                   1
#define CAN_ESR1_BOFFINT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_BOFFINT_SHIFT))&CAN_ESR1_BOFFINT_MASK)
#define CAN_ESR1_RX_MASK                         0x8u
#define CAN_ESR1_RX_SHIFT                        3
#define CAN_ESR1_RX_WIDTH                        1
#define CAN_ESR1_RX(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_RX_SHIFT))&CAN_ESR1_RX_MASK)
#define CAN_ESR1_FLTCONF_MASK                    0x30u
#define CAN_ESR1_FLTCONF_SHIFT                   4
#define CAN_ESR1_FLTCONF_WIDTH                   2
#define CAN_ESR1_FLTCONF(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_FLTCONF_SHIFT))&CAN_ESR1_FLTCONF_MASK)
#define CAN_ESR1_TX_MASK                         0x40u
#define CAN_ESR1_TX_SHIFT                        6
#define CAN_ESR1_TX_WIDTH                        1
#define CAN_ESR1_TX(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_TX_SHIFT))&CAN_ESR1_TX_MASK)
#define CAN_ESR1_IDLE_MASK                       0x80u
#define CAN_ESR1_IDLE_SHIFT                      7
#define CAN_ESR1_IDLE_WIDTH                      1
#define CAN_ESR1_IDLE(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_IDLE_SHIFT))&CAN_ESR1_IDLE_MASK)
#define CAN_ESR1_RXWRN_MASK                      0x100u
#define CAN_ESR1_RXWRN_SHIFT                     8
#define CAN_ESR1_RXWRN_WIDTH                     1
#define CAN_ESR1_RXWRN(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_RXWRN_SHIFT))&CAN_ESR1_RXWRN_MASK)
#define CAN_ESR1_TXWRN_MASK                      0x200u
#define CAN_ESR1_TXWRN_SHIFT                     9
#define CAN_ESR1_TXWRN_WIDTH                     1
#define CAN_ESR1_TXWRN(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_TXWRN_SHIFT))&CAN_ESR1_TXWRN_MASK)
#define CAN_ESR1_STFERR_MASK                     0x400u
#define CAN_ESR1_STFERR_SHIFT                    10
#define CAN_ESR1_STFERR_WIDTH                    1
#define CAN_ESR1_STFERR(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_STFERR_SHIFT))&CAN_ESR1_STFERR_MASK)
#define CAN_ESR1_FRMERR_MASK                     0x800u
#define CAN_ESR1_FRMERR_SHIFT                    11
#define CAN_ESR1_FRMERR_WIDTH                    1
#define CAN_ESR1_FRMERR(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_FRMERR_SHIFT))&CAN_ESR1_FRMERR_MASK)
#define CAN_ESR1_CRCERR_MASK                     0x1000u
#define CAN_ESR1_CRCERR_SHIFT                    12
#define CAN_ESR1_CRCERR_WIDTH                    1
#define CAN_ESR1_CRCERR(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_CRCERR_SHIFT))&CAN_ESR1_CRCERR_MASK)
#define CAN_ESR1_ACKERR_MASK                     0x2000u
#define CAN_ESR1_ACKERR_SHIFT                    13
#define CAN_ESR1_ACKERR_WIDTH                    1
#define CAN_ESR1_ACKERR(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_ACKERR_SHIFT))&CAN_ESR1_ACKERR_MASK)
#define CAN_ESR1_BIT0ERR_MASK                    0x4000u
#define CAN_ESR1_BIT0ERR_SHIFT                   14
#define CAN_ESR1_BIT0ERR_WIDTH                   1
#define CAN_ESR1_BIT0ERR(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_BIT0ERR_SHIFT))&CAN_ESR1_BIT0ERR_MASK)
#define CAN_ESR1_BIT1ERR_MASK                    0x8000u
#define CAN_ESR1_BIT1ERR_SHIFT                   15
#define CAN_ESR1_BIT1ERR_WIDTH                   1
#define CAN_ESR1_BIT1ERR(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_BIT1ERR_SHIFT))&CAN_ESR1_BIT1ERR_MASK)
#define CAN_ESR1_RWRNINT_MASK                    0x10000u
#define CAN_ESR1_RWRNINT_SHIFT                   16
#define CAN_ESR1_RWRNINT_WIDTH                   1
#define CAN_ESR1_RWRNINT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_RWRNINT_SHIFT))&CAN_ESR1_RWRNINT_MASK)
#define CAN_ESR1_TWRNINT_MASK                    0x20000u
#define CAN_ESR1_TWRNINT_SHIFT                   17
#define CAN_ESR1_TWRNINT_WIDTH                   1
#define CAN_ESR1_TWRNINT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_TWRNINT_SHIFT))&CAN_ESR1_TWRNINT_MASK)
#define CAN_ESR1_SYNCH_MASK                      0x40000u
#define CAN_ESR1_SYNCH_SHIFT                     18
#define CAN_ESR1_SYNCH_WIDTH                     1
#define CAN_ESR1_SYNCH(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_SYNCH_SHIFT))&CAN_ESR1_SYNCH_MASK)
#define CAN_ESR1_BOFFDONEINT_MASK                0x80000u
#define CAN_ESR1_BOFFDONEINT_SHIFT               19
#define CAN_ESR1_BOFFDONEINT_WIDTH               1
#define CAN_ESR1_BOFFDONEINT(x)                  (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_BOFFDONEINT_SHIFT))&CAN_ESR1_BOFFDONEINT_MASK)
#define CAN_ESR1_ERROVR_MASK                     0x200000u
#define CAN_ESR1_ERROVR_SHIFT                    21
#define CAN_ESR1_ERROVR_WIDTH                    1
#define CAN_ESR1_ERROVR(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_ERROVR_SHIFT))&CAN_ESR1_ERROVR_MASK)
/* IMASK1 Bit Fields */
#define CAN_IMASK1_BUF31TO0M_MASK                0xFFFFFFFFu
#define CAN_IMASK1_BUF31TO0M_SHIFT               0
#define CAN_IMASK1_BUF31TO0M_WIDTH               32
#define CAN_IMASK1_BUF31TO0M(x)                  (((uint32_t)(((uint32_t)(x))<<CAN_IMASK1_BUF31TO0M_SHIFT))&CAN_IMASK1_BUF31TO0M_MASK)
/* IFLAG1 Bit Fields */
#define CAN_IFLAG1_BUF0I_MASK                    0x1u
#define CAN_IFLAG1_BUF0I_SHIFT                   0
#define CAN_IFLAG1_BUF0I_WIDTH                   1
#define CAN_IFLAG1_BUF0I(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF0I_SHIFT))&CAN_IFLAG1_BUF0I_MASK)
#define CAN_IFLAG1_BUF4TO1I_MASK                 0x1Eu
#define CAN_IFLAG1_BUF4TO1I_SHIFT                1
#define CAN_IFLAG1_BUF4TO1I_WIDTH                4
#define CAN_IFLAG1_BUF4TO1I(x)                   (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF4TO1I_SHIFT))&CAN_IFLAG1_BUF4TO1I_MASK)
#define CAN_IFLAG1_BUF5I_MASK                    0x20u
#define CAN_IFLAG1_BUF5I_SHIFT                   5
#define CAN_IFLAG1_BUF5I_WIDTH                   1
#define CAN_IFLAG1_BUF5I(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF5I_SHIFT))&CAN_IFLAG1_BUF5I_MASK)
#define CAN_IFLAG1_BUF6I_MASK                    0x40u
#define CAN_IFLAG1_BUF6I_SHIFT                   6
#define CAN_IFLAG1_BUF6I_WIDTH                   1
#define CAN_IFLAG1_BUF6I(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF6I_SHIFT))&CAN_IFLAG1_BUF6I_MASK)
#define CAN_IFLAG1_BUF7I_MASK                    0x80u
#define CAN_IFLAG1_BUF7I_SHIFT                   7
#define CAN_IFLAG1_BUF7I_WIDTH                   1
#define CAN_IFLAG1_BUF7I(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF7I_SHIFT))&CAN_IFLAG1_BUF7I_MASK)
#define CAN_IFLAG1_BUF31TO8I_MASK                0xFFFFFF00u
#define CAN_IFLAG1_BUF31TO8I_SHIFT               8
#define CAN_IFLAG1_BUF31TO8I_WIDTH               24
#define CAN_IFLAG1_BUF31TO8I(x)                  (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF31TO8I_SHIFT))&CAN_IFLAG1_BUF31TO8I_MASK)
/* CTRL2 Bit Fields */
#define CAN_CTRL2_EACEN_MASK                     0x10000u
#define CAN_CTRL2_EACEN_SHIFT                    16
#define CAN_CTRL2_EACEN_WIDTH                    1
#define CAN_CTRL2_EACEN(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_EACEN_SHIFT))&CAN_CTRL2_EACEN_MASK)
#define CAN_CTRL2_RRS_MASK                       0x20000u
#define CAN_CTRL2_RRS_SHIFT                      17
#define CAN_CTRL2_RRS_WIDTH                      1
#define CAN_CTRL2_RRS(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_RRS_SHIFT))&CAN_CTRL2_RRS_MASK)
#define CAN_CTRL2_MRP_MASK                       0x40000u
#define CAN_CTRL2_MRP_SHIFT                      18
#define CAN_CTRL2_MRP_WIDTH                      1
#define CAN_CTRL2_MRP(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_MRP_SHIFT))&CAN_CTRL2_MRP_MASK)
#define CAN_CTRL2_TASD_MASK                      0xF80000u
#define CAN_CTRL2_TASD_SHIFT                     19
#define CAN_CTRL2_TASD_WIDTH                     5
#define CAN_CTRL2_TASD(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_TASD_SHIFT))&CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_MASK                      0xF000000u
#define CAN_CTRL2_RFFN_SHIFT                     24
#define CAN_CTRL2_RFFN_WIDTH                     4
#define CAN_CTRL2_RFFN(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_RFFN_SHIFT))&CAN_CTRL2_RFFN_MASK)
#define CAN_CTRL2_BOFFDONEMSK_MASK               0x40000000u
#define CAN_CTRL2_BOFFDONEMSK_SHIFT              30
#define CAN_CTRL2_BOFFDONEMSK_WIDTH              1
#define CAN_CTRL2_BOFFDONEMSK(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_BOFFDONEMSK_SHIFT))&CAN_CTRL2_BOFFDONEMSK_MASK)
/* ESR2 Bit Fields */
#define CAN_ESR2_IMB_MASK                        0x2000u
#define CAN_ESR2_IMB_SHIFT                       13
#define CAN_ESR2_IMB_WIDTH                       1
#define CAN_ESR2_IMB(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_ESR2_IMB_SHIFT))&CAN_ESR2_IMB_MASK)
#define CAN_ESR2_VPS_MASK                        0x4000u
#define CAN_ESR2_VPS_SHIFT                       14
#define CAN_ESR2_VPS_WIDTH                       1
#define CAN_ESR2_VPS(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_ESR2_VPS_SHIFT))&CAN_ESR2_VPS_MASK)
#define CAN_ESR2_LPTM_MASK                       0x7F0000u
#define CAN_ESR2_LPTM_SHIFT                      16
#define CAN_ESR2_LPTM_WIDTH                      7
#define CAN_ESR2_LPTM(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_ESR2_LPTM_SHIFT))&CAN_ESR2_LPTM_MASK)
/* CRCR Bit Fields */
#define CAN_CRCR_TXCRC_MASK                      0x7FFFu
#define CAN_CRCR_TXCRC_SHIFT                     0
#define CAN_CRCR_TXCRC_WIDTH                     15
#define CAN_CRCR_TXCRC(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CRCR_TXCRC_SHIFT))&CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_MASK                      0x7F0000u
#define CAN_CRCR_MBCRC_SHIFT                     16
#define CAN_CRCR_MBCRC_WIDTH                     7
#define CAN_CRCR_MBCRC(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CRCR_MBCRC_SHIFT))&CAN_CRCR_MBCRC_MASK)
/* RXFGMASK Bit Fields */
#define CAN_RXFGMASK_FGM_MASK                    0xFFFFFFFFu
#define CAN_RXFGMASK_FGM_SHIFT                   0
#define CAN_RXFGMASK_FGM_WIDTH                   32
#define CAN_RXFGMASK_FGM(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_RXFGMASK_FGM_SHIFT))&CAN_RXFGMASK_FGM_MASK)
/* RXFIR Bit Fields */
#define CAN_RXFIR_IDHIT_MASK                     0x1FFu
#define CAN_RXFIR_IDHIT_SHIFT                    0
#define CAN_RXFIR_IDHIT_WIDTH                    9
#define CAN_RXFIR_IDHIT(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_RXFIR_IDHIT_SHIFT))&CAN_RXFIR_IDHIT_MASK)
/* CBT Bit Fields */
#define CAN_CBT_EPSEG2_MASK                      0x1Fu
#define CAN_CBT_EPSEG2_SHIFT                     0
#define CAN_CBT_EPSEG2_WIDTH                     5
#define CAN_CBT_EPSEG2(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CBT_EPSEG2_SHIFT))&CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_MASK                      0x3E0u
#define CAN_CBT_EPSEG1_SHIFT                     5
#define CAN_CBT_EPSEG1_WIDTH                     5
#define CAN_CBT_EPSEG1(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CBT_EPSEG1_SHIFT))&CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_MASK                    0xFC00u
#define CAN_CBT_EPROPSEG_SHIFT                   10
#define CAN_CBT_EPROPSEG_WIDTH                   6
#define CAN_CBT_EPROPSEG(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_CBT_EPROPSEG_SHIFT))&CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_MASK                        0xF0000u
#define CAN_CBT_ERJW_SHIFT                       16
#define CAN_CBT_ERJW_WIDTH                       4
#define CAN_CBT_ERJW(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_CBT_ERJW_SHIFT))&CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_MASK                    0x7FE00000u
#define CAN_CBT_EPRESDIV_SHIFT                   21
#define CAN_CBT_EPRESDIV_WIDTH                   10
#define CAN_CBT_EPRESDIV(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_CBT_EPRESDIV_SHIFT))&CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF_MASK                         0x80000000u
#define CAN_CBT_BTF_SHIFT                        31
#define CAN_CBT_BTF_WIDTH                        1
#define CAN_CBT_BTF(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_CBT_BTF_SHIFT))&CAN_CBT_BTF_MASK)
/* CS Bit Fields */
#define CAN_CS_TIME_STAMP_MASK                   0xFFFFu
#define CAN_CS_TIME_STAMP_SHIFT                  0
#define CAN_CS_TIME_STAMP_WIDTH                  16
#define CAN_CS_TIME_STAMP(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CS_TIME_STAMP_SHIFT))&CAN_CS_TIME_STAMP_MASK)
#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC_WIDTH                         4
#define CAN_CS_DLC(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_DLC_SHIFT))&CAN_CS_DLC_MASK)
#define CAN_CS_RTR_MASK                          0x100000u
#define CAN_CS_RTR_SHIFT                         20
#define CAN_CS_RTR_WIDTH                         1
#define CAN_CS_RTR(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_RTR_SHIFT))&CAN_CS_RTR_MASK)
#define CAN_CS_IDE_MASK                          0x200000u
#define CAN_CS_IDE_SHIFT                         21
#define CAN_CS_IDE_WIDTH                         1
#define CAN_CS_IDE(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_IDE_SHIFT))&CAN_CS_IDE_MASK)
#define CAN_CS_SRR_MASK                          0x400000u
#define CAN_CS_SRR_SHIFT                         22
#define CAN_CS_SRR_WIDTH                         1
#define CAN_CS_SRR(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_SRR_SHIFT))&CAN_CS_SRR_MASK)
#define CAN_CS_CODE_MASK                         0xF000000u
#define CAN_CS_CODE_SHIFT                        24
#define CAN_CS_CODE_WIDTH                        4
#define CAN_CS_CODE(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_CS_CODE_SHIFT))&CAN_CS_CODE_MASK)
#define CAN_CS_ESI_MASK                          0x20000000u
#define CAN_CS_ESI_SHIFT                         29
#define CAN_CS_ESI_WIDTH                         1
#define CAN_CS_ESI(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_ESI_SHIFT))&CAN_CS_ESI_MASK)
#define CAN_CS_BRS_MASK                          0x40000000u
#define CAN_CS_BRS_SHIFT                         30
#define CAN_CS_BRS_WIDTH                         1
#define CAN_CS_BRS(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_BRS_SHIFT))&CAN_CS_BRS_MASK)
#define CAN_CS_EDL_MASK                          0x80000000u
#define CAN_CS_EDL_SHIFT                         31
#define CAN_CS_EDL_WIDTH                         1
#define CAN_CS_EDL(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_EDL_SHIFT))&CAN_CS_EDL_MASK)
/* ID Bit Fields */
#define CAN_ID_EXT_MASK                          0x3FFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT_WIDTH                         18
#define CAN_ID_EXT(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_ID_EXT_SHIFT))&CAN_ID_EXT_MASK)
#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD_WIDTH                         11
#define CAN_ID_STD(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK)
#define CAN_ID_PRIO_MASK                         0xE0000000u
#define CAN_ID_PRIO_SHIFT                        29
#define CAN_ID_PRIO_WIDTH                        3
#define CAN_ID_PRIO(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_ID_PRIO_SHIFT))&CAN_ID_PRIO_MASK)
/* WORD0 Bit Fields */
#define CAN_WORD0_DATA_BYTE_3_MASK               0xFFu
#define CAN_WORD0_DATA_BYTE_3_SHIFT              0
#define CAN_WORD0_DATA_BYTE_3_WIDTH              8
#define CAN_WORD0_DATA_BYTE_3(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_3_SHIFT))&CAN_WORD0_DATA_BYTE_3_MASK)
#define CAN_WORD0_DATA_BYTE_2_MASK               0xFF00u
#define CAN_WORD0_DATA_BYTE_2_SHIFT              8
#define CAN_WORD0_DATA_BYTE_2_WIDTH              8
#define CAN_WORD0_DATA_BYTE_2(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_2_SHIFT))&CAN_WORD0_DATA_BYTE_2_MASK)
#define CAN_WORD0_DATA_BYTE_1_MASK               0xFF0000u
#define CAN_WORD0_DATA_BYTE_1_SHIFT              16
#define CAN_WORD0_DATA_BYTE_1_WIDTH              8
#define CAN_WORD0_DATA_BYTE_1(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_1_SHIFT))&CAN_WORD0_DATA_BYTE_1_MASK)
#define CAN_WORD0_DATA_BYTE_0_MASK               0xFF000000u
#define CAN_WORD0_DATA_BYTE_0_SHIFT              24
#define CAN_WORD0_DATA_BYTE_0_WIDTH              8
#define CAN_WORD0_DATA_BYTE_0(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_0_SHIFT))&CAN_WORD0_DATA_BYTE_0_MASK)
/* WORD1 Bit Fields */
#define CAN_WORD1_DATA_BYTE_7_MASK               0xFFu
#define CAN_WORD1_DATA_BYTE_7_SHIFT              0
#define CAN_WORD1_DATA_BYTE_7_WIDTH              8
#define CAN_WORD1_DATA_BYTE_7(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_7_SHIFT))&CAN_WORD1_DATA_BYTE_7_MASK)
#define CAN_WORD1_DATA_BYTE_6_MASK               0xFF00u
#define CAN_WORD1_DATA_BYTE_6_SHIFT              8
#define CAN_WORD1_DATA_BYTE_6_WIDTH              8
#define CAN_WORD1_DATA_BYTE_6(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_6_SHIFT))&CAN_WORD1_DATA_BYTE_6_MASK)
#define CAN_WORD1_DATA_BYTE_5_MASK               0xFF0000u
#define CAN_WORD1_DATA_BYTE_5_SHIFT              16
#define CAN_WORD1_DATA_BYTE_5_WIDTH              8
#define CAN_WORD1_DATA_BYTE_5(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_5_SHIFT))&CAN_WORD1_DATA_BYTE_5_MASK)
#define CAN_WORD1_DATA_BYTE_4_MASK               0xFF000000u
#define CAN_WORD1_DATA_BYTE_4_SHIFT              24
#define CAN_WORD1_DATA_BYTE_4_WIDTH              8
#define CAN_WORD1_DATA_BYTE_4(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_4_SHIFT))&CAN_WORD1_DATA_BYTE_4_MASK)
/* RXIMR Bit Fields */
#define CAN_RXIMR_MI_MASK                        0xFFFFFFFFu
#define CAN_RXIMR_MI_SHIFT                       0
#define CAN_RXIMR_MI_WIDTH                       32
#define CAN_RXIMR_MI(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_RXIMR_MI_SHIFT))&CAN_RXIMR_MI_MASK)

/*!
 * @}
 */ /* end of group CAN_Register_Masks */


/* CAN - Peripheral instance base addresses */
/** Peripheral CAN0 base address */
#define CAN0_BASE                                (0x40024000u)
/** Peripheral CAN0 base pointer */
#define CAN0                                     ((CAN_Type *)CAN0_BASE)
#define CAN0_BASE_PTR                            (CAN0)
/** Peripheral CAN1 base address */
#define CAN1_BASE                                (0x40025000u)
/** Peripheral CAN1 base pointer */
#define CAN1                                     ((CAN_Type *)CAN1_BASE)
#define CAN1_BASE_PTR                            (CAN1)
/** Array initializer of CAN peripheral base addresses */
#define CAN_BASE_ADDRS                           { CAN0_BASE, CAN1_BASE }
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { CAN0, CAN1 }

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register instance definitions */
/* CAN0 */
#define CAN0_MCR                                 CAN_MCR_REG(CAN0)
#define CAN0_CTRL1                               CAN_CTRL1_REG(CAN0)
#define CAN0_TIMER                               CAN_TIMER_REG(CAN0)
#define CAN0_RXMGMASK                            CAN_RXMGMASK_REG(CAN0)
#define CAN0_RX14MASK                            CAN_RX14MASK_REG(CAN0)
#define CAN0_RX15MASK                            CAN_RX15MASK_REG(CAN0)
#define CAN0_ECR                                 CAN_ECR_REG(CAN0)
#define CAN0_ESR1                                CAN_ESR1_REG(CAN0)
#define CAN0_IMASK1                              CAN_IMASK1_REG(CAN0)
#define CAN0_IFLAG1                              CAN_IFLAG1_REG(CAN0)
#define CAN0_CTRL2                               CAN_CTRL2_REG(CAN0)
#define CAN0_ESR2                                CAN_ESR2_REG(CAN0)
#define CAN0_CRCR                                CAN_CRCR_REG(CAN0)
#define CAN0_RXFGMASK                            CAN_RXFGMASK_REG(CAN0)
#define CAN0_RXFIR                               CAN_RXFIR_REG(CAN0)
#define CAN0_CBT                                 CAN_CBT_REG(CAN0)
#define CAN0_CS0                                 CAN_CS_REG(CAN0,0)
#define CAN0_ID0                                 CAN_ID_REG(CAN0,0)
#define CAN0_WORD00                              CAN_WORD0_REG(CAN0,0)
#define CAN0_WORD10                              CAN_WORD1_REG(CAN0,0)
#define CAN0_CS1                                 CAN_CS_REG(CAN0,1)
#define CAN0_ID1                                 CAN_ID_REG(CAN0,1)
#define CAN0_WORD01                              CAN_WORD0_REG(CAN0,1)
#define CAN0_WORD11                              CAN_WORD1_REG(CAN0,1)
#define CAN0_CS2                                 CAN_CS_REG(CAN0,2)
#define CAN0_ID2                                 CAN_ID_REG(CAN0,2)
#define CAN0_WORD02                              CAN_WORD0_REG(CAN0,2)
#define CAN0_WORD12                              CAN_WORD1_REG(CAN0,2)
#define CAN0_CS3                                 CAN_CS_REG(CAN0,3)
#define CAN0_ID3                                 CAN_ID_REG(CAN0,3)
#define CAN0_WORD03                              CAN_WORD0_REG(CAN0,3)
#define CAN0_WORD13                              CAN_WORD1_REG(CAN0,3)
#define CAN0_CS4                                 CAN_CS_REG(CAN0,4)
#define CAN0_ID4                                 CAN_ID_REG(CAN0,4)
#define CAN0_WORD04                              CAN_WORD0_REG(CAN0,4)
#define CAN0_WORD14                              CAN_WORD1_REG(CAN0,4)
#define CAN0_CS5                                 CAN_CS_REG(CAN0,5)
#define CAN0_ID5                                 CAN_ID_REG(CAN0,5)
#define CAN0_WORD05                              CAN_WORD0_REG(CAN0,5)
#define CAN0_WORD15                              CAN_WORD1_REG(CAN0,5)
#define CAN0_CS6                                 CAN_CS_REG(CAN0,6)
#define CAN0_ID6                                 CAN_ID_REG(CAN0,6)
#define CAN0_WORD06                              CAN_WORD0_REG(CAN0,6)
#define CAN0_WORD16                              CAN_WORD1_REG(CAN0,6)
#define CAN0_CS7                                 CAN_CS_REG(CAN0,7)
#define CAN0_ID7                                 CAN_ID_REG(CAN0,7)
#define CAN0_WORD07                              CAN_WORD0_REG(CAN0,7)
#define CAN0_WORD17                              CAN_WORD1_REG(CAN0,7)
#define CAN0_CS8                                 CAN_CS_REG(CAN0,8)
#define CAN0_ID8                                 CAN_ID_REG(CAN0,8)
#define CAN0_WORD08                              CAN_WORD0_REG(CAN0,8)
#define CAN0_WORD18                              CAN_WORD1_REG(CAN0,8)
#define CAN0_CS9                                 CAN_CS_REG(CAN0,9)
#define CAN0_ID9                                 CAN_ID_REG(CAN0,9)
#define CAN0_WORD09                              CAN_WORD0_REG(CAN0,9)
#define CAN0_WORD19                              CAN_WORD1_REG(CAN0,9)
#define CAN0_CS10                                CAN_CS_REG(CAN0,10)
#define CAN0_ID10                                CAN_ID_REG(CAN0,10)
#define CAN0_WORD010                             CAN_WORD0_REG(CAN0,10)
#define CAN0_WORD110                             CAN_WORD1_REG(CAN0,10)
#define CAN0_CS11                                CAN_CS_REG(CAN0,11)
#define CAN0_ID11                                CAN_ID_REG(CAN0,11)
#define CAN0_WORD011                             CAN_WORD0_REG(CAN0,11)
#define CAN0_WORD111                             CAN_WORD1_REG(CAN0,11)
#define CAN0_CS12                                CAN_CS_REG(CAN0,12)
#define CAN0_ID12                                CAN_ID_REG(CAN0,12)
#define CAN0_WORD012                             CAN_WORD0_REG(CAN0,12)
#define CAN0_WORD112                             CAN_WORD1_REG(CAN0,12)
#define CAN0_CS13                                CAN_CS_REG(CAN0,13)
#define CAN0_ID13                                CAN_ID_REG(CAN0,13)
#define CAN0_WORD013                             CAN_WORD0_REG(CAN0,13)
#define CAN0_WORD113                             CAN_WORD1_REG(CAN0,13)
#define CAN0_CS14                                CAN_CS_REG(CAN0,14)
#define CAN0_ID14                                CAN_ID_REG(CAN0,14)
#define CAN0_WORD014                             CAN_WORD0_REG(CAN0,14)
#define CAN0_WORD114                             CAN_WORD1_REG(CAN0,14)
#define CAN0_CS15                                CAN_CS_REG(CAN0,15)
#define CAN0_ID15                                CAN_ID_REG(CAN0,15)
#define CAN0_WORD015                             CAN_WORD0_REG(CAN0,15)
#define CAN0_WORD115                             CAN_WORD1_REG(CAN0,15)
#define CAN0_RXIMR0                              CAN_RXIMR_REG(CAN0,0)
#define CAN0_RXIMR1                              CAN_RXIMR_REG(CAN0,1)
#define CAN0_RXIMR2                              CAN_RXIMR_REG(CAN0,2)
#define CAN0_RXIMR3                              CAN_RXIMR_REG(CAN0,3)
#define CAN0_RXIMR4                              CAN_RXIMR_REG(CAN0,4)
#define CAN0_RXIMR5                              CAN_RXIMR_REG(CAN0,5)
#define CAN0_RXIMR6                              CAN_RXIMR_REG(CAN0,6)
#define CAN0_RXIMR7                              CAN_RXIMR_REG(CAN0,7)
#define CAN0_RXIMR8                              CAN_RXIMR_REG(CAN0,8)
#define CAN0_RXIMR9                              CAN_RXIMR_REG(CAN0,9)
#define CAN0_RXIMR10                             CAN_RXIMR_REG(CAN0,10)
#define CAN0_RXIMR11                             CAN_RXIMR_REG(CAN0,11)
#define CAN0_RXIMR12                             CAN_RXIMR_REG(CAN0,12)
#define CAN0_RXIMR13                             CAN_RXIMR_REG(CAN0,13)
#define CAN0_RXIMR14                             CAN_RXIMR_REG(CAN0,14)
#define CAN0_RXIMR15                             CAN_RXIMR_REG(CAN0,15)
/* CAN1 */
#define CAN1_MCR                                 CAN_MCR_REG(CAN1)
#define CAN1_CTRL1                               CAN_CTRL1_REG(CAN1)
#define CAN1_TIMER                               CAN_TIMER_REG(CAN1)
#define CAN1_RXMGMASK                            CAN_RXMGMASK_REG(CAN1)
#define CAN1_RX14MASK                            CAN_RX14MASK_REG(CAN1)
#define CAN1_RX15MASK                            CAN_RX15MASK_REG(CAN1)
#define CAN1_ECR                                 CAN_ECR_REG(CAN1)
#define CAN1_ESR1                                CAN_ESR1_REG(CAN1)
#define CAN1_IMASK1                              CAN_IMASK1_REG(CAN1)
#define CAN1_IFLAG1                              CAN_IFLAG1_REG(CAN1)
#define CAN1_CTRL2                               CAN_CTRL2_REG(CAN1)
#define CAN1_ESR2                                CAN_ESR2_REG(CAN1)
#define CAN1_CRCR                                CAN_CRCR_REG(CAN1)
#define CAN1_RXFGMASK                            CAN_RXFGMASK_REG(CAN1)
#define CAN1_RXFIR                               CAN_RXFIR_REG(CAN1)
#define CAN1_CBT                                 CAN_CBT_REG(CAN1)
#define CAN1_CS0                                 CAN_CS_REG(CAN1,0)
#define CAN1_ID0                                 CAN_ID_REG(CAN1,0)
#define CAN1_WORD00                              CAN_WORD0_REG(CAN1,0)
#define CAN1_WORD10                              CAN_WORD1_REG(CAN1,0)
#define CAN1_CS1                                 CAN_CS_REG(CAN1,1)
#define CAN1_ID1                                 CAN_ID_REG(CAN1,1)
#define CAN1_WORD01                              CAN_WORD0_REG(CAN1,1)
#define CAN1_WORD11                              CAN_WORD1_REG(CAN1,1)
#define CAN1_CS2                                 CAN_CS_REG(CAN1,2)
#define CAN1_ID2                                 CAN_ID_REG(CAN1,2)
#define CAN1_WORD02                              CAN_WORD0_REG(CAN1,2)
#define CAN1_WORD12                              CAN_WORD1_REG(CAN1,2)
#define CAN1_CS3                                 CAN_CS_REG(CAN1,3)
#define CAN1_ID3                                 CAN_ID_REG(CAN1,3)
#define CAN1_WORD03                              CAN_WORD0_REG(CAN1,3)
#define CAN1_WORD13                              CAN_WORD1_REG(CAN1,3)
#define CAN1_CS4                                 CAN_CS_REG(CAN1,4)
#define CAN1_ID4                                 CAN_ID_REG(CAN1,4)
#define CAN1_WORD04                              CAN_WORD0_REG(CAN1,4)
#define CAN1_WORD14                              CAN_WORD1_REG(CAN1,4)
#define CAN1_CS5                                 CAN_CS_REG(CAN1,5)
#define CAN1_ID5                                 CAN_ID_REG(CAN1,5)
#define CAN1_WORD05                              CAN_WORD0_REG(CAN1,5)
#define CAN1_WORD15                              CAN_WORD1_REG(CAN1,5)
#define CAN1_CS6                                 CAN_CS_REG(CAN1,6)
#define CAN1_ID6                                 CAN_ID_REG(CAN1,6)
#define CAN1_WORD06                              CAN_WORD0_REG(CAN1,6)
#define CAN1_WORD16                              CAN_WORD1_REG(CAN1,6)
#define CAN1_CS7                                 CAN_CS_REG(CAN1,7)
#define CAN1_ID7                                 CAN_ID_REG(CAN1,7)
#define CAN1_WORD07                              CAN_WORD0_REG(CAN1,7)
#define CAN1_WORD17                              CAN_WORD1_REG(CAN1,7)
#define CAN1_CS8                                 CAN_CS_REG(CAN1,8)
#define CAN1_ID8                                 CAN_ID_REG(CAN1,8)
#define CAN1_WORD08                              CAN_WORD0_REG(CAN1,8)
#define CAN1_WORD18                              CAN_WORD1_REG(CAN1,8)
#define CAN1_CS9                                 CAN_CS_REG(CAN1,9)
#define CAN1_ID9                                 CAN_ID_REG(CAN1,9)
#define CAN1_WORD09                              CAN_WORD0_REG(CAN1,9)
#define CAN1_WORD19                              CAN_WORD1_REG(CAN1,9)
#define CAN1_CS10                                CAN_CS_REG(CAN1,10)
#define CAN1_ID10                                CAN_ID_REG(CAN1,10)
#define CAN1_WORD010                             CAN_WORD0_REG(CAN1,10)
#define CAN1_WORD110                             CAN_WORD1_REG(CAN1,10)
#define CAN1_CS11                                CAN_CS_REG(CAN1,11)
#define CAN1_ID11                                CAN_ID_REG(CAN1,11)
#define CAN1_WORD011                             CAN_WORD0_REG(CAN1,11)
#define CAN1_WORD111                             CAN_WORD1_REG(CAN1,11)
#define CAN1_CS12                                CAN_CS_REG(CAN1,12)
#define CAN1_ID12                                CAN_ID_REG(CAN1,12)
#define CAN1_WORD012                             CAN_WORD0_REG(CAN1,12)
#define CAN1_WORD112                             CAN_WORD1_REG(CAN1,12)
#define CAN1_CS13                                CAN_CS_REG(CAN1,13)
#define CAN1_ID13                                CAN_ID_REG(CAN1,13)
#define CAN1_WORD013                             CAN_WORD0_REG(CAN1,13)
#define CAN1_WORD113                             CAN_WORD1_REG(CAN1,13)
#define CAN1_CS14                                CAN_CS_REG(CAN1,14)
#define CAN1_ID14                                CAN_ID_REG(CAN1,14)
#define CAN1_WORD014                             CAN_WORD0_REG(CAN1,14)
#define CAN1_WORD114                             CAN_WORD1_REG(CAN1,14)
#define CAN1_CS15                                CAN_CS_REG(CAN1,15)
#define CAN1_ID15                                CAN_ID_REG(CAN1,15)
#define CAN1_WORD015                             CAN_WORD0_REG(CAN1,15)
#define CAN1_WORD115                             CAN_WORD1_REG(CAN1,15)
#define CAN1_RXIMR0                              CAN_RXIMR_REG(CAN1,0)
#define CAN1_RXIMR1                              CAN_RXIMR_REG(CAN1,1)
#define CAN1_RXIMR2                              CAN_RXIMR_REG(CAN1,2)
#define CAN1_RXIMR3                              CAN_RXIMR_REG(CAN1,3)
#define CAN1_RXIMR4                              CAN_RXIMR_REG(CAN1,4)
#define CAN1_RXIMR5                              CAN_RXIMR_REG(CAN1,5)
#define CAN1_RXIMR6                              CAN_RXIMR_REG(CAN1,6)
#define CAN1_RXIMR7                              CAN_RXIMR_REG(CAN1,7)
#define CAN1_RXIMR8                              CAN_RXIMR_REG(CAN1,8)
#define CAN1_RXIMR9                              CAN_RXIMR_REG(CAN1,9)
#define CAN1_RXIMR10                             CAN_RXIMR_REG(CAN1,10)
#define CAN1_RXIMR11                             CAN_RXIMR_REG(CAN1,11)
#define CAN1_RXIMR12                             CAN_RXIMR_REG(CAN1,12)
#define CAN1_RXIMR13                             CAN_RXIMR_REG(CAN1,13)
#define CAN1_RXIMR14                             CAN_RXIMR_REG(CAN1,14)
#define CAN1_RXIMR15                             CAN_RXIMR_REG(CAN1,15)

/* CAN - Register array accessors */
#define CAN0_CS(index)                           CAN_CS_REG(CAN0,index)
#define CAN1_CS(index)                           CAN_CS_REG(CAN1,index)
#define CAN0_ID(index)                           CAN_ID_REG(CAN0,index)
#define CAN1_ID(index)                           CAN_ID_REG(CAN1,index)
#define CAN0_WORD0(index)                        CAN_WORD0_REG(CAN0,index)
#define CAN1_WORD0(index)                        CAN_WORD0_REG(CAN1,index)
#define CAN0_WORD1(index)                        CAN_WORD1_REG(CAN0,index)
#define CAN1_WORD1(index)                        CAN_WORD1_REG(CAN1,index)
#define CAN0_RXIMR(index)                        CAN_RXIMR_REG(CAN0,index)
#define CAN1_RXIMR(index)                        CAN_RXIMR_REG(CAN1,index)

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */

#define CAN_IMASK1_BUFLM_MASK                    CAN_IMASK1_BUF31TO0M_MASK
#define CAN_IMASK1_BUFLM_SHIFT                   CAN_IMASK1_BUF31TO0M_SHIFT
#define CAN_IMASK1_BUFLM_WIDTH                   CAN_IMASK1_BUF31TO0M_WIDTH
#define CAN_IMASK1_BUFLM(x)                      CAN_IMASK1_BUF31TO0M(x)


/*!
 * @}
 */ /* end of group CAN_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR0;                                /**< CMP Control Register 0, offset: 0x0 */
  __IO uint8_t CR1;                                /**< CMP Control Register 1, offset: 0x1 */
  __IO uint8_t FPR;                                /**< CMP Filter Period Register, offset: 0x2 */
  __IO uint8_t SCR;                                /**< CMP Status and Control Register, offset: 0x3 */
  __IO uint8_t DACCR;                              /**< DAC Control Register, offset: 0x4 */
  __IO uint8_t MUXCR;                              /**< MUX Control Register, offset: 0x5 */
} CMP_Type, *CMP_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register accessors */
#define CMP_CR0_REG(base)                        ((base)->CR0)
#define CMP_CR1_REG(base)                        ((base)->CR1)
#define CMP_FPR_REG(base)                        ((base)->FPR)
#define CMP_SCR_REG(base)                        ((base)->SCR)
#define CMP_DACCR_REG(base)                      ((base)->DACCR)
#define CMP_MUXCR_REG(base)                      ((base)->MUXCR)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/* CR0 Bit Fields */
#define CMP_CR0_HYSTCTR_MASK                     0x3u
#define CMP_CR0_HYSTCTR_SHIFT                    0
#define CMP_CR0_HYSTCTR_WIDTH                    2
#define CMP_CR0_HYSTCTR(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_CR0_HYSTCTR_SHIFT))&CMP_CR0_HYSTCTR_MASK)
#define CMP_CR0_FILTER_CNT_MASK                  0x70u
#define CMP_CR0_FILTER_CNT_SHIFT                 4
#define CMP_CR0_FILTER_CNT_WIDTH                 3
#define CMP_CR0_FILTER_CNT(x)                    (((uint8_t)(((uint8_t)(x))<<CMP_CR0_FILTER_CNT_SHIFT))&CMP_CR0_FILTER_CNT_MASK)
/* CR1 Bit Fields */
#define CMP_CR1_EN_MASK                          0x1u
#define CMP_CR1_EN_SHIFT                         0
#define CMP_CR1_EN_WIDTH                         1
#define CMP_CR1_EN(x)                            (((uint8_t)(((uint8_t)(x))<<CMP_CR1_EN_SHIFT))&CMP_CR1_EN_MASK)
#define CMP_CR1_OPE_MASK                         0x2u
#define CMP_CR1_OPE_SHIFT                        1
#define CMP_CR1_OPE_WIDTH                        1
#define CMP_CR1_OPE(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_CR1_OPE_SHIFT))&CMP_CR1_OPE_MASK)
#define CMP_CR1_COS_MASK                         0x4u
#define CMP_CR1_COS_SHIFT                        2
#define CMP_CR1_COS_WIDTH                        1
#define CMP_CR1_COS(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_CR1_COS_SHIFT))&CMP_CR1_COS_MASK)
#define CMP_CR1_INV_MASK                         0x8u
#define CMP_CR1_INV_SHIFT                        3
#define CMP_CR1_INV_WIDTH                        1
#define CMP_CR1_INV(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_CR1_INV_SHIFT))&CMP_CR1_INV_MASK)
#define CMP_CR1_PMODE_MASK                       0x10u
#define CMP_CR1_PMODE_SHIFT                      4
#define CMP_CR1_PMODE_WIDTH                      1
#define CMP_CR1_PMODE(x)                         (((uint8_t)(((uint8_t)(x))<<CMP_CR1_PMODE_SHIFT))&CMP_CR1_PMODE_MASK)
#define CMP_CR1_TRIGM_MASK                       0x20u
#define CMP_CR1_TRIGM_SHIFT                      5
#define CMP_CR1_TRIGM_WIDTH                      1
#define CMP_CR1_TRIGM(x)                         (((uint8_t)(((uint8_t)(x))<<CMP_CR1_TRIGM_SHIFT))&CMP_CR1_TRIGM_MASK)
#define CMP_CR1_WE_MASK                          0x40u
#define CMP_CR1_WE_SHIFT                         6
#define CMP_CR1_WE_WIDTH                         1
#define CMP_CR1_WE(x)                            (((uint8_t)(((uint8_t)(x))<<CMP_CR1_WE_SHIFT))&CMP_CR1_WE_MASK)
#define CMP_CR1_SE_MASK                          0x80u
#define CMP_CR1_SE_SHIFT                         7
#define CMP_CR1_SE_WIDTH                         1
#define CMP_CR1_SE(x)                            (((uint8_t)(((uint8_t)(x))<<CMP_CR1_SE_SHIFT))&CMP_CR1_SE_MASK)
/* FPR Bit Fields */
#define CMP_FPR_FILT_PER_MASK                    0xFFu
#define CMP_FPR_FILT_PER_SHIFT                   0
#define CMP_FPR_FILT_PER_WIDTH                   8
#define CMP_FPR_FILT_PER(x)                      (((uint8_t)(((uint8_t)(x))<<CMP_FPR_FILT_PER_SHIFT))&CMP_FPR_FILT_PER_MASK)
/* SCR Bit Fields */
#define CMP_SCR_COUT_MASK                        0x1u
#define CMP_SCR_COUT_SHIFT                       0
#define CMP_SCR_COUT_WIDTH                       1
#define CMP_SCR_COUT(x)                          (((uint8_t)(((uint8_t)(x))<<CMP_SCR_COUT_SHIFT))&CMP_SCR_COUT_MASK)
#define CMP_SCR_CFF_MASK                         0x2u
#define CMP_SCR_CFF_SHIFT                        1
#define CMP_SCR_CFF_WIDTH                        1
#define CMP_SCR_CFF(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_SCR_CFF_SHIFT))&CMP_SCR_CFF_MASK)
#define CMP_SCR_CFR_MASK                         0x4u
#define CMP_SCR_CFR_SHIFT                        2
#define CMP_SCR_CFR_WIDTH                        1
#define CMP_SCR_CFR(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_SCR_CFR_SHIFT))&CMP_SCR_CFR_MASK)
#define CMP_SCR_IEF_MASK                         0x8u
#define CMP_SCR_IEF_SHIFT                        3
#define CMP_SCR_IEF_WIDTH                        1
#define CMP_SCR_IEF(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_SCR_IEF_SHIFT))&CMP_SCR_IEF_MASK)
#define CMP_SCR_IER_MASK                         0x10u
#define CMP_SCR_IER_SHIFT                        4
#define CMP_SCR_IER_WIDTH                        1
#define CMP_SCR_IER(x)                           (((uint8_t)(((uint8_t)(x))<<CMP_SCR_IER_SHIFT))&CMP_SCR_IER_MASK)
#define CMP_SCR_DMAEN_MASK                       0x40u
#define CMP_SCR_DMAEN_SHIFT                      6
#define CMP_SCR_DMAEN_WIDTH                      1
#define CMP_SCR_DMAEN(x)                         (((uint8_t)(((uint8_t)(x))<<CMP_SCR_DMAEN_SHIFT))&CMP_SCR_DMAEN_MASK)
/* DACCR Bit Fields */
#define CMP_DACCR_VOSEL_MASK                     0x3Fu
#define CMP_DACCR_VOSEL_SHIFT                    0
#define CMP_DACCR_VOSEL_WIDTH                    6
#define CMP_DACCR_VOSEL(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_DACCR_VOSEL_SHIFT))&CMP_DACCR_VOSEL_MASK)
#define CMP_DACCR_VRSEL_MASK                     0x40u
#define CMP_DACCR_VRSEL_SHIFT                    6
#define CMP_DACCR_VRSEL_WIDTH                    1
#define CMP_DACCR_VRSEL(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_DACCR_VRSEL_SHIFT))&CMP_DACCR_VRSEL_MASK)
#define CMP_DACCR_DACEN_MASK                     0x80u
#define CMP_DACCR_DACEN_SHIFT                    7
#define CMP_DACCR_DACEN_WIDTH                    1
#define CMP_DACCR_DACEN(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_DACCR_DACEN_SHIFT))&CMP_DACCR_DACEN_MASK)
/* MUXCR Bit Fields */
#define CMP_MUXCR_MSEL_MASK                      0x7u
#define CMP_MUXCR_MSEL_SHIFT                     0
#define CMP_MUXCR_MSEL_WIDTH                     3
#define CMP_MUXCR_MSEL(x)                        (((uint8_t)(((uint8_t)(x))<<CMP_MUXCR_MSEL_SHIFT))&CMP_MUXCR_MSEL_MASK)
#define CMP_MUXCR_PSEL_MASK                      0x38u
#define CMP_MUXCR_PSEL_SHIFT                     3
#define CMP_MUXCR_PSEL_WIDTH                     3
#define CMP_MUXCR_PSEL(x)                        (((uint8_t)(((uint8_t)(x))<<CMP_MUXCR_PSEL_SHIFT))&CMP_MUXCR_PSEL_MASK)

/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
#define CMP0_BASE_PTR                            (CMP0)
/** Peripheral CMP1 base address */
#define CMP1_BASE                                (0x40073008u)
/** Peripheral CMP1 base pointer */
#define CMP1                                     ((CMP_Type *)CMP1_BASE)
#define CMP1_BASE_PTR                            (CMP1)
/** Peripheral CMP2 base address */
#define CMP2_BASE                                (0x40073010u)
/** Peripheral CMP2 base pointer */
#define CMP2                                     ((CMP_Type *)CMP2_BASE)
#define CMP2_BASE_PTR                            (CMP2)
/** Peripheral CMP3 base address */
#define CMP3_BASE                                (0x40073018u)
/** Peripheral CMP3 base pointer */
#define CMP3                                     ((CMP_Type *)CMP3_BASE)
#define CMP3_BASE_PTR                            (CMP3)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMP0_BASE, CMP1_BASE, CMP2_BASE, CMP3_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMP0, CMP1, CMP2, CMP3 }

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register instance definitions */
/* CMP0 */
#define CMP0_CR0                                 CMP_CR0_REG(CMP0)
#define CMP0_CR1                                 CMP_CR1_REG(CMP0)
#define CMP0_FPR                                 CMP_FPR_REG(CMP0)
#define CMP0_SCR                                 CMP_SCR_REG(CMP0)
#define CMP0_DACCR                               CMP_DACCR_REG(CMP0)
#define CMP0_MUXCR                               CMP_MUXCR_REG(CMP0)
/* CMP1 */
#define CMP1_CR0                                 CMP_CR0_REG(CMP1)
#define CMP1_CR1                                 CMP_CR1_REG(CMP1)
#define CMP1_FPR                                 CMP_FPR_REG(CMP1)
#define CMP1_SCR                                 CMP_SCR_REG(CMP1)
#define CMP1_DACCR                               CMP_DACCR_REG(CMP1)
#define CMP1_MUXCR                               CMP_MUXCR_REG(CMP1)
/* CMP2 */
#define CMP2_CR0                                 CMP_CR0_REG(CMP2)
#define CMP2_CR1                                 CMP_CR1_REG(CMP2)
#define CMP2_FPR                                 CMP_FPR_REG(CMP2)
#define CMP2_SCR                                 CMP_SCR_REG(CMP2)
#define CMP2_DACCR                               CMP_DACCR_REG(CMP2)
#define CMP2_MUXCR                               CMP_MUXCR_REG(CMP2)
/* CMP3 */
#define CMP3_CR0                                 CMP_CR0_REG(CMP3)
#define CMP3_CR1                                 CMP_CR1_REG(CMP3)
#define CMP3_FPR                                 CMP_FPR_REG(CMP3)
#define CMP3_SCR                                 CMP_SCR_REG(CMP3)
#define CMP3_DACCR                               CMP_DACCR_REG(CMP3)
#define CMP3_MUXCR                               CMP_MUXCR_REG(CMP3)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  union {                                          /* offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint16_t DATAL;                             /**< CRC_DATAL register., offset: 0x0 */
      __IO uint16_t DATAH;                             /**< CRC_DATAH register., offset: 0x2 */
    } ACCESS16BIT;
    __IO uint32_t DATA;                              /**< CRC Data register, offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint8_t DATALL;                             /**< CRC_DATALL register., offset: 0x0 */
      __IO uint8_t DATALU;                             /**< CRC_DATALU register., offset: 0x1 */
      __IO uint8_t DATAHL;                             /**< CRC_DATAHL register., offset: 0x2 */
      __IO uint8_t DATAHU;                             /**< CRC_DATAHU register., offset: 0x3 */
    } ACCESS8BIT;
  };
  union {                                          /* offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint16_t GPOLYL;                            /**< CRC_GPOLYL register., offset: 0x4 */
      __IO uint16_t GPOLYH;                            /**< CRC_GPOLYH register., offset: 0x6 */
    } GPOLY_ACCESS16BIT;
    __IO uint32_t GPOLY;                             /**< CRC Polynomial register, offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint8_t GPOLYLL;                            /**< CRC_GPOLYLL register., offset: 0x4 */
      __IO uint8_t GPOLYLU;                            /**< CRC_GPOLYLU register., offset: 0x5 */
      __IO uint8_t GPOLYHL;                            /**< CRC_GPOLYHL register., offset: 0x6 */
      __IO uint8_t GPOLYHU;                            /**< CRC_GPOLYHU register., offset: 0x7 */
    } GPOLY_ACCESS8BIT;
  };
  union {                                          /* offset: 0x8 */
    __IO uint32_t CTRL;                              /**< CRC Control register, offset: 0x8 */
    struct {                                         /* offset: 0x8 */
           uint8_t RESERVED_0[3];
      __IO uint8_t CTRLHU;                             /**< CRC_CTRLHU register., offset: 0xB */
    } CTRL_ACCESS8BIT;
  };
} CRC_Type, *CRC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register accessors */
#define CRC_DATAL_REG(base)                      ((base)->ACCESS16BIT.DATAL)
#define CRC_DATAH_REG(base)                      ((base)->ACCESS16BIT.DATAH)
#define CRC_DATA_REG(base)                       ((base)->DATA)
#define CRC_DATALL_REG(base)                     ((base)->ACCESS8BIT.DATALL)
#define CRC_DATALU_REG(base)                     ((base)->ACCESS8BIT.DATALU)
#define CRC_DATAHL_REG(base)                     ((base)->ACCESS8BIT.DATAHL)
#define CRC_DATAHU_REG(base)                     ((base)->ACCESS8BIT.DATAHU)
#define CRC_GPOLYL_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYL)
#define CRC_GPOLYH_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYH)
#define CRC_GPOLY_REG(base)                      ((base)->GPOLY)
#define CRC_GPOLYLL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLL)
#define CRC_GPOLYLU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLU)
#define CRC_GPOLYHL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHL)
#define CRC_GPOLYHU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHU)
#define CRC_CTRL_REG(base)                       ((base)->CTRL)
#define CRC_CTRLHU_REG(base)                     ((base)->CTRL_ACCESS8BIT.CTRLHU)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/* DATAL Bit Fields */
#define CRC_DATAL_DATAL_MASK                     0xFFFFu
#define CRC_DATAL_DATAL_SHIFT                    0
#define CRC_DATAL_DATAL_WIDTH                    16
#define CRC_DATAL_DATAL(x)                       (((uint16_t)(((uint16_t)(x))<<CRC_DATAL_DATAL_SHIFT))&CRC_DATAL_DATAL_MASK)
/* DATAH Bit Fields */
#define CRC_DATAH_DATAH_MASK                     0xFFFFu
#define CRC_DATAH_DATAH_SHIFT                    0
#define CRC_DATAH_DATAH_WIDTH                    16
#define CRC_DATAH_DATAH(x)                       (((uint16_t)(((uint16_t)(x))<<CRC_DATAH_DATAH_SHIFT))&CRC_DATAH_DATAH_MASK)
/* DATA Bit Fields */
#define CRC_DATA_LL_MASK                         0xFFu
#define CRC_DATA_LL_SHIFT                        0
#define CRC_DATA_LL_WIDTH                        8
#define CRC_DATA_LL(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_LL_SHIFT))&CRC_DATA_LL_MASK)
#define CRC_DATA_LU_MASK                         0xFF00u
#define CRC_DATA_LU_SHIFT                        8
#define CRC_DATA_LU_WIDTH                        8
#define CRC_DATA_LU(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_LU_SHIFT))&CRC_DATA_LU_MASK)
#define CRC_DATA_HL_MASK                         0xFF0000u
#define CRC_DATA_HL_SHIFT                        16
#define CRC_DATA_HL_WIDTH                        8
#define CRC_DATA_HL(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_HL_SHIFT))&CRC_DATA_HL_MASK)
#define CRC_DATA_HU_MASK                         0xFF000000u
#define CRC_DATA_HU_SHIFT                        24
#define CRC_DATA_HU_WIDTH                        8
#define CRC_DATA_HU(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_HU_SHIFT))&CRC_DATA_HU_MASK)
/* DATALL Bit Fields */
#define CRC_DATALL_DATALL_MASK                   0xFFu
#define CRC_DATALL_DATALL_SHIFT                  0
#define CRC_DATALL_DATALL_WIDTH                  8
#define CRC_DATALL_DATALL(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATALL_DATALL_SHIFT))&CRC_DATALL_DATALL_MASK)
/* DATALU Bit Fields */
#define CRC_DATALU_DATALU_MASK                   0xFFu
#define CRC_DATALU_DATALU_SHIFT                  0
#define CRC_DATALU_DATALU_WIDTH                  8
#define CRC_DATALU_DATALU(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATALU_DATALU_SHIFT))&CRC_DATALU_DATALU_MASK)
/* DATAHL Bit Fields */
#define CRC_DATAHL_DATAHL_MASK                   0xFFu
#define CRC_DATAHL_DATAHL_SHIFT                  0
#define CRC_DATAHL_DATAHL_WIDTH                  8
#define CRC_DATAHL_DATAHL(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATAHL_DATAHL_SHIFT))&CRC_DATAHL_DATAHL_MASK)
/* DATAHU Bit Fields */
#define CRC_DATAHU_DATAHU_MASK                   0xFFu
#define CRC_DATAHU_DATAHU_SHIFT                  0
#define CRC_DATAHU_DATAHU_WIDTH                  8
#define CRC_DATAHU_DATAHU(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATAHU_DATAHU_SHIFT))&CRC_DATAHU_DATAHU_MASK)
/* GPOLYL Bit Fields */
#define CRC_GPOLYL_GPOLYL_MASK                   0xFFFFu
#define CRC_GPOLYL_GPOLYL_SHIFT                  0
#define CRC_GPOLYL_GPOLYL_WIDTH                  16
#define CRC_GPOLYL_GPOLYL(x)                     (((uint16_t)(((uint16_t)(x))<<CRC_GPOLYL_GPOLYL_SHIFT))&CRC_GPOLYL_GPOLYL_MASK)
/* GPOLYH Bit Fields */
#define CRC_GPOLYH_GPOLYH_MASK                   0xFFFFu
#define CRC_GPOLYH_GPOLYH_SHIFT                  0
#define CRC_GPOLYH_GPOLYH_WIDTH                  16
#define CRC_GPOLYH_GPOLYH(x)                     (((uint16_t)(((uint16_t)(x))<<CRC_GPOLYH_GPOLYH_SHIFT))&CRC_GPOLYH_GPOLYH_MASK)
/* GPOLY Bit Fields */
#define CRC_GPOLY_LOW_MASK                       0xFFFFu
#define CRC_GPOLY_LOW_SHIFT                      0
#define CRC_GPOLY_LOW_WIDTH                      16
#define CRC_GPOLY_LOW(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_GPOLY_LOW_SHIFT))&CRC_GPOLY_LOW_MASK)
#define CRC_GPOLY_HIGH_MASK                      0xFFFF0000u
#define CRC_GPOLY_HIGH_SHIFT                     16
#define CRC_GPOLY_HIGH_WIDTH                     16
#define CRC_GPOLY_HIGH(x)                        (((uint32_t)(((uint32_t)(x))<<CRC_GPOLY_HIGH_SHIFT))&CRC_GPOLY_HIGH_MASK)
/* GPOLYLL Bit Fields */
#define CRC_GPOLYLL_GPOLYLL_MASK                 0xFFu
#define CRC_GPOLYLL_GPOLYLL_SHIFT                0
#define CRC_GPOLYLL_GPOLYLL_WIDTH                8
#define CRC_GPOLYLL_GPOLYLL(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYLL_GPOLYLL_SHIFT))&CRC_GPOLYLL_GPOLYLL_MASK)
/* GPOLYLU Bit Fields */
#define CRC_GPOLYLU_GPOLYLU_MASK                 0xFFu
#define CRC_GPOLYLU_GPOLYLU_SHIFT                0
#define CRC_GPOLYLU_GPOLYLU_WIDTH                8
#define CRC_GPOLYLU_GPOLYLU(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYLU_GPOLYLU_SHIFT))&CRC_GPOLYLU_GPOLYLU_MASK)
/* GPOLYHL Bit Fields */
#define CRC_GPOLYHL_GPOLYHL_MASK                 0xFFu
#define CRC_GPOLYHL_GPOLYHL_SHIFT                0
#define CRC_GPOLYHL_GPOLYHL_WIDTH                8
#define CRC_GPOLYHL_GPOLYHL(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYHL_GPOLYHL_SHIFT))&CRC_GPOLYHL_GPOLYHL_MASK)
/* GPOLYHU Bit Fields */
#define CRC_GPOLYHU_GPOLYHU_MASK                 0xFFu
#define CRC_GPOLYHU_GPOLYHU_SHIFT                0
#define CRC_GPOLYHU_GPOLYHU_WIDTH                8
#define CRC_GPOLYHU_GPOLYHU(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYHU_GPOLYHU_SHIFT))&CRC_GPOLYHU_GPOLYHU_MASK)
/* CTRL Bit Fields */
#define CRC_CTRL_TCRC_MASK                       0x1000000u
#define CRC_CTRL_TCRC_SHIFT                      24
#define CRC_CTRL_TCRC_WIDTH                      1
#define CRC_CTRL_TCRC(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_TCRC_SHIFT))&CRC_CTRL_TCRC_MASK)
#define CRC_CTRL_WAS_MASK                        0x2000000u
#define CRC_CTRL_WAS_SHIFT                       25
#define CRC_CTRL_WAS_WIDTH                       1
#define CRC_CTRL_WAS(x)                          (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_WAS_SHIFT))&CRC_CTRL_WAS_MASK)
#define CRC_CTRL_FXOR_MASK                       0x4000000u
#define CRC_CTRL_FXOR_SHIFT                      26
#define CRC_CTRL_FXOR_WIDTH                      1
#define CRC_CTRL_FXOR(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_FXOR_SHIFT))&CRC_CTRL_FXOR_MASK)
#define CRC_CTRL_TOTR_MASK                       0x30000000u
#define CRC_CTRL_TOTR_SHIFT                      28
#define CRC_CTRL_TOTR_WIDTH                      2
#define CRC_CTRL_TOTR(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_TOTR_SHIFT))&CRC_CTRL_TOTR_MASK)
#define CRC_CTRL_TOT_MASK                        0xC0000000u
#define CRC_CTRL_TOT_SHIFT                       30
#define CRC_CTRL_TOT_WIDTH                       2
#define CRC_CTRL_TOT(x)                          (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_TOT_SHIFT))&CRC_CTRL_TOT_MASK)
/* CTRLHU Bit Fields */
#define CRC_CTRLHU_TCRC_MASK                     0x1u
#define CRC_CTRLHU_TCRC_SHIFT                    0
#define CRC_CTRLHU_TCRC_WIDTH                    1
#define CRC_CTRLHU_TCRC(x)                       (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_TCRC_SHIFT))&CRC_CTRLHU_TCRC_MASK)
#define CRC_CTRLHU_WAS_MASK                      0x2u
#define CRC_CTRLHU_WAS_SHIFT                     1
#define CRC_CTRLHU_WAS_WIDTH                     1
#define CRC_CTRLHU_WAS(x)                        (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_WAS_SHIFT))&CRC_CTRLHU_WAS_MASK)
#define CRC_CTRLHU_FXOR_MASK                     0x4u
#define CRC_CTRLHU_FXOR_SHIFT                    2
#define CRC_CTRLHU_FXOR_WIDTH                    1
#define CRC_CTRLHU_FXOR(x)                       (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_FXOR_SHIFT))&CRC_CTRLHU_FXOR_MASK)
#define CRC_CTRLHU_TOTR_MASK                     0x30u
#define CRC_CTRLHU_TOTR_SHIFT                    4
#define CRC_CTRLHU_TOTR_WIDTH                    2
#define CRC_CTRLHU_TOTR(x)                       (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_TOTR_SHIFT))&CRC_CTRLHU_TOTR_MASK)
#define CRC_CTRLHU_TOT_MASK                      0xC0u
#define CRC_CTRLHU_TOT_SHIFT                     6
#define CRC_CTRLHU_TOT_WIDTH                     2
#define CRC_CTRLHU_TOT(x)                        (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_TOT_SHIFT))&CRC_CTRLHU_TOT_MASK)

/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE                                 (0x40032000u)
/** Peripheral CRC base pointer */
#define CRC0                                     ((CRC_Type *)CRC_BASE)
#define CRC_BASE_PTR                             (CRC0)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC0 }

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register instance definitions */
/* CRC */
#define CRC_DATA                                 CRC_DATA_REG(CRC0)
#define CRC_DATAL                                CRC_DATAL_REG(CRC0)
#define CRC_DATALL                               CRC_DATALL_REG(CRC0)
#define CRC_DATALU                               CRC_DATALU_REG(CRC0)
#define CRC_DATAH                                CRC_DATAH_REG(CRC0)
#define CRC_DATAHL                               CRC_DATAHL_REG(CRC0)
#define CRC_DATAHU                               CRC_DATAHU_REG(CRC0)
#define CRC_GPOLY                                CRC_GPOLY_REG(CRC0)
#define CRC_GPOLYL                               CRC_GPOLYL_REG(CRC0)
#define CRC_GPOLYLL                              CRC_GPOLYLL_REG(CRC0)
#define CRC_GPOLYLU                              CRC_GPOLYLU_REG(CRC0)
#define CRC_GPOLYH                               CRC_GPOLYH_REG(CRC0)
#define CRC_GPOLYHL                              CRC_GPOLYHL_REG(CRC0)
#define CRC_GPOLYHU                              CRC_GPOLYHU_REG(CRC0)
#define CRC_CTRL                                 CRC_CTRL_REG(CRC0)
#define CRC_CTRLHU                               CRC_CTRLHU_REG(CRC0)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DAC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral_Access_Layer DAC Peripheral Access Layer
 * @{
 */

/** DAC - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x2 */
    __IO uint8_t DATL;                               /**< DAC Data Low Register, array offset: 0x0, array step: 0x2 */
    __IO uint8_t DATH;                               /**< DAC Data High Register, array offset: 0x1, array step: 0x2 */
  } DAT[16];
  __IO uint8_t SR;                                 /**< DAC Status Register, offset: 0x20 */
  __IO uint8_t C0;                                 /**< DAC Control Register, offset: 0x21 */
  __IO uint8_t C1;                                 /**< DAC Control Register 1, offset: 0x22 */
  __IO uint8_t C2;                                 /**< DAC Control Register 2, offset: 0x23 */
} DAC_Type, *DAC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register accessors */
#define DAC_DATL_REG(base,index)                 ((base)->DAT[index].DATL)
#define DAC_DATL_COUNT                           16
#define DAC_DATH_REG(base,index)                 ((base)->DAT[index].DATH)
#define DAC_DATH_COUNT                           16
#define DAC_SR_REG(base)                         ((base)->SR)
#define DAC_C0_REG(base)                         ((base)->C0)
#define DAC_C1_REG(base)                         ((base)->C1)
#define DAC_C2_REG(base)                         ((base)->C2)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/* DATL Bit Fields */
#define DAC_DATL_DATA0_MASK                      0xFFu
#define DAC_DATL_DATA0_SHIFT                     0
#define DAC_DATL_DATA0_WIDTH                     8
#define DAC_DATL_DATA0(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_DATL_DATA0_SHIFT))&DAC_DATL_DATA0_MASK)
/* DATH Bit Fields */
#define DAC_DATH_DATA1_MASK                      0xFu
#define DAC_DATH_DATA1_SHIFT                     0
#define DAC_DATH_DATA1_WIDTH                     4
#define DAC_DATH_DATA1(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_DATH_DATA1_SHIFT))&DAC_DATH_DATA1_MASK)
/* SR Bit Fields */
#define DAC_SR_DACBFRPBF_MASK                    0x1u
#define DAC_SR_DACBFRPBF_SHIFT                   0
#define DAC_SR_DACBFRPBF_WIDTH                   1
#define DAC_SR_DACBFRPBF(x)                      (((uint8_t)(((uint8_t)(x))<<DAC_SR_DACBFRPBF_SHIFT))&DAC_SR_DACBFRPBF_MASK)
#define DAC_SR_DACBFRPTF_MASK                    0x2u
#define DAC_SR_DACBFRPTF_SHIFT                   1
#define DAC_SR_DACBFRPTF_WIDTH                   1
#define DAC_SR_DACBFRPTF(x)                      (((uint8_t)(((uint8_t)(x))<<DAC_SR_DACBFRPTF_SHIFT))&DAC_SR_DACBFRPTF_MASK)
#define DAC_SR_DACBFWMF_MASK                     0x4u
#define DAC_SR_DACBFWMF_SHIFT                    2
#define DAC_SR_DACBFWMF_WIDTH                    1
#define DAC_SR_DACBFWMF(x)                       (((uint8_t)(((uint8_t)(x))<<DAC_SR_DACBFWMF_SHIFT))&DAC_SR_DACBFWMF_MASK)
/* C0 Bit Fields */
#define DAC_C0_DACBBIEN_MASK                     0x1u
#define DAC_C0_DACBBIEN_SHIFT                    0
#define DAC_C0_DACBBIEN_WIDTH                    1
#define DAC_C0_DACBBIEN(x)                       (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACBBIEN_SHIFT))&DAC_C0_DACBBIEN_MASK)
#define DAC_C0_DACBTIEN_MASK                     0x2u
#define DAC_C0_DACBTIEN_SHIFT                    1
#define DAC_C0_DACBTIEN_WIDTH                    1
#define DAC_C0_DACBTIEN(x)                       (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACBTIEN_SHIFT))&DAC_C0_DACBTIEN_MASK)
#define DAC_C0_DACBWIEN_MASK                     0x4u
#define DAC_C0_DACBWIEN_SHIFT                    2
#define DAC_C0_DACBWIEN_WIDTH                    1
#define DAC_C0_DACBWIEN(x)                       (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACBWIEN_SHIFT))&DAC_C0_DACBWIEN_MASK)
#define DAC_C0_LPEN_MASK                         0x8u
#define DAC_C0_LPEN_SHIFT                        3
#define DAC_C0_LPEN_WIDTH                        1
#define DAC_C0_LPEN(x)                           (((uint8_t)(((uint8_t)(x))<<DAC_C0_LPEN_SHIFT))&DAC_C0_LPEN_MASK)
#define DAC_C0_DACSWTRG_MASK                     0x10u
#define DAC_C0_DACSWTRG_SHIFT                    4
#define DAC_C0_DACSWTRG_WIDTH                    1
#define DAC_C0_DACSWTRG(x)                       (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACSWTRG_SHIFT))&DAC_C0_DACSWTRG_MASK)
#define DAC_C0_DACTRGSEL_MASK                    0x20u
#define DAC_C0_DACTRGSEL_SHIFT                   5
#define DAC_C0_DACTRGSEL_WIDTH                   1
#define DAC_C0_DACTRGSEL(x)                      (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACTRGSEL_SHIFT))&DAC_C0_DACTRGSEL_MASK)
#define DAC_C0_DACRFS_MASK                       0x40u
#define DAC_C0_DACRFS_SHIFT                      6
#define DAC_C0_DACRFS_WIDTH                      1
#define DAC_C0_DACRFS(x)                         (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACRFS_SHIFT))&DAC_C0_DACRFS_MASK)
#define DAC_C0_DACEN_MASK                        0x80u
#define DAC_C0_DACEN_SHIFT                       7
#define DAC_C0_DACEN_WIDTH                       1
#define DAC_C0_DACEN(x)                          (((uint8_t)(((uint8_t)(x))<<DAC_C0_DACEN_SHIFT))&DAC_C0_DACEN_MASK)
/* C1 Bit Fields */
#define DAC_C1_DACBFEN_MASK                      0x1u
#define DAC_C1_DACBFEN_SHIFT                     0
#define DAC_C1_DACBFEN_WIDTH                     1
#define DAC_C1_DACBFEN(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C1_DACBFEN_SHIFT))&DAC_C1_DACBFEN_MASK)
#define DAC_C1_DACBFMD_MASK                      0x6u
#define DAC_C1_DACBFMD_SHIFT                     1
#define DAC_C1_DACBFMD_WIDTH                     2
#define DAC_C1_DACBFMD(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C1_DACBFMD_SHIFT))&DAC_C1_DACBFMD_MASK)
#define DAC_C1_DACBFWM_MASK                      0x18u
#define DAC_C1_DACBFWM_SHIFT                     3
#define DAC_C1_DACBFWM_WIDTH                     2
#define DAC_C1_DACBFWM(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C1_DACBFWM_SHIFT))&DAC_C1_DACBFWM_MASK)
#define DAC_C1_DMAEN_MASK                        0x80u
#define DAC_C1_DMAEN_SHIFT                       7
#define DAC_C1_DMAEN_WIDTH                       1
#define DAC_C1_DMAEN(x)                          (((uint8_t)(((uint8_t)(x))<<DAC_C1_DMAEN_SHIFT))&DAC_C1_DMAEN_MASK)
/* C2 Bit Fields */
#define DAC_C2_DACBFUP_MASK                      0xFu
#define DAC_C2_DACBFUP_SHIFT                     0
#define DAC_C2_DACBFUP_WIDTH                     4
#define DAC_C2_DACBFUP(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C2_DACBFUP_SHIFT))&DAC_C2_DACBFUP_MASK)
#define DAC_C2_DACBFRP_MASK                      0xF0u
#define DAC_C2_DACBFRP_SHIFT                     4
#define DAC_C2_DACBFRP_WIDTH                     4
#define DAC_C2_DACBFRP(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C2_DACBFRP_SHIFT))&DAC_C2_DACBFRP_MASK)

/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DAC0 base address */
#define DAC0_BASE                                (0x4003F000u)
/** Peripheral DAC0 base pointer */
#define DAC0                                     ((DAC_Type *)DAC0_BASE)
#define DAC0_BASE_PTR                            (DAC0)
/** Array initializer of DAC peripheral base addresses */
#define DAC_BASE_ADDRS                           { DAC0_BASE }
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DAC0 }

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register instance definitions */
/* DAC0 */
#define DAC0_DAT0L                               DAC_DATL_REG(DAC0,0)
#define DAC0_DAT0H                               DAC_DATH_REG(DAC0,0)
#define DAC0_DAT1L                               DAC_DATL_REG(DAC0,1)
#define DAC0_DAT1H                               DAC_DATH_REG(DAC0,1)
#define DAC0_DAT2L                               DAC_DATL_REG(DAC0,2)
#define DAC0_DAT2H                               DAC_DATH_REG(DAC0,2)
#define DAC0_DAT3L                               DAC_DATL_REG(DAC0,3)
#define DAC0_DAT3H                               DAC_DATH_REG(DAC0,3)
#define DAC0_DAT4L                               DAC_DATL_REG(DAC0,4)
#define DAC0_DAT4H                               DAC_DATH_REG(DAC0,4)
#define DAC0_DAT5L                               DAC_DATL_REG(DAC0,5)
#define DAC0_DAT5H                               DAC_DATH_REG(DAC0,5)
#define DAC0_DAT6L                               DAC_DATL_REG(DAC0,6)
#define DAC0_DAT6H                               DAC_DATH_REG(DAC0,6)
#define DAC0_DAT7L                               DAC_DATL_REG(DAC0,7)
#define DAC0_DAT7H                               DAC_DATH_REG(DAC0,7)
#define DAC0_DAT8L                               DAC_DATL_REG(DAC0,8)
#define DAC0_DAT8H                               DAC_DATH_REG(DAC0,8)
#define DAC0_DAT9L                               DAC_DATL_REG(DAC0,9)
#define DAC0_DAT9H                               DAC_DATH_REG(DAC0,9)
#define DAC0_DAT10L                              DAC_DATL_REG(DAC0,10)
#define DAC0_DAT10H                              DAC_DATH_REG(DAC0,10)
#define DAC0_DAT11L                              DAC_DATL_REG(DAC0,11)
#define DAC0_DAT11H                              DAC_DATH_REG(DAC0,11)
#define DAC0_DAT12L                              DAC_DATL_REG(DAC0,12)
#define DAC0_DAT12H                              DAC_DATH_REG(DAC0,12)
#define DAC0_DAT13L                              DAC_DATL_REG(DAC0,13)
#define DAC0_DAT13H                              DAC_DATH_REG(DAC0,13)
#define DAC0_DAT14L                              DAC_DATL_REG(DAC0,14)
#define DAC0_DAT14H                              DAC_DATH_REG(DAC0,14)
#define DAC0_DAT15L                              DAC_DATL_REG(DAC0,15)
#define DAC0_DAT15H                              DAC_DATH_REG(DAC0,15)
#define DAC0_SR                                  DAC_SR_REG(DAC0)
#define DAC0_C0                                  DAC_C0_REG(DAC0)
#define DAC0_C1                                  DAC_C1_REG(DAC0)
#define DAC0_C2                                  DAC_C2_REG(DAC0)

/* DAC - Register array accessors */
#define DAC0_DATL(index)                         DAC_DATL_REG(DAC0,index)
#define DAC0_DATH(index)                         DAC_DATH_REG(DAC0,index)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DAC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< Control Register, offset: 0x0 */
  __I  uint32_t ES;                                /**< Error Status Register, offset: 0x4 */
       uint8_t RESERVED_0[4];
  __IO uint32_t ERQ;                               /**< Enable Request Register, offset: 0xC */
       uint8_t RESERVED_1[4];
  __IO uint32_t EEI;                               /**< Enable Error Interrupt Register, offset: 0x14 */
  __O  uint8_t CEEI;                               /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  __O  uint8_t SEEI;                               /**< Set Enable Error Interrupt Register, offset: 0x19 */
  __O  uint8_t CERQ;                               /**< Clear Enable Request Register, offset: 0x1A */
  __O  uint8_t SERQ;                               /**< Set Enable Request Register, offset: 0x1B */
  __O  uint8_t CDNE;                               /**< Clear DONE Status Bit Register, offset: 0x1C */
  __O  uint8_t SSRT;                               /**< Set START Bit Register, offset: 0x1D */
  __O  uint8_t CERR;                               /**< Clear Error Register, offset: 0x1E */
  __O  uint8_t CINT;                               /**< Clear Interrupt Request Register, offset: 0x1F */
       uint8_t RESERVED_2[4];
  __IO uint32_t INT;                               /**< Interrupt Request Register, offset: 0x24 */
       uint8_t RESERVED_3[4];
  __IO uint32_t ERR;                               /**< Error Register, offset: 0x2C */
       uint8_t RESERVED_4[4];
  __I  uint32_t HRS;                               /**< Hardware Request Status Register, offset: 0x34 */
       uint8_t RESERVED_5[12];
  __IO uint32_t EARS;                              /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
       uint8_t RESERVED_6[184];
  __IO uint8_t DCHPRI3;                            /**< Channel n Priority Register, offset: 0x100 */
  __IO uint8_t DCHPRI2;                            /**< Channel n Priority Register, offset: 0x101 */
  __IO uint8_t DCHPRI1;                            /**< Channel n Priority Register, offset: 0x102 */
  __IO uint8_t DCHPRI0;                            /**< Channel n Priority Register, offset: 0x103 */
  __IO uint8_t DCHPRI7;                            /**< Channel n Priority Register, offset: 0x104 */
  __IO uint8_t DCHPRI6;                            /**< Channel n Priority Register, offset: 0x105 */
  __IO uint8_t DCHPRI5;                            /**< Channel n Priority Register, offset: 0x106 */
  __IO uint8_t DCHPRI4;                            /**< Channel n Priority Register, offset: 0x107 */
  __IO uint8_t DCHPRI11;                           /**< Channel n Priority Register, offset: 0x108 */
  __IO uint8_t DCHPRI10;                           /**< Channel n Priority Register, offset: 0x109 */
  __IO uint8_t DCHPRI9;                            /**< Channel n Priority Register, offset: 0x10A */
  __IO uint8_t DCHPRI8;                            /**< Channel n Priority Register, offset: 0x10B */
  __IO uint8_t DCHPRI15;                           /**< Channel n Priority Register, offset: 0x10C */
  __IO uint8_t DCHPRI14;                           /**< Channel n Priority Register, offset: 0x10D */
  __IO uint8_t DCHPRI13;                           /**< Channel n Priority Register, offset: 0x10E */
  __IO uint8_t DCHPRI12;                           /**< Channel n Priority Register, offset: 0x10F */
       uint8_t RESERVED_7[3824];
  struct {                                         /* offset: 0x1000, array step: 0x20 */
    __IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    __IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    __IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLNO;                       /**< TCD Minor Byte Count (Minor Loop Mapping Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    };
    __IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    __IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    __IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    };
    __IO uint32_t DLAST_SGA;                         /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    __IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    };
  } TCD[16];
} DMA_Type, *DMA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register accessors */
#define DMA_CR_REG(base)                         ((base)->CR)
#define DMA_ES_REG(base)                         ((base)->ES)
#define DMA_ERQ_REG(base)                        ((base)->ERQ)
#define DMA_EEI_REG(base)                        ((base)->EEI)
#define DMA_CEEI_REG(base)                       ((base)->CEEI)
#define DMA_SEEI_REG(base)                       ((base)->SEEI)
#define DMA_CERQ_REG(base)                       ((base)->CERQ)
#define DMA_SERQ_REG(base)                       ((base)->SERQ)
#define DMA_CDNE_REG(base)                       ((base)->CDNE)
#define DMA_SSRT_REG(base)                       ((base)->SSRT)
#define DMA_CERR_REG(base)                       ((base)->CERR)
#define DMA_CINT_REG(base)                       ((base)->CINT)
#define DMA_INT_REG(base)                        ((base)->INT)
#define DMA_ERR_REG(base)                        ((base)->ERR)
#define DMA_HRS_REG(base)                        ((base)->HRS)
#define DMA_EARS_REG(base)                       ((base)->EARS)
#define DMA_DCHPRI3_REG(base)                    ((base)->DCHPRI3)
#define DMA_DCHPRI2_REG(base)                    ((base)->DCHPRI2)
#define DMA_DCHPRI1_REG(base)                    ((base)->DCHPRI1)
#define DMA_DCHPRI0_REG(base)                    ((base)->DCHPRI0)
#define DMA_DCHPRI7_REG(base)                    ((base)->DCHPRI7)
#define DMA_DCHPRI6_REG(base)                    ((base)->DCHPRI6)
#define DMA_DCHPRI5_REG(base)                    ((base)->DCHPRI5)
#define DMA_DCHPRI4_REG(base)                    ((base)->DCHPRI4)
#define DMA_DCHPRI11_REG(base)                   ((base)->DCHPRI11)
#define DMA_DCHPRI10_REG(base)                   ((base)->DCHPRI10)
#define DMA_DCHPRI9_REG(base)                    ((base)->DCHPRI9)
#define DMA_DCHPRI8_REG(base)                    ((base)->DCHPRI8)
#define DMA_DCHPRI15_REG(base)                   ((base)->DCHPRI15)
#define DMA_DCHPRI14_REG(base)                   ((base)->DCHPRI14)
#define DMA_DCHPRI13_REG(base)                   ((base)->DCHPRI13)
#define DMA_DCHPRI12_REG(base)                   ((base)->DCHPRI12)
#define DMA_SADDR_REG(base,index)                ((base)->TCD[index].SADDR)
#define DMA_SADDR_COUNT                          16
#define DMA_SOFF_REG(base,index)                 ((base)->TCD[index].SOFF)
#define DMA_SOFF_COUNT                           16
#define DMA_ATTR_REG(base,index)                 ((base)->TCD[index].ATTR)
#define DMA_ATTR_COUNT                           16
#define DMA_NBYTES_MLNO_REG(base,index)          ((base)->TCD[index].NBYTES_MLNO)
#define DMA_NBYTES_MLNO_COUNT                    16
#define DMA_NBYTES_MLOFFNO_REG(base,index)       ((base)->TCD[index].NBYTES_MLOFFNO)
#define DMA_NBYTES_MLOFFNO_COUNT                 16
#define DMA_NBYTES_MLOFFYES_REG(base,index)      ((base)->TCD[index].NBYTES_MLOFFYES)
#define DMA_NBYTES_MLOFFYES_COUNT                16
#define DMA_SLAST_REG(base,index)                ((base)->TCD[index].SLAST)
#define DMA_SLAST_COUNT                          16
#define DMA_DADDR_REG(base,index)                ((base)->TCD[index].DADDR)
#define DMA_DADDR_COUNT                          16
#define DMA_DOFF_REG(base,index)                 ((base)->TCD[index].DOFF)
#define DMA_DOFF_COUNT                           16
#define DMA_CITER_ELINKNO_REG(base,index)        ((base)->TCD[index].CITER_ELINKNO)
#define DMA_CITER_ELINKNO_COUNT                  16
#define DMA_CITER_ELINKYES_REG(base,index)       ((base)->TCD[index].CITER_ELINKYES)
#define DMA_CITER_ELINKYES_COUNT                 16
#define DMA_DLAST_SGA_REG(base,index)            ((base)->TCD[index].DLAST_SGA)
#define DMA_DLAST_SGA_COUNT                      16
#define DMA_CSR_REG(base,index)                  ((base)->TCD[index].CSR)
#define DMA_CSR_COUNT                            16
#define DMA_BITER_ELINKNO_REG(base,index)        ((base)->TCD[index].BITER_ELINKNO)
#define DMA_BITER_ELINKNO_COUNT                  16
#define DMA_BITER_ELINKYES_REG(base,index)       ((base)->TCD[index].BITER_ELINKYES)
#define DMA_BITER_ELINKYES_COUNT                 16

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/* CR Bit Fields */
#define DMA_CR_EDBG_MASK                         0x2u
#define DMA_CR_EDBG_SHIFT                        1
#define DMA_CR_EDBG_WIDTH                        1
#define DMA_CR_EDBG(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_CR_EDBG_SHIFT))&DMA_CR_EDBG_MASK)
#define DMA_CR_ERCA_MASK                         0x4u
#define DMA_CR_ERCA_SHIFT                        2
#define DMA_CR_ERCA_WIDTH                        1
#define DMA_CR_ERCA(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_CR_ERCA_SHIFT))&DMA_CR_ERCA_MASK)
#define DMA_CR_HOE_MASK                          0x10u
#define DMA_CR_HOE_SHIFT                         4
#define DMA_CR_HOE_WIDTH                         1
#define DMA_CR_HOE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_CR_HOE_SHIFT))&DMA_CR_HOE_MASK)
#define DMA_CR_HALT_MASK                         0x20u
#define DMA_CR_HALT_SHIFT                        5
#define DMA_CR_HALT_WIDTH                        1
#define DMA_CR_HALT(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_CR_HALT_SHIFT))&DMA_CR_HALT_MASK)
#define DMA_CR_CLM_MASK                          0x40u
#define DMA_CR_CLM_SHIFT                         6
#define DMA_CR_CLM_WIDTH                         1
#define DMA_CR_CLM(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_CR_CLM_SHIFT))&DMA_CR_CLM_MASK)
#define DMA_CR_EMLM_MASK                         0x80u
#define DMA_CR_EMLM_SHIFT                        7
#define DMA_CR_EMLM_WIDTH                        1
#define DMA_CR_EMLM(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_CR_EMLM_SHIFT))&DMA_CR_EMLM_MASK)
#define DMA_CR_ECX_MASK                          0x10000u
#define DMA_CR_ECX_SHIFT                         16
#define DMA_CR_ECX_WIDTH                         1
#define DMA_CR_ECX(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_CR_ECX_SHIFT))&DMA_CR_ECX_MASK)
#define DMA_CR_CX_MASK                           0x20000u
#define DMA_CR_CX_SHIFT                          17
#define DMA_CR_CX_WIDTH                          1
#define DMA_CR_CX(x)                             (((uint32_t)(((uint32_t)(x))<<DMA_CR_CX_SHIFT))&DMA_CR_CX_MASK)
/* ES Bit Fields */
#define DMA_ES_DBE_MASK                          0x1u
#define DMA_ES_DBE_SHIFT                         0
#define DMA_ES_DBE_WIDTH                         1
#define DMA_ES_DBE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_DBE_SHIFT))&DMA_ES_DBE_MASK)
#define DMA_ES_SBE_MASK                          0x2u
#define DMA_ES_SBE_SHIFT                         1
#define DMA_ES_SBE_WIDTH                         1
#define DMA_ES_SBE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_SBE_SHIFT))&DMA_ES_SBE_MASK)
#define DMA_ES_SGE_MASK                          0x4u
#define DMA_ES_SGE_SHIFT                         2
#define DMA_ES_SGE_WIDTH                         1
#define DMA_ES_SGE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_SGE_SHIFT))&DMA_ES_SGE_MASK)
#define DMA_ES_NCE_MASK                          0x8u
#define DMA_ES_NCE_SHIFT                         3
#define DMA_ES_NCE_WIDTH                         1
#define DMA_ES_NCE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_NCE_SHIFT))&DMA_ES_NCE_MASK)
#define DMA_ES_DOE_MASK                          0x10u
#define DMA_ES_DOE_SHIFT                         4
#define DMA_ES_DOE_WIDTH                         1
#define DMA_ES_DOE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_DOE_SHIFT))&DMA_ES_DOE_MASK)
#define DMA_ES_DAE_MASK                          0x20u
#define DMA_ES_DAE_SHIFT                         5
#define DMA_ES_DAE_WIDTH                         1
#define DMA_ES_DAE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_DAE_SHIFT))&DMA_ES_DAE_MASK)
#define DMA_ES_SOE_MASK                          0x40u
#define DMA_ES_SOE_SHIFT                         6
#define DMA_ES_SOE_WIDTH                         1
#define DMA_ES_SOE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_SOE_SHIFT))&DMA_ES_SOE_MASK)
#define DMA_ES_SAE_MASK                          0x80u
#define DMA_ES_SAE_SHIFT                         7
#define DMA_ES_SAE_WIDTH                         1
#define DMA_ES_SAE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_SAE_SHIFT))&DMA_ES_SAE_MASK)
#define DMA_ES_ERRCHN_MASK                       0xF00u
#define DMA_ES_ERRCHN_SHIFT                      8
#define DMA_ES_ERRCHN_WIDTH                      4
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ES_ERRCHN_SHIFT))&DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          0x4000u
#define DMA_ES_CPE_SHIFT                         14
#define DMA_ES_CPE_WIDTH                         1
#define DMA_ES_CPE(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_CPE_SHIFT))&DMA_ES_CPE_MASK)
#define DMA_ES_ECX_MASK                          0x10000u
#define DMA_ES_ECX_SHIFT                         16
#define DMA_ES_ECX_WIDTH                         1
#define DMA_ES_ECX(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_ECX_SHIFT))&DMA_ES_ECX_MASK)
#define DMA_ES_VLD_MASK                          0x80000000u
#define DMA_ES_VLD_SHIFT                         31
#define DMA_ES_VLD_WIDTH                         1
#define DMA_ES_VLD(x)                            (((uint32_t)(((uint32_t)(x))<<DMA_ES_VLD_SHIFT))&DMA_ES_VLD_MASK)
/* ERQ Bit Fields */
#define DMA_ERQ_ERQ0_MASK                        0x1u
#define DMA_ERQ_ERQ0_SHIFT                       0
#define DMA_ERQ_ERQ0_WIDTH                       1
#define DMA_ERQ_ERQ0(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ0_SHIFT))&DMA_ERQ_ERQ0_MASK)
#define DMA_ERQ_ERQ1_MASK                        0x2u
#define DMA_ERQ_ERQ1_SHIFT                       1
#define DMA_ERQ_ERQ1_WIDTH                       1
#define DMA_ERQ_ERQ1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ1_SHIFT))&DMA_ERQ_ERQ1_MASK)
#define DMA_ERQ_ERQ2_MASK                        0x4u
#define DMA_ERQ_ERQ2_SHIFT                       2
#define DMA_ERQ_ERQ2_WIDTH                       1
#define DMA_ERQ_ERQ2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ2_SHIFT))&DMA_ERQ_ERQ2_MASK)
#define DMA_ERQ_ERQ3_MASK                        0x8u
#define DMA_ERQ_ERQ3_SHIFT                       3
#define DMA_ERQ_ERQ3_WIDTH                       1
#define DMA_ERQ_ERQ3(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ3_SHIFT))&DMA_ERQ_ERQ3_MASK)
#define DMA_ERQ_ERQ4_MASK                        0x10u
#define DMA_ERQ_ERQ4_SHIFT                       4
#define DMA_ERQ_ERQ4_WIDTH                       1
#define DMA_ERQ_ERQ4(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ4_SHIFT))&DMA_ERQ_ERQ4_MASK)
#define DMA_ERQ_ERQ5_MASK                        0x20u
#define DMA_ERQ_ERQ5_SHIFT                       5
#define DMA_ERQ_ERQ5_WIDTH                       1
#define DMA_ERQ_ERQ5(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ5_SHIFT))&DMA_ERQ_ERQ5_MASK)
#define DMA_ERQ_ERQ6_MASK                        0x40u
#define DMA_ERQ_ERQ6_SHIFT                       6
#define DMA_ERQ_ERQ6_WIDTH                       1
#define DMA_ERQ_ERQ6(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ6_SHIFT))&DMA_ERQ_ERQ6_MASK)
#define DMA_ERQ_ERQ7_MASK                        0x80u
#define DMA_ERQ_ERQ7_SHIFT                       7
#define DMA_ERQ_ERQ7_WIDTH                       1
#define DMA_ERQ_ERQ7(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ7_SHIFT))&DMA_ERQ_ERQ7_MASK)
#define DMA_ERQ_ERQ8_MASK                        0x100u
#define DMA_ERQ_ERQ8_SHIFT                       8
#define DMA_ERQ_ERQ8_WIDTH                       1
#define DMA_ERQ_ERQ8(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ8_SHIFT))&DMA_ERQ_ERQ8_MASK)
#define DMA_ERQ_ERQ9_MASK                        0x200u
#define DMA_ERQ_ERQ9_SHIFT                       9
#define DMA_ERQ_ERQ9_WIDTH                       1
#define DMA_ERQ_ERQ9(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ9_SHIFT))&DMA_ERQ_ERQ9_MASK)
#define DMA_ERQ_ERQ10_MASK                       0x400u
#define DMA_ERQ_ERQ10_SHIFT                      10
#define DMA_ERQ_ERQ10_WIDTH                      1
#define DMA_ERQ_ERQ10(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ10_SHIFT))&DMA_ERQ_ERQ10_MASK)
#define DMA_ERQ_ERQ11_MASK                       0x800u
#define DMA_ERQ_ERQ11_SHIFT                      11
#define DMA_ERQ_ERQ11_WIDTH                      1
#define DMA_ERQ_ERQ11(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ11_SHIFT))&DMA_ERQ_ERQ11_MASK)
#define DMA_ERQ_ERQ12_MASK                       0x1000u
#define DMA_ERQ_ERQ12_SHIFT                      12
#define DMA_ERQ_ERQ12_WIDTH                      1
#define DMA_ERQ_ERQ12(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ12_SHIFT))&DMA_ERQ_ERQ12_MASK)
#define DMA_ERQ_ERQ13_MASK                       0x2000u
#define DMA_ERQ_ERQ13_SHIFT                      13
#define DMA_ERQ_ERQ13_WIDTH                      1
#define DMA_ERQ_ERQ13(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ13_SHIFT))&DMA_ERQ_ERQ13_MASK)
#define DMA_ERQ_ERQ14_MASK                       0x4000u
#define DMA_ERQ_ERQ14_SHIFT                      14
#define DMA_ERQ_ERQ14_WIDTH                      1
#define DMA_ERQ_ERQ14(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ14_SHIFT))&DMA_ERQ_ERQ14_MASK)
#define DMA_ERQ_ERQ15_MASK                       0x8000u
#define DMA_ERQ_ERQ15_SHIFT                      15
#define DMA_ERQ_ERQ15_WIDTH                      1
#define DMA_ERQ_ERQ15(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERQ_ERQ15_SHIFT))&DMA_ERQ_ERQ15_MASK)
/* EEI Bit Fields */
#define DMA_EEI_EEI0_MASK                        0x1u
#define DMA_EEI_EEI0_SHIFT                       0
#define DMA_EEI_EEI0_WIDTH                       1
#define DMA_EEI_EEI0(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI0_SHIFT))&DMA_EEI_EEI0_MASK)
#define DMA_EEI_EEI1_MASK                        0x2u
#define DMA_EEI_EEI1_SHIFT                       1
#define DMA_EEI_EEI1_WIDTH                       1
#define DMA_EEI_EEI1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI1_SHIFT))&DMA_EEI_EEI1_MASK)
#define DMA_EEI_EEI2_MASK                        0x4u
#define DMA_EEI_EEI2_SHIFT                       2
#define DMA_EEI_EEI2_WIDTH                       1
#define DMA_EEI_EEI2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI2_SHIFT))&DMA_EEI_EEI2_MASK)
#define DMA_EEI_EEI3_MASK                        0x8u
#define DMA_EEI_EEI3_SHIFT                       3
#define DMA_EEI_EEI3_WIDTH                       1
#define DMA_EEI_EEI3(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI3_SHIFT))&DMA_EEI_EEI3_MASK)
#define DMA_EEI_EEI4_MASK                        0x10u
#define DMA_EEI_EEI4_SHIFT                       4
#define DMA_EEI_EEI4_WIDTH                       1
#define DMA_EEI_EEI4(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI4_SHIFT))&DMA_EEI_EEI4_MASK)
#define DMA_EEI_EEI5_MASK                        0x20u
#define DMA_EEI_EEI5_SHIFT                       5
#define DMA_EEI_EEI5_WIDTH                       1
#define DMA_EEI_EEI5(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI5_SHIFT))&DMA_EEI_EEI5_MASK)
#define DMA_EEI_EEI6_MASK                        0x40u
#define DMA_EEI_EEI6_SHIFT                       6
#define DMA_EEI_EEI6_WIDTH                       1
#define DMA_EEI_EEI6(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI6_SHIFT))&DMA_EEI_EEI6_MASK)
#define DMA_EEI_EEI7_MASK                        0x80u
#define DMA_EEI_EEI7_SHIFT                       7
#define DMA_EEI_EEI7_WIDTH                       1
#define DMA_EEI_EEI7(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI7_SHIFT))&DMA_EEI_EEI7_MASK)
#define DMA_EEI_EEI8_MASK                        0x100u
#define DMA_EEI_EEI8_SHIFT                       8
#define DMA_EEI_EEI8_WIDTH                       1
#define DMA_EEI_EEI8(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI8_SHIFT))&DMA_EEI_EEI8_MASK)
#define DMA_EEI_EEI9_MASK                        0x200u
#define DMA_EEI_EEI9_SHIFT                       9
#define DMA_EEI_EEI9_WIDTH                       1
#define DMA_EEI_EEI9(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI9_SHIFT))&DMA_EEI_EEI9_MASK)
#define DMA_EEI_EEI10_MASK                       0x400u
#define DMA_EEI_EEI10_SHIFT                      10
#define DMA_EEI_EEI10_WIDTH                      1
#define DMA_EEI_EEI10(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI10_SHIFT))&DMA_EEI_EEI10_MASK)
#define DMA_EEI_EEI11_MASK                       0x800u
#define DMA_EEI_EEI11_SHIFT                      11
#define DMA_EEI_EEI11_WIDTH                      1
#define DMA_EEI_EEI11(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI11_SHIFT))&DMA_EEI_EEI11_MASK)
#define DMA_EEI_EEI12_MASK                       0x1000u
#define DMA_EEI_EEI12_SHIFT                      12
#define DMA_EEI_EEI12_WIDTH                      1
#define DMA_EEI_EEI12(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI12_SHIFT))&DMA_EEI_EEI12_MASK)
#define DMA_EEI_EEI13_MASK                       0x2000u
#define DMA_EEI_EEI13_SHIFT                      13
#define DMA_EEI_EEI13_WIDTH                      1
#define DMA_EEI_EEI13(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI13_SHIFT))&DMA_EEI_EEI13_MASK)
#define DMA_EEI_EEI14_MASK                       0x4000u
#define DMA_EEI_EEI14_SHIFT                      14
#define DMA_EEI_EEI14_WIDTH                      1
#define DMA_EEI_EEI14(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI14_SHIFT))&DMA_EEI_EEI14_MASK)
#define DMA_EEI_EEI15_MASK                       0x8000u
#define DMA_EEI_EEI15_SHIFT                      15
#define DMA_EEI_EEI15_WIDTH                      1
#define DMA_EEI_EEI15(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_EEI_EEI15_SHIFT))&DMA_EEI_EEI15_MASK)
/* CEEI Bit Fields */
#define DMA_CEEI_CEEI_MASK                       0xFu
#define DMA_CEEI_CEEI_SHIFT                      0
#define DMA_CEEI_CEEI_WIDTH                      4
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_CEEI_SHIFT))&DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       0x40u
#define DMA_CEEI_CAEE_SHIFT                      6
#define DMA_CEEI_CAEE_WIDTH                      1
#define DMA_CEEI_CAEE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_CAEE_SHIFT))&DMA_CEEI_CAEE_MASK)
#define DMA_CEEI_NOP_MASK                        0x80u
#define DMA_CEEI_NOP_SHIFT                       7
#define DMA_CEEI_NOP_WIDTH                       1
#define DMA_CEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_NOP_SHIFT))&DMA_CEEI_NOP_MASK)
/* SEEI Bit Fields */
#define DMA_SEEI_SEEI_MASK                       0xFu
#define DMA_SEEI_SEEI_SHIFT                      0
#define DMA_SEEI_SEEI_WIDTH                      4
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_SEEI_SHIFT))&DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       0x40u
#define DMA_SEEI_SAEE_SHIFT                      6
#define DMA_SEEI_SAEE_WIDTH                      1
#define DMA_SEEI_SAEE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_SAEE_SHIFT))&DMA_SEEI_SAEE_MASK)
#define DMA_SEEI_NOP_MASK                        0x80u
#define DMA_SEEI_NOP_SHIFT                       7
#define DMA_SEEI_NOP_WIDTH                       1
#define DMA_SEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_NOP_SHIFT))&DMA_SEEI_NOP_MASK)
/* CERQ Bit Fields */
#define DMA_CERQ_CERQ_MASK                       0xFu
#define DMA_CERQ_CERQ_SHIFT                      0
#define DMA_CERQ_CERQ_WIDTH                      4
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_CERQ_SHIFT))&DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       0x40u
#define DMA_CERQ_CAER_SHIFT                      6
#define DMA_CERQ_CAER_WIDTH                      1
#define DMA_CERQ_CAER(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_CAER_SHIFT))&DMA_CERQ_CAER_MASK)
#define DMA_CERQ_NOP_MASK                        0x80u
#define DMA_CERQ_NOP_SHIFT                       7
#define DMA_CERQ_NOP_WIDTH                       1
#define DMA_CERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_NOP_SHIFT))&DMA_CERQ_NOP_MASK)
/* SERQ Bit Fields */
#define DMA_SERQ_SERQ_MASK                       0xFu
#define DMA_SERQ_SERQ_SHIFT                      0
#define DMA_SERQ_SERQ_WIDTH                      4
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_SERQ_SHIFT))&DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       0x40u
#define DMA_SERQ_SAER_SHIFT                      6
#define DMA_SERQ_SAER_WIDTH                      1
#define DMA_SERQ_SAER(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_SAER_SHIFT))&DMA_SERQ_SAER_MASK)
#define DMA_SERQ_NOP_MASK                        0x80u
#define DMA_SERQ_NOP_SHIFT                       7
#define DMA_SERQ_NOP_WIDTH                       1
#define DMA_SERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_NOP_SHIFT))&DMA_SERQ_NOP_MASK)
/* CDNE Bit Fields */
#define DMA_CDNE_CDNE_MASK                       0xFu
#define DMA_CDNE_CDNE_SHIFT                      0
#define DMA_CDNE_CDNE_WIDTH                      4
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_CDNE_SHIFT))&DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       0x40u
#define DMA_CDNE_CADN_SHIFT                      6
#define DMA_CDNE_CADN_WIDTH                      1
#define DMA_CDNE_CADN(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_CADN_SHIFT))&DMA_CDNE_CADN_MASK)
#define DMA_CDNE_NOP_MASK                        0x80u
#define DMA_CDNE_NOP_SHIFT                       7
#define DMA_CDNE_NOP_WIDTH                       1
#define DMA_CDNE_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_NOP_SHIFT))&DMA_CDNE_NOP_MASK)
/* SSRT Bit Fields */
#define DMA_SSRT_SSRT_MASK                       0xFu
#define DMA_SSRT_SSRT_SHIFT                      0
#define DMA_SSRT_SSRT_WIDTH                      4
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_SSRT_SHIFT))&DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       0x40u
#define DMA_SSRT_SAST_SHIFT                      6
#define DMA_SSRT_SAST_WIDTH                      1
#define DMA_SSRT_SAST(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_SAST_SHIFT))&DMA_SSRT_SAST_MASK)
#define DMA_SSRT_NOP_MASK                        0x80u
#define DMA_SSRT_NOP_SHIFT                       7
#define DMA_SSRT_NOP_WIDTH                       1
#define DMA_SSRT_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_NOP_SHIFT))&DMA_SSRT_NOP_MASK)
/* CERR Bit Fields */
#define DMA_CERR_CERR_MASK                       0xFu
#define DMA_CERR_CERR_SHIFT                      0
#define DMA_CERR_CERR_WIDTH                      4
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERR_CERR_SHIFT))&DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       0x40u
#define DMA_CERR_CAEI_SHIFT                      6
#define DMA_CERR_CAEI_WIDTH                      1
#define DMA_CERR_CAEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERR_CAEI_SHIFT))&DMA_CERR_CAEI_MASK)
#define DMA_CERR_NOP_MASK                        0x80u
#define DMA_CERR_NOP_SHIFT                       7
#define DMA_CERR_NOP_WIDTH                       1
#define DMA_CERR_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_CERR_NOP_SHIFT))&DMA_CERR_NOP_MASK)
/* CINT Bit Fields */
#define DMA_CINT_CINT_MASK                       0xFu
#define DMA_CINT_CINT_SHIFT                      0
#define DMA_CINT_CINT_WIDTH                      4
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CINT_CINT_SHIFT))&DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR_MASK                       0x40u
#define DMA_CINT_CAIR_SHIFT                      6
#define DMA_CINT_CAIR_WIDTH                      1
#define DMA_CINT_CAIR(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CINT_CAIR_SHIFT))&DMA_CINT_CAIR_MASK)
#define DMA_CINT_NOP_MASK                        0x80u
#define DMA_CINT_NOP_SHIFT                       7
#define DMA_CINT_NOP_WIDTH                       1
#define DMA_CINT_NOP(x)                          (((uint8_t)(((uint8_t)(x))<<DMA_CINT_NOP_SHIFT))&DMA_CINT_NOP_MASK)
/* INT Bit Fields */
#define DMA_INT_INT0_MASK                        0x1u
#define DMA_INT_INT0_SHIFT                       0
#define DMA_INT_INT0_WIDTH                       1
#define DMA_INT_INT0(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT0_SHIFT))&DMA_INT_INT0_MASK)
#define DMA_INT_INT1_MASK                        0x2u
#define DMA_INT_INT1_SHIFT                       1
#define DMA_INT_INT1_WIDTH                       1
#define DMA_INT_INT1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT1_SHIFT))&DMA_INT_INT1_MASK)
#define DMA_INT_INT2_MASK                        0x4u
#define DMA_INT_INT2_SHIFT                       2
#define DMA_INT_INT2_WIDTH                       1
#define DMA_INT_INT2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT2_SHIFT))&DMA_INT_INT2_MASK)
#define DMA_INT_INT3_MASK                        0x8u
#define DMA_INT_INT3_SHIFT                       3
#define DMA_INT_INT3_WIDTH                       1
#define DMA_INT_INT3(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT3_SHIFT))&DMA_INT_INT3_MASK)
#define DMA_INT_INT4_MASK                        0x10u
#define DMA_INT_INT4_SHIFT                       4
#define DMA_INT_INT4_WIDTH                       1
#define DMA_INT_INT4(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT4_SHIFT))&DMA_INT_INT4_MASK)
#define DMA_INT_INT5_MASK                        0x20u
#define DMA_INT_INT5_SHIFT                       5
#define DMA_INT_INT5_WIDTH                       1
#define DMA_INT_INT5(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT5_SHIFT))&DMA_INT_INT5_MASK)
#define DMA_INT_INT6_MASK                        0x40u
#define DMA_INT_INT6_SHIFT                       6
#define DMA_INT_INT6_WIDTH                       1
#define DMA_INT_INT6(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT6_SHIFT))&DMA_INT_INT6_MASK)
#define DMA_INT_INT7_MASK                        0x80u
#define DMA_INT_INT7_SHIFT                       7
#define DMA_INT_INT7_WIDTH                       1
#define DMA_INT_INT7(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT7_SHIFT))&DMA_INT_INT7_MASK)
#define DMA_INT_INT8_MASK                        0x100u
#define DMA_INT_INT8_SHIFT                       8
#define DMA_INT_INT8_WIDTH                       1
#define DMA_INT_INT8(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT8_SHIFT))&DMA_INT_INT8_MASK)
#define DMA_INT_INT9_MASK                        0x200u
#define DMA_INT_INT9_SHIFT                       9
#define DMA_INT_INT9_WIDTH                       1
#define DMA_INT_INT9(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT9_SHIFT))&DMA_INT_INT9_MASK)
#define DMA_INT_INT10_MASK                       0x400u
#define DMA_INT_INT10_SHIFT                      10
#define DMA_INT_INT10_WIDTH                      1
#define DMA_INT_INT10(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT10_SHIFT))&DMA_INT_INT10_MASK)
#define DMA_INT_INT11_MASK                       0x800u
#define DMA_INT_INT11_SHIFT                      11
#define DMA_INT_INT11_WIDTH                      1
#define DMA_INT_INT11(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT11_SHIFT))&DMA_INT_INT11_MASK)
#define DMA_INT_INT12_MASK                       0x1000u
#define DMA_INT_INT12_SHIFT                      12
#define DMA_INT_INT12_WIDTH                      1
#define DMA_INT_INT12(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT12_SHIFT))&DMA_INT_INT12_MASK)
#define DMA_INT_INT13_MASK                       0x2000u
#define DMA_INT_INT13_SHIFT                      13
#define DMA_INT_INT13_WIDTH                      1
#define DMA_INT_INT13(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT13_SHIFT))&DMA_INT_INT13_MASK)
#define DMA_INT_INT14_MASK                       0x4000u
#define DMA_INT_INT14_SHIFT                      14
#define DMA_INT_INT14_WIDTH                      1
#define DMA_INT_INT14(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT14_SHIFT))&DMA_INT_INT14_MASK)
#define DMA_INT_INT15_MASK                       0x8000u
#define DMA_INT_INT15_SHIFT                      15
#define DMA_INT_INT15_WIDTH                      1
#define DMA_INT_INT15(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_INT_INT15_SHIFT))&DMA_INT_INT15_MASK)
/* ERR Bit Fields */
#define DMA_ERR_ERR0_MASK                        0x1u
#define DMA_ERR_ERR0_SHIFT                       0
#define DMA_ERR_ERR0_WIDTH                       1
#define DMA_ERR_ERR0(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR0_SHIFT))&DMA_ERR_ERR0_MASK)
#define DMA_ERR_ERR1_MASK                        0x2u
#define DMA_ERR_ERR1_SHIFT                       1
#define DMA_ERR_ERR1_WIDTH                       1
#define DMA_ERR_ERR1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR1_SHIFT))&DMA_ERR_ERR1_MASK)
#define DMA_ERR_ERR2_MASK                        0x4u
#define DMA_ERR_ERR2_SHIFT                       2
#define DMA_ERR_ERR2_WIDTH                       1
#define DMA_ERR_ERR2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR2_SHIFT))&DMA_ERR_ERR2_MASK)
#define DMA_ERR_ERR3_MASK                        0x8u
#define DMA_ERR_ERR3_SHIFT                       3
#define DMA_ERR_ERR3_WIDTH                       1
#define DMA_ERR_ERR3(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR3_SHIFT))&DMA_ERR_ERR3_MASK)
#define DMA_ERR_ERR4_MASK                        0x10u
#define DMA_ERR_ERR4_SHIFT                       4
#define DMA_ERR_ERR4_WIDTH                       1
#define DMA_ERR_ERR4(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR4_SHIFT))&DMA_ERR_ERR4_MASK)
#define DMA_ERR_ERR5_MASK                        0x20u
#define DMA_ERR_ERR5_SHIFT                       5
#define DMA_ERR_ERR5_WIDTH                       1
#define DMA_ERR_ERR5(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR5_SHIFT))&DMA_ERR_ERR5_MASK)
#define DMA_ERR_ERR6_MASK                        0x40u
#define DMA_ERR_ERR6_SHIFT                       6
#define DMA_ERR_ERR6_WIDTH                       1
#define DMA_ERR_ERR6(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR6_SHIFT))&DMA_ERR_ERR6_MASK)
#define DMA_ERR_ERR7_MASK                        0x80u
#define DMA_ERR_ERR7_SHIFT                       7
#define DMA_ERR_ERR7_WIDTH                       1
#define DMA_ERR_ERR7(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR7_SHIFT))&DMA_ERR_ERR7_MASK)
#define DMA_ERR_ERR8_MASK                        0x100u
#define DMA_ERR_ERR8_SHIFT                       8
#define DMA_ERR_ERR8_WIDTH                       1
#define DMA_ERR_ERR8(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR8_SHIFT))&DMA_ERR_ERR8_MASK)
#define DMA_ERR_ERR9_MASK                        0x200u
#define DMA_ERR_ERR9_SHIFT                       9
#define DMA_ERR_ERR9_WIDTH                       1
#define DMA_ERR_ERR9(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR9_SHIFT))&DMA_ERR_ERR9_MASK)
#define DMA_ERR_ERR10_MASK                       0x400u
#define DMA_ERR_ERR10_SHIFT                      10
#define DMA_ERR_ERR10_WIDTH                      1
#define DMA_ERR_ERR10(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR10_SHIFT))&DMA_ERR_ERR10_MASK)
#define DMA_ERR_ERR11_MASK                       0x800u
#define DMA_ERR_ERR11_SHIFT                      11
#define DMA_ERR_ERR11_WIDTH                      1
#define DMA_ERR_ERR11(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR11_SHIFT))&DMA_ERR_ERR11_MASK)
#define DMA_ERR_ERR12_MASK                       0x1000u
#define DMA_ERR_ERR12_SHIFT                      12
#define DMA_ERR_ERR12_WIDTH                      1
#define DMA_ERR_ERR12(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR12_SHIFT))&DMA_ERR_ERR12_MASK)
#define DMA_ERR_ERR13_MASK                       0x2000u
#define DMA_ERR_ERR13_SHIFT                      13
#define DMA_ERR_ERR13_WIDTH                      1
#define DMA_ERR_ERR13(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR13_SHIFT))&DMA_ERR_ERR13_MASK)
#define DMA_ERR_ERR14_MASK                       0x4000u
#define DMA_ERR_ERR14_SHIFT                      14
#define DMA_ERR_ERR14_WIDTH                      1
#define DMA_ERR_ERR14(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR14_SHIFT))&DMA_ERR_ERR14_MASK)
#define DMA_ERR_ERR15_MASK                       0x8000u
#define DMA_ERR_ERR15_SHIFT                      15
#define DMA_ERR_ERR15_WIDTH                      1
#define DMA_ERR_ERR15(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ERR_ERR15_SHIFT))&DMA_ERR_ERR15_MASK)
/* HRS Bit Fields */
#define DMA_HRS_HRS0_MASK                        0x1u
#define DMA_HRS_HRS0_SHIFT                       0
#define DMA_HRS_HRS0_WIDTH                       1
#define DMA_HRS_HRS0(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS0_SHIFT))&DMA_HRS_HRS0_MASK)
#define DMA_HRS_HRS1_MASK                        0x2u
#define DMA_HRS_HRS1_SHIFT                       1
#define DMA_HRS_HRS1_WIDTH                       1
#define DMA_HRS_HRS1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS1_SHIFT))&DMA_HRS_HRS1_MASK)
#define DMA_HRS_HRS2_MASK                        0x4u
#define DMA_HRS_HRS2_SHIFT                       2
#define DMA_HRS_HRS2_WIDTH                       1
#define DMA_HRS_HRS2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS2_SHIFT))&DMA_HRS_HRS2_MASK)
#define DMA_HRS_HRS3_MASK                        0x8u
#define DMA_HRS_HRS3_SHIFT                       3
#define DMA_HRS_HRS3_WIDTH                       1
#define DMA_HRS_HRS3(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS3_SHIFT))&DMA_HRS_HRS3_MASK)
#define DMA_HRS_HRS4_MASK                        0x10u
#define DMA_HRS_HRS4_SHIFT                       4
#define DMA_HRS_HRS4_WIDTH                       1
#define DMA_HRS_HRS4(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS4_SHIFT))&DMA_HRS_HRS4_MASK)
#define DMA_HRS_HRS5_MASK                        0x20u
#define DMA_HRS_HRS5_SHIFT                       5
#define DMA_HRS_HRS5_WIDTH                       1
#define DMA_HRS_HRS5(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS5_SHIFT))&DMA_HRS_HRS5_MASK)
#define DMA_HRS_HRS6_MASK                        0x40u
#define DMA_HRS_HRS6_SHIFT                       6
#define DMA_HRS_HRS6_WIDTH                       1
#define DMA_HRS_HRS6(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS6_SHIFT))&DMA_HRS_HRS6_MASK)
#define DMA_HRS_HRS7_MASK                        0x80u
#define DMA_HRS_HRS7_SHIFT                       7
#define DMA_HRS_HRS7_WIDTH                       1
#define DMA_HRS_HRS7(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS7_SHIFT))&DMA_HRS_HRS7_MASK)
#define DMA_HRS_HRS8_MASK                        0x100u
#define DMA_HRS_HRS8_SHIFT                       8
#define DMA_HRS_HRS8_WIDTH                       1
#define DMA_HRS_HRS8(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS8_SHIFT))&DMA_HRS_HRS8_MASK)
#define DMA_HRS_HRS9_MASK                        0x200u
#define DMA_HRS_HRS9_SHIFT                       9
#define DMA_HRS_HRS9_WIDTH                       1
#define DMA_HRS_HRS9(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS9_SHIFT))&DMA_HRS_HRS9_MASK)
#define DMA_HRS_HRS10_MASK                       0x400u
#define DMA_HRS_HRS10_SHIFT                      10
#define DMA_HRS_HRS10_WIDTH                      1
#define DMA_HRS_HRS10(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS10_SHIFT))&DMA_HRS_HRS10_MASK)
#define DMA_HRS_HRS11_MASK                       0x800u
#define DMA_HRS_HRS11_SHIFT                      11
#define DMA_HRS_HRS11_WIDTH                      1
#define DMA_HRS_HRS11(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS11_SHIFT))&DMA_HRS_HRS11_MASK)
#define DMA_HRS_HRS12_MASK                       0x1000u
#define DMA_HRS_HRS12_SHIFT                      12
#define DMA_HRS_HRS12_WIDTH                      1
#define DMA_HRS_HRS12(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS12_SHIFT))&DMA_HRS_HRS12_MASK)
#define DMA_HRS_HRS13_MASK                       0x2000u
#define DMA_HRS_HRS13_SHIFT                      13
#define DMA_HRS_HRS13_WIDTH                      1
#define DMA_HRS_HRS13(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS13_SHIFT))&DMA_HRS_HRS13_MASK)
#define DMA_HRS_HRS14_MASK                       0x4000u
#define DMA_HRS_HRS14_SHIFT                      14
#define DMA_HRS_HRS14_WIDTH                      1
#define DMA_HRS_HRS14(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS14_SHIFT))&DMA_HRS_HRS14_MASK)
#define DMA_HRS_HRS15_MASK                       0x8000u
#define DMA_HRS_HRS15_SHIFT                      15
#define DMA_HRS_HRS15_WIDTH                      1
#define DMA_HRS_HRS15(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_HRS_HRS15_SHIFT))&DMA_HRS_HRS15_MASK)
/* EARS Bit Fields */
#define DMA_EARS_EDREQ_0_MASK                    0x1u
#define DMA_EARS_EDREQ_0_SHIFT                   0
#define DMA_EARS_EDREQ_0_WIDTH                   1
#define DMA_EARS_EDREQ_0(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_0_SHIFT))&DMA_EARS_EDREQ_0_MASK)
#define DMA_EARS_EDREQ_1_MASK                    0x2u
#define DMA_EARS_EDREQ_1_SHIFT                   1
#define DMA_EARS_EDREQ_1_WIDTH                   1
#define DMA_EARS_EDREQ_1(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_1_SHIFT))&DMA_EARS_EDREQ_1_MASK)
#define DMA_EARS_EDREQ_2_MASK                    0x4u
#define DMA_EARS_EDREQ_2_SHIFT                   2
#define DMA_EARS_EDREQ_2_WIDTH                   1
#define DMA_EARS_EDREQ_2(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_2_SHIFT))&DMA_EARS_EDREQ_2_MASK)
#define DMA_EARS_EDREQ_3_MASK                    0x8u
#define DMA_EARS_EDREQ_3_SHIFT                   3
#define DMA_EARS_EDREQ_3_WIDTH                   1
#define DMA_EARS_EDREQ_3(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_3_SHIFT))&DMA_EARS_EDREQ_3_MASK)
#define DMA_EARS_EDREQ_4_MASK                    0x10u
#define DMA_EARS_EDREQ_4_SHIFT                   4
#define DMA_EARS_EDREQ_4_WIDTH                   1
#define DMA_EARS_EDREQ_4(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_4_SHIFT))&DMA_EARS_EDREQ_4_MASK)
#define DMA_EARS_EDREQ_5_MASK                    0x20u
#define DMA_EARS_EDREQ_5_SHIFT                   5
#define DMA_EARS_EDREQ_5_WIDTH                   1
#define DMA_EARS_EDREQ_5(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_5_SHIFT))&DMA_EARS_EDREQ_5_MASK)
#define DMA_EARS_EDREQ_6_MASK                    0x40u
#define DMA_EARS_EDREQ_6_SHIFT                   6
#define DMA_EARS_EDREQ_6_WIDTH                   1
#define DMA_EARS_EDREQ_6(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_6_SHIFT))&DMA_EARS_EDREQ_6_MASK)
#define DMA_EARS_EDREQ_7_MASK                    0x80u
#define DMA_EARS_EDREQ_7_SHIFT                   7
#define DMA_EARS_EDREQ_7_WIDTH                   1
#define DMA_EARS_EDREQ_7(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_7_SHIFT))&DMA_EARS_EDREQ_7_MASK)
#define DMA_EARS_EDREQ_8_MASK                    0x100u
#define DMA_EARS_EDREQ_8_SHIFT                   8
#define DMA_EARS_EDREQ_8_WIDTH                   1
#define DMA_EARS_EDREQ_8(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_8_SHIFT))&DMA_EARS_EDREQ_8_MASK)
#define DMA_EARS_EDREQ_9_MASK                    0x200u
#define DMA_EARS_EDREQ_9_SHIFT                   9
#define DMA_EARS_EDREQ_9_WIDTH                   1
#define DMA_EARS_EDREQ_9(x)                      (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_9_SHIFT))&DMA_EARS_EDREQ_9_MASK)
#define DMA_EARS_EDREQ_10_MASK                   0x400u
#define DMA_EARS_EDREQ_10_SHIFT                  10
#define DMA_EARS_EDREQ_10_WIDTH                  1
#define DMA_EARS_EDREQ_10(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_10_SHIFT))&DMA_EARS_EDREQ_10_MASK)
#define DMA_EARS_EDREQ_11_MASK                   0x800u
#define DMA_EARS_EDREQ_11_SHIFT                  11
#define DMA_EARS_EDREQ_11_WIDTH                  1
#define DMA_EARS_EDREQ_11(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_11_SHIFT))&DMA_EARS_EDREQ_11_MASK)
#define DMA_EARS_EDREQ_12_MASK                   0x1000u
#define DMA_EARS_EDREQ_12_SHIFT                  12
#define DMA_EARS_EDREQ_12_WIDTH                  1
#define DMA_EARS_EDREQ_12(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_12_SHIFT))&DMA_EARS_EDREQ_12_MASK)
#define DMA_EARS_EDREQ_13_MASK                   0x2000u
#define DMA_EARS_EDREQ_13_SHIFT                  13
#define DMA_EARS_EDREQ_13_WIDTH                  1
#define DMA_EARS_EDREQ_13(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_13_SHIFT))&DMA_EARS_EDREQ_13_MASK)
#define DMA_EARS_EDREQ_14_MASK                   0x4000u
#define DMA_EARS_EDREQ_14_SHIFT                  14
#define DMA_EARS_EDREQ_14_WIDTH                  1
#define DMA_EARS_EDREQ_14(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_14_SHIFT))&DMA_EARS_EDREQ_14_MASK)
#define DMA_EARS_EDREQ_15_MASK                   0x8000u
#define DMA_EARS_EDREQ_15_SHIFT                  15
#define DMA_EARS_EDREQ_15_WIDTH                  1
#define DMA_EARS_EDREQ_15(x)                     (((uint32_t)(((uint32_t)(x))<<DMA_EARS_EDREQ_15_SHIFT))&DMA_EARS_EDREQ_15_MASK)
/* DCHPRI3 Bit Fields */
#define DMA_DCHPRI3_CHPRI_MASK                   0xFu
#define DMA_DCHPRI3_CHPRI_SHIFT                  0
#define DMA_DCHPRI3_CHPRI_WIDTH                  4
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_CHPRI_SHIFT))&DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_DPA_MASK                     0x40u
#define DMA_DCHPRI3_DPA_SHIFT                    6
#define DMA_DCHPRI3_DPA_WIDTH                    1
#define DMA_DCHPRI3_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_DPA_SHIFT))&DMA_DCHPRI3_DPA_MASK)
#define DMA_DCHPRI3_ECP_MASK                     0x80u
#define DMA_DCHPRI3_ECP_SHIFT                    7
#define DMA_DCHPRI3_ECP_WIDTH                    1
#define DMA_DCHPRI3_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_ECP_SHIFT))&DMA_DCHPRI3_ECP_MASK)
/* DCHPRI2 Bit Fields */
#define DMA_DCHPRI2_CHPRI_MASK                   0xFu
#define DMA_DCHPRI2_CHPRI_SHIFT                  0
#define DMA_DCHPRI2_CHPRI_WIDTH                  4
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_CHPRI_SHIFT))&DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_DPA_MASK                     0x40u
#define DMA_DCHPRI2_DPA_SHIFT                    6
#define DMA_DCHPRI2_DPA_WIDTH                    1
#define DMA_DCHPRI2_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_DPA_SHIFT))&DMA_DCHPRI2_DPA_MASK)
#define DMA_DCHPRI2_ECP_MASK                     0x80u
#define DMA_DCHPRI2_ECP_SHIFT                    7
#define DMA_DCHPRI2_ECP_WIDTH                    1
#define DMA_DCHPRI2_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_ECP_SHIFT))&DMA_DCHPRI2_ECP_MASK)
/* DCHPRI1 Bit Fields */
#define DMA_DCHPRI1_CHPRI_MASK                   0xFu
#define DMA_DCHPRI1_CHPRI_SHIFT                  0
#define DMA_DCHPRI1_CHPRI_WIDTH                  4
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_CHPRI_SHIFT))&DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_DPA_MASK                     0x40u
#define DMA_DCHPRI1_DPA_SHIFT                    6
#define DMA_DCHPRI1_DPA_WIDTH                    1
#define DMA_DCHPRI1_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_DPA_SHIFT))&DMA_DCHPRI1_DPA_MASK)
#define DMA_DCHPRI1_ECP_MASK                     0x80u
#define DMA_DCHPRI1_ECP_SHIFT                    7
#define DMA_DCHPRI1_ECP_WIDTH                    1
#define DMA_DCHPRI1_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_ECP_SHIFT))&DMA_DCHPRI1_ECP_MASK)
/* DCHPRI0 Bit Fields */
#define DMA_DCHPRI0_CHPRI_MASK                   0xFu
#define DMA_DCHPRI0_CHPRI_SHIFT                  0
#define DMA_DCHPRI0_CHPRI_WIDTH                  4
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_CHPRI_SHIFT))&DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_DPA_MASK                     0x40u
#define DMA_DCHPRI0_DPA_SHIFT                    6
#define DMA_DCHPRI0_DPA_WIDTH                    1
#define DMA_DCHPRI0_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_DPA_SHIFT))&DMA_DCHPRI0_DPA_MASK)
#define DMA_DCHPRI0_ECP_MASK                     0x80u
#define DMA_DCHPRI0_ECP_SHIFT                    7
#define DMA_DCHPRI0_ECP_WIDTH                    1
#define DMA_DCHPRI0_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_ECP_SHIFT))&DMA_DCHPRI0_ECP_MASK)
/* DCHPRI7 Bit Fields */
#define DMA_DCHPRI7_CHPRI_MASK                   0xFu
#define DMA_DCHPRI7_CHPRI_SHIFT                  0
#define DMA_DCHPRI7_CHPRI_WIDTH                  4
#define DMA_DCHPRI7_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_CHPRI_SHIFT))&DMA_DCHPRI7_CHPRI_MASK)
#define DMA_DCHPRI7_DPA_MASK                     0x40u
#define DMA_DCHPRI7_DPA_SHIFT                    6
#define DMA_DCHPRI7_DPA_WIDTH                    1
#define DMA_DCHPRI7_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_DPA_SHIFT))&DMA_DCHPRI7_DPA_MASK)
#define DMA_DCHPRI7_ECP_MASK                     0x80u
#define DMA_DCHPRI7_ECP_SHIFT                    7
#define DMA_DCHPRI7_ECP_WIDTH                    1
#define DMA_DCHPRI7_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_ECP_SHIFT))&DMA_DCHPRI7_ECP_MASK)
/* DCHPRI6 Bit Fields */
#define DMA_DCHPRI6_CHPRI_MASK                   0xFu
#define DMA_DCHPRI6_CHPRI_SHIFT                  0
#define DMA_DCHPRI6_CHPRI_WIDTH                  4
#define DMA_DCHPRI6_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_CHPRI_SHIFT))&DMA_DCHPRI6_CHPRI_MASK)
#define DMA_DCHPRI6_DPA_MASK                     0x40u
#define DMA_DCHPRI6_DPA_SHIFT                    6
#define DMA_DCHPRI6_DPA_WIDTH                    1
#define DMA_DCHPRI6_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_DPA_SHIFT))&DMA_DCHPRI6_DPA_MASK)
#define DMA_DCHPRI6_ECP_MASK                     0x80u
#define DMA_DCHPRI6_ECP_SHIFT                    7
#define DMA_DCHPRI6_ECP_WIDTH                    1
#define DMA_DCHPRI6_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_ECP_SHIFT))&DMA_DCHPRI6_ECP_MASK)
/* DCHPRI5 Bit Fields */
#define DMA_DCHPRI5_CHPRI_MASK                   0xFu
#define DMA_DCHPRI5_CHPRI_SHIFT                  0
#define DMA_DCHPRI5_CHPRI_WIDTH                  4
#define DMA_DCHPRI5_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_CHPRI_SHIFT))&DMA_DCHPRI5_CHPRI_MASK)
#define DMA_DCHPRI5_DPA_MASK                     0x40u
#define DMA_DCHPRI5_DPA_SHIFT                    6
#define DMA_DCHPRI5_DPA_WIDTH                    1
#define DMA_DCHPRI5_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_DPA_SHIFT))&DMA_DCHPRI5_DPA_MASK)
#define DMA_DCHPRI5_ECP_MASK                     0x80u
#define DMA_DCHPRI5_ECP_SHIFT                    7
#define DMA_DCHPRI5_ECP_WIDTH                    1
#define DMA_DCHPRI5_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_ECP_SHIFT))&DMA_DCHPRI5_ECP_MASK)
/* DCHPRI4 Bit Fields */
#define DMA_DCHPRI4_CHPRI_MASK                   0xFu
#define DMA_DCHPRI4_CHPRI_SHIFT                  0
#define DMA_DCHPRI4_CHPRI_WIDTH                  4
#define DMA_DCHPRI4_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_CHPRI_SHIFT))&DMA_DCHPRI4_CHPRI_MASK)
#define DMA_DCHPRI4_DPA_MASK                     0x40u
#define DMA_DCHPRI4_DPA_SHIFT                    6
#define DMA_DCHPRI4_DPA_WIDTH                    1
#define DMA_DCHPRI4_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_DPA_SHIFT))&DMA_DCHPRI4_DPA_MASK)
#define DMA_DCHPRI4_ECP_MASK                     0x80u
#define DMA_DCHPRI4_ECP_SHIFT                    7
#define DMA_DCHPRI4_ECP_WIDTH                    1
#define DMA_DCHPRI4_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_ECP_SHIFT))&DMA_DCHPRI4_ECP_MASK)
/* DCHPRI11 Bit Fields */
#define DMA_DCHPRI11_CHPRI_MASK                  0xFu
#define DMA_DCHPRI11_CHPRI_SHIFT                 0
#define DMA_DCHPRI11_CHPRI_WIDTH                 4
#define DMA_DCHPRI11_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_CHPRI_SHIFT))&DMA_DCHPRI11_CHPRI_MASK)
#define DMA_DCHPRI11_DPA_MASK                    0x40u
#define DMA_DCHPRI11_DPA_SHIFT                   6
#define DMA_DCHPRI11_DPA_WIDTH                   1
#define DMA_DCHPRI11_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_DPA_SHIFT))&DMA_DCHPRI11_DPA_MASK)
#define DMA_DCHPRI11_ECP_MASK                    0x80u
#define DMA_DCHPRI11_ECP_SHIFT                   7
#define DMA_DCHPRI11_ECP_WIDTH                   1
#define DMA_DCHPRI11_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_ECP_SHIFT))&DMA_DCHPRI11_ECP_MASK)
/* DCHPRI10 Bit Fields */
#define DMA_DCHPRI10_CHPRI_MASK                  0xFu
#define DMA_DCHPRI10_CHPRI_SHIFT                 0
#define DMA_DCHPRI10_CHPRI_WIDTH                 4
#define DMA_DCHPRI10_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_CHPRI_SHIFT))&DMA_DCHPRI10_CHPRI_MASK)
#define DMA_DCHPRI10_DPA_MASK                    0x40u
#define DMA_DCHPRI10_DPA_SHIFT                   6
#define DMA_DCHPRI10_DPA_WIDTH                   1
#define DMA_DCHPRI10_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_DPA_SHIFT))&DMA_DCHPRI10_DPA_MASK)
#define DMA_DCHPRI10_ECP_MASK                    0x80u
#define DMA_DCHPRI10_ECP_SHIFT                   7
#define DMA_DCHPRI10_ECP_WIDTH                   1
#define DMA_DCHPRI10_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_ECP_SHIFT))&DMA_DCHPRI10_ECP_MASK)
/* DCHPRI9 Bit Fields */
#define DMA_DCHPRI9_CHPRI_MASK                   0xFu
#define DMA_DCHPRI9_CHPRI_SHIFT                  0
#define DMA_DCHPRI9_CHPRI_WIDTH                  4
#define DMA_DCHPRI9_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_CHPRI_SHIFT))&DMA_DCHPRI9_CHPRI_MASK)
#define DMA_DCHPRI9_DPA_MASK                     0x40u
#define DMA_DCHPRI9_DPA_SHIFT                    6
#define DMA_DCHPRI9_DPA_WIDTH                    1
#define DMA_DCHPRI9_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_DPA_SHIFT))&DMA_DCHPRI9_DPA_MASK)
#define DMA_DCHPRI9_ECP_MASK                     0x80u
#define DMA_DCHPRI9_ECP_SHIFT                    7
#define DMA_DCHPRI9_ECP_WIDTH                    1
#define DMA_DCHPRI9_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_ECP_SHIFT))&DMA_DCHPRI9_ECP_MASK)
/* DCHPRI8 Bit Fields */
#define DMA_DCHPRI8_CHPRI_MASK                   0xFu
#define DMA_DCHPRI8_CHPRI_SHIFT                  0
#define DMA_DCHPRI8_CHPRI_WIDTH                  4
#define DMA_DCHPRI8_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_CHPRI_SHIFT))&DMA_DCHPRI8_CHPRI_MASK)
#define DMA_DCHPRI8_DPA_MASK                     0x40u
#define DMA_DCHPRI8_DPA_SHIFT                    6
#define DMA_DCHPRI8_DPA_WIDTH                    1
#define DMA_DCHPRI8_DPA(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_DPA_SHIFT))&DMA_DCHPRI8_DPA_MASK)
#define DMA_DCHPRI8_ECP_MASK                     0x80u
#define DMA_DCHPRI8_ECP_SHIFT                    7
#define DMA_DCHPRI8_ECP_WIDTH                    1
#define DMA_DCHPRI8_ECP(x)                       (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_ECP_SHIFT))&DMA_DCHPRI8_ECP_MASK)
/* DCHPRI15 Bit Fields */
#define DMA_DCHPRI15_CHPRI_MASK                  0xFu
#define DMA_DCHPRI15_CHPRI_SHIFT                 0
#define DMA_DCHPRI15_CHPRI_WIDTH                 4
#define DMA_DCHPRI15_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_CHPRI_SHIFT))&DMA_DCHPRI15_CHPRI_MASK)
#define DMA_DCHPRI15_DPA_MASK                    0x40u
#define DMA_DCHPRI15_DPA_SHIFT                   6
#define DMA_DCHPRI15_DPA_WIDTH                   1
#define DMA_DCHPRI15_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_DPA_SHIFT))&DMA_DCHPRI15_DPA_MASK)
#define DMA_DCHPRI15_ECP_MASK                    0x80u
#define DMA_DCHPRI15_ECP_SHIFT                   7
#define DMA_DCHPRI15_ECP_WIDTH                   1
#define DMA_DCHPRI15_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_ECP_SHIFT))&DMA_DCHPRI15_ECP_MASK)
/* DCHPRI14 Bit Fields */
#define DMA_DCHPRI14_CHPRI_MASK                  0xFu
#define DMA_DCHPRI14_CHPRI_SHIFT                 0
#define DMA_DCHPRI14_CHPRI_WIDTH                 4
#define DMA_DCHPRI14_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_CHPRI_SHIFT))&DMA_DCHPRI14_CHPRI_MASK)
#define DMA_DCHPRI14_DPA_MASK                    0x40u
#define DMA_DCHPRI14_DPA_SHIFT                   6
#define DMA_DCHPRI14_DPA_WIDTH                   1
#define DMA_DCHPRI14_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_DPA_SHIFT))&DMA_DCHPRI14_DPA_MASK)
#define DMA_DCHPRI14_ECP_MASK                    0x80u
#define DMA_DCHPRI14_ECP_SHIFT                   7
#define DMA_DCHPRI14_ECP_WIDTH                   1
#define DMA_DCHPRI14_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_ECP_SHIFT))&DMA_DCHPRI14_ECP_MASK)
/* DCHPRI13 Bit Fields */
#define DMA_DCHPRI13_CHPRI_MASK                  0xFu
#define DMA_DCHPRI13_CHPRI_SHIFT                 0
#define DMA_DCHPRI13_CHPRI_WIDTH                 4
#define DMA_DCHPRI13_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_CHPRI_SHIFT))&DMA_DCHPRI13_CHPRI_MASK)
#define DMA_DCHPRI13_DPA_MASK                    0x40u
#define DMA_DCHPRI13_DPA_SHIFT                   6
#define DMA_DCHPRI13_DPA_WIDTH                   1
#define DMA_DCHPRI13_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_DPA_SHIFT))&DMA_DCHPRI13_DPA_MASK)
#define DMA_DCHPRI13_ECP_MASK                    0x80u
#define DMA_DCHPRI13_ECP_SHIFT                   7
#define DMA_DCHPRI13_ECP_WIDTH                   1
#define DMA_DCHPRI13_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_ECP_SHIFT))&DMA_DCHPRI13_ECP_MASK)
/* DCHPRI12 Bit Fields */
#define DMA_DCHPRI12_CHPRI_MASK                  0xFu
#define DMA_DCHPRI12_CHPRI_SHIFT                 0
#define DMA_DCHPRI12_CHPRI_WIDTH                 4
#define DMA_DCHPRI12_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_CHPRI_SHIFT))&DMA_DCHPRI12_CHPRI_MASK)
#define DMA_DCHPRI12_DPA_MASK                    0x40u
#define DMA_DCHPRI12_DPA_SHIFT                   6
#define DMA_DCHPRI12_DPA_WIDTH                   1
#define DMA_DCHPRI12_DPA(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_DPA_SHIFT))&DMA_DCHPRI12_DPA_MASK)
#define DMA_DCHPRI12_ECP_MASK                    0x80u
#define DMA_DCHPRI12_ECP_SHIFT                   7
#define DMA_DCHPRI12_ECP_WIDTH                   1
#define DMA_DCHPRI12_ECP(x)                      (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_ECP_SHIFT))&DMA_DCHPRI12_ECP_MASK)
/* SADDR Bit Fields */
#define DMA_SADDR_SADDR_MASK                     0xFFFFFFFFu
#define DMA_SADDR_SADDR_SHIFT                    0
#define DMA_SADDR_SADDR_WIDTH                    32
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SADDR_SADDR_SHIFT))&DMA_SADDR_SADDR_MASK)
/* SOFF Bit Fields */
#define DMA_SOFF_SOFF_MASK                       0xFFFFu
#define DMA_SOFF_SOFF_SHIFT                      0
#define DMA_SOFF_SOFF_WIDTH                      16
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_SOFF_SOFF_SHIFT))&DMA_SOFF_SOFF_MASK)
/* ATTR Bit Fields */
#define DMA_ATTR_DSIZE_MASK                      0x7u
#define DMA_ATTR_DSIZE_SHIFT                     0
#define DMA_ATTR_DSIZE_WIDTH                     3
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DSIZE_SHIFT))&DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       0xF8u
#define DMA_ATTR_DMOD_SHIFT                      3
#define DMA_ATTR_DMOD_WIDTH                      5
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DMOD_SHIFT))&DMA_ATTR_DMOD_MASK)
#define DMA_ATTR_SSIZE_MASK                      0x700u
#define DMA_ATTR_SSIZE_SHIFT                     8
#define DMA_ATTR_SSIZE_WIDTH                     3
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SSIZE_SHIFT))&DMA_ATTR_SSIZE_MASK)
#define DMA_ATTR_SMOD_MASK                       0xF800u
#define DMA_ATTR_SMOD_SHIFT                      11
#define DMA_ATTR_SMOD_WIDTH                      5
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SMOD_SHIFT))&DMA_ATTR_SMOD_MASK)
/* NBYTES_MLNO Bit Fields */
#define DMA_NBYTES_MLNO_NBYTES_MASK              0xFFFFFFFFu
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             0
#define DMA_NBYTES_MLNO_NBYTES_WIDTH             32
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLNO_NBYTES_SHIFT))&DMA_NBYTES_MLNO_NBYTES_MASK)
/* NBYTES_MLOFFNO Bit Fields */
#define DMA_NBYTES_MLOFFNO_NBYTES_MASK           0x3FFFFFFFu
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT          0
#define DMA_NBYTES_MLOFFNO_NBYTES_WIDTH          30
#define DMA_NBYTES_MLOFFNO_NBYTES(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_NBYTES_SHIFT))&DMA_NBYTES_MLOFFNO_NBYTES_MASK)
#define DMA_NBYTES_MLOFFNO_DMLOE_MASK            0x40000000u
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT           30
#define DMA_NBYTES_MLOFFNO_DMLOE_WIDTH           1
#define DMA_NBYTES_MLOFFNO_DMLOE(x)              (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_DMLOE_SHIFT))&DMA_NBYTES_MLOFFNO_DMLOE_MASK)
#define DMA_NBYTES_MLOFFNO_SMLOE_MASK            0x80000000u
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT           31
#define DMA_NBYTES_MLOFFNO_SMLOE_WIDTH           1
#define DMA_NBYTES_MLOFFNO_SMLOE(x)              (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_SMLOE_SHIFT))&DMA_NBYTES_MLOFFNO_SMLOE_MASK)
/* NBYTES_MLOFFYES Bit Fields */
#define DMA_NBYTES_MLOFFYES_NBYTES_MASK          0x3FFu
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT         0
#define DMA_NBYTES_MLOFFYES_NBYTES_WIDTH         10
#define DMA_NBYTES_MLOFFYES_NBYTES(x)            (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_NBYTES_SHIFT))&DMA_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           0x3FFFFC00u
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          10
#define DMA_NBYTES_MLOFFYES_MLOFF_WIDTH          20
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_MLOFF_SHIFT))&DMA_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_NBYTES_MLOFFYES_DMLOE_MASK           0x40000000u
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT          30
#define DMA_NBYTES_MLOFFYES_DMLOE_WIDTH          1
#define DMA_NBYTES_MLOFFYES_DMLOE(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_DMLOE_SHIFT))&DMA_NBYTES_MLOFFYES_DMLOE_MASK)
#define DMA_NBYTES_MLOFFYES_SMLOE_MASK           0x80000000u
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT          31
#define DMA_NBYTES_MLOFFYES_SMLOE_WIDTH          1
#define DMA_NBYTES_MLOFFYES_SMLOE(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_SMLOE_SHIFT))&DMA_NBYTES_MLOFFYES_SMLOE_MASK)
/* SLAST Bit Fields */
#define DMA_SLAST_SLAST_MASK                     0xFFFFFFFFu
#define DMA_SLAST_SLAST_SHIFT                    0
#define DMA_SLAST_SLAST_WIDTH                    32
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SLAST_SLAST_SHIFT))&DMA_SLAST_SLAST_MASK)
/* DADDR Bit Fields */
#define DMA_DADDR_DADDR_MASK                     0xFFFFFFFFu
#define DMA_DADDR_DADDR_SHIFT                    0
#define DMA_DADDR_DADDR_WIDTH                    32
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_DADDR_DADDR_SHIFT))&DMA_DADDR_DADDR_MASK)
/* DOFF Bit Fields */
#define DMA_DOFF_DOFF_MASK                       0xFFFFu
#define DMA_DOFF_DOFF_SHIFT                      0
#define DMA_DOFF_DOFF_WIDTH                      16
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_DOFF_DOFF_SHIFT))&DMA_DOFF_DOFF_MASK)
/* CITER_ELINKNO Bit Fields */
#define DMA_CITER_ELINKNO_CITER_MASK             0x7FFFu
#define DMA_CITER_ELINKNO_CITER_SHIFT            0
#define DMA_CITER_ELINKNO_CITER_WIDTH            15
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKNO_CITER_SHIFT))&DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_CITER_ELINKNO_ELINK_SHIFT            15
#define DMA_CITER_ELINKNO_ELINK_WIDTH            1
#define DMA_CITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKNO_ELINK_SHIFT))&DMA_CITER_ELINKNO_ELINK_MASK)
/* CITER_ELINKYES Bit Fields */
#define DMA_CITER_ELINKYES_CITER_MASK            0x1FFu
#define DMA_CITER_ELINKYES_CITER_SHIFT           0
#define DMA_CITER_ELINKYES_CITER_WIDTH           9
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_CITER_SHIFT))&DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_CITER_ELINKYES_LINKCH_WIDTH          4
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_LINKCH_SHIFT))&DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_CITER_ELINKYES_ELINK_SHIFT           15
#define DMA_CITER_ELINKYES_ELINK_WIDTH           1
#define DMA_CITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_ELINK_SHIFT))&DMA_CITER_ELINKYES_ELINK_MASK)
/* DLAST_SGA Bit Fields */
#define DMA_DLAST_SGA_DLASTSGA_MASK              0xFFFFFFFFu
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             0
#define DMA_DLAST_SGA_DLASTSGA_WIDTH             32
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x))<<DMA_DLAST_SGA_DLASTSGA_SHIFT))&DMA_DLAST_SGA_DLASTSGA_MASK)
/* CSR Bit Fields */
#define DMA_CSR_START_MASK                       0x1u
#define DMA_CSR_START_SHIFT                      0
#define DMA_CSR_START_WIDTH                      1
#define DMA_CSR_START(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_CSR_START_SHIFT))&DMA_CSR_START_MASK)
#define DMA_CSR_INTMAJOR_MASK                    0x2u
#define DMA_CSR_INTMAJOR_SHIFT                   1
#define DMA_CSR_INTMAJOR_WIDTH                   1
#define DMA_CSR_INTMAJOR(x)                      (((uint16_t)(((uint16_t)(x))<<DMA_CSR_INTMAJOR_SHIFT))&DMA_CSR_INTMAJOR_MASK)
#define DMA_CSR_INTHALF_MASK                     0x4u
#define DMA_CSR_INTHALF_SHIFT                    2
#define DMA_CSR_INTHALF_WIDTH                    1
#define DMA_CSR_INTHALF(x)                       (((uint16_t)(((uint16_t)(x))<<DMA_CSR_INTHALF_SHIFT))&DMA_CSR_INTHALF_MASK)
#define DMA_CSR_DREQ_MASK                        0x8u
#define DMA_CSR_DREQ_SHIFT                       3
#define DMA_CSR_DREQ_WIDTH                       1
#define DMA_CSR_DREQ(x)                          (((uint16_t)(((uint16_t)(x))<<DMA_CSR_DREQ_SHIFT))&DMA_CSR_DREQ_MASK)
#define DMA_CSR_ESG_MASK                         0x10u
#define DMA_CSR_ESG_SHIFT                        4
#define DMA_CSR_ESG_WIDTH                        1
#define DMA_CSR_ESG(x)                           (((uint16_t)(((uint16_t)(x))<<DMA_CSR_ESG_SHIFT))&DMA_CSR_ESG_MASK)
#define DMA_CSR_MAJORELINK_MASK                  0x20u
#define DMA_CSR_MAJORELINK_SHIFT                 5
#define DMA_CSR_MAJORELINK_WIDTH                 1
#define DMA_CSR_MAJORELINK(x)                    (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORELINK_SHIFT))&DMA_CSR_MAJORELINK_MASK)
#define DMA_CSR_ACTIVE_MASK                      0x40u
#define DMA_CSR_ACTIVE_SHIFT                     6
#define DMA_CSR_ACTIVE_WIDTH                     1
#define DMA_CSR_ACTIVE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_CSR_ACTIVE_SHIFT))&DMA_CSR_ACTIVE_MASK)
#define DMA_CSR_DONE_MASK                        0x80u
#define DMA_CSR_DONE_SHIFT                       7
#define DMA_CSR_DONE_WIDTH                       1
#define DMA_CSR_DONE(x)                          (((uint16_t)(((uint16_t)(x))<<DMA_CSR_DONE_SHIFT))&DMA_CSR_DONE_MASK)
#define DMA_CSR_MAJORLINKCH_MASK                 0xF00u
#define DMA_CSR_MAJORLINKCH_SHIFT                8
#define DMA_CSR_MAJORLINKCH_WIDTH                4
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORLINKCH_SHIFT))&DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         0xC000u
#define DMA_CSR_BWC_SHIFT                        14
#define DMA_CSR_BWC_WIDTH                        2
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x))<<DMA_CSR_BWC_SHIFT))&DMA_CSR_BWC_MASK)
/* BITER_ELINKNO Bit Fields */
#define DMA_BITER_ELINKNO_BITER_MASK             0x7FFFu
#define DMA_BITER_ELINKNO_BITER_SHIFT            0
#define DMA_BITER_ELINKNO_BITER_WIDTH            15
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_BITER_SHIFT))&DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_BITER_ELINKNO_ELINK_SHIFT            15
#define DMA_BITER_ELINKNO_ELINK_WIDTH            1
#define DMA_BITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_ELINK_SHIFT))&DMA_BITER_ELINKNO_ELINK_MASK)
/* BITER_ELINKYES Bit Fields */
#define DMA_BITER_ELINKYES_BITER_MASK            0x1FFu
#define DMA_BITER_ELINKYES_BITER_SHIFT           0
#define DMA_BITER_ELINKYES_BITER_WIDTH           9
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_BITER_SHIFT))&DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_BITER_ELINKYES_LINKCH_WIDTH          4
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_LINKCH_SHIFT))&DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_BITER_ELINKYES_ELINK_SHIFT           15
#define DMA_BITER_ELINKYES_ELINK_WIDTH           1
#define DMA_BITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_ELINK_SHIFT))&DMA_BITER_ELINKYES_ELINK_MASK)

/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base address */
#define DMA_BASE                                 (0x40008000u)
/** Peripheral DMA base pointer */
#define DMA0                                     ((DMA_Type *)DMA_BASE)
#define DMA_BASE_PTR                             (DMA0)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register instance definitions */
/* DMA */
#define DMA_CR                                   DMA_CR_REG(DMA0)
#define DMA_ES                                   DMA_ES_REG(DMA0)
#define DMA_ERQ                                  DMA_ERQ_REG(DMA0)
#define DMA_EEI                                  DMA_EEI_REG(DMA0)
#define DMA_CEEI                                 DMA_CEEI_REG(DMA0)
#define DMA_SEEI                                 DMA_SEEI_REG(DMA0)
#define DMA_CERQ                                 DMA_CERQ_REG(DMA0)
#define DMA_SERQ                                 DMA_SERQ_REG(DMA0)
#define DMA_CDNE                                 DMA_CDNE_REG(DMA0)
#define DMA_SSRT                                 DMA_SSRT_REG(DMA0)
#define DMA_CERR                                 DMA_CERR_REG(DMA0)
#define DMA_CINT                                 DMA_CINT_REG(DMA0)
#define DMA_INT                                  DMA_INT_REG(DMA0)
#define DMA_ERR                                  DMA_ERR_REG(DMA0)
#define DMA_HRS                                  DMA_HRS_REG(DMA0)
#define DMA_EARS                                 DMA_EARS_REG(DMA0)
#define DMA_DCHPRI3                              DMA_DCHPRI3_REG(DMA0)
#define DMA_DCHPRI2                              DMA_DCHPRI2_REG(DMA0)
#define DMA_DCHPRI1                              DMA_DCHPRI1_REG(DMA0)
#define DMA_DCHPRI0                              DMA_DCHPRI0_REG(DMA0)
#define DMA_DCHPRI7                              DMA_DCHPRI7_REG(DMA0)
#define DMA_DCHPRI6                              DMA_DCHPRI6_REG(DMA0)
#define DMA_DCHPRI5                              DMA_DCHPRI5_REG(DMA0)
#define DMA_DCHPRI4                              DMA_DCHPRI4_REG(DMA0)
#define DMA_DCHPRI11                             DMA_DCHPRI11_REG(DMA0)
#define DMA_DCHPRI10                             DMA_DCHPRI10_REG(DMA0)
#define DMA_DCHPRI9                              DMA_DCHPRI9_REG(DMA0)
#define DMA_DCHPRI8                              DMA_DCHPRI8_REG(DMA0)
#define DMA_DCHPRI15                             DMA_DCHPRI15_REG(DMA0)
#define DMA_DCHPRI14                             DMA_DCHPRI14_REG(DMA0)
#define DMA_DCHPRI13                             DMA_DCHPRI13_REG(DMA0)
#define DMA_DCHPRI12                             DMA_DCHPRI12_REG(DMA0)
#define DMA_TCD0_SADDR                           DMA_SADDR_REG(DMA0,0)
#define DMA_TCD0_SOFF                            DMA_SOFF_REG(DMA0,0)
#define DMA_TCD0_ATTR                            DMA_ATTR_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,0)
#define DMA_TCD0_SLAST                           DMA_SLAST_REG(DMA0,0)
#define DMA_TCD0_DADDR                           DMA_DADDR_REG(DMA0,0)
#define DMA_TCD0_DOFF                            DMA_DOFF_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD0_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,0)
#define DMA_TCD0_CSR                             DMA_CSR_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD1_SADDR                           DMA_SADDR_REG(DMA0,1)
#define DMA_TCD1_SOFF                            DMA_SOFF_REG(DMA0,1)
#define DMA_TCD1_ATTR                            DMA_ATTR_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,1)
#define DMA_TCD1_SLAST                           DMA_SLAST_REG(DMA0,1)
#define DMA_TCD1_DADDR                           DMA_DADDR_REG(DMA0,1)
#define DMA_TCD1_DOFF                            DMA_DOFF_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD1_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,1)
#define DMA_TCD1_CSR                             DMA_CSR_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD2_SADDR                           DMA_SADDR_REG(DMA0,2)
#define DMA_TCD2_SOFF                            DMA_SOFF_REG(DMA0,2)
#define DMA_TCD2_ATTR                            DMA_ATTR_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,2)
#define DMA_TCD2_SLAST                           DMA_SLAST_REG(DMA0,2)
#define DMA_TCD2_DADDR                           DMA_DADDR_REG(DMA0,2)
#define DMA_TCD2_DOFF                            DMA_DOFF_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD2_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,2)
#define DMA_TCD2_CSR                             DMA_CSR_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD3_SADDR                           DMA_SADDR_REG(DMA0,3)
#define DMA_TCD3_SOFF                            DMA_SOFF_REG(DMA0,3)
#define DMA_TCD3_ATTR                            DMA_ATTR_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,3)
#define DMA_TCD3_SLAST                           DMA_SLAST_REG(DMA0,3)
#define DMA_TCD3_DADDR                           DMA_DADDR_REG(DMA0,3)
#define DMA_TCD3_DOFF                            DMA_DOFF_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD3_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,3)
#define DMA_TCD3_CSR                             DMA_CSR_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD4_SADDR                           DMA_SADDR_REG(DMA0,4)
#define DMA_TCD4_SOFF                            DMA_SOFF_REG(DMA0,4)
#define DMA_TCD4_ATTR                            DMA_ATTR_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,4)
#define DMA_TCD4_SLAST                           DMA_SLAST_REG(DMA0,4)
#define DMA_TCD4_DADDR                           DMA_DADDR_REG(DMA0,4)
#define DMA_TCD4_DOFF                            DMA_DOFF_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD4_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,4)
#define DMA_TCD4_CSR                             DMA_CSR_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD5_SADDR                           DMA_SADDR_REG(DMA0,5)
#define DMA_TCD5_SOFF                            DMA_SOFF_REG(DMA0,5)
#define DMA_TCD5_ATTR                            DMA_ATTR_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,5)
#define DMA_TCD5_SLAST                           DMA_SLAST_REG(DMA0,5)
#define DMA_TCD5_DADDR                           DMA_DADDR_REG(DMA0,5)
#define DMA_TCD5_DOFF                            DMA_DOFF_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD5_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,5)
#define DMA_TCD5_CSR                             DMA_CSR_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD6_SADDR                           DMA_SADDR_REG(DMA0,6)
#define DMA_TCD6_SOFF                            DMA_SOFF_REG(DMA0,6)
#define DMA_TCD6_ATTR                            DMA_ATTR_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,6)
#define DMA_TCD6_SLAST                           DMA_SLAST_REG(DMA0,6)
#define DMA_TCD6_DADDR                           DMA_DADDR_REG(DMA0,6)
#define DMA_TCD6_DOFF                            DMA_DOFF_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD6_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,6)
#define DMA_TCD6_CSR                             DMA_CSR_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD7_SADDR                           DMA_SADDR_REG(DMA0,7)
#define DMA_TCD7_SOFF                            DMA_SOFF_REG(DMA0,7)
#define DMA_TCD7_ATTR                            DMA_ATTR_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,7)
#define DMA_TCD7_SLAST                           DMA_SLAST_REG(DMA0,7)
#define DMA_TCD7_DADDR                           DMA_DADDR_REG(DMA0,7)
#define DMA_TCD7_DOFF                            DMA_DOFF_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD7_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,7)
#define DMA_TCD7_CSR                             DMA_CSR_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD8_SADDR                           DMA_SADDR_REG(DMA0,8)
#define DMA_TCD8_SOFF                            DMA_SOFF_REG(DMA0,8)
#define DMA_TCD8_ATTR                            DMA_ATTR_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,8)
#define DMA_TCD8_SLAST                           DMA_SLAST_REG(DMA0,8)
#define DMA_TCD8_DADDR                           DMA_DADDR_REG(DMA0,8)
#define DMA_TCD8_DOFF                            DMA_DOFF_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD8_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,8)
#define DMA_TCD8_CSR                             DMA_CSR_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD9_SADDR                           DMA_SADDR_REG(DMA0,9)
#define DMA_TCD9_SOFF                            DMA_SOFF_REG(DMA0,9)
#define DMA_TCD9_ATTR                            DMA_ATTR_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,9)
#define DMA_TCD9_SLAST                           DMA_SLAST_REG(DMA0,9)
#define DMA_TCD9_DADDR                           DMA_DADDR_REG(DMA0,9)
#define DMA_TCD9_DOFF                            DMA_DOFF_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD9_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,9)
#define DMA_TCD9_CSR                             DMA_CSR_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD10_SADDR                          DMA_SADDR_REG(DMA0,10)
#define DMA_TCD10_SOFF                           DMA_SOFF_REG(DMA0,10)
#define DMA_TCD10_ATTR                           DMA_ATTR_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,10)
#define DMA_TCD10_SLAST                          DMA_SLAST_REG(DMA0,10)
#define DMA_TCD10_DADDR                          DMA_DADDR_REG(DMA0,10)
#define DMA_TCD10_DOFF                           DMA_DOFF_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD10_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,10)
#define DMA_TCD10_CSR                            DMA_CSR_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD11_SADDR                          DMA_SADDR_REG(DMA0,11)
#define DMA_TCD11_SOFF                           DMA_SOFF_REG(DMA0,11)
#define DMA_TCD11_ATTR                           DMA_ATTR_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,11)
#define DMA_TCD11_SLAST                          DMA_SLAST_REG(DMA0,11)
#define DMA_TCD11_DADDR                          DMA_DADDR_REG(DMA0,11)
#define DMA_TCD11_DOFF                           DMA_DOFF_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD11_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,11)
#define DMA_TCD11_CSR                            DMA_CSR_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD12_SADDR                          DMA_SADDR_REG(DMA0,12)
#define DMA_TCD12_SOFF                           DMA_SOFF_REG(DMA0,12)
#define DMA_TCD12_ATTR                           DMA_ATTR_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,12)
#define DMA_TCD12_SLAST                          DMA_SLAST_REG(DMA0,12)
#define DMA_TCD12_DADDR                          DMA_DADDR_REG(DMA0,12)
#define DMA_TCD12_DOFF                           DMA_DOFF_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD12_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,12)
#define DMA_TCD12_CSR                            DMA_CSR_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD13_SADDR                          DMA_SADDR_REG(DMA0,13)
#define DMA_TCD13_SOFF                           DMA_SOFF_REG(DMA0,13)
#define DMA_TCD13_ATTR                           DMA_ATTR_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,13)
#define DMA_TCD13_SLAST                          DMA_SLAST_REG(DMA0,13)
#define DMA_TCD13_DADDR                          DMA_DADDR_REG(DMA0,13)
#define DMA_TCD13_DOFF                           DMA_DOFF_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD13_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,13)
#define DMA_TCD13_CSR                            DMA_CSR_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD14_SADDR                          DMA_SADDR_REG(DMA0,14)
#define DMA_TCD14_SOFF                           DMA_SOFF_REG(DMA0,14)
#define DMA_TCD14_ATTR                           DMA_ATTR_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,14)
#define DMA_TCD14_SLAST                          DMA_SLAST_REG(DMA0,14)
#define DMA_TCD14_DADDR                          DMA_DADDR_REG(DMA0,14)
#define DMA_TCD14_DOFF                           DMA_DOFF_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD14_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,14)
#define DMA_TCD14_CSR                            DMA_CSR_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD15_SADDR                          DMA_SADDR_REG(DMA0,15)
#define DMA_TCD15_SOFF                           DMA_SOFF_REG(DMA0,15)
#define DMA_TCD15_ATTR                           DMA_ATTR_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,15)
#define DMA_TCD15_SLAST                          DMA_SLAST_REG(DMA0,15)
#define DMA_TCD15_DADDR                          DMA_DADDR_REG(DMA0,15)
#define DMA_TCD15_DOFF                           DMA_DOFF_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,15)
#define DMA_TCD15_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,15)
#define DMA_TCD15_CSR                            DMA_CSR_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,15)

/* DMA - Register array accessors */
#define DMA_SADDR(index)                         DMA_SADDR_REG(DMA0,index)
#define DMA_SOFF(index)                          DMA_SOFF_REG(DMA0,index)
#define DMA_ATTR(index)                          DMA_ATTR_REG(DMA0,index)
#define DMA_NBYTES_MLNO(index)                   DMA_NBYTES_MLNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFNO(index)                DMA_NBYTES_MLOFFNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFYES(index)               DMA_NBYTES_MLOFFYES_REG(DMA0,index)
#define DMA_SLAST(index)                         DMA_SLAST_REG(DMA0,index)
#define DMA_DADDR(index)                         DMA_DADDR_REG(DMA0,index)
#define DMA_DOFF(index)                          DMA_DOFF_REG(DMA0,index)
#define DMA_CITER_ELINKNO(index)                 DMA_CITER_ELINKNO_REG(DMA0,index)
#define DMA_CITER_ELINKYES(index)                DMA_CITER_ELINKYES_REG(DMA0,index)
#define DMA_DLAST_SGA(index)                     DMA_DLAST_SGA_REG(DMA0,index)
#define DMA_CSR(index)                           DMA_CSR_REG(DMA0,index)
#define DMA_BITER_ELINKNO(index)                 DMA_BITER_ELINKNO_REG(DMA0,index)
#define DMA_BITER_ELINKYES(index)                DMA_BITER_ELINKYES_REG(DMA0,index)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Register Layout Typedef */
typedef struct {
  __IO uint8_t CHCFG[16];                          /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_Type, *DMAMUX_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register accessors */
#define DMAMUX_CHCFG_REG(base,index)             ((base)->CHCFG[index])
#define DMAMUX_CHCFG_COUNT                       16

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/* CHCFG Bit Fields */
#define DMAMUX_CHCFG_SOURCE_MASK                 0x3Fu
#define DMAMUX_CHCFG_SOURCE_SHIFT                0
#define DMAMUX_CHCFG_SOURCE_WIDTH                6
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x))<<DMAMUX_CHCFG_SOURCE_SHIFT))&DMAMUX_CHCFG_SOURCE_MASK)
#define DMAMUX_CHCFG_TRIG_MASK                   0x40u
#define DMAMUX_CHCFG_TRIG_SHIFT                  6
#define DMAMUX_CHCFG_TRIG_WIDTH                  1
#define DMAMUX_CHCFG_TRIG(x)                     (((uint8_t)(((uint8_t)(x))<<DMAMUX_CHCFG_TRIG_SHIFT))&DMAMUX_CHCFG_TRIG_MASK)
#define DMAMUX_CHCFG_ENBL_MASK                   0x80u
#define DMAMUX_CHCFG_ENBL_SHIFT                  7
#define DMAMUX_CHCFG_ENBL_WIDTH                  1
#define DMAMUX_CHCFG_ENBL(x)                     (((uint8_t)(((uint8_t)(x))<<DMAMUX_CHCFG_ENBL_SHIFT))&DMAMUX_CHCFG_ENBL_MASK)

/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE                              (0x40021000u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX                                   ((DMAMUX_Type *)DMAMUX_BASE)
#define DMAMUX_BASE_PTR                          (DMAMUX)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX }

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register instance definitions */
/* DMAMUX */
#define DMAMUX_CHCFG0                            DMAMUX_CHCFG_REG(DMAMUX,0)
#define DMAMUX_CHCFG1                            DMAMUX_CHCFG_REG(DMAMUX,1)
#define DMAMUX_CHCFG2                            DMAMUX_CHCFG_REG(DMAMUX,2)
#define DMAMUX_CHCFG3                            DMAMUX_CHCFG_REG(DMAMUX,3)
#define DMAMUX_CHCFG4                            DMAMUX_CHCFG_REG(DMAMUX,4)
#define DMAMUX_CHCFG5                            DMAMUX_CHCFG_REG(DMAMUX,5)
#define DMAMUX_CHCFG6                            DMAMUX_CHCFG_REG(DMAMUX,6)
#define DMAMUX_CHCFG7                            DMAMUX_CHCFG_REG(DMAMUX,7)
#define DMAMUX_CHCFG8                            DMAMUX_CHCFG_REG(DMAMUX,8)
#define DMAMUX_CHCFG9                            DMAMUX_CHCFG_REG(DMAMUX,9)
#define DMAMUX_CHCFG10                           DMAMUX_CHCFG_REG(DMAMUX,10)
#define DMAMUX_CHCFG11                           DMAMUX_CHCFG_REG(DMAMUX,11)
#define DMAMUX_CHCFG12                           DMAMUX_CHCFG_REG(DMAMUX,12)
#define DMAMUX_CHCFG13                           DMAMUX_CHCFG_REG(DMAMUX,13)
#define DMAMUX_CHCFG14                           DMAMUX_CHCFG_REG(DMAMUX,14)
#define DMAMUX_CHCFG15                           DMAMUX_CHCFG_REG(DMAMUX,15)

/* DMAMUX - Register array accessors */
#define DMAMUX_CHCFG(index)                      DMAMUX_CHCFG_REG(DMAMUX,index)

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- ENC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ENC_Peripheral_Access_Layer ENC Peripheral Access Layer
 * @{
 */

/** ENC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x0 */
  __IO uint16_t FILT;                              /**< Input Filter Register, offset: 0x2 */
  __IO uint16_t WTR;                               /**< Watchdog Timeout Register, offset: 0x4 */
  __IO uint16_t POSD;                              /**< Position Difference Counter Register, offset: 0x6 */
  __I  uint16_t POSDH;                             /**< Position Difference Hold Register, offset: 0x8 */
  __IO uint16_t REV;                               /**< Revolution Counter Register, offset: 0xA */
  __I  uint16_t REVH;                              /**< Revolution Hold Register, offset: 0xC */
  __IO uint16_t UPOS;                              /**< Upper Position Counter Register, offset: 0xE */
  __IO uint16_t LPOS;                              /**< Lower Position Counter Register, offset: 0x10 */
  __I  uint16_t UPOSH;                             /**< Upper Position Hold Register, offset: 0x12 */
  __I  uint16_t LPOSH;                             /**< Lower Position Hold Register, offset: 0x14 */
  __IO uint16_t UINIT;                             /**< Upper Initialization Register, offset: 0x16 */
  __IO uint16_t LINIT;                             /**< Lower Initialization Register, offset: 0x18 */
  __I  uint16_t IMR;                               /**< Input Monitor Register, offset: 0x1A */
  __IO uint16_t TST;                               /**< Test Register, offset: 0x1C */
  __IO uint16_t CTRL2;                             /**< Control 2 Register, offset: 0x1E */
  __IO uint16_t UMOD;                              /**< Upper Modulus Register, offset: 0x20 */
  __IO uint16_t LMOD;                              /**< Lower Modulus Register, offset: 0x22 */
  __IO uint16_t UCOMP;                             /**< Upper Position Compare Register, offset: 0x24 */
  __IO uint16_t LCOMP;                             /**< Lower Position Compare Register, offset: 0x26 */
} ENC_Type, *ENC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- ENC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ENC_Register_Accessor_Macros ENC - Register accessor macros
 * @{
 */


/* ENC - Register accessors */
#define ENC_CTRL_REG(base)                       ((base)->CTRL)
#define ENC_FILT_REG(base)                       ((base)->FILT)
#define ENC_WTR_REG(base)                        ((base)->WTR)
#define ENC_POSD_REG(base)                       ((base)->POSD)
#define ENC_POSDH_REG(base)                      ((base)->POSDH)
#define ENC_REV_REG(base)                        ((base)->REV)
#define ENC_REVH_REG(base)                       ((base)->REVH)
#define ENC_UPOS_REG(base)                       ((base)->UPOS)
#define ENC_LPOS_REG(base)                       ((base)->LPOS)
#define ENC_UPOSH_REG(base)                      ((base)->UPOSH)
#define ENC_LPOSH_REG(base)                      ((base)->LPOSH)
#define ENC_UINIT_REG(base)                      ((base)->UINIT)
#define ENC_LINIT_REG(base)                      ((base)->LINIT)
#define ENC_IMR_REG(base)                        ((base)->IMR)
#define ENC_TST_REG(base)                        ((base)->TST)
#define ENC_CTRL2_REG(base)                      ((base)->CTRL2)
#define ENC_UMOD_REG(base)                       ((base)->UMOD)
#define ENC_LMOD_REG(base)                       ((base)->LMOD)
#define ENC_UCOMP_REG(base)                      ((base)->UCOMP)
#define ENC_LCOMP_REG(base)                      ((base)->LCOMP)

/*!
 * @}
 */ /* end of group ENC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- ENC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ENC_Register_Masks ENC Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define ENC_CTRL_CMPIE_MASK                      0x1u
#define ENC_CTRL_CMPIE_SHIFT                     0
#define ENC_CTRL_CMPIE_WIDTH                     1
#define ENC_CTRL_CMPIE(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_CMPIE_SHIFT))&ENC_CTRL_CMPIE_MASK)
#define ENC_CTRL_CMPIRQ_MASK                     0x2u
#define ENC_CTRL_CMPIRQ_SHIFT                    1
#define ENC_CTRL_CMPIRQ_WIDTH                    1
#define ENC_CTRL_CMPIRQ(x)                       (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_CMPIRQ_SHIFT))&ENC_CTRL_CMPIRQ_MASK)
#define ENC_CTRL_WDE_MASK                        0x4u
#define ENC_CTRL_WDE_SHIFT                       2
#define ENC_CTRL_WDE_WIDTH                       1
#define ENC_CTRL_WDE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_WDE_SHIFT))&ENC_CTRL_WDE_MASK)
#define ENC_CTRL_DIE_MASK                        0x8u
#define ENC_CTRL_DIE_SHIFT                       3
#define ENC_CTRL_DIE_WIDTH                       1
#define ENC_CTRL_DIE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_DIE_SHIFT))&ENC_CTRL_DIE_MASK)
#define ENC_CTRL_DIRQ_MASK                       0x10u
#define ENC_CTRL_DIRQ_SHIFT                      4
#define ENC_CTRL_DIRQ_WIDTH                      1
#define ENC_CTRL_DIRQ(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_DIRQ_SHIFT))&ENC_CTRL_DIRQ_MASK)
#define ENC_CTRL_XNE_MASK                        0x20u
#define ENC_CTRL_XNE_SHIFT                       5
#define ENC_CTRL_XNE_WIDTH                       1
#define ENC_CTRL_XNE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_XNE_SHIFT))&ENC_CTRL_XNE_MASK)
#define ENC_CTRL_XIP_MASK                        0x40u
#define ENC_CTRL_XIP_SHIFT                       6
#define ENC_CTRL_XIP_WIDTH                       1
#define ENC_CTRL_XIP(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_XIP_SHIFT))&ENC_CTRL_XIP_MASK)
#define ENC_CTRL_XIE_MASK                        0x80u
#define ENC_CTRL_XIE_SHIFT                       7
#define ENC_CTRL_XIE_WIDTH                       1
#define ENC_CTRL_XIE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_XIE_SHIFT))&ENC_CTRL_XIE_MASK)
#define ENC_CTRL_XIRQ_MASK                       0x100u
#define ENC_CTRL_XIRQ_SHIFT                      8
#define ENC_CTRL_XIRQ_WIDTH                      1
#define ENC_CTRL_XIRQ(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_XIRQ_SHIFT))&ENC_CTRL_XIRQ_MASK)
#define ENC_CTRL_PH1_MASK                        0x200u
#define ENC_CTRL_PH1_SHIFT                       9
#define ENC_CTRL_PH1_WIDTH                       1
#define ENC_CTRL_PH1(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_PH1_SHIFT))&ENC_CTRL_PH1_MASK)
#define ENC_CTRL_REV_MASK                        0x400u
#define ENC_CTRL_REV_SHIFT                       10
#define ENC_CTRL_REV_WIDTH                       1
#define ENC_CTRL_REV(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_REV_SHIFT))&ENC_CTRL_REV_MASK)
#define ENC_CTRL_SWIP_MASK                       0x800u
#define ENC_CTRL_SWIP_SHIFT                      11
#define ENC_CTRL_SWIP_WIDTH                      1
#define ENC_CTRL_SWIP(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_SWIP_SHIFT))&ENC_CTRL_SWIP_MASK)
#define ENC_CTRL_HNE_MASK                        0x1000u
#define ENC_CTRL_HNE_SHIFT                       12
#define ENC_CTRL_HNE_WIDTH                       1
#define ENC_CTRL_HNE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_HNE_SHIFT))&ENC_CTRL_HNE_MASK)
#define ENC_CTRL_HIP_MASK                        0x2000u
#define ENC_CTRL_HIP_SHIFT                       13
#define ENC_CTRL_HIP_WIDTH                       1
#define ENC_CTRL_HIP(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_HIP_SHIFT))&ENC_CTRL_HIP_MASK)
#define ENC_CTRL_HIE_MASK                        0x4000u
#define ENC_CTRL_HIE_SHIFT                       14
#define ENC_CTRL_HIE_WIDTH                       1
#define ENC_CTRL_HIE(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_HIE_SHIFT))&ENC_CTRL_HIE_MASK)
#define ENC_CTRL_HIRQ_MASK                       0x8000u
#define ENC_CTRL_HIRQ_SHIFT                      15
#define ENC_CTRL_HIRQ_WIDTH                      1
#define ENC_CTRL_HIRQ(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL_HIRQ_SHIFT))&ENC_CTRL_HIRQ_MASK)
/* FILT Bit Fields */
#define ENC_FILT_FILT_PER_MASK                   0xFFu
#define ENC_FILT_FILT_PER_SHIFT                  0
#define ENC_FILT_FILT_PER_WIDTH                  8
#define ENC_FILT_FILT_PER(x)                     (((uint16_t)(((uint16_t)(x))<<ENC_FILT_FILT_PER_SHIFT))&ENC_FILT_FILT_PER_MASK)
#define ENC_FILT_FILT_CNT_MASK                   0x700u
#define ENC_FILT_FILT_CNT_SHIFT                  8
#define ENC_FILT_FILT_CNT_WIDTH                  3
#define ENC_FILT_FILT_CNT(x)                     (((uint16_t)(((uint16_t)(x))<<ENC_FILT_FILT_CNT_SHIFT))&ENC_FILT_FILT_CNT_MASK)
/* WTR Bit Fields */
#define ENC_WTR_WDOG_MASK                        0xFFFFu
#define ENC_WTR_WDOG_SHIFT                       0
#define ENC_WTR_WDOG_WIDTH                       16
#define ENC_WTR_WDOG(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_WTR_WDOG_SHIFT))&ENC_WTR_WDOG_MASK)
/* POSD Bit Fields */
#define ENC_POSD_POSD_MASK                       0xFFFFu
#define ENC_POSD_POSD_SHIFT                      0
#define ENC_POSD_POSD_WIDTH                      16
#define ENC_POSD_POSD(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_POSD_POSD_SHIFT))&ENC_POSD_POSD_MASK)
/* POSDH Bit Fields */
#define ENC_POSDH_POSDH_MASK                     0xFFFFu
#define ENC_POSDH_POSDH_SHIFT                    0
#define ENC_POSDH_POSDH_WIDTH                    16
#define ENC_POSDH_POSDH(x)                       (((uint16_t)(((uint16_t)(x))<<ENC_POSDH_POSDH_SHIFT))&ENC_POSDH_POSDH_MASK)
/* REV Bit Fields */
#define ENC_REV_REV_MASK                         0xFFFFu
#define ENC_REV_REV_SHIFT                        0
#define ENC_REV_REV_WIDTH                        16
#define ENC_REV_REV(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_REV_REV_SHIFT))&ENC_REV_REV_MASK)
/* REVH Bit Fields */
#define ENC_REVH_REVH_MASK                       0xFFFFu
#define ENC_REVH_REVH_SHIFT                      0
#define ENC_REVH_REVH_WIDTH                      16
#define ENC_REVH_REVH(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_REVH_REVH_SHIFT))&ENC_REVH_REVH_MASK)
/* UPOS Bit Fields */
#define ENC_UPOS_POS_MASK                        0xFFFFu
#define ENC_UPOS_POS_SHIFT                       0
#define ENC_UPOS_POS_WIDTH                       16
#define ENC_UPOS_POS(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_UPOS_POS_SHIFT))&ENC_UPOS_POS_MASK)
/* LPOS Bit Fields */
#define ENC_LPOS_POS_MASK                        0xFFFFu
#define ENC_LPOS_POS_SHIFT                       0
#define ENC_LPOS_POS_WIDTH                       16
#define ENC_LPOS_POS(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_LPOS_POS_SHIFT))&ENC_LPOS_POS_MASK)
/* UPOSH Bit Fields */
#define ENC_UPOSH_POSH_MASK                      0xFFFFu
#define ENC_UPOSH_POSH_SHIFT                     0
#define ENC_UPOSH_POSH_WIDTH                     16
#define ENC_UPOSH_POSH(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_UPOSH_POSH_SHIFT))&ENC_UPOSH_POSH_MASK)
/* LPOSH Bit Fields */
#define ENC_LPOSH_POSH_MASK                      0xFFFFu
#define ENC_LPOSH_POSH_SHIFT                     0
#define ENC_LPOSH_POSH_WIDTH                     16
#define ENC_LPOSH_POSH(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_LPOSH_POSH_SHIFT))&ENC_LPOSH_POSH_MASK)
/* UINIT Bit Fields */
#define ENC_UINIT_INIT_MASK                      0xFFFFu
#define ENC_UINIT_INIT_SHIFT                     0
#define ENC_UINIT_INIT_WIDTH                     16
#define ENC_UINIT_INIT(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_UINIT_INIT_SHIFT))&ENC_UINIT_INIT_MASK)
/* LINIT Bit Fields */
#define ENC_LINIT_INIT_MASK                      0xFFFFu
#define ENC_LINIT_INIT_SHIFT                     0
#define ENC_LINIT_INIT_WIDTH                     16
#define ENC_LINIT_INIT(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_LINIT_INIT_SHIFT))&ENC_LINIT_INIT_MASK)
/* IMR Bit Fields */
#define ENC_IMR_HOME_MASK                        0x1u
#define ENC_IMR_HOME_SHIFT                       0
#define ENC_IMR_HOME_WIDTH                       1
#define ENC_IMR_HOME(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_IMR_HOME_SHIFT))&ENC_IMR_HOME_MASK)
#define ENC_IMR_INDEX_MASK                       0x2u
#define ENC_IMR_INDEX_SHIFT                      1
#define ENC_IMR_INDEX_WIDTH                      1
#define ENC_IMR_INDEX(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_IMR_INDEX_SHIFT))&ENC_IMR_INDEX_MASK)
#define ENC_IMR_PHB_MASK                         0x4u
#define ENC_IMR_PHB_SHIFT                        2
#define ENC_IMR_PHB_WIDTH                        1
#define ENC_IMR_PHB(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_IMR_PHB_SHIFT))&ENC_IMR_PHB_MASK)
#define ENC_IMR_PHA_MASK                         0x8u
#define ENC_IMR_PHA_SHIFT                        3
#define ENC_IMR_PHA_WIDTH                        1
#define ENC_IMR_PHA(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_IMR_PHA_SHIFT))&ENC_IMR_PHA_MASK)
#define ENC_IMR_FHOM_MASK                        0x10u
#define ENC_IMR_FHOM_SHIFT                       4
#define ENC_IMR_FHOM_WIDTH                       1
#define ENC_IMR_FHOM(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_IMR_FHOM_SHIFT))&ENC_IMR_FHOM_MASK)
#define ENC_IMR_FIND_MASK                        0x20u
#define ENC_IMR_FIND_SHIFT                       5
#define ENC_IMR_FIND_WIDTH                       1
#define ENC_IMR_FIND(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_IMR_FIND_SHIFT))&ENC_IMR_FIND_MASK)
#define ENC_IMR_FPHB_MASK                        0x40u
#define ENC_IMR_FPHB_SHIFT                       6
#define ENC_IMR_FPHB_WIDTH                       1
#define ENC_IMR_FPHB(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_IMR_FPHB_SHIFT))&ENC_IMR_FPHB_MASK)
#define ENC_IMR_FPHA_MASK                        0x80u
#define ENC_IMR_FPHA_SHIFT                       7
#define ENC_IMR_FPHA_WIDTH                       1
#define ENC_IMR_FPHA(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_IMR_FPHA_SHIFT))&ENC_IMR_FPHA_MASK)
/* TST Bit Fields */
#define ENC_TST_TEST_COUNT_MASK                  0xFFu
#define ENC_TST_TEST_COUNT_SHIFT                 0
#define ENC_TST_TEST_COUNT_WIDTH                 8
#define ENC_TST_TEST_COUNT(x)                    (((uint16_t)(((uint16_t)(x))<<ENC_TST_TEST_COUNT_SHIFT))&ENC_TST_TEST_COUNT_MASK)
#define ENC_TST_TEST_PERIOD_MASK                 0x1F00u
#define ENC_TST_TEST_PERIOD_SHIFT                8
#define ENC_TST_TEST_PERIOD_WIDTH                5
#define ENC_TST_TEST_PERIOD(x)                   (((uint16_t)(((uint16_t)(x))<<ENC_TST_TEST_PERIOD_SHIFT))&ENC_TST_TEST_PERIOD_MASK)
#define ENC_TST_QDN_MASK                         0x2000u
#define ENC_TST_QDN_SHIFT                        13
#define ENC_TST_QDN_WIDTH                        1
#define ENC_TST_QDN(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_TST_QDN_SHIFT))&ENC_TST_QDN_MASK)
#define ENC_TST_TCE_MASK                         0x4000u
#define ENC_TST_TCE_SHIFT                        14
#define ENC_TST_TCE_WIDTH                        1
#define ENC_TST_TCE(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_TST_TCE_SHIFT))&ENC_TST_TCE_MASK)
#define ENC_TST_TEN_MASK                         0x8000u
#define ENC_TST_TEN_SHIFT                        15
#define ENC_TST_TEN_WIDTH                        1
#define ENC_TST_TEN(x)                           (((uint16_t)(((uint16_t)(x))<<ENC_TST_TEN_SHIFT))&ENC_TST_TEN_MASK)
/* CTRL2 Bit Fields */
#define ENC_CTRL2_UPDHLD_MASK                    0x1u
#define ENC_CTRL2_UPDHLD_SHIFT                   0
#define ENC_CTRL2_UPDHLD_WIDTH                   1
#define ENC_CTRL2_UPDHLD(x)                      (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_UPDHLD_SHIFT))&ENC_CTRL2_UPDHLD_MASK)
#define ENC_CTRL2_UPDPOS_MASK                    0x2u
#define ENC_CTRL2_UPDPOS_SHIFT                   1
#define ENC_CTRL2_UPDPOS_WIDTH                   1
#define ENC_CTRL2_UPDPOS(x)                      (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_UPDPOS_SHIFT))&ENC_CTRL2_UPDPOS_MASK)
#define ENC_CTRL2_MOD_MASK                       0x4u
#define ENC_CTRL2_MOD_SHIFT                      2
#define ENC_CTRL2_MOD_WIDTH                      1
#define ENC_CTRL2_MOD(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_MOD_SHIFT))&ENC_CTRL2_MOD_MASK)
#define ENC_CTRL2_DIR_MASK                       0x8u
#define ENC_CTRL2_DIR_SHIFT                      3
#define ENC_CTRL2_DIR_WIDTH                      1
#define ENC_CTRL2_DIR(x)                         (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_DIR_SHIFT))&ENC_CTRL2_DIR_MASK)
#define ENC_CTRL2_RUIE_MASK                      0x10u
#define ENC_CTRL2_RUIE_SHIFT                     4
#define ENC_CTRL2_RUIE_WIDTH                     1
#define ENC_CTRL2_RUIE(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_RUIE_SHIFT))&ENC_CTRL2_RUIE_MASK)
#define ENC_CTRL2_RUIRQ_MASK                     0x20u
#define ENC_CTRL2_RUIRQ_SHIFT                    5
#define ENC_CTRL2_RUIRQ_WIDTH                    1
#define ENC_CTRL2_RUIRQ(x)                       (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_RUIRQ_SHIFT))&ENC_CTRL2_RUIRQ_MASK)
#define ENC_CTRL2_ROIE_MASK                      0x40u
#define ENC_CTRL2_ROIE_SHIFT                     6
#define ENC_CTRL2_ROIE_WIDTH                     1
#define ENC_CTRL2_ROIE(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_ROIE_SHIFT))&ENC_CTRL2_ROIE_MASK)
#define ENC_CTRL2_ROIRQ_MASK                     0x80u
#define ENC_CTRL2_ROIRQ_SHIFT                    7
#define ENC_CTRL2_ROIRQ_WIDTH                    1
#define ENC_CTRL2_ROIRQ(x)                       (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_ROIRQ_SHIFT))&ENC_CTRL2_ROIRQ_MASK)
#define ENC_CTRL2_REVMOD_MASK                    0x100u
#define ENC_CTRL2_REVMOD_SHIFT                   8
#define ENC_CTRL2_REVMOD_WIDTH                   1
#define ENC_CTRL2_REVMOD(x)                      (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_REVMOD_SHIFT))&ENC_CTRL2_REVMOD_MASK)
#define ENC_CTRL2_OUTCTL_MASK                    0x200u
#define ENC_CTRL2_OUTCTL_SHIFT                   9
#define ENC_CTRL2_OUTCTL_WIDTH                   1
#define ENC_CTRL2_OUTCTL(x)                      (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_OUTCTL_SHIFT))&ENC_CTRL2_OUTCTL_MASK)
#define ENC_CTRL2_SABIE_MASK                     0x400u
#define ENC_CTRL2_SABIE_SHIFT                    10
#define ENC_CTRL2_SABIE_WIDTH                    1
#define ENC_CTRL2_SABIE(x)                       (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_SABIE_SHIFT))&ENC_CTRL2_SABIE_MASK)
#define ENC_CTRL2_SABIRQ_MASK                    0x800u
#define ENC_CTRL2_SABIRQ_SHIFT                   11
#define ENC_CTRL2_SABIRQ_WIDTH                   1
#define ENC_CTRL2_SABIRQ(x)                      (((uint16_t)(((uint16_t)(x))<<ENC_CTRL2_SABIRQ_SHIFT))&ENC_CTRL2_SABIRQ_MASK)
/* UMOD Bit Fields */
#define ENC_UMOD_MOD_MASK                        0xFFFFu
#define ENC_UMOD_MOD_SHIFT                       0
#define ENC_UMOD_MOD_WIDTH                       16
#define ENC_UMOD_MOD(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_UMOD_MOD_SHIFT))&ENC_UMOD_MOD_MASK)
/* LMOD Bit Fields */
#define ENC_LMOD_MOD_MASK                        0xFFFFu
#define ENC_LMOD_MOD_SHIFT                       0
#define ENC_LMOD_MOD_WIDTH                       16
#define ENC_LMOD_MOD(x)                          (((uint16_t)(((uint16_t)(x))<<ENC_LMOD_MOD_SHIFT))&ENC_LMOD_MOD_MASK)
/* UCOMP Bit Fields */
#define ENC_UCOMP_COMP_MASK                      0xFFFFu
#define ENC_UCOMP_COMP_SHIFT                     0
#define ENC_UCOMP_COMP_WIDTH                     16
#define ENC_UCOMP_COMP(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_UCOMP_COMP_SHIFT))&ENC_UCOMP_COMP_MASK)
/* LCOMP Bit Fields */
#define ENC_LCOMP_COMP_MASK                      0xFFFFu
#define ENC_LCOMP_COMP_SHIFT                     0
#define ENC_LCOMP_COMP_WIDTH                     16
#define ENC_LCOMP_COMP(x)                        (((uint16_t)(((uint16_t)(x))<<ENC_LCOMP_COMP_SHIFT))&ENC_LCOMP_COMP_MASK)

/*!
 * @}
 */ /* end of group ENC_Register_Masks */


/* ENC - Peripheral instance base addresses */
/** Peripheral ENC0 base address */
#define ENC0_BASE                                (0x40055000u)
/** Peripheral ENC0 base pointer */
#define ENC0                                     ((ENC_Type *)ENC0_BASE)
#define ENC0_BASE_PTR                            (ENC0)
/** Array initializer of ENC peripheral base addresses */
#define ENC_BASE_ADDRS                           { ENC0_BASE }
/** Array initializer of ENC peripheral base pointers */
#define ENC_BASE_PTRS                            { ENC0 }

/* ----------------------------------------------------------------------------
   -- ENC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ENC_Register_Accessor_Macros ENC - Register accessor macros
 * @{
 */


/* ENC - Register instance definitions */
/* ENC0 */
#define ENC0_CTRL                                ENC_CTRL_REG(ENC0)
#define ENC0_FILT                                ENC_FILT_REG(ENC0)
#define ENC0_WTR                                 ENC_WTR_REG(ENC0)
#define ENC0_POSD                                ENC_POSD_REG(ENC0)
#define ENC0_POSDH                               ENC_POSDH_REG(ENC0)
#define ENC0_REV                                 ENC_REV_REG(ENC0)
#define ENC0_REVH                                ENC_REVH_REG(ENC0)
#define ENC0_UPOS                                ENC_UPOS_REG(ENC0)
#define ENC0_LPOS                                ENC_LPOS_REG(ENC0)
#define ENC0_UPOSH                               ENC_UPOSH_REG(ENC0)
#define ENC0_LPOSH                               ENC_LPOSH_REG(ENC0)
#define ENC0_UINIT                               ENC_UINIT_REG(ENC0)
#define ENC0_LINIT                               ENC_LINIT_REG(ENC0)
#define ENC0_IMR                                 ENC_IMR_REG(ENC0)
#define ENC0_TST                                 ENC_TST_REG(ENC0)
#define ENC0_CTRL2                               ENC_CTRL2_REG(ENC0)
#define ENC0_UMOD                                ENC_UMOD_REG(ENC0)
#define ENC0_LMOD                                ENC_LMOD_REG(ENC0)
#define ENC0_UCOMP                               ENC_UCOMP_REG(ENC0)
#define ENC0_LCOMP                               ENC_LCOMP_REG(ENC0)

/*!
 * @}
 */ /* end of group ENC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group ENC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct {
  __IO uint8_t CTRL;                               /**< Control Register, offset: 0x0 */
  __O  uint8_t SERV;                               /**< Service Register, offset: 0x1 */
  __IO uint8_t CMPL;                               /**< Compare Low Register, offset: 0x2 */
  __IO uint8_t CMPH;                               /**< Compare High Register, offset: 0x3 */
} EWM_Type, *EWM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register accessors */
#define EWM_CTRL_REG(base)                       ((base)->CTRL)
#define EWM_SERV_REG(base)                       ((base)->SERV)
#define EWM_CMPL_REG(base)                       ((base)->CMPL)
#define EWM_CMPH_REG(base)                       ((base)->CMPH)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define EWM_CTRL_EWMEN_MASK                      0x1u
#define EWM_CTRL_EWMEN_SHIFT                     0
#define EWM_CTRL_EWMEN_WIDTH                     1
#define EWM_CTRL_EWMEN(x)                        (((uint8_t)(((uint8_t)(x))<<EWM_CTRL_EWMEN_SHIFT))&EWM_CTRL_EWMEN_MASK)
#define EWM_CTRL_ASSIN_MASK                      0x2u
#define EWM_CTRL_ASSIN_SHIFT                     1
#define EWM_CTRL_ASSIN_WIDTH                     1
#define EWM_CTRL_ASSIN(x)                        (((uint8_t)(((uint8_t)(x))<<EWM_CTRL_ASSIN_SHIFT))&EWM_CTRL_ASSIN_MASK)
#define EWM_CTRL_INEN_MASK                       0x4u
#define EWM_CTRL_INEN_SHIFT                      2
#define EWM_CTRL_INEN_WIDTH                      1
#define EWM_CTRL_INEN(x)                         (((uint8_t)(((uint8_t)(x))<<EWM_CTRL_INEN_SHIFT))&EWM_CTRL_INEN_MASK)
#define EWM_CTRL_INTEN_MASK                      0x8u
#define EWM_CTRL_INTEN_SHIFT                     3
#define EWM_CTRL_INTEN_WIDTH                     1
#define EWM_CTRL_INTEN(x)                        (((uint8_t)(((uint8_t)(x))<<EWM_CTRL_INTEN_SHIFT))&EWM_CTRL_INTEN_MASK)
/* SERV Bit Fields */
#define EWM_SERV_SERVICE_MASK                    0xFFu
#define EWM_SERV_SERVICE_SHIFT                   0
#define EWM_SERV_SERVICE_WIDTH                   8
#define EWM_SERV_SERVICE(x)                      (((uint8_t)(((uint8_t)(x))<<EWM_SERV_SERVICE_SHIFT))&EWM_SERV_SERVICE_MASK)
/* CMPL Bit Fields */
#define EWM_CMPL_COMPAREL_MASK                   0xFFu
#define EWM_CMPL_COMPAREL_SHIFT                  0
#define EWM_CMPL_COMPAREL_WIDTH                  8
#define EWM_CMPL_COMPAREL(x)                     (((uint8_t)(((uint8_t)(x))<<EWM_CMPL_COMPAREL_SHIFT))&EWM_CMPL_COMPAREL_MASK)
/* CMPH Bit Fields */
#define EWM_CMPH_COMPAREH_MASK                   0xFFu
#define EWM_CMPH_COMPAREH_SHIFT                  0
#define EWM_CMPH_COMPAREH_WIDTH                  8
#define EWM_CMPH_COMPAREH(x)                     (((uint8_t)(((uint8_t)(x))<<EWM_CMPH_COMPAREH_SHIFT))&EWM_CMPH_COMPAREH_MASK)

/*!
 * @}
 */ /* end of group EWM_Register_Masks */


/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE                                 (0x40061000u)
/** Peripheral EWM base pointer */
#define EWM                                      ((EWM_Type *)EWM_BASE)
#define EWM_BASE_PTR                             (EWM)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS                           { EWM_BASE }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM }

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register instance definitions */
/* EWM */
#define EWM_CTRL                                 EWM_CTRL_REG(EWM)
#define EWM_SERV                                 EWM_SERV_REG(EWM)
#define EWM_CMPL                                 EWM_CMPL_REG(EWM)
#define EWM_CMPH                                 EWM_CMPH_REG(EWM)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Peripheral_Access_Layer FMC Peripheral Access Layer
 * @{
 */

/** FMC - Register Layout Typedef */
typedef struct {
  __IO uint32_t PFAPR;                             /**< Flash Access Protection Register, offset: 0x0 */
  __IO uint32_t PFB0CR;                            /**< Flash Bank 0 Control Register, offset: 0x4 */
       uint8_t RESERVED_0[248];
  __IO uint32_t TAGVDW0S[2];                       /**< Cache Tag Storage, array offset: 0x100, array step: 0x4 */
  __IO uint32_t TAGVDW1S[2];                       /**< Cache Tag Storage, array offset: 0x108, array step: 0x4 */
  __IO uint32_t TAGVDW2S[2];                       /**< Cache Tag Storage, array offset: 0x110, array step: 0x4 */
  __IO uint32_t TAGVDW3S[2];                       /**< Cache Tag Storage, array offset: 0x118, array step: 0x4 */
       uint8_t RESERVED_1[224];
  struct {                                         /* offset: 0x200, array step: index*0x20, index2*0x10 */
    __IO uint32_t DATA_UM;                           /**< Cache Data Storage (uppermost word), array offset: 0x200, array step: index*0x20, index2*0x10 */
    __IO uint32_t DATA_MU;                           /**< Cache Data Storage (mid-upper word), array offset: 0x204, array step: index*0x20, index2*0x10 */
    __IO uint32_t DATA_ML;                           /**< Cache Data Storage (mid-lower word), array offset: 0x208, array step: index*0x20, index2*0x10 */
    __IO uint32_t DATA_LM;                           /**< Cache Data Storage (lowermost word), array offset: 0x20C, array step: index*0x20, index2*0x10 */
  } SET[4][2];
} FMC_Type, *FMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register accessors */
#define FMC_PFAPR_REG(base)                      ((base)->PFAPR)
#define FMC_PFB0CR_REG(base)                     ((base)->PFB0CR)
#define FMC_TAGVDW0S_REG(base,index)             ((base)->TAGVDW0S[index])
#define FMC_TAGVDW0S_COUNT                       2
#define FMC_TAGVDW1S_REG(base,index)             ((base)->TAGVDW1S[index])
#define FMC_TAGVDW1S_COUNT                       2
#define FMC_TAGVDW2S_REG(base,index)             ((base)->TAGVDW2S[index])
#define FMC_TAGVDW2S_COUNT                       2
#define FMC_TAGVDW3S_REG(base,index)             ((base)->TAGVDW3S[index])
#define FMC_TAGVDW3S_COUNT                       2
#define FMC_DATA_UM_REG(base,index,index2)       ((base)->SET[index][index2].DATA_UM)
#define FMC_DATA_UM_COUNT                        4
#define FMC_DATA_UM_COUNT2                       2
#define FMC_DATA_MU_REG(base,index,index2)       ((base)->SET[index][index2].DATA_MU)
#define FMC_DATA_MU_COUNT                        4
#define FMC_DATA_MU_COUNT2                       2
#define FMC_DATA_ML_REG(base,index,index2)       ((base)->SET[index][index2].DATA_ML)
#define FMC_DATA_ML_COUNT                        4
#define FMC_DATA_ML_COUNT2                       2
#define FMC_DATA_LM_REG(base,index,index2)       ((base)->SET[index][index2].DATA_LM)
#define FMC_DATA_LM_COUNT                        4
#define FMC_DATA_LM_COUNT2                       2

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Masks FMC Register Masks
 * @{
 */

/* PFAPR Bit Fields */
#define FMC_PFAPR_M0AP_MASK                      0x3u
#define FMC_PFAPR_M0AP_SHIFT                     0
#define FMC_PFAPR_M0AP_WIDTH                     2
#define FMC_PFAPR_M0AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M0AP_SHIFT))&FMC_PFAPR_M0AP_MASK)
#define FMC_PFAPR_M1AP_MASK                      0xCu
#define FMC_PFAPR_M1AP_SHIFT                     2
#define FMC_PFAPR_M1AP_WIDTH                     2
#define FMC_PFAPR_M1AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M1AP_SHIFT))&FMC_PFAPR_M1AP_MASK)
#define FMC_PFAPR_M2AP_MASK                      0x30u
#define FMC_PFAPR_M2AP_SHIFT                     4
#define FMC_PFAPR_M2AP_WIDTH                     2
#define FMC_PFAPR_M2AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M2AP_SHIFT))&FMC_PFAPR_M2AP_MASK)
#define FMC_PFAPR_M0PFD_MASK                     0x10000u
#define FMC_PFAPR_M0PFD_SHIFT                    16
#define FMC_PFAPR_M0PFD_WIDTH                    1
#define FMC_PFAPR_M0PFD(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M0PFD_SHIFT))&FMC_PFAPR_M0PFD_MASK)
#define FMC_PFAPR_M1PFD_MASK                     0x20000u
#define FMC_PFAPR_M1PFD_SHIFT                    17
#define FMC_PFAPR_M1PFD_WIDTH                    1
#define FMC_PFAPR_M1PFD(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M1PFD_SHIFT))&FMC_PFAPR_M1PFD_MASK)
#define FMC_PFAPR_M2PFD_MASK                     0x40000u
#define FMC_PFAPR_M2PFD_SHIFT                    18
#define FMC_PFAPR_M2PFD_WIDTH                    1
#define FMC_PFAPR_M2PFD(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M2PFD_SHIFT))&FMC_PFAPR_M2PFD_MASK)
/* PFB0CR Bit Fields */
#define FMC_PFB0CR_B0SEBE_MASK                   0x1u
#define FMC_PFB0CR_B0SEBE_SHIFT                  0
#define FMC_PFB0CR_B0SEBE_WIDTH                  1
#define FMC_PFB0CR_B0SEBE(x)                     (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0SEBE_SHIFT))&FMC_PFB0CR_B0SEBE_MASK)
#define FMC_PFB0CR_B0IPE_MASK                    0x2u
#define FMC_PFB0CR_B0IPE_SHIFT                   1
#define FMC_PFB0CR_B0IPE_WIDTH                   1
#define FMC_PFB0CR_B0IPE(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0IPE_SHIFT))&FMC_PFB0CR_B0IPE_MASK)
#define FMC_PFB0CR_B0DPE_MASK                    0x4u
#define FMC_PFB0CR_B0DPE_SHIFT                   2
#define FMC_PFB0CR_B0DPE_WIDTH                   1
#define FMC_PFB0CR_B0DPE(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0DPE_SHIFT))&FMC_PFB0CR_B0DPE_MASK)
#define FMC_PFB0CR_B0ICE_MASK                    0x8u
#define FMC_PFB0CR_B0ICE_SHIFT                   3
#define FMC_PFB0CR_B0ICE_WIDTH                   1
#define FMC_PFB0CR_B0ICE(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0ICE_SHIFT))&FMC_PFB0CR_B0ICE_MASK)
#define FMC_PFB0CR_B0DCE_MASK                    0x10u
#define FMC_PFB0CR_B0DCE_SHIFT                   4
#define FMC_PFB0CR_B0DCE_WIDTH                   1
#define FMC_PFB0CR_B0DCE(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0DCE_SHIFT))&FMC_PFB0CR_B0DCE_MASK)
#define FMC_PFB0CR_CRC_MASK                      0xE0u
#define FMC_PFB0CR_CRC_SHIFT                     5
#define FMC_PFB0CR_CRC_WIDTH                     3
#define FMC_PFB0CR_CRC(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CRC_SHIFT))&FMC_PFB0CR_CRC_MASK)
#define FMC_PFB0CR_B0MW_MASK                     0x60000u
#define FMC_PFB0CR_B0MW_SHIFT                    17
#define FMC_PFB0CR_B0MW_WIDTH                    2
#define FMC_PFB0CR_B0MW(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0MW_SHIFT))&FMC_PFB0CR_B0MW_MASK)
#define FMC_PFB0CR_S_B_INV_MASK                  0x80000u
#define FMC_PFB0CR_S_B_INV_SHIFT                 19
#define FMC_PFB0CR_S_B_INV_WIDTH                 1
#define FMC_PFB0CR_S_B_INV(x)                    (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_S_B_INV_SHIFT))&FMC_PFB0CR_S_B_INV_MASK)
#define FMC_PFB0CR_CINV_WAY_MASK                 0xF00000u
#define FMC_PFB0CR_CINV_WAY_SHIFT                20
#define FMC_PFB0CR_CINV_WAY_WIDTH                4
#define FMC_PFB0CR_CINV_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CINV_WAY_SHIFT))&FMC_PFB0CR_CINV_WAY_MASK)
#define FMC_PFB0CR_CLCK_WAY_MASK                 0xF000000u
#define FMC_PFB0CR_CLCK_WAY_SHIFT                24
#define FMC_PFB0CR_CLCK_WAY_WIDTH                4
#define FMC_PFB0CR_CLCK_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CLCK_WAY_SHIFT))&FMC_PFB0CR_CLCK_WAY_MASK)
#define FMC_PFB0CR_B0RWSC_MASK                   0xF0000000u
#define FMC_PFB0CR_B0RWSC_SHIFT                  28
#define FMC_PFB0CR_B0RWSC_WIDTH                  4
#define FMC_PFB0CR_B0RWSC(x)                     (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0RWSC_SHIFT))&FMC_PFB0CR_B0RWSC_MASK)
/* TAGVDW0S Bit Fields */
#define FMC_TAGVDW0S_valid_MASK                  0x1u
#define FMC_TAGVDW0S_valid_SHIFT                 0
#define FMC_TAGVDW0S_valid_WIDTH                 1
#define FMC_TAGVDW0S_valid(x)                    (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW0S_valid_SHIFT))&FMC_TAGVDW0S_valid_MASK)
#define FMC_TAGVDW0S_cache_tag_MASK              0xFFFE0u
#define FMC_TAGVDW0S_cache_tag_SHIFT             5
#define FMC_TAGVDW0S_cache_tag_WIDTH             15
#define FMC_TAGVDW0S_cache_tag(x)                (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW0S_cache_tag_SHIFT))&FMC_TAGVDW0S_cache_tag_MASK)
/* TAGVDW1S Bit Fields */
#define FMC_TAGVDW1S_valid_MASK                  0x1u
#define FMC_TAGVDW1S_valid_SHIFT                 0
#define FMC_TAGVDW1S_valid_WIDTH                 1
#define FMC_TAGVDW1S_valid(x)                    (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW1S_valid_SHIFT))&FMC_TAGVDW1S_valid_MASK)
#define FMC_TAGVDW1S_cache_tag_MASK              0xFFFE0u
#define FMC_TAGVDW1S_cache_tag_SHIFT             5
#define FMC_TAGVDW1S_cache_tag_WIDTH             15
#define FMC_TAGVDW1S_cache_tag(x)                (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW1S_cache_tag_SHIFT))&FMC_TAGVDW1S_cache_tag_MASK)
/* TAGVDW2S Bit Fields */
#define FMC_TAGVDW2S_valid_MASK                  0x1u
#define FMC_TAGVDW2S_valid_SHIFT                 0
#define FMC_TAGVDW2S_valid_WIDTH                 1
#define FMC_TAGVDW2S_valid(x)                    (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW2S_valid_SHIFT))&FMC_TAGVDW2S_valid_MASK)
#define FMC_TAGVDW2S_cache_tag_MASK              0xFFFE0u
#define FMC_TAGVDW2S_cache_tag_SHIFT             5
#define FMC_TAGVDW2S_cache_tag_WIDTH             15
#define FMC_TAGVDW2S_cache_tag(x)                (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW2S_cache_tag_SHIFT))&FMC_TAGVDW2S_cache_tag_MASK)
/* TAGVDW3S Bit Fields */
#define FMC_TAGVDW3S_valid_MASK                  0x1u
#define FMC_TAGVDW3S_valid_SHIFT                 0
#define FMC_TAGVDW3S_valid_WIDTH                 1
#define FMC_TAGVDW3S_valid(x)                    (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW3S_valid_SHIFT))&FMC_TAGVDW3S_valid_MASK)
#define FMC_TAGVDW3S_cache_tag_MASK              0xFFFE0u
#define FMC_TAGVDW3S_cache_tag_SHIFT             5
#define FMC_TAGVDW3S_cache_tag_WIDTH             15
#define FMC_TAGVDW3S_cache_tag(x)                (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW3S_cache_tag_SHIFT))&FMC_TAGVDW3S_cache_tag_MASK)
/* DATA_UM Bit Fields */
#define FMC_DATA_UM_data_MASK                    0xFFFFFFFFu
#define FMC_DATA_UM_data_SHIFT                   0
#define FMC_DATA_UM_data_WIDTH                   32
#define FMC_DATA_UM_data(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_DATA_UM_data_SHIFT))&FMC_DATA_UM_data_MASK)
/* DATA_MU Bit Fields */
#define FMC_DATA_MU_data_MASK                    0xFFFFFFFFu
#define FMC_DATA_MU_data_SHIFT                   0
#define FMC_DATA_MU_data_WIDTH                   32
#define FMC_DATA_MU_data(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_DATA_MU_data_SHIFT))&FMC_DATA_MU_data_MASK)
/* DATA_ML Bit Fields */
#define FMC_DATA_ML_data_MASK                    0xFFFFFFFFu
#define FMC_DATA_ML_data_SHIFT                   0
#define FMC_DATA_ML_data_WIDTH                   32
#define FMC_DATA_ML_data(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_DATA_ML_data_SHIFT))&FMC_DATA_ML_data_MASK)
/* DATA_LM Bit Fields */
#define FMC_DATA_LM_data_MASK                    0xFFFFFFFFu
#define FMC_DATA_LM_data_SHIFT                   0
#define FMC_DATA_LM_data_WIDTH                   32
#define FMC_DATA_LM_data(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_DATA_LM_data_SHIFT))&FMC_DATA_LM_data_MASK)

/*!
 * @}
 */ /* end of group FMC_Register_Masks */


/* FMC - Peripheral instance base addresses */
/** Peripheral FMC base address */
#define FMC_BASE                                 (0x4001F000u)
/** Peripheral FMC base pointer */
#define FMC                                      ((FMC_Type *)FMC_BASE)
#define FMC_BASE_PTR                             (FMC)
/** Array initializer of FMC peripheral base addresses */
#define FMC_BASE_ADDRS                           { FMC_BASE }
/** Array initializer of FMC peripheral base pointers */
#define FMC_BASE_PTRS                            { FMC }

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register instance definitions */
/* FMC */
#define FMC_PFAPR                                FMC_PFAPR_REG(FMC)
#define FMC_PFB0CR                               FMC_PFB0CR_REG(FMC)
#define FMC_TAGVDW0S0                            FMC_TAGVDW0S_REG(FMC,0)
#define FMC_TAGVDW0S1                            FMC_TAGVDW0S_REG(FMC,1)
#define FMC_TAGVDW1S0                            FMC_TAGVDW1S_REG(FMC,0)
#define FMC_TAGVDW1S1                            FMC_TAGVDW1S_REG(FMC,1)
#define FMC_TAGVDW2S0                            FMC_TAGVDW2S_REG(FMC,0)
#define FMC_TAGVDW2S1                            FMC_TAGVDW2S_REG(FMC,1)
#define FMC_TAGVDW3S0                            FMC_TAGVDW3S_REG(FMC,0)
#define FMC_TAGVDW3S1                            FMC_TAGVDW3S_REG(FMC,1)
#define FMC_DATAW0S0UM                           FMC_DATA_UM_REG(FMC,0,0)
#define FMC_DATAW0S0MU                           FMC_DATA_MU_REG(FMC,0,0)
#define FMC_DATAW0S0ML                           FMC_DATA_ML_REG(FMC,0,0)
#define FMC_DATAW0S0LM                           FMC_DATA_LM_REG(FMC,0,0)
#define FMC_DATAW0S1UM                           FMC_DATA_UM_REG(FMC,0,1)
#define FMC_DATAW0S1MU                           FMC_DATA_MU_REG(FMC,0,1)
#define FMC_DATAW0S1ML                           FMC_DATA_ML_REG(FMC,0,1)
#define FMC_DATAW0S1LM                           FMC_DATA_LM_REG(FMC,0,1)
#define FMC_DATAW1S0UM                           FMC_DATA_UM_REG(FMC,1,0)
#define FMC_DATAW1S0MU                           FMC_DATA_MU_REG(FMC,1,0)
#define FMC_DATAW1S0ML                           FMC_DATA_ML_REG(FMC,1,0)
#define FMC_DATAW1S0LM                           FMC_DATA_LM_REG(FMC,1,0)
#define FMC_DATAW1S1UM                           FMC_DATA_UM_REG(FMC,1,1)
#define FMC_DATAW1S1MU                           FMC_DATA_MU_REG(FMC,1,1)
#define FMC_DATAW1S1ML                           FMC_DATA_ML_REG(FMC,1,1)
#define FMC_DATAW1S1LM                           FMC_DATA_LM_REG(FMC,1,1)
#define FMC_DATAW2S0UM                           FMC_DATA_UM_REG(FMC,2,0)
#define FMC_DATAW2S0MU                           FMC_DATA_MU_REG(FMC,2,0)
#define FMC_DATAW2S0ML                           FMC_DATA_ML_REG(FMC,2,0)
#define FMC_DATAW2S0LM                           FMC_DATA_LM_REG(FMC,2,0)
#define FMC_DATAW2S1UM                           FMC_DATA_UM_REG(FMC,2,1)
#define FMC_DATAW2S1MU                           FMC_DATA_MU_REG(FMC,2,1)
#define FMC_DATAW2S1ML                           FMC_DATA_ML_REG(FMC,2,1)
#define FMC_DATAW2S1LM                           FMC_DATA_LM_REG(FMC,2,1)
#define FMC_DATAW3S0UM                           FMC_DATA_UM_REG(FMC,3,0)
#define FMC_DATAW3S0MU                           FMC_DATA_MU_REG(FMC,3,0)
#define FMC_DATAW3S0ML                           FMC_DATA_ML_REG(FMC,3,0)
#define FMC_DATAW3S0LM                           FMC_DATA_LM_REG(FMC,3,0)
#define FMC_DATAW3S1UM                           FMC_DATA_UM_REG(FMC,3,1)
#define FMC_DATAW3S1MU                           FMC_DATA_MU_REG(FMC,3,1)
#define FMC_DATAW3S1ML                           FMC_DATA_ML_REG(FMC,3,1)
#define FMC_DATAW3S1LM                           FMC_DATA_LM_REG(FMC,3,1)

/* FMC - Register array accessors */
#define FMC_TAGVDW0S(index)                      FMC_TAGVDW0S_REG(FMC,index)
#define FMC_TAGVDW1S(index)                      FMC_TAGVDW1S_REG(FMC,index)
#define FMC_TAGVDW2S(index)                      FMC_TAGVDW2S_REG(FMC,index)
#define FMC_TAGVDW3S(index)                      FMC_TAGVDW3S_REG(FMC,index)
#define FMC_DATA_UM(index,index2)                FMC_DATA_UM_REG(FMC,index,index2)
#define FMC_DATA_MU(index,index2)                FMC_DATA_MU_REG(FMC,index,index2)
#define FMC_DATA_ML(index,index2)                FMC_DATA_ML_REG(FMC,index,index2)
#define FMC_DATA_LM(index,index2)                FMC_DATA_LM_REG(FMC,index,index2)

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Peripheral_Access_Layer FTFA Peripheral Access Layer
 * @{
 */

/** FTFA - Register Layout Typedef */
typedef struct {
  __IO uint8_t FSTAT;                              /**< Flash Status Register, offset: 0x0 */
  __IO uint8_t FCNFG;                              /**< Flash Configuration Register, offset: 0x1 */
  __I  uint8_t FSEC;                               /**< Flash Security Register, offset: 0x2 */
  __I  uint8_t FOPT;                               /**< Flash Option Register, offset: 0x3 */
  __IO uint8_t FCCOB3;                             /**< Flash Common Command Object Registers, offset: 0x4 */
  __IO uint8_t FCCOB2;                             /**< Flash Common Command Object Registers, offset: 0x5 */
  __IO uint8_t FCCOB1;                             /**< Flash Common Command Object Registers, offset: 0x6 */
  __IO uint8_t FCCOB0;                             /**< Flash Common Command Object Registers, offset: 0x7 */
  __IO uint8_t FCCOB7;                             /**< Flash Common Command Object Registers, offset: 0x8 */
  __IO uint8_t FCCOB6;                             /**< Flash Common Command Object Registers, offset: 0x9 */
  __IO uint8_t FCCOB5;                             /**< Flash Common Command Object Registers, offset: 0xA */
  __IO uint8_t FCCOB4;                             /**< Flash Common Command Object Registers, offset: 0xB */
  __IO uint8_t FCCOBB;                             /**< Flash Common Command Object Registers, offset: 0xC */
  __IO uint8_t FCCOBA;                             /**< Flash Common Command Object Registers, offset: 0xD */
  __IO uint8_t FCCOB9;                             /**< Flash Common Command Object Registers, offset: 0xE */
  __IO uint8_t FCCOB8;                             /**< Flash Common Command Object Registers, offset: 0xF */
  __IO uint8_t FPROT3;                             /**< Program Flash Protection Registers, offset: 0x10 */
  __IO uint8_t FPROT2;                             /**< Program Flash Protection Registers, offset: 0x11 */
  __IO uint8_t FPROT1;                             /**< Program Flash Protection Registers, offset: 0x12 */
  __IO uint8_t FPROT0;                             /**< Program Flash Protection Registers, offset: 0x13 */
       uint8_t RESERVED_0[4];
  __I  uint8_t XACCH3;                             /**< Execute-only Access Registers, offset: 0x18 */
  __I  uint8_t XACCH2;                             /**< Execute-only Access Registers, offset: 0x19 */
  __I  uint8_t XACCH1;                             /**< Execute-only Access Registers, offset: 0x1A */
  __I  uint8_t XACCH0;                             /**< Execute-only Access Registers, offset: 0x1B */
  __I  uint8_t XACCL3;                             /**< Execute-only Access Registers, offset: 0x1C */
  __I  uint8_t XACCL2;                             /**< Execute-only Access Registers, offset: 0x1D */
  __I  uint8_t XACCL1;                             /**< Execute-only Access Registers, offset: 0x1E */
  __I  uint8_t XACCL0;                             /**< Execute-only Access Registers, offset: 0x1F */
  __I  uint8_t SACCH3;                             /**< Supervisor-only Access Registers, offset: 0x20 */
  __I  uint8_t SACCH2;                             /**< Supervisor-only Access Registers, offset: 0x21 */
  __I  uint8_t SACCH1;                             /**< Supervisor-only Access Registers, offset: 0x22 */
  __I  uint8_t SACCH0;                             /**< Supervisor-only Access Registers, offset: 0x23 */
  __I  uint8_t SACCL3;                             /**< Supervisor-only Access Registers, offset: 0x24 */
  __I  uint8_t SACCL2;                             /**< Supervisor-only Access Registers, offset: 0x25 */
  __I  uint8_t SACCL1;                             /**< Supervisor-only Access Registers, offset: 0x26 */
  __I  uint8_t SACCL0;                             /**< Supervisor-only Access Registers, offset: 0x27 */
  __I  uint8_t FACSS;                              /**< Flash Access Segment Size Register, offset: 0x28 */
       uint8_t RESERVED_1[2];
  __I  uint8_t FACSN;                              /**< Flash Access Segment Number Register, offset: 0x2B */
} FTFA_Type, *FTFA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register accessors */
#define FTFA_FSTAT_REG(base)                     ((base)->FSTAT)
#define FTFA_FCNFG_REG(base)                     ((base)->FCNFG)
#define FTFA_FSEC_REG(base)                      ((base)->FSEC)
#define FTFA_FOPT_REG(base)                      ((base)->FOPT)
#define FTFA_FCCOB3_REG(base)                    ((base)->FCCOB3)
#define FTFA_FCCOB2_REG(base)                    ((base)->FCCOB2)
#define FTFA_FCCOB1_REG(base)                    ((base)->FCCOB1)
#define FTFA_FCCOB0_REG(base)                    ((base)->FCCOB0)
#define FTFA_FCCOB7_REG(base)                    ((base)->FCCOB7)
#define FTFA_FCCOB6_REG(base)                    ((base)->FCCOB6)
#define FTFA_FCCOB5_REG(base)                    ((base)->FCCOB5)
#define FTFA_FCCOB4_REG(base)                    ((base)->FCCOB4)
#define FTFA_FCCOBB_REG(base)                    ((base)->FCCOBB)
#define FTFA_FCCOBA_REG(base)                    ((base)->FCCOBA)
#define FTFA_FCCOB9_REG(base)                    ((base)->FCCOB9)
#define FTFA_FCCOB8_REG(base)                    ((base)->FCCOB8)
#define FTFA_FPROT3_REG(base)                    ((base)->FPROT3)
#define FTFA_FPROT2_REG(base)                    ((base)->FPROT2)
#define FTFA_FPROT1_REG(base)                    ((base)->FPROT1)
#define FTFA_FPROT0_REG(base)                    ((base)->FPROT0)
#define FTFA_XACCH3_REG(base)                    ((base)->XACCH3)
#define FTFA_XACCH2_REG(base)                    ((base)->XACCH2)
#define FTFA_XACCH1_REG(base)                    ((base)->XACCH1)
#define FTFA_XACCH0_REG(base)                    ((base)->XACCH0)
#define FTFA_XACCL3_REG(base)                    ((base)->XACCL3)
#define FTFA_XACCL2_REG(base)                    ((base)->XACCL2)
#define FTFA_XACCL1_REG(base)                    ((base)->XACCL1)
#define FTFA_XACCL0_REG(base)                    ((base)->XACCL0)
#define FTFA_SACCH3_REG(base)                    ((base)->SACCH3)
#define FTFA_SACCH2_REG(base)                    ((base)->SACCH2)
#define FTFA_SACCH1_REG(base)                    ((base)->SACCH1)
#define FTFA_SACCH0_REG(base)                    ((base)->SACCH0)
#define FTFA_SACCL3_REG(base)                    ((base)->SACCL3)
#define FTFA_SACCL2_REG(base)                    ((base)->SACCL2)
#define FTFA_SACCL1_REG(base)                    ((base)->SACCL1)
#define FTFA_SACCL0_REG(base)                    ((base)->SACCL0)
#define FTFA_FACSS_REG(base)                     ((base)->FACSS)
#define FTFA_FACSN_REG(base)                     ((base)->FACSN)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTFA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Masks FTFA Register Masks
 * @{
 */

/* FSTAT Bit Fields */
#define FTFA_FSTAT_MGSTAT0_MASK                  0x1u
#define FTFA_FSTAT_MGSTAT0_SHIFT                 0
#define FTFA_FSTAT_MGSTAT0_WIDTH                 1
#define FTFA_FSTAT_MGSTAT0(x)                    (((uint8_t)(((uint8_t)(x))<<FTFA_FSTAT_MGSTAT0_SHIFT))&FTFA_FSTAT_MGSTAT0_MASK)
#define FTFA_FSTAT_FPVIOL_MASK                   0x10u
#define FTFA_FSTAT_FPVIOL_SHIFT                  4
#define FTFA_FSTAT_FPVIOL_WIDTH                  1
#define FTFA_FSTAT_FPVIOL(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FSTAT_FPVIOL_SHIFT))&FTFA_FSTAT_FPVIOL_MASK)
#define FTFA_FSTAT_ACCERR_MASK                   0x20u
#define FTFA_FSTAT_ACCERR_SHIFT                  5
#define FTFA_FSTAT_ACCERR_WIDTH                  1
#define FTFA_FSTAT_ACCERR(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FSTAT_ACCERR_SHIFT))&FTFA_FSTAT_ACCERR_MASK)
#define FTFA_FSTAT_RDCOLERR_MASK                 0x40u
#define FTFA_FSTAT_RDCOLERR_SHIFT                6
#define FTFA_FSTAT_RDCOLERR_WIDTH                1
#define FTFA_FSTAT_RDCOLERR(x)                   (((uint8_t)(((uint8_t)(x))<<FTFA_FSTAT_RDCOLERR_SHIFT))&FTFA_FSTAT_RDCOLERR_MASK)
#define FTFA_FSTAT_CCIF_MASK                     0x80u
#define FTFA_FSTAT_CCIF_SHIFT                    7
#define FTFA_FSTAT_CCIF_WIDTH                    1
#define FTFA_FSTAT_CCIF(x)                       (((uint8_t)(((uint8_t)(x))<<FTFA_FSTAT_CCIF_SHIFT))&FTFA_FSTAT_CCIF_MASK)
/* FCNFG Bit Fields */
#define FTFA_FCNFG_ERSSUSP_MASK                  0x10u
#define FTFA_FCNFG_ERSSUSP_SHIFT                 4
#define FTFA_FCNFG_ERSSUSP_WIDTH                 1
#define FTFA_FCNFG_ERSSUSP(x)                    (((uint8_t)(((uint8_t)(x))<<FTFA_FCNFG_ERSSUSP_SHIFT))&FTFA_FCNFG_ERSSUSP_MASK)
#define FTFA_FCNFG_ERSAREQ_MASK                  0x20u
#define FTFA_FCNFG_ERSAREQ_SHIFT                 5
#define FTFA_FCNFG_ERSAREQ_WIDTH                 1
#define FTFA_FCNFG_ERSAREQ(x)                    (((uint8_t)(((uint8_t)(x))<<FTFA_FCNFG_ERSAREQ_SHIFT))&FTFA_FCNFG_ERSAREQ_MASK)
#define FTFA_FCNFG_RDCOLLIE_MASK                 0x40u
#define FTFA_FCNFG_RDCOLLIE_SHIFT                6
#define FTFA_FCNFG_RDCOLLIE_WIDTH                1
#define FTFA_FCNFG_RDCOLLIE(x)                   (((uint8_t)(((uint8_t)(x))<<FTFA_FCNFG_RDCOLLIE_SHIFT))&FTFA_FCNFG_RDCOLLIE_MASK)
#define FTFA_FCNFG_CCIE_MASK                     0x80u
#define FTFA_FCNFG_CCIE_SHIFT                    7
#define FTFA_FCNFG_CCIE_WIDTH                    1
#define FTFA_FCNFG_CCIE(x)                       (((uint8_t)(((uint8_t)(x))<<FTFA_FCNFG_CCIE_SHIFT))&FTFA_FCNFG_CCIE_MASK)
/* FSEC Bit Fields */
#define FTFA_FSEC_SEC_MASK                       0x3u
#define FTFA_FSEC_SEC_SHIFT                      0
#define FTFA_FSEC_SEC_WIDTH                      2
#define FTFA_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_SEC_SHIFT))&FTFA_FSEC_SEC_MASK)
#define FTFA_FSEC_FSLACC_MASK                    0xCu
#define FTFA_FSEC_FSLACC_SHIFT                   2
#define FTFA_FSEC_FSLACC_WIDTH                   2
#define FTFA_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_FSLACC_SHIFT))&FTFA_FSEC_FSLACC_MASK)
#define FTFA_FSEC_MEEN_MASK                      0x30u
#define FTFA_FSEC_MEEN_SHIFT                     4
#define FTFA_FSEC_MEEN_WIDTH                     2
#define FTFA_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_MEEN_SHIFT))&FTFA_FSEC_MEEN_MASK)
#define FTFA_FSEC_KEYEN_MASK                     0xC0u
#define FTFA_FSEC_KEYEN_SHIFT                    6
#define FTFA_FSEC_KEYEN_WIDTH                    2
#define FTFA_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_KEYEN_SHIFT))&FTFA_FSEC_KEYEN_MASK)
/* FOPT Bit Fields */
#define FTFA_FOPT_OPT_MASK                       0xFFu
#define FTFA_FOPT_OPT_SHIFT                      0
#define FTFA_FOPT_OPT_WIDTH                      8
#define FTFA_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FOPT_OPT_SHIFT))&FTFA_FOPT_OPT_MASK)
/* FCCOB3 Bit Fields */
#define FTFA_FCCOB3_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB3_CCOBn_SHIFT                  0
#define FTFA_FCCOB3_CCOBn_WIDTH                  8
#define FTFA_FCCOB3_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB3_CCOBn_SHIFT))&FTFA_FCCOB3_CCOBn_MASK)
/* FCCOB2 Bit Fields */
#define FTFA_FCCOB2_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB2_CCOBn_SHIFT                  0
#define FTFA_FCCOB2_CCOBn_WIDTH                  8
#define FTFA_FCCOB2_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB2_CCOBn_SHIFT))&FTFA_FCCOB2_CCOBn_MASK)
/* FCCOB1 Bit Fields */
#define FTFA_FCCOB1_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB1_CCOBn_SHIFT                  0
#define FTFA_FCCOB1_CCOBn_WIDTH                  8
#define FTFA_FCCOB1_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB1_CCOBn_SHIFT))&FTFA_FCCOB1_CCOBn_MASK)
/* FCCOB0 Bit Fields */
#define FTFA_FCCOB0_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB0_CCOBn_SHIFT                  0
#define FTFA_FCCOB0_CCOBn_WIDTH                  8
#define FTFA_FCCOB0_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB0_CCOBn_SHIFT))&FTFA_FCCOB0_CCOBn_MASK)
/* FCCOB7 Bit Fields */
#define FTFA_FCCOB7_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB7_CCOBn_SHIFT                  0
#define FTFA_FCCOB7_CCOBn_WIDTH                  8
#define FTFA_FCCOB7_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB7_CCOBn_SHIFT))&FTFA_FCCOB7_CCOBn_MASK)
/* FCCOB6 Bit Fields */
#define FTFA_FCCOB6_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB6_CCOBn_SHIFT                  0
#define FTFA_FCCOB6_CCOBn_WIDTH                  8
#define FTFA_FCCOB6_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB6_CCOBn_SHIFT))&FTFA_FCCOB6_CCOBn_MASK)
/* FCCOB5 Bit Fields */
#define FTFA_FCCOB5_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB5_CCOBn_SHIFT                  0
#define FTFA_FCCOB5_CCOBn_WIDTH                  8
#define FTFA_FCCOB5_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB5_CCOBn_SHIFT))&FTFA_FCCOB5_CCOBn_MASK)
/* FCCOB4 Bit Fields */
#define FTFA_FCCOB4_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB4_CCOBn_SHIFT                  0
#define FTFA_FCCOB4_CCOBn_WIDTH                  8
#define FTFA_FCCOB4_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB4_CCOBn_SHIFT))&FTFA_FCCOB4_CCOBn_MASK)
/* FCCOBB Bit Fields */
#define FTFA_FCCOBB_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBB_CCOBn_SHIFT                  0
#define FTFA_FCCOBB_CCOBn_WIDTH                  8
#define FTFA_FCCOBB_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBB_CCOBn_SHIFT))&FTFA_FCCOBB_CCOBn_MASK)
/* FCCOBA Bit Fields */
#define FTFA_FCCOBA_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBA_CCOBn_SHIFT                  0
#define FTFA_FCCOBA_CCOBn_WIDTH                  8
#define FTFA_FCCOBA_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBA_CCOBn_SHIFT))&FTFA_FCCOBA_CCOBn_MASK)
/* FCCOB9 Bit Fields */
#define FTFA_FCCOB9_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB9_CCOBn_SHIFT                  0
#define FTFA_FCCOB9_CCOBn_WIDTH                  8
#define FTFA_FCCOB9_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB9_CCOBn_SHIFT))&FTFA_FCCOB9_CCOBn_MASK)
/* FCCOB8 Bit Fields */
#define FTFA_FCCOB8_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB8_CCOBn_SHIFT                  0
#define FTFA_FCCOB8_CCOBn_WIDTH                  8
#define FTFA_FCCOB8_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB8_CCOBn_SHIFT))&FTFA_FCCOB8_CCOBn_MASK)
/* FPROT3 Bit Fields */
#define FTFA_FPROT3_PROT_MASK                    0xFFu
#define FTFA_FPROT3_PROT_SHIFT                   0
#define FTFA_FPROT3_PROT_WIDTH                   8
#define FTFA_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT3_PROT_SHIFT))&FTFA_FPROT3_PROT_MASK)
/* FPROT2 Bit Fields */
#define FTFA_FPROT2_PROT_MASK                    0xFFu
#define FTFA_FPROT2_PROT_SHIFT                   0
#define FTFA_FPROT2_PROT_WIDTH                   8
#define FTFA_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT2_PROT_SHIFT))&FTFA_FPROT2_PROT_MASK)
/* FPROT1 Bit Fields */
#define FTFA_FPROT1_PROT_MASK                    0xFFu
#define FTFA_FPROT1_PROT_SHIFT                   0
#define FTFA_FPROT1_PROT_WIDTH                   8
#define FTFA_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT1_PROT_SHIFT))&FTFA_FPROT1_PROT_MASK)
/* FPROT0 Bit Fields */
#define FTFA_FPROT0_PROT_MASK                    0xFFu
#define FTFA_FPROT0_PROT_SHIFT                   0
#define FTFA_FPROT0_PROT_WIDTH                   8
#define FTFA_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT0_PROT_SHIFT))&FTFA_FPROT0_PROT_MASK)
/* XACCH3 Bit Fields */
#define FTFA_XACCH3_XA_MASK                      0xFFu
#define FTFA_XACCH3_XA_SHIFT                     0
#define FTFA_XACCH3_XA_WIDTH                     8
#define FTFA_XACCH3_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH3_XA_SHIFT))&FTFA_XACCH3_XA_MASK)
/* XACCH2 Bit Fields */
#define FTFA_XACCH2_XA_MASK                      0xFFu
#define FTFA_XACCH2_XA_SHIFT                     0
#define FTFA_XACCH2_XA_WIDTH                     8
#define FTFA_XACCH2_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH2_XA_SHIFT))&FTFA_XACCH2_XA_MASK)
/* XACCH1 Bit Fields */
#define FTFA_XACCH1_XA_MASK                      0xFFu
#define FTFA_XACCH1_XA_SHIFT                     0
#define FTFA_XACCH1_XA_WIDTH                     8
#define FTFA_XACCH1_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH1_XA_SHIFT))&FTFA_XACCH1_XA_MASK)
/* XACCH0 Bit Fields */
#define FTFA_XACCH0_XA_MASK                      0xFFu
#define FTFA_XACCH0_XA_SHIFT                     0
#define FTFA_XACCH0_XA_WIDTH                     8
#define FTFA_XACCH0_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH0_XA_SHIFT))&FTFA_XACCH0_XA_MASK)
/* XACCL3 Bit Fields */
#define FTFA_XACCL3_XA_MASK                      0xFFu
#define FTFA_XACCL3_XA_SHIFT                     0
#define FTFA_XACCL3_XA_WIDTH                     8
#define FTFA_XACCL3_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL3_XA_SHIFT))&FTFA_XACCL3_XA_MASK)
/* XACCL2 Bit Fields */
#define FTFA_XACCL2_XA_MASK                      0xFFu
#define FTFA_XACCL2_XA_SHIFT                     0
#define FTFA_XACCL2_XA_WIDTH                     8
#define FTFA_XACCL2_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL2_XA_SHIFT))&FTFA_XACCL2_XA_MASK)
/* XACCL1 Bit Fields */
#define FTFA_XACCL1_XA_MASK                      0xFFu
#define FTFA_XACCL1_XA_SHIFT                     0
#define FTFA_XACCL1_XA_WIDTH                     8
#define FTFA_XACCL1_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL1_XA_SHIFT))&FTFA_XACCL1_XA_MASK)
/* XACCL0 Bit Fields */
#define FTFA_XACCL0_XA_MASK                      0xFFu
#define FTFA_XACCL0_XA_SHIFT                     0
#define FTFA_XACCL0_XA_WIDTH                     8
#define FTFA_XACCL0_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL0_XA_SHIFT))&FTFA_XACCL0_XA_MASK)
/* SACCH3 Bit Fields */
#define FTFA_SACCH3_SA_MASK                      0xFFu
#define FTFA_SACCH3_SA_SHIFT                     0
#define FTFA_SACCH3_SA_WIDTH                     8
#define FTFA_SACCH3_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH3_SA_SHIFT))&FTFA_SACCH3_SA_MASK)
/* SACCH2 Bit Fields */
#define FTFA_SACCH2_SA_MASK                      0xFFu
#define FTFA_SACCH2_SA_SHIFT                     0
#define FTFA_SACCH2_SA_WIDTH                     8
#define FTFA_SACCH2_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH2_SA_SHIFT))&FTFA_SACCH2_SA_MASK)
/* SACCH1 Bit Fields */
#define FTFA_SACCH1_SA_MASK                      0xFFu
#define FTFA_SACCH1_SA_SHIFT                     0
#define FTFA_SACCH1_SA_WIDTH                     8
#define FTFA_SACCH1_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH1_SA_SHIFT))&FTFA_SACCH1_SA_MASK)
/* SACCH0 Bit Fields */
#define FTFA_SACCH0_SA_MASK                      0xFFu
#define FTFA_SACCH0_SA_SHIFT                     0
#define FTFA_SACCH0_SA_WIDTH                     8
#define FTFA_SACCH0_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH0_SA_SHIFT))&FTFA_SACCH0_SA_MASK)
/* SACCL3 Bit Fields */
#define FTFA_SACCL3_SA_MASK                      0xFFu
#define FTFA_SACCL3_SA_SHIFT                     0
#define FTFA_SACCL3_SA_WIDTH                     8
#define FTFA_SACCL3_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL3_SA_SHIFT))&FTFA_SACCL3_SA_MASK)
/* SACCL2 Bit Fields */
#define FTFA_SACCL2_SA_MASK                      0xFFu
#define FTFA_SACCL2_SA_SHIFT                     0
#define FTFA_SACCL2_SA_WIDTH                     8
#define FTFA_SACCL2_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL2_SA_SHIFT))&FTFA_SACCL2_SA_MASK)
/* SACCL1 Bit Fields */
#define FTFA_SACCL1_SA_MASK                      0xFFu
#define FTFA_SACCL1_SA_SHIFT                     0
#define FTFA_SACCL1_SA_WIDTH                     8
#define FTFA_SACCL1_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL1_SA_SHIFT))&FTFA_SACCL1_SA_MASK)
/* SACCL0 Bit Fields */
#define FTFA_SACCL0_SA_MASK                      0xFFu
#define FTFA_SACCL0_SA_SHIFT                     0
#define FTFA_SACCL0_SA_WIDTH                     8
#define FTFA_SACCL0_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL0_SA_SHIFT))&FTFA_SACCL0_SA_MASK)
/* FACSS Bit Fields */
#define FTFA_FACSS_SGSIZE_MASK                   0xFFu
#define FTFA_FACSS_SGSIZE_SHIFT                  0
#define FTFA_FACSS_SGSIZE_WIDTH                  8
#define FTFA_FACSS_SGSIZE(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FACSS_SGSIZE_SHIFT))&FTFA_FACSS_SGSIZE_MASK)
/* FACSN Bit Fields */
#define FTFA_FACSN_NUMSG_MASK                    0xFFu
#define FTFA_FACSN_NUMSG_SHIFT                   0
#define FTFA_FACSN_NUMSG_WIDTH                   8
#define FTFA_FACSN_NUMSG(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FACSN_NUMSG_SHIFT))&FTFA_FACSN_NUMSG_MASK)

/*!
 * @}
 */ /* end of group FTFA_Register_Masks */


/* FTFA - Peripheral instance base addresses */
/** Peripheral FTFA base address */
#define FTFA_BASE                                (0x40020000u)
/** Peripheral FTFA base pointer */
#define FTFA                                     ((FTFA_Type *)FTFA_BASE)
#define FTFA_BASE_PTR                            (FTFA)
/** Array initializer of FTFA peripheral base addresses */
#define FTFA_BASE_ADDRS                          { FTFA_BASE }
/** Array initializer of FTFA peripheral base pointers */
#define FTFA_BASE_PTRS                           { FTFA }

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register instance definitions */
/* FTFA */
#define FTFA_FSTAT                               FTFA_FSTAT_REG(FTFA)
#define FTFA_FCNFG                               FTFA_FCNFG_REG(FTFA)
#define FTFA_FSEC                                FTFA_FSEC_REG(FTFA)
#define FTFA_FOPT                                FTFA_FOPT_REG(FTFA)
#define FTFA_FCCOB3                              FTFA_FCCOB3_REG(FTFA)
#define FTFA_FCCOB2                              FTFA_FCCOB2_REG(FTFA)
#define FTFA_FCCOB1                              FTFA_FCCOB1_REG(FTFA)
#define FTFA_FCCOB0                              FTFA_FCCOB0_REG(FTFA)
#define FTFA_FCCOB7                              FTFA_FCCOB7_REG(FTFA)
#define FTFA_FCCOB6                              FTFA_FCCOB6_REG(FTFA)
#define FTFA_FCCOB5                              FTFA_FCCOB5_REG(FTFA)
#define FTFA_FCCOB4                              FTFA_FCCOB4_REG(FTFA)
#define FTFA_FCCOBB                              FTFA_FCCOBB_REG(FTFA)
#define FTFA_FCCOBA                              FTFA_FCCOBA_REG(FTFA)
#define FTFA_FCCOB9                              FTFA_FCCOB9_REG(FTFA)
#define FTFA_FCCOB8                              FTFA_FCCOB8_REG(FTFA)
#define FTFA_FPROT3                              FTFA_FPROT3_REG(FTFA)
#define FTFA_FPROT2                              FTFA_FPROT2_REG(FTFA)
#define FTFA_FPROT1                              FTFA_FPROT1_REG(FTFA)
#define FTFA_FPROT0                              FTFA_FPROT0_REG(FTFA)
#define FTFA_XACCH3                              FTFA_XACCH3_REG(FTFA)
#define FTFA_XACCH2                              FTFA_XACCH2_REG(FTFA)
#define FTFA_XACCH1                              FTFA_XACCH1_REG(FTFA)
#define FTFA_XACCH0                              FTFA_XACCH0_REG(FTFA)
#define FTFA_XACCL3                              FTFA_XACCL3_REG(FTFA)
#define FTFA_XACCL2                              FTFA_XACCL2_REG(FTFA)
#define FTFA_XACCL1                              FTFA_XACCL1_REG(FTFA)
#define FTFA_XACCL0                              FTFA_XACCL0_REG(FTFA)
#define FTFA_SACCH3                              FTFA_SACCH3_REG(FTFA)
#define FTFA_SACCH2                              FTFA_SACCH2_REG(FTFA)
#define FTFA_SACCH1                              FTFA_SACCH1_REG(FTFA)
#define FTFA_SACCH0                              FTFA_SACCH0_REG(FTFA)
#define FTFA_SACCL3                              FTFA_SACCL3_REG(FTFA)
#define FTFA_SACCL2                              FTFA_SACCL2_REG(FTFA)
#define FTFA_SACCL1                              FTFA_SACCL1_REG(FTFA)
#define FTFA_SACCL0                              FTFA_SACCL0_REG(FTFA)
#define FTFA_FACSS                               FTFA_FACSS_REG(FTFA)
#define FTFA_FACSN                               FTFA_FACSN_REG(FTFA)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FTFA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status And Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[8];
  __IO uint32_t CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  __IO uint32_t STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  __IO uint32_t MODE;                              /**< Features Mode Selection, offset: 0x54 */
  __IO uint32_t SYNC;                              /**< Synchronization, offset: 0x58 */
  __IO uint32_t OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  __IO uint32_t OUTMASK;                           /**< Output Mask, offset: 0x60 */
  __IO uint32_t COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  __IO uint32_t DEADTIME;                          /**< Deadtime Insertion Control, offset: 0x68 */
  __IO uint32_t EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  __IO uint32_t POL;                               /**< Channels Polarity, offset: 0x70 */
  __IO uint32_t FMS;                               /**< Fault Mode Status, offset: 0x74 */
  __IO uint32_t FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  __IO uint32_t FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80 */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
  __IO uint32_t FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  __IO uint32_t SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  __IO uint32_t INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  __IO uint32_t SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  __IO uint32_t PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
} FTM_Type, *FTM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register accessors */
#define FTM_SC_REG(base)                         ((base)->SC)
#define FTM_CNT_REG(base)                        ((base)->CNT)
#define FTM_MOD_REG(base)                        ((base)->MOD)
#define FTM_CnSC_REG(base,index)                 ((base)->CONTROLS[index].CnSC)
#define FTM_CnSC_COUNT                           8
#define FTM_CnV_REG(base,index)                  ((base)->CONTROLS[index].CnV)
#define FTM_CnV_COUNT                            8
#define FTM_CNTIN_REG(base)                      ((base)->CNTIN)
#define FTM_STATUS_REG(base)                     ((base)->STATUS)
#define FTM_MODE_REG(base)                       ((base)->MODE)
#define FTM_SYNC_REG(base)                       ((base)->SYNC)
#define FTM_OUTINIT_REG(base)                    ((base)->OUTINIT)
#define FTM_OUTMASK_REG(base)                    ((base)->OUTMASK)
#define FTM_COMBINE_REG(base)                    ((base)->COMBINE)
#define FTM_DEADTIME_REG(base)                   ((base)->DEADTIME)
#define FTM_EXTTRIG_REG(base)                    ((base)->EXTTRIG)
#define FTM_POL_REG(base)                        ((base)->POL)
#define FTM_FMS_REG(base)                        ((base)->FMS)
#define FTM_FILTER_REG(base)                     ((base)->FILTER)
#define FTM_FLTCTRL_REG(base)                    ((base)->FLTCTRL)
#define FTM_QDCTRL_REG(base)                     ((base)->QDCTRL)
#define FTM_CONF_REG(base)                       ((base)->CONF)
#define FTM_FLTPOL_REG(base)                     ((base)->FLTPOL)
#define FTM_SYNCONF_REG(base)                    ((base)->SYNCONF)
#define FTM_INVCTRL_REG(base)                    ((base)->INVCTRL)
#define FTM_SWOCTRL_REG(base)                    ((base)->SWOCTRL)
#define FTM_PWMLOAD_REG(base)                    ((base)->PWMLOAD)

/*!
 * @}
 */ /* end of group FTM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/* SC Bit Fields */
#define FTM_SC_PS_MASK                           0x7u
#define FTM_SC_PS_SHIFT                          0
#define FTM_SC_PS_WIDTH                          3
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x))<<FTM_SC_PS_SHIFT))&FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         0x18u
#define FTM_SC_CLKS_SHIFT                        3
#define FTM_SC_CLKS_WIDTH                        2
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_SC_CLKS_SHIFT))&FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS_MASK                        0x20u
#define FTM_SC_CPWMS_SHIFT                       5
#define FTM_SC_CPWMS_WIDTH                       1
#define FTM_SC_CPWMS(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_SC_CPWMS_SHIFT))&FTM_SC_CPWMS_MASK)
#define FTM_SC_TOIE_MASK                         0x40u
#define FTM_SC_TOIE_SHIFT                        6
#define FTM_SC_TOIE_WIDTH                        1
#define FTM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_SC_TOIE_SHIFT))&FTM_SC_TOIE_MASK)
#define FTM_SC_TOF_MASK                          0x80u
#define FTM_SC_TOF_SHIFT                         7
#define FTM_SC_TOF_WIDTH                         1
#define FTM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x))<<FTM_SC_TOF_SHIFT))&FTM_SC_TOF_MASK)
/* CNT Bit Fields */
#define FTM_CNT_COUNT_MASK                       0xFFFFu
#define FTM_CNT_COUNT_SHIFT                      0
#define FTM_CNT_COUNT_WIDTH                      16
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CNT_COUNT_SHIFT))&FTM_CNT_COUNT_MASK)
/* MOD Bit Fields */
#define FTM_MOD_MOD_MASK                         0xFFFFu
#define FTM_MOD_MOD_SHIFT                        0
#define FTM_MOD_MOD_WIDTH                        16
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_MOD_MOD_SHIFT))&FTM_MOD_MOD_MASK)
/* CnSC Bit Fields */
#define FTM_CnSC_DMA_MASK                        0x1u
#define FTM_CnSC_DMA_SHIFT                       0
#define FTM_CnSC_DMA_WIDTH                       1
#define FTM_CnSC_DMA(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_DMA_SHIFT))&FTM_CnSC_DMA_MASK)
#define FTM_CnSC_ICRST_MASK                      0x2u
#define FTM_CnSC_ICRST_SHIFT                     1
#define FTM_CnSC_ICRST_WIDTH                     1
#define FTM_CnSC_ICRST(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_ICRST_SHIFT))&FTM_CnSC_ICRST_MASK)
#define FTM_CnSC_ELSA_MASK                       0x4u
#define FTM_CnSC_ELSA_SHIFT                      2
#define FTM_CnSC_ELSA_WIDTH                      1
#define FTM_CnSC_ELSA(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_ELSA_SHIFT))&FTM_CnSC_ELSA_MASK)
#define FTM_CnSC_ELSB_MASK                       0x8u
#define FTM_CnSC_ELSB_SHIFT                      3
#define FTM_CnSC_ELSB_WIDTH                      1
#define FTM_CnSC_ELSB(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_ELSB_SHIFT))&FTM_CnSC_ELSB_MASK)
#define FTM_CnSC_MSA_MASK                        0x10u
#define FTM_CnSC_MSA_SHIFT                       4
#define FTM_CnSC_MSA_WIDTH                       1
#define FTM_CnSC_MSA(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_MSA_SHIFT))&FTM_CnSC_MSA_MASK)
#define FTM_CnSC_MSB_MASK                        0x20u
#define FTM_CnSC_MSB_SHIFT                       5
#define FTM_CnSC_MSB_WIDTH                       1
#define FTM_CnSC_MSB(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_MSB_SHIFT))&FTM_CnSC_MSB_MASK)
#define FTM_CnSC_CHIE_MASK                       0x40u
#define FTM_CnSC_CHIE_SHIFT                      6
#define FTM_CnSC_CHIE_WIDTH                      1
#define FTM_CnSC_CHIE(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_CHIE_SHIFT))&FTM_CnSC_CHIE_MASK)
#define FTM_CnSC_CHF_MASK                        0x80u
#define FTM_CnSC_CHF_SHIFT                       7
#define FTM_CnSC_CHF_WIDTH                       1
#define FTM_CnSC_CHF(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_CnSC_CHF_SHIFT))&FTM_CnSC_CHF_MASK)
/* CnV Bit Fields */
#define FTM_CnV_VAL_MASK                         0xFFFFu
#define FTM_CnV_VAL_SHIFT                        0
#define FTM_CnV_VAL_WIDTH                        16
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_CnV_VAL_SHIFT))&FTM_CnV_VAL_MASK)
/* CNTIN Bit Fields */
#define FTM_CNTIN_INIT_MASK                      0xFFFFu
#define FTM_CNTIN_INIT_SHIFT                     0
#define FTM_CNTIN_INIT_WIDTH                     16
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_CNTIN_INIT_SHIFT))&FTM_CNTIN_INIT_MASK)
/* STATUS Bit Fields */
#define FTM_STATUS_CH0F_MASK                     0x1u
#define FTM_STATUS_CH0F_SHIFT                    0
#define FTM_STATUS_CH0F_WIDTH                    1
#define FTM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH0F_SHIFT))&FTM_STATUS_CH0F_MASK)
#define FTM_STATUS_CH1F_MASK                     0x2u
#define FTM_STATUS_CH1F_SHIFT                    1
#define FTM_STATUS_CH1F_WIDTH                    1
#define FTM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH1F_SHIFT))&FTM_STATUS_CH1F_MASK)
#define FTM_STATUS_CH2F_MASK                     0x4u
#define FTM_STATUS_CH2F_SHIFT                    2
#define FTM_STATUS_CH2F_WIDTH                    1
#define FTM_STATUS_CH2F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH2F_SHIFT))&FTM_STATUS_CH2F_MASK)
#define FTM_STATUS_CH3F_MASK                     0x8u
#define FTM_STATUS_CH3F_SHIFT                    3
#define FTM_STATUS_CH3F_WIDTH                    1
#define FTM_STATUS_CH3F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH3F_SHIFT))&FTM_STATUS_CH3F_MASK)
#define FTM_STATUS_CH4F_MASK                     0x10u
#define FTM_STATUS_CH4F_SHIFT                    4
#define FTM_STATUS_CH4F_WIDTH                    1
#define FTM_STATUS_CH4F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH4F_SHIFT))&FTM_STATUS_CH4F_MASK)
#define FTM_STATUS_CH5F_MASK                     0x20u
#define FTM_STATUS_CH5F_SHIFT                    5
#define FTM_STATUS_CH5F_WIDTH                    1
#define FTM_STATUS_CH5F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH5F_SHIFT))&FTM_STATUS_CH5F_MASK)
#define FTM_STATUS_CH6F_MASK                     0x40u
#define FTM_STATUS_CH6F_SHIFT                    6
#define FTM_STATUS_CH6F_WIDTH                    1
#define FTM_STATUS_CH6F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH6F_SHIFT))&FTM_STATUS_CH6F_MASK)
#define FTM_STATUS_CH7F_MASK                     0x80u
#define FTM_STATUS_CH7F_SHIFT                    7
#define FTM_STATUS_CH7F_WIDTH                    1
#define FTM_STATUS_CH7F(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_STATUS_CH7F_SHIFT))&FTM_STATUS_CH7F_MASK)
/* MODE Bit Fields */
#define FTM_MODE_FTMEN_MASK                      0x1u
#define FTM_MODE_FTMEN_SHIFT                     0
#define FTM_MODE_FTMEN_WIDTH                     1
#define FTM_MODE_FTMEN(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FTMEN_SHIFT))&FTM_MODE_FTMEN_MASK)
#define FTM_MODE_INIT_MASK                       0x2u
#define FTM_MODE_INIT_SHIFT                      1
#define FTM_MODE_INIT_WIDTH                      1
#define FTM_MODE_INIT(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_MODE_INIT_SHIFT))&FTM_MODE_INIT_MASK)
#define FTM_MODE_WPDIS_MASK                      0x4u
#define FTM_MODE_WPDIS_SHIFT                     2
#define FTM_MODE_WPDIS_WIDTH                     1
#define FTM_MODE_WPDIS(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_MODE_WPDIS_SHIFT))&FTM_MODE_WPDIS_MASK)
#define FTM_MODE_PWMSYNC_MASK                    0x8u
#define FTM_MODE_PWMSYNC_SHIFT                   3
#define FTM_MODE_PWMSYNC_WIDTH                   1
#define FTM_MODE_PWMSYNC(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_MODE_PWMSYNC_SHIFT))&FTM_MODE_PWMSYNC_MASK)
#define FTM_MODE_CAPTEST_MASK                    0x10u
#define FTM_MODE_CAPTEST_SHIFT                   4
#define FTM_MODE_CAPTEST_WIDTH                   1
#define FTM_MODE_CAPTEST(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_MODE_CAPTEST_SHIFT))&FTM_MODE_CAPTEST_MASK)
#define FTM_MODE_FAULTM_MASK                     0x60u
#define FTM_MODE_FAULTM_SHIFT                    5
#define FTM_MODE_FAULTM_WIDTH                    2
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FAULTM_SHIFT))&FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE_MASK                    0x80u
#define FTM_MODE_FAULTIE_SHIFT                   7
#define FTM_MODE_FAULTIE_WIDTH                   1
#define FTM_MODE_FAULTIE(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FAULTIE_SHIFT))&FTM_MODE_FAULTIE_MASK)
/* SYNC Bit Fields */
#define FTM_SYNC_CNTMIN_MASK                     0x1u
#define FTM_SYNC_CNTMIN_SHIFT                    0
#define FTM_SYNC_CNTMIN_WIDTH                    1
#define FTM_SYNC_CNTMIN(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_CNTMIN_SHIFT))&FTM_SYNC_CNTMIN_MASK)
#define FTM_SYNC_CNTMAX_MASK                     0x2u
#define FTM_SYNC_CNTMAX_SHIFT                    1
#define FTM_SYNC_CNTMAX_WIDTH                    1
#define FTM_SYNC_CNTMAX(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_CNTMAX_SHIFT))&FTM_SYNC_CNTMAX_MASK)
#define FTM_SYNC_REINIT_MASK                     0x4u
#define FTM_SYNC_REINIT_SHIFT                    2
#define FTM_SYNC_REINIT_WIDTH                    1
#define FTM_SYNC_REINIT(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_REINIT_SHIFT))&FTM_SYNC_REINIT_MASK)
#define FTM_SYNC_SYNCHOM_MASK                    0x8u
#define FTM_SYNC_SYNCHOM_SHIFT                   3
#define FTM_SYNC_SYNCHOM_WIDTH                   1
#define FTM_SYNC_SYNCHOM(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_SYNCHOM_SHIFT))&FTM_SYNC_SYNCHOM_MASK)
#define FTM_SYNC_TRIG0_MASK                      0x10u
#define FTM_SYNC_TRIG0_SHIFT                     4
#define FTM_SYNC_TRIG0_WIDTH                     1
#define FTM_SYNC_TRIG0(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_TRIG0_SHIFT))&FTM_SYNC_TRIG0_MASK)
#define FTM_SYNC_TRIG1_MASK                      0x20u
#define FTM_SYNC_TRIG1_SHIFT                     5
#define FTM_SYNC_TRIG1_WIDTH                     1
#define FTM_SYNC_TRIG1(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_TRIG1_SHIFT))&FTM_SYNC_TRIG1_MASK)
#define FTM_SYNC_TRIG2_MASK                      0x40u
#define FTM_SYNC_TRIG2_SHIFT                     6
#define FTM_SYNC_TRIG2_WIDTH                     1
#define FTM_SYNC_TRIG2(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_TRIG2_SHIFT))&FTM_SYNC_TRIG2_MASK)
#define FTM_SYNC_SWSYNC_MASK                     0x80u
#define FTM_SYNC_SWSYNC_SHIFT                    7
#define FTM_SYNC_SWSYNC_WIDTH                    1
#define FTM_SYNC_SWSYNC(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_SYNC_SWSYNC_SHIFT))&FTM_SYNC_SWSYNC_MASK)
/* OUTINIT Bit Fields */
#define FTM_OUTINIT_CH0OI_MASK                   0x1u
#define FTM_OUTINIT_CH0OI_SHIFT                  0
#define FTM_OUTINIT_CH0OI_WIDTH                  1
#define FTM_OUTINIT_CH0OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH0OI_SHIFT))&FTM_OUTINIT_CH0OI_MASK)
#define FTM_OUTINIT_CH1OI_MASK                   0x2u
#define FTM_OUTINIT_CH1OI_SHIFT                  1
#define FTM_OUTINIT_CH1OI_WIDTH                  1
#define FTM_OUTINIT_CH1OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH1OI_SHIFT))&FTM_OUTINIT_CH1OI_MASK)
#define FTM_OUTINIT_CH2OI_MASK                   0x4u
#define FTM_OUTINIT_CH2OI_SHIFT                  2
#define FTM_OUTINIT_CH2OI_WIDTH                  1
#define FTM_OUTINIT_CH2OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH2OI_SHIFT))&FTM_OUTINIT_CH2OI_MASK)
#define FTM_OUTINIT_CH3OI_MASK                   0x8u
#define FTM_OUTINIT_CH3OI_SHIFT                  3
#define FTM_OUTINIT_CH3OI_WIDTH                  1
#define FTM_OUTINIT_CH3OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH3OI_SHIFT))&FTM_OUTINIT_CH3OI_MASK)
#define FTM_OUTINIT_CH4OI_MASK                   0x10u
#define FTM_OUTINIT_CH4OI_SHIFT                  4
#define FTM_OUTINIT_CH4OI_WIDTH                  1
#define FTM_OUTINIT_CH4OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH4OI_SHIFT))&FTM_OUTINIT_CH4OI_MASK)
#define FTM_OUTINIT_CH5OI_MASK                   0x20u
#define FTM_OUTINIT_CH5OI_SHIFT                  5
#define FTM_OUTINIT_CH5OI_WIDTH                  1
#define FTM_OUTINIT_CH5OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH5OI_SHIFT))&FTM_OUTINIT_CH5OI_MASK)
#define FTM_OUTINIT_CH6OI_MASK                   0x40u
#define FTM_OUTINIT_CH6OI_SHIFT                  6
#define FTM_OUTINIT_CH6OI_WIDTH                  1
#define FTM_OUTINIT_CH6OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH6OI_SHIFT))&FTM_OUTINIT_CH6OI_MASK)
#define FTM_OUTINIT_CH7OI_MASK                   0x80u
#define FTM_OUTINIT_CH7OI_SHIFT                  7
#define FTM_OUTINIT_CH7OI_WIDTH                  1
#define FTM_OUTINIT_CH7OI(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTINIT_CH7OI_SHIFT))&FTM_OUTINIT_CH7OI_MASK)
/* OUTMASK Bit Fields */
#define FTM_OUTMASK_CH0OM_MASK                   0x1u
#define FTM_OUTMASK_CH0OM_SHIFT                  0
#define FTM_OUTMASK_CH0OM_WIDTH                  1
#define FTM_OUTMASK_CH0OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH0OM_SHIFT))&FTM_OUTMASK_CH0OM_MASK)
#define FTM_OUTMASK_CH1OM_MASK                   0x2u
#define FTM_OUTMASK_CH1OM_SHIFT                  1
#define FTM_OUTMASK_CH1OM_WIDTH                  1
#define FTM_OUTMASK_CH1OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH1OM_SHIFT))&FTM_OUTMASK_CH1OM_MASK)
#define FTM_OUTMASK_CH2OM_MASK                   0x4u
#define FTM_OUTMASK_CH2OM_SHIFT                  2
#define FTM_OUTMASK_CH2OM_WIDTH                  1
#define FTM_OUTMASK_CH2OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH2OM_SHIFT))&FTM_OUTMASK_CH2OM_MASK)
#define FTM_OUTMASK_CH3OM_MASK                   0x8u
#define FTM_OUTMASK_CH3OM_SHIFT                  3
#define FTM_OUTMASK_CH3OM_WIDTH                  1
#define FTM_OUTMASK_CH3OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH3OM_SHIFT))&FTM_OUTMASK_CH3OM_MASK)
#define FTM_OUTMASK_CH4OM_MASK                   0x10u
#define FTM_OUTMASK_CH4OM_SHIFT                  4
#define FTM_OUTMASK_CH4OM_WIDTH                  1
#define FTM_OUTMASK_CH4OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH4OM_SHIFT))&FTM_OUTMASK_CH4OM_MASK)
#define FTM_OUTMASK_CH5OM_MASK                   0x20u
#define FTM_OUTMASK_CH5OM_SHIFT                  5
#define FTM_OUTMASK_CH5OM_WIDTH                  1
#define FTM_OUTMASK_CH5OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH5OM_SHIFT))&FTM_OUTMASK_CH5OM_MASK)
#define FTM_OUTMASK_CH6OM_MASK                   0x40u
#define FTM_OUTMASK_CH6OM_SHIFT                  6
#define FTM_OUTMASK_CH6OM_WIDTH                  1
#define FTM_OUTMASK_CH6OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH6OM_SHIFT))&FTM_OUTMASK_CH6OM_MASK)
#define FTM_OUTMASK_CH7OM_MASK                   0x80u
#define FTM_OUTMASK_CH7OM_SHIFT                  7
#define FTM_OUTMASK_CH7OM_WIDTH                  1
#define FTM_OUTMASK_CH7OM(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_OUTMASK_CH7OM_SHIFT))&FTM_OUTMASK_CH7OM_MASK)
/* COMBINE Bit Fields */
#define FTM_COMBINE_COMBINE0_MASK                0x1u
#define FTM_COMBINE_COMBINE0_SHIFT               0
#define FTM_COMBINE_COMBINE0_WIDTH               1
#define FTM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMBINE0_SHIFT))&FTM_COMBINE_COMBINE0_MASK)
#define FTM_COMBINE_COMP0_MASK                   0x2u
#define FTM_COMBINE_COMP0_SHIFT                  1
#define FTM_COMBINE_COMP0_WIDTH                  1
#define FTM_COMBINE_COMP0(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMP0_SHIFT))&FTM_COMBINE_COMP0_MASK)
#define FTM_COMBINE_DECAPEN0_MASK                0x4u
#define FTM_COMBINE_DECAPEN0_SHIFT               2
#define FTM_COMBINE_DECAPEN0_WIDTH               1
#define FTM_COMBINE_DECAPEN0(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAPEN0_SHIFT))&FTM_COMBINE_DECAPEN0_MASK)
#define FTM_COMBINE_DECAP0_MASK                  0x8u
#define FTM_COMBINE_DECAP0_SHIFT                 3
#define FTM_COMBINE_DECAP0_WIDTH                 1
#define FTM_COMBINE_DECAP0(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAP0_SHIFT))&FTM_COMBINE_DECAP0_MASK)
#define FTM_COMBINE_DTEN0_MASK                   0x10u
#define FTM_COMBINE_DTEN0_SHIFT                  4
#define FTM_COMBINE_DTEN0_WIDTH                  1
#define FTM_COMBINE_DTEN0(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DTEN0_SHIFT))&FTM_COMBINE_DTEN0_MASK)
#define FTM_COMBINE_SYNCEN0_MASK                 0x20u
#define FTM_COMBINE_SYNCEN0_SHIFT                5
#define FTM_COMBINE_SYNCEN0_WIDTH                1
#define FTM_COMBINE_SYNCEN0(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_SYNCEN0_SHIFT))&FTM_COMBINE_SYNCEN0_MASK)
#define FTM_COMBINE_FAULTEN0_MASK                0x40u
#define FTM_COMBINE_FAULTEN0_SHIFT               6
#define FTM_COMBINE_FAULTEN0_WIDTH               1
#define FTM_COMBINE_FAULTEN0(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_FAULTEN0_SHIFT))&FTM_COMBINE_FAULTEN0_MASK)
#define FTM_COMBINE_COMBINE1_MASK                0x100u
#define FTM_COMBINE_COMBINE1_SHIFT               8
#define FTM_COMBINE_COMBINE1_WIDTH               1
#define FTM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMBINE1_SHIFT))&FTM_COMBINE_COMBINE1_MASK)
#define FTM_COMBINE_COMP1_MASK                   0x200u
#define FTM_COMBINE_COMP1_SHIFT                  9
#define FTM_COMBINE_COMP1_WIDTH                  1
#define FTM_COMBINE_COMP1(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMP1_SHIFT))&FTM_COMBINE_COMP1_MASK)
#define FTM_COMBINE_DECAPEN1_MASK                0x400u
#define FTM_COMBINE_DECAPEN1_SHIFT               10
#define FTM_COMBINE_DECAPEN1_WIDTH               1
#define FTM_COMBINE_DECAPEN1(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAPEN1_SHIFT))&FTM_COMBINE_DECAPEN1_MASK)
#define FTM_COMBINE_DECAP1_MASK                  0x800u
#define FTM_COMBINE_DECAP1_SHIFT                 11
#define FTM_COMBINE_DECAP1_WIDTH                 1
#define FTM_COMBINE_DECAP1(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAP1_SHIFT))&FTM_COMBINE_DECAP1_MASK)
#define FTM_COMBINE_DTEN1_MASK                   0x1000u
#define FTM_COMBINE_DTEN1_SHIFT                  12
#define FTM_COMBINE_DTEN1_WIDTH                  1
#define FTM_COMBINE_DTEN1(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DTEN1_SHIFT))&FTM_COMBINE_DTEN1_MASK)
#define FTM_COMBINE_SYNCEN1_MASK                 0x2000u
#define FTM_COMBINE_SYNCEN1_SHIFT                13
#define FTM_COMBINE_SYNCEN1_WIDTH                1
#define FTM_COMBINE_SYNCEN1(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_SYNCEN1_SHIFT))&FTM_COMBINE_SYNCEN1_MASK)
#define FTM_COMBINE_FAULTEN1_MASK                0x4000u
#define FTM_COMBINE_FAULTEN1_SHIFT               14
#define FTM_COMBINE_FAULTEN1_WIDTH               1
#define FTM_COMBINE_FAULTEN1(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_FAULTEN1_SHIFT))&FTM_COMBINE_FAULTEN1_MASK)
#define FTM_COMBINE_COMBINE2_MASK                0x10000u
#define FTM_COMBINE_COMBINE2_SHIFT               16
#define FTM_COMBINE_COMBINE2_WIDTH               1
#define FTM_COMBINE_COMBINE2(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMBINE2_SHIFT))&FTM_COMBINE_COMBINE2_MASK)
#define FTM_COMBINE_COMP2_MASK                   0x20000u
#define FTM_COMBINE_COMP2_SHIFT                  17
#define FTM_COMBINE_COMP2_WIDTH                  1
#define FTM_COMBINE_COMP2(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMP2_SHIFT))&FTM_COMBINE_COMP2_MASK)
#define FTM_COMBINE_DECAPEN2_MASK                0x40000u
#define FTM_COMBINE_DECAPEN2_SHIFT               18
#define FTM_COMBINE_DECAPEN2_WIDTH               1
#define FTM_COMBINE_DECAPEN2(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAPEN2_SHIFT))&FTM_COMBINE_DECAPEN2_MASK)
#define FTM_COMBINE_DECAP2_MASK                  0x80000u
#define FTM_COMBINE_DECAP2_SHIFT                 19
#define FTM_COMBINE_DECAP2_WIDTH                 1
#define FTM_COMBINE_DECAP2(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAP2_SHIFT))&FTM_COMBINE_DECAP2_MASK)
#define FTM_COMBINE_DTEN2_MASK                   0x100000u
#define FTM_COMBINE_DTEN2_SHIFT                  20
#define FTM_COMBINE_DTEN2_WIDTH                  1
#define FTM_COMBINE_DTEN2(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DTEN2_SHIFT))&FTM_COMBINE_DTEN2_MASK)
#define FTM_COMBINE_SYNCEN2_MASK                 0x200000u
#define FTM_COMBINE_SYNCEN2_SHIFT                21
#define FTM_COMBINE_SYNCEN2_WIDTH                1
#define FTM_COMBINE_SYNCEN2(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_SYNCEN2_SHIFT))&FTM_COMBINE_SYNCEN2_MASK)
#define FTM_COMBINE_FAULTEN2_MASK                0x400000u
#define FTM_COMBINE_FAULTEN2_SHIFT               22
#define FTM_COMBINE_FAULTEN2_WIDTH               1
#define FTM_COMBINE_FAULTEN2(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_FAULTEN2_SHIFT))&FTM_COMBINE_FAULTEN2_MASK)
#define FTM_COMBINE_COMBINE3_MASK                0x1000000u
#define FTM_COMBINE_COMBINE3_SHIFT               24
#define FTM_COMBINE_COMBINE3_WIDTH               1
#define FTM_COMBINE_COMBINE3(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMBINE3_SHIFT))&FTM_COMBINE_COMBINE3_MASK)
#define FTM_COMBINE_COMP3_MASK                   0x2000000u
#define FTM_COMBINE_COMP3_SHIFT                  25
#define FTM_COMBINE_COMP3_WIDTH                  1
#define FTM_COMBINE_COMP3(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_COMP3_SHIFT))&FTM_COMBINE_COMP3_MASK)
#define FTM_COMBINE_DECAPEN3_MASK                0x4000000u
#define FTM_COMBINE_DECAPEN3_SHIFT               26
#define FTM_COMBINE_DECAPEN3_WIDTH               1
#define FTM_COMBINE_DECAPEN3(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAPEN3_SHIFT))&FTM_COMBINE_DECAPEN3_MASK)
#define FTM_COMBINE_DECAP3_MASK                  0x8000000u
#define FTM_COMBINE_DECAP3_SHIFT                 27
#define FTM_COMBINE_DECAP3_WIDTH                 1
#define FTM_COMBINE_DECAP3(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DECAP3_SHIFT))&FTM_COMBINE_DECAP3_MASK)
#define FTM_COMBINE_DTEN3_MASK                   0x10000000u
#define FTM_COMBINE_DTEN3_SHIFT                  28
#define FTM_COMBINE_DTEN3_WIDTH                  1
#define FTM_COMBINE_DTEN3(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_DTEN3_SHIFT))&FTM_COMBINE_DTEN3_MASK)
#define FTM_COMBINE_SYNCEN3_MASK                 0x20000000u
#define FTM_COMBINE_SYNCEN3_SHIFT                29
#define FTM_COMBINE_SYNCEN3_WIDTH                1
#define FTM_COMBINE_SYNCEN3(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_SYNCEN3_SHIFT))&FTM_COMBINE_SYNCEN3_MASK)
#define FTM_COMBINE_FAULTEN3_MASK                0x40000000u
#define FTM_COMBINE_FAULTEN3_SHIFT               30
#define FTM_COMBINE_FAULTEN3_WIDTH               1
#define FTM_COMBINE_FAULTEN3(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_COMBINE_FAULTEN3_SHIFT))&FTM_COMBINE_FAULTEN3_MASK)
/* DEADTIME Bit Fields */
#define FTM_DEADTIME_DTVAL_MASK                  0x3Fu
#define FTM_DEADTIME_DTVAL_SHIFT                 0
#define FTM_DEADTIME_DTVAL_WIDTH                 6
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTVAL_SHIFT))&FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   0xC0u
#define FTM_DEADTIME_DTPS_SHIFT                  6
#define FTM_DEADTIME_DTPS_WIDTH                  2
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTPS_SHIFT))&FTM_DEADTIME_DTPS_MASK)
/* EXTTRIG Bit Fields */
#define FTM_EXTTRIG_CH2TRIG_MASK                 0x1u
#define FTM_EXTTRIG_CH2TRIG_SHIFT                0
#define FTM_EXTTRIG_CH2TRIG_WIDTH                1
#define FTM_EXTTRIG_CH2TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH2TRIG_SHIFT))&FTM_EXTTRIG_CH2TRIG_MASK)
#define FTM_EXTTRIG_CH3TRIG_MASK                 0x2u
#define FTM_EXTTRIG_CH3TRIG_SHIFT                1
#define FTM_EXTTRIG_CH3TRIG_WIDTH                1
#define FTM_EXTTRIG_CH3TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH3TRIG_SHIFT))&FTM_EXTTRIG_CH3TRIG_MASK)
#define FTM_EXTTRIG_CH4TRIG_MASK                 0x4u
#define FTM_EXTTRIG_CH4TRIG_SHIFT                2
#define FTM_EXTTRIG_CH4TRIG_WIDTH                1
#define FTM_EXTTRIG_CH4TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH4TRIG_SHIFT))&FTM_EXTTRIG_CH4TRIG_MASK)
#define FTM_EXTTRIG_CH5TRIG_MASK                 0x8u
#define FTM_EXTTRIG_CH5TRIG_SHIFT                3
#define FTM_EXTTRIG_CH5TRIG_WIDTH                1
#define FTM_EXTTRIG_CH5TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH5TRIG_SHIFT))&FTM_EXTTRIG_CH5TRIG_MASK)
#define FTM_EXTTRIG_CH0TRIG_MASK                 0x10u
#define FTM_EXTTRIG_CH0TRIG_SHIFT                4
#define FTM_EXTTRIG_CH0TRIG_WIDTH                1
#define FTM_EXTTRIG_CH0TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH0TRIG_SHIFT))&FTM_EXTTRIG_CH0TRIG_MASK)
#define FTM_EXTTRIG_CH1TRIG_MASK                 0x20u
#define FTM_EXTTRIG_CH1TRIG_SHIFT                5
#define FTM_EXTTRIG_CH1TRIG_WIDTH                1
#define FTM_EXTTRIG_CH1TRIG(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_CH1TRIG_SHIFT))&FTM_EXTTRIG_CH1TRIG_MASK)
#define FTM_EXTTRIG_INITTRIGEN_MASK              0x40u
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             6
#define FTM_EXTTRIG_INITTRIGEN_WIDTH             1
#define FTM_EXTTRIG_INITTRIGEN(x)                (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_INITTRIGEN_SHIFT))&FTM_EXTTRIG_INITTRIGEN_MASK)
#define FTM_EXTTRIG_TRIGF_MASK                   0x80u
#define FTM_EXTTRIG_TRIGF_SHIFT                  7
#define FTM_EXTTRIG_TRIGF_WIDTH                  1
#define FTM_EXTTRIG_TRIGF(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_EXTTRIG_TRIGF_SHIFT))&FTM_EXTTRIG_TRIGF_MASK)
/* POL Bit Fields */
#define FTM_POL_POL0_MASK                        0x1u
#define FTM_POL_POL0_SHIFT                       0
#define FTM_POL_POL0_WIDTH                       1
#define FTM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL0_SHIFT))&FTM_POL_POL0_MASK)
#define FTM_POL_POL1_MASK                        0x2u
#define FTM_POL_POL1_SHIFT                       1
#define FTM_POL_POL1_WIDTH                       1
#define FTM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL1_SHIFT))&FTM_POL_POL1_MASK)
#define FTM_POL_POL2_MASK                        0x4u
#define FTM_POL_POL2_SHIFT                       2
#define FTM_POL_POL2_WIDTH                       1
#define FTM_POL_POL2(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL2_SHIFT))&FTM_POL_POL2_MASK)
#define FTM_POL_POL3_MASK                        0x8u
#define FTM_POL_POL3_SHIFT                       3
#define FTM_POL_POL3_WIDTH                       1
#define FTM_POL_POL3(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL3_SHIFT))&FTM_POL_POL3_MASK)
#define FTM_POL_POL4_MASK                        0x10u
#define FTM_POL_POL4_SHIFT                       4
#define FTM_POL_POL4_WIDTH                       1
#define FTM_POL_POL4(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL4_SHIFT))&FTM_POL_POL4_MASK)
#define FTM_POL_POL5_MASK                        0x20u
#define FTM_POL_POL5_SHIFT                       5
#define FTM_POL_POL5_WIDTH                       1
#define FTM_POL_POL5(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL5_SHIFT))&FTM_POL_POL5_MASK)
#define FTM_POL_POL6_MASK                        0x40u
#define FTM_POL_POL6_SHIFT                       6
#define FTM_POL_POL6_WIDTH                       1
#define FTM_POL_POL6(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL6_SHIFT))&FTM_POL_POL6_MASK)
#define FTM_POL_POL7_MASK                        0x80u
#define FTM_POL_POL7_SHIFT                       7
#define FTM_POL_POL7_WIDTH                       1
#define FTM_POL_POL7(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_POL_POL7_SHIFT))&FTM_POL_POL7_MASK)
/* FMS Bit Fields */
#define FTM_FMS_FAULTF0_MASK                     0x1u
#define FTM_FMS_FAULTF0_SHIFT                    0
#define FTM_FMS_FAULTF0_WIDTH                    1
#define FTM_FMS_FAULTF0(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTF0_SHIFT))&FTM_FMS_FAULTF0_MASK)
#define FTM_FMS_FAULTF1_MASK                     0x2u
#define FTM_FMS_FAULTF1_SHIFT                    1
#define FTM_FMS_FAULTF1_WIDTH                    1
#define FTM_FMS_FAULTF1(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTF1_SHIFT))&FTM_FMS_FAULTF1_MASK)
#define FTM_FMS_FAULTF2_MASK                     0x4u
#define FTM_FMS_FAULTF2_SHIFT                    2
#define FTM_FMS_FAULTF2_WIDTH                    1
#define FTM_FMS_FAULTF2(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTF2_SHIFT))&FTM_FMS_FAULTF2_MASK)
#define FTM_FMS_FAULTF3_MASK                     0x8u
#define FTM_FMS_FAULTF3_SHIFT                    3
#define FTM_FMS_FAULTF3_WIDTH                    1
#define FTM_FMS_FAULTF3(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTF3_SHIFT))&FTM_FMS_FAULTF3_MASK)
#define FTM_FMS_FAULTIN_MASK                     0x20u
#define FTM_FMS_FAULTIN_SHIFT                    5
#define FTM_FMS_FAULTIN_WIDTH                    1
#define FTM_FMS_FAULTIN(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTIN_SHIFT))&FTM_FMS_FAULTIN_MASK)
#define FTM_FMS_WPEN_MASK                        0x40u
#define FTM_FMS_WPEN_SHIFT                       6
#define FTM_FMS_WPEN_WIDTH                       1
#define FTM_FMS_WPEN(x)                          (((uint32_t)(((uint32_t)(x))<<FTM_FMS_WPEN_SHIFT))&FTM_FMS_WPEN_MASK)
#define FTM_FMS_FAULTF_MASK                      0x80u
#define FTM_FMS_FAULTF_SHIFT                     7
#define FTM_FMS_FAULTF_WIDTH                     1
#define FTM_FMS_FAULTF(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_FMS_FAULTF_SHIFT))&FTM_FMS_FAULTF_MASK)
/* FILTER Bit Fields */
#define FTM_FILTER_CH0FVAL_MASK                  0xFu
#define FTM_FILTER_CH0FVAL_SHIFT                 0
#define FTM_FILTER_CH0FVAL_WIDTH                 4
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH0FVAL_SHIFT))&FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  0xF0u
#define FTM_FILTER_CH1FVAL_SHIFT                 4
#define FTM_FILTER_CH1FVAL_WIDTH                 4
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH1FVAL_SHIFT))&FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  0xF00u
#define FTM_FILTER_CH2FVAL_SHIFT                 8
#define FTM_FILTER_CH2FVAL_WIDTH                 4
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH2FVAL_SHIFT))&FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  0xF000u
#define FTM_FILTER_CH3FVAL_SHIFT                 12
#define FTM_FILTER_CH3FVAL_WIDTH                 4
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH3FVAL_SHIFT))&FTM_FILTER_CH3FVAL_MASK)
/* FLTCTRL Bit Fields */
#define FTM_FLTCTRL_FAULT0EN_MASK                0x1u
#define FTM_FLTCTRL_FAULT0EN_SHIFT               0
#define FTM_FLTCTRL_FAULT0EN_WIDTH               1
#define FTM_FLTCTRL_FAULT0EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FAULT0EN_SHIFT))&FTM_FLTCTRL_FAULT0EN_MASK)
#define FTM_FLTCTRL_FAULT1EN_MASK                0x2u
#define FTM_FLTCTRL_FAULT1EN_SHIFT               1
#define FTM_FLTCTRL_FAULT1EN_WIDTH               1
#define FTM_FLTCTRL_FAULT1EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FAULT1EN_SHIFT))&FTM_FLTCTRL_FAULT1EN_MASK)
#define FTM_FLTCTRL_FAULT2EN_MASK                0x4u
#define FTM_FLTCTRL_FAULT2EN_SHIFT               2
#define FTM_FLTCTRL_FAULT2EN_WIDTH               1
#define FTM_FLTCTRL_FAULT2EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FAULT2EN_SHIFT))&FTM_FLTCTRL_FAULT2EN_MASK)
#define FTM_FLTCTRL_FAULT3EN_MASK                0x8u
#define FTM_FLTCTRL_FAULT3EN_SHIFT               3
#define FTM_FLTCTRL_FAULT3EN_WIDTH               1
#define FTM_FLTCTRL_FAULT3EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FAULT3EN_SHIFT))&FTM_FLTCTRL_FAULT3EN_MASK)
#define FTM_FLTCTRL_FFLTR0EN_MASK                0x10u
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               4
#define FTM_FLTCTRL_FFLTR0EN_WIDTH               1
#define FTM_FLTCTRL_FFLTR0EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFLTR0EN_SHIFT))&FTM_FLTCTRL_FFLTR0EN_MASK)
#define FTM_FLTCTRL_FFLTR1EN_MASK                0x20u
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               5
#define FTM_FLTCTRL_FFLTR1EN_WIDTH               1
#define FTM_FLTCTRL_FFLTR1EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFLTR1EN_SHIFT))&FTM_FLTCTRL_FFLTR1EN_MASK)
#define FTM_FLTCTRL_FFLTR2EN_MASK                0x40u
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               6
#define FTM_FLTCTRL_FFLTR2EN_WIDTH               1
#define FTM_FLTCTRL_FFLTR2EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFLTR2EN_SHIFT))&FTM_FLTCTRL_FFLTR2EN_MASK)
#define FTM_FLTCTRL_FFLTR3EN_MASK                0x80u
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               7
#define FTM_FLTCTRL_FFLTR3EN_WIDTH               1
#define FTM_FLTCTRL_FFLTR3EN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFLTR3EN_SHIFT))&FTM_FLTCTRL_FFLTR3EN_MASK)
#define FTM_FLTCTRL_FFVAL_MASK                   0xF00u
#define FTM_FLTCTRL_FFVAL_SHIFT                  8
#define FTM_FLTCTRL_FFVAL_WIDTH                  4
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFVAL_SHIFT))&FTM_FLTCTRL_FFVAL_MASK)
/* QDCTRL Bit Fields */
#define FTM_QDCTRL_QUADEN_MASK                   0x1u
#define FTM_QDCTRL_QUADEN_SHIFT                  0
#define FTM_QDCTRL_QUADEN_WIDTH                  1
#define FTM_QDCTRL_QUADEN(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_QUADEN_SHIFT))&FTM_QDCTRL_QUADEN_MASK)
#define FTM_QDCTRL_TOFDIR_MASK                   0x2u
#define FTM_QDCTRL_TOFDIR_SHIFT                  1
#define FTM_QDCTRL_TOFDIR_WIDTH                  1
#define FTM_QDCTRL_TOFDIR(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_TOFDIR_SHIFT))&FTM_QDCTRL_TOFDIR_MASK)
#define FTM_QDCTRL_QUADIR_MASK                   0x4u
#define FTM_QDCTRL_QUADIR_SHIFT                  2
#define FTM_QDCTRL_QUADIR_WIDTH                  1
#define FTM_QDCTRL_QUADIR(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_QUADIR_SHIFT))&FTM_QDCTRL_QUADIR_MASK)
#define FTM_QDCTRL_QUADMODE_MASK                 0x8u
#define FTM_QDCTRL_QUADMODE_SHIFT                3
#define FTM_QDCTRL_QUADMODE_WIDTH                1
#define FTM_QDCTRL_QUADMODE(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_QUADMODE_SHIFT))&FTM_QDCTRL_QUADMODE_MASK)
#define FTM_QDCTRL_PHBPOL_MASK                   0x10u
#define FTM_QDCTRL_PHBPOL_SHIFT                  4
#define FTM_QDCTRL_PHBPOL_WIDTH                  1
#define FTM_QDCTRL_PHBPOL(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_PHBPOL_SHIFT))&FTM_QDCTRL_PHBPOL_MASK)
#define FTM_QDCTRL_PHAPOL_MASK                   0x20u
#define FTM_QDCTRL_PHAPOL_SHIFT                  5
#define FTM_QDCTRL_PHAPOL_WIDTH                  1
#define FTM_QDCTRL_PHAPOL(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_PHAPOL_SHIFT))&FTM_QDCTRL_PHAPOL_MASK)
#define FTM_QDCTRL_PHBFLTREN_MASK                0x40u
#define FTM_QDCTRL_PHBFLTREN_SHIFT               6
#define FTM_QDCTRL_PHBFLTREN_WIDTH               1
#define FTM_QDCTRL_PHBFLTREN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_PHBFLTREN_SHIFT))&FTM_QDCTRL_PHBFLTREN_MASK)
#define FTM_QDCTRL_PHAFLTREN_MASK                0x80u
#define FTM_QDCTRL_PHAFLTREN_SHIFT               7
#define FTM_QDCTRL_PHAFLTREN_WIDTH               1
#define FTM_QDCTRL_PHAFLTREN(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_QDCTRL_PHAFLTREN_SHIFT))&FTM_QDCTRL_PHAFLTREN_MASK)
/* CONF Bit Fields */
#define FTM_CONF_NUMTOF_MASK                     0x1Fu
#define FTM_CONF_NUMTOF_SHIFT                    0
#define FTM_CONF_NUMTOF_WIDTH                    5
#define FTM_CONF_NUMTOF(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_CONF_NUMTOF_SHIFT))&FTM_CONF_NUMTOF_MASK)
#define FTM_CONF_BDMMODE_MASK                    0xC0u
#define FTM_CONF_BDMMODE_SHIFT                   6
#define FTM_CONF_BDMMODE_WIDTH                   2
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_CONF_BDMMODE_SHIFT))&FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     0x200u
#define FTM_CONF_GTBEEN_SHIFT                    9
#define FTM_CONF_GTBEEN_WIDTH                    1
#define FTM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_CONF_GTBEEN_SHIFT))&FTM_CONF_GTBEEN_MASK)
#define FTM_CONF_GTBEOUT_MASK                    0x400u
#define FTM_CONF_GTBEOUT_SHIFT                   10
#define FTM_CONF_GTBEOUT_WIDTH                   1
#define FTM_CONF_GTBEOUT(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_CONF_GTBEOUT_SHIFT))&FTM_CONF_GTBEOUT_MASK)
/* FLTPOL Bit Fields */
#define FTM_FLTPOL_FLT0POL_MASK                  0x1u
#define FTM_FLTPOL_FLT0POL_SHIFT                 0
#define FTM_FLTPOL_FLT0POL_WIDTH                 1
#define FTM_FLTPOL_FLT0POL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FLTPOL_FLT0POL_SHIFT))&FTM_FLTPOL_FLT0POL_MASK)
#define FTM_FLTPOL_FLT1POL_MASK                  0x2u
#define FTM_FLTPOL_FLT1POL_SHIFT                 1
#define FTM_FLTPOL_FLT1POL_WIDTH                 1
#define FTM_FLTPOL_FLT1POL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FLTPOL_FLT1POL_SHIFT))&FTM_FLTPOL_FLT1POL_MASK)
#define FTM_FLTPOL_FLT2POL_MASK                  0x4u
#define FTM_FLTPOL_FLT2POL_SHIFT                 2
#define FTM_FLTPOL_FLT2POL_WIDTH                 1
#define FTM_FLTPOL_FLT2POL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FLTPOL_FLT2POL_SHIFT))&FTM_FLTPOL_FLT2POL_MASK)
#define FTM_FLTPOL_FLT3POL_MASK                  0x8u
#define FTM_FLTPOL_FLT3POL_SHIFT                 3
#define FTM_FLTPOL_FLT3POL_WIDTH                 1
#define FTM_FLTPOL_FLT3POL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FLTPOL_FLT3POL_SHIFT))&FTM_FLTPOL_FLT3POL_MASK)
/* SYNCONF Bit Fields */
#define FTM_SYNCONF_HWTRIGMODE_MASK              0x1u
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             0
#define FTM_SYNCONF_HWTRIGMODE_WIDTH             1
#define FTM_SYNCONF_HWTRIGMODE(x)                (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWTRIGMODE_SHIFT))&FTM_SYNCONF_HWTRIGMODE_MASK)
#define FTM_SYNCONF_CNTINC_MASK                  0x4u
#define FTM_SYNCONF_CNTINC_SHIFT                 2
#define FTM_SYNCONF_CNTINC_WIDTH                 1
#define FTM_SYNCONF_CNTINC(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_CNTINC_SHIFT))&FTM_SYNCONF_CNTINC_MASK)
#define FTM_SYNCONF_INVC_MASK                    0x10u
#define FTM_SYNCONF_INVC_SHIFT                   4
#define FTM_SYNCONF_INVC_WIDTH                   1
#define FTM_SYNCONF_INVC(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_INVC_SHIFT))&FTM_SYNCONF_INVC_MASK)
#define FTM_SYNCONF_SWOC_MASK                    0x20u
#define FTM_SYNCONF_SWOC_SHIFT                   5
#define FTM_SYNCONF_SWOC_WIDTH                   1
#define FTM_SYNCONF_SWOC(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWOC_SHIFT))&FTM_SYNCONF_SWOC_MASK)
#define FTM_SYNCONF_SYNCMODE_MASK                0x80u
#define FTM_SYNCONF_SYNCMODE_SHIFT               7
#define FTM_SYNCONF_SYNCMODE_WIDTH               1
#define FTM_SYNCONF_SYNCMODE(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SYNCMODE_SHIFT))&FTM_SYNCONF_SYNCMODE_MASK)
#define FTM_SYNCONF_SWRSTCNT_MASK                0x100u
#define FTM_SYNCONF_SWRSTCNT_SHIFT               8
#define FTM_SYNCONF_SWRSTCNT_WIDTH               1
#define FTM_SYNCONF_SWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWRSTCNT_SHIFT))&FTM_SYNCONF_SWRSTCNT_MASK)
#define FTM_SYNCONF_SWWRBUF_MASK                 0x200u
#define FTM_SYNCONF_SWWRBUF_SHIFT                9
#define FTM_SYNCONF_SWWRBUF_WIDTH                1
#define FTM_SYNCONF_SWWRBUF(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWWRBUF_SHIFT))&FTM_SYNCONF_SWWRBUF_MASK)
#define FTM_SYNCONF_SWOM_MASK                    0x400u
#define FTM_SYNCONF_SWOM_SHIFT                   10
#define FTM_SYNCONF_SWOM_WIDTH                   1
#define FTM_SYNCONF_SWOM(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWOM_SHIFT))&FTM_SYNCONF_SWOM_MASK)
#define FTM_SYNCONF_SWINVC_MASK                  0x800u
#define FTM_SYNCONF_SWINVC_SHIFT                 11
#define FTM_SYNCONF_SWINVC_WIDTH                 1
#define FTM_SYNCONF_SWINVC(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWINVC_SHIFT))&FTM_SYNCONF_SWINVC_MASK)
#define FTM_SYNCONF_SWSOC_MASK                   0x1000u
#define FTM_SYNCONF_SWSOC_SHIFT                  12
#define FTM_SYNCONF_SWSOC_WIDTH                  1
#define FTM_SYNCONF_SWSOC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_SWSOC_SHIFT))&FTM_SYNCONF_SWSOC_MASK)
#define FTM_SYNCONF_HWRSTCNT_MASK                0x10000u
#define FTM_SYNCONF_HWRSTCNT_SHIFT               16
#define FTM_SYNCONF_HWRSTCNT_WIDTH               1
#define FTM_SYNCONF_HWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWRSTCNT_SHIFT))&FTM_SYNCONF_HWRSTCNT_MASK)
#define FTM_SYNCONF_HWWRBUF_MASK                 0x20000u
#define FTM_SYNCONF_HWWRBUF_SHIFT                17
#define FTM_SYNCONF_HWWRBUF_WIDTH                1
#define FTM_SYNCONF_HWWRBUF(x)                   (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWWRBUF_SHIFT))&FTM_SYNCONF_HWWRBUF_MASK)
#define FTM_SYNCONF_HWOM_MASK                    0x40000u
#define FTM_SYNCONF_HWOM_SHIFT                   18
#define FTM_SYNCONF_HWOM_WIDTH                   1
#define FTM_SYNCONF_HWOM(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWOM_SHIFT))&FTM_SYNCONF_HWOM_MASK)
#define FTM_SYNCONF_HWINVC_MASK                  0x80000u
#define FTM_SYNCONF_HWINVC_SHIFT                 19
#define FTM_SYNCONF_HWINVC_WIDTH                 1
#define FTM_SYNCONF_HWINVC(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWINVC_SHIFT))&FTM_SYNCONF_HWINVC_MASK)
#define FTM_SYNCONF_HWSOC_MASK                   0x100000u
#define FTM_SYNCONF_HWSOC_SHIFT                  20
#define FTM_SYNCONF_HWSOC_WIDTH                  1
#define FTM_SYNCONF_HWSOC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SYNCONF_HWSOC_SHIFT))&FTM_SYNCONF_HWSOC_MASK)
/* INVCTRL Bit Fields */
#define FTM_INVCTRL_INV0EN_MASK                  0x1u
#define FTM_INVCTRL_INV0EN_SHIFT                 0
#define FTM_INVCTRL_INV0EN_WIDTH                 1
#define FTM_INVCTRL_INV0EN(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_INVCTRL_INV0EN_SHIFT))&FTM_INVCTRL_INV0EN_MASK)
#define FTM_INVCTRL_INV1EN_MASK                  0x2u
#define FTM_INVCTRL_INV1EN_SHIFT                 1
#define FTM_INVCTRL_INV1EN_WIDTH                 1
#define FTM_INVCTRL_INV1EN(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_INVCTRL_INV1EN_SHIFT))&FTM_INVCTRL_INV1EN_MASK)
#define FTM_INVCTRL_INV2EN_MASK                  0x4u
#define FTM_INVCTRL_INV2EN_SHIFT                 2
#define FTM_INVCTRL_INV2EN_WIDTH                 1
#define FTM_INVCTRL_INV2EN(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_INVCTRL_INV2EN_SHIFT))&FTM_INVCTRL_INV2EN_MASK)
#define FTM_INVCTRL_INV3EN_MASK                  0x8u
#define FTM_INVCTRL_INV3EN_SHIFT                 3
#define FTM_INVCTRL_INV3EN_WIDTH                 1
#define FTM_INVCTRL_INV3EN(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_INVCTRL_INV3EN_SHIFT))&FTM_INVCTRL_INV3EN_MASK)
/* SWOCTRL Bit Fields */
#define FTM_SWOCTRL_CH0OC_MASK                   0x1u
#define FTM_SWOCTRL_CH0OC_SHIFT                  0
#define FTM_SWOCTRL_CH0OC_WIDTH                  1
#define FTM_SWOCTRL_CH0OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH0OC_SHIFT))&FTM_SWOCTRL_CH0OC_MASK)
#define FTM_SWOCTRL_CH1OC_MASK                   0x2u
#define FTM_SWOCTRL_CH1OC_SHIFT                  1
#define FTM_SWOCTRL_CH1OC_WIDTH                  1
#define FTM_SWOCTRL_CH1OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH1OC_SHIFT))&FTM_SWOCTRL_CH1OC_MASK)
#define FTM_SWOCTRL_CH2OC_MASK                   0x4u
#define FTM_SWOCTRL_CH2OC_SHIFT                  2
#define FTM_SWOCTRL_CH2OC_WIDTH                  1
#define FTM_SWOCTRL_CH2OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH2OC_SHIFT))&FTM_SWOCTRL_CH2OC_MASK)
#define FTM_SWOCTRL_CH3OC_MASK                   0x8u
#define FTM_SWOCTRL_CH3OC_SHIFT                  3
#define FTM_SWOCTRL_CH3OC_WIDTH                  1
#define FTM_SWOCTRL_CH3OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH3OC_SHIFT))&FTM_SWOCTRL_CH3OC_MASK)
#define FTM_SWOCTRL_CH4OC_MASK                   0x10u
#define FTM_SWOCTRL_CH4OC_SHIFT                  4
#define FTM_SWOCTRL_CH4OC_WIDTH                  1
#define FTM_SWOCTRL_CH4OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH4OC_SHIFT))&FTM_SWOCTRL_CH4OC_MASK)
#define FTM_SWOCTRL_CH5OC_MASK                   0x20u
#define FTM_SWOCTRL_CH5OC_SHIFT                  5
#define FTM_SWOCTRL_CH5OC_WIDTH                  1
#define FTM_SWOCTRL_CH5OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH5OC_SHIFT))&FTM_SWOCTRL_CH5OC_MASK)
#define FTM_SWOCTRL_CH6OC_MASK                   0x40u
#define FTM_SWOCTRL_CH6OC_SHIFT                  6
#define FTM_SWOCTRL_CH6OC_WIDTH                  1
#define FTM_SWOCTRL_CH6OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH6OC_SHIFT))&FTM_SWOCTRL_CH6OC_MASK)
#define FTM_SWOCTRL_CH7OC_MASK                   0x80u
#define FTM_SWOCTRL_CH7OC_SHIFT                  7
#define FTM_SWOCTRL_CH7OC_WIDTH                  1
#define FTM_SWOCTRL_CH7OC(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH7OC_SHIFT))&FTM_SWOCTRL_CH7OC_MASK)
#define FTM_SWOCTRL_CH0OCV_MASK                  0x100u
#define FTM_SWOCTRL_CH0OCV_SHIFT                 8
#define FTM_SWOCTRL_CH0OCV_WIDTH                 1
#define FTM_SWOCTRL_CH0OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH0OCV_SHIFT))&FTM_SWOCTRL_CH0OCV_MASK)
#define FTM_SWOCTRL_CH1OCV_MASK                  0x200u
#define FTM_SWOCTRL_CH1OCV_SHIFT                 9
#define FTM_SWOCTRL_CH1OCV_WIDTH                 1
#define FTM_SWOCTRL_CH1OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH1OCV_SHIFT))&FTM_SWOCTRL_CH1OCV_MASK)
#define FTM_SWOCTRL_CH2OCV_MASK                  0x400u
#define FTM_SWOCTRL_CH2OCV_SHIFT                 10
#define FTM_SWOCTRL_CH2OCV_WIDTH                 1
#define FTM_SWOCTRL_CH2OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH2OCV_SHIFT))&FTM_SWOCTRL_CH2OCV_MASK)
#define FTM_SWOCTRL_CH3OCV_MASK                  0x800u
#define FTM_SWOCTRL_CH3OCV_SHIFT                 11
#define FTM_SWOCTRL_CH3OCV_WIDTH                 1
#define FTM_SWOCTRL_CH3OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH3OCV_SHIFT))&FTM_SWOCTRL_CH3OCV_MASK)
#define FTM_SWOCTRL_CH4OCV_MASK                  0x1000u
#define FTM_SWOCTRL_CH4OCV_SHIFT                 12
#define FTM_SWOCTRL_CH4OCV_WIDTH                 1
#define FTM_SWOCTRL_CH4OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH4OCV_SHIFT))&FTM_SWOCTRL_CH4OCV_MASK)
#define FTM_SWOCTRL_CH5OCV_MASK                  0x2000u
#define FTM_SWOCTRL_CH5OCV_SHIFT                 13
#define FTM_SWOCTRL_CH5OCV_WIDTH                 1
#define FTM_SWOCTRL_CH5OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH5OCV_SHIFT))&FTM_SWOCTRL_CH5OCV_MASK)
#define FTM_SWOCTRL_CH6OCV_MASK                  0x4000u
#define FTM_SWOCTRL_CH6OCV_SHIFT                 14
#define FTM_SWOCTRL_CH6OCV_WIDTH                 1
#define FTM_SWOCTRL_CH6OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH6OCV_SHIFT))&FTM_SWOCTRL_CH6OCV_MASK)
#define FTM_SWOCTRL_CH7OCV_MASK                  0x8000u
#define FTM_SWOCTRL_CH7OCV_SHIFT                 15
#define FTM_SWOCTRL_CH7OCV_WIDTH                 1
#define FTM_SWOCTRL_CH7OCV(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_SWOCTRL_CH7OCV_SHIFT))&FTM_SWOCTRL_CH7OCV_MASK)
/* PWMLOAD Bit Fields */
#define FTM_PWMLOAD_CH0SEL_MASK                  0x1u
#define FTM_PWMLOAD_CH0SEL_SHIFT                 0
#define FTM_PWMLOAD_CH0SEL_WIDTH                 1
#define FTM_PWMLOAD_CH0SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH0SEL_SHIFT))&FTM_PWMLOAD_CH0SEL_MASK)
#define FTM_PWMLOAD_CH1SEL_MASK                  0x2u
#define FTM_PWMLOAD_CH1SEL_SHIFT                 1
#define FTM_PWMLOAD_CH1SEL_WIDTH                 1
#define FTM_PWMLOAD_CH1SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH1SEL_SHIFT))&FTM_PWMLOAD_CH1SEL_MASK)
#define FTM_PWMLOAD_CH2SEL_MASK                  0x4u
#define FTM_PWMLOAD_CH2SEL_SHIFT                 2
#define FTM_PWMLOAD_CH2SEL_WIDTH                 1
#define FTM_PWMLOAD_CH2SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH2SEL_SHIFT))&FTM_PWMLOAD_CH2SEL_MASK)
#define FTM_PWMLOAD_CH3SEL_MASK                  0x8u
#define FTM_PWMLOAD_CH3SEL_SHIFT                 3
#define FTM_PWMLOAD_CH3SEL_WIDTH                 1
#define FTM_PWMLOAD_CH3SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH3SEL_SHIFT))&FTM_PWMLOAD_CH3SEL_MASK)
#define FTM_PWMLOAD_CH4SEL_MASK                  0x10u
#define FTM_PWMLOAD_CH4SEL_SHIFT                 4
#define FTM_PWMLOAD_CH4SEL_WIDTH                 1
#define FTM_PWMLOAD_CH4SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH4SEL_SHIFT))&FTM_PWMLOAD_CH4SEL_MASK)
#define FTM_PWMLOAD_CH5SEL_MASK                  0x20u
#define FTM_PWMLOAD_CH5SEL_SHIFT                 5
#define FTM_PWMLOAD_CH5SEL_WIDTH                 1
#define FTM_PWMLOAD_CH5SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH5SEL_SHIFT))&FTM_PWMLOAD_CH5SEL_MASK)
#define FTM_PWMLOAD_CH6SEL_MASK                  0x40u
#define FTM_PWMLOAD_CH6SEL_SHIFT                 6
#define FTM_PWMLOAD_CH6SEL_WIDTH                 1
#define FTM_PWMLOAD_CH6SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH6SEL_SHIFT))&FTM_PWMLOAD_CH6SEL_MASK)
#define FTM_PWMLOAD_CH7SEL_MASK                  0x80u
#define FTM_PWMLOAD_CH7SEL_SHIFT                 7
#define FTM_PWMLOAD_CH7SEL_WIDTH                 1
#define FTM_PWMLOAD_CH7SEL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_CH7SEL_SHIFT))&FTM_PWMLOAD_CH7SEL_MASK)
#define FTM_PWMLOAD_LDOK_MASK                    0x200u
#define FTM_PWMLOAD_LDOK_SHIFT                   9
#define FTM_PWMLOAD_LDOK_WIDTH                   1
#define FTM_PWMLOAD_LDOK(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_PWMLOAD_LDOK_SHIFT))&FTM_PWMLOAD_LDOK_MASK)

/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base address */
#define FTM0_BASE                                (0x40038000u)
/** Peripheral FTM0 base pointer */
#define FTM0                                     ((FTM_Type *)FTM0_BASE)
#define FTM0_BASE_PTR                            (FTM0)
/** Peripheral FTM1 base address */
#define FTM1_BASE                                (0x40039000u)
/** Peripheral FTM1 base pointer */
#define FTM1                                     ((FTM_Type *)FTM1_BASE)
#define FTM1_BASE_PTR                            (FTM1)
/** Peripheral FTM3 base address */
#define FTM3_BASE                                (0x40026000u)
/** Peripheral FTM3 base pointer */
#define FTM3                                     ((FTM_Type *)FTM3_BASE)
#define FTM3_BASE_PTR                            (FTM3)
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM3_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM3 }

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register instance definitions */
/* FTM0 */
#define FTM0_SC                                  FTM_SC_REG(FTM0)
#define FTM0_CNT                                 FTM_CNT_REG(FTM0)
#define FTM0_MOD                                 FTM_MOD_REG(FTM0)
#define FTM0_C0SC                                FTM_CnSC_REG(FTM0,0)
#define FTM0_C0V                                 FTM_CnV_REG(FTM0,0)
#define FTM0_C1SC                                FTM_CnSC_REG(FTM0,1)
#define FTM0_C1V                                 FTM_CnV_REG(FTM0,1)
#define FTM0_C2SC                                FTM_CnSC_REG(FTM0,2)
#define FTM0_C2V                                 FTM_CnV_REG(FTM0,2)
#define FTM0_C3SC                                FTM_CnSC_REG(FTM0,3)
#define FTM0_C3V                                 FTM_CnV_REG(FTM0,3)
#define FTM0_C4SC                                FTM_CnSC_REG(FTM0,4)
#define FTM0_C4V                                 FTM_CnV_REG(FTM0,4)
#define FTM0_C5SC                                FTM_CnSC_REG(FTM0,5)
#define FTM0_C5V                                 FTM_CnV_REG(FTM0,5)
#define FTM0_C6SC                                FTM_CnSC_REG(FTM0,6)
#define FTM0_C6V                                 FTM_CnV_REG(FTM0,6)
#define FTM0_C7SC                                FTM_CnSC_REG(FTM0,7)
#define FTM0_C7V                                 FTM_CnV_REG(FTM0,7)
#define FTM0_CNTIN                               FTM_CNTIN_REG(FTM0)
#define FTM0_STATUS                              FTM_STATUS_REG(FTM0)
#define FTM0_MODE                                FTM_MODE_REG(FTM0)
#define FTM0_SYNC                                FTM_SYNC_REG(FTM0)
#define FTM0_OUTINIT                             FTM_OUTINIT_REG(FTM0)
#define FTM0_OUTMASK                             FTM_OUTMASK_REG(FTM0)
#define FTM0_COMBINE                             FTM_COMBINE_REG(FTM0)
#define FTM0_DEADTIME                            FTM_DEADTIME_REG(FTM0)
#define FTM0_EXTTRIG                             FTM_EXTTRIG_REG(FTM0)
#define FTM0_POL                                 FTM_POL_REG(FTM0)
#define FTM0_FMS                                 FTM_FMS_REG(FTM0)
#define FTM0_FILTER                              FTM_FILTER_REG(FTM0)
#define FTM0_FLTCTRL                             FTM_FLTCTRL_REG(FTM0)
#define FTM0_QDCTRL                              FTM_QDCTRL_REG(FTM0)
#define FTM0_CONF                                FTM_CONF_REG(FTM0)
#define FTM0_FLTPOL                              FTM_FLTPOL_REG(FTM0)
#define FTM0_SYNCONF                             FTM_SYNCONF_REG(FTM0)
#define FTM0_INVCTRL                             FTM_INVCTRL_REG(FTM0)
#define FTM0_SWOCTRL                             FTM_SWOCTRL_REG(FTM0)
#define FTM0_PWMLOAD                             FTM_PWMLOAD_REG(FTM0)
/* FTM1 */
#define FTM1_SC                                  FTM_SC_REG(FTM1)
#define FTM1_CNT                                 FTM_CNT_REG(FTM1)
#define FTM1_MOD                                 FTM_MOD_REG(FTM1)
#define FTM1_C0SC                                FTM_CnSC_REG(FTM1,0)
#define FTM1_C0V                                 FTM_CnV_REG(FTM1,0)
#define FTM1_C1SC                                FTM_CnSC_REG(FTM1,1)
#define FTM1_C1V                                 FTM_CnV_REG(FTM1,1)
#define FTM1_CNTIN                               FTM_CNTIN_REG(FTM1)
#define FTM1_STATUS                              FTM_STATUS_REG(FTM1)
#define FTM1_MODE                                FTM_MODE_REG(FTM1)
#define FTM1_SYNC                                FTM_SYNC_REG(FTM1)
#define FTM1_OUTINIT                             FTM_OUTINIT_REG(FTM1)
#define FTM1_OUTMASK                             FTM_OUTMASK_REG(FTM1)
#define FTM1_COMBINE                             FTM_COMBINE_REG(FTM1)
#define FTM1_DEADTIME                            FTM_DEADTIME_REG(FTM1)
#define FTM1_EXTTRIG                             FTM_EXTTRIG_REG(FTM1)
#define FTM1_POL                                 FTM_POL_REG(FTM1)
#define FTM1_FMS                                 FTM_FMS_REG(FTM1)
#define FTM1_FILTER                              FTM_FILTER_REG(FTM1)
#define FTM1_FLTCTRL                             FTM_FLTCTRL_REG(FTM1)
#define FTM1_QDCTRL                              FTM_QDCTRL_REG(FTM1)
#define FTM1_CONF                                FTM_CONF_REG(FTM1)
#define FTM1_FLTPOL                              FTM_FLTPOL_REG(FTM1)
#define FTM1_SYNCONF                             FTM_SYNCONF_REG(FTM1)
#define FTM1_INVCTRL                             FTM_INVCTRL_REG(FTM1)
#define FTM1_SWOCTRL                             FTM_SWOCTRL_REG(FTM1)
#define FTM1_PWMLOAD                             FTM_PWMLOAD_REG(FTM1)
/* FTM3 */
#define FTM3_SC                                  FTM_SC_REG(FTM3)
#define FTM3_CNT                                 FTM_CNT_REG(FTM3)
#define FTM3_MOD                                 FTM_MOD_REG(FTM3)
#define FTM3_C0SC                                FTM_CnSC_REG(FTM3,0)
#define FTM3_C0V                                 FTM_CnV_REG(FTM3,0)
#define FTM3_C1SC                                FTM_CnSC_REG(FTM3,1)
#define FTM3_C1V                                 FTM_CnV_REG(FTM3,1)
#define FTM3_C2SC                                FTM_CnSC_REG(FTM3,2)
#define FTM3_C2V                                 FTM_CnV_REG(FTM3,2)
#define FTM3_C3SC                                FTM_CnSC_REG(FTM3,3)
#define FTM3_C3V                                 FTM_CnV_REG(FTM3,3)
#define FTM3_C4SC                                FTM_CnSC_REG(FTM3,4)
#define FTM3_C4V                                 FTM_CnV_REG(FTM3,4)
#define FTM3_C5SC                                FTM_CnSC_REG(FTM3,5)
#define FTM3_C5V                                 FTM_CnV_REG(FTM3,5)
#define FTM3_C6SC                                FTM_CnSC_REG(FTM3,6)
#define FTM3_C6V                                 FTM_CnV_REG(FTM3,6)
#define FTM3_C7SC                                FTM_CnSC_REG(FTM3,7)
#define FTM3_C7V                                 FTM_CnV_REG(FTM3,7)
#define FTM3_CNTIN                               FTM_CNTIN_REG(FTM3)
#define FTM3_STATUS                              FTM_STATUS_REG(FTM3)
#define FTM3_MODE                                FTM_MODE_REG(FTM3)
#define FTM3_SYNC                                FTM_SYNC_REG(FTM3)
#define FTM3_OUTINIT                             FTM_OUTINIT_REG(FTM3)
#define FTM3_OUTMASK                             FTM_OUTMASK_REG(FTM3)
#define FTM3_COMBINE                             FTM_COMBINE_REG(FTM3)
#define FTM3_DEADTIME                            FTM_DEADTIME_REG(FTM3)
#define FTM3_EXTTRIG                             FTM_EXTTRIG_REG(FTM3)
#define FTM3_POL                                 FTM_POL_REG(FTM3)
#define FTM3_FMS                                 FTM_FMS_REG(FTM3)
#define FTM3_FILTER                              FTM_FILTER_REG(FTM3)
#define FTM3_FLTCTRL                             FTM_FLTCTRL_REG(FTM3)
#define FTM3_QDCTRL                              FTM_QDCTRL_REG(FTM3)
#define FTM3_CONF                                FTM_CONF_REG(FTM3)
#define FTM3_FLTPOL                              FTM_FLTPOL_REG(FTM3)
#define FTM3_SYNCONF                             FTM_SYNCONF_REG(FTM3)
#define FTM3_INVCTRL                             FTM_INVCTRL_REG(FTM3)
#define FTM3_SWOCTRL                             FTM_SWOCTRL_REG(FTM3)
#define FTM3_PWMLOAD                             FTM_PWMLOAD_REG(FTM3)

/* FTM - Register array accessors */
#define FTM0_CnSC(index)                         FTM_CnSC_REG(FTM0,index)
#define FTM1_CnSC(index)                         FTM_CnSC_REG(FTM1,index)
#define FTM3_CnSC(index)                         FTM_CnSC_REG(FTM3,index)
#define FTM0_CnV(index)                          FTM_CnV_REG(FTM0,index)
#define FTM1_CnV(index)                          FTM_CnV_REG(FTM1,index)
#define FTM3_CnV(index)                          FTM_CnV_REG(FTM3,index)

/*!
 * @}
 */ /* end of group FTM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FTM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} GPIO_Type, *GPIO_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register accessors */
#define GPIO_PDOR_REG(base)                      ((base)->PDOR)
#define GPIO_PSOR_REG(base)                      ((base)->PSOR)
#define GPIO_PCOR_REG(base)                      ((base)->PCOR)
#define GPIO_PTOR_REG(base)                      ((base)->PTOR)
#define GPIO_PDIR_REG(base)                      ((base)->PDIR)
#define GPIO_PDDR_REG(base)                      ((base)->PDDR)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/* PDOR Bit Fields */
#define GPIO_PDOR_PDO_MASK                       0xFFFFFFFFu
#define GPIO_PDOR_PDO_SHIFT                      0
#define GPIO_PDOR_PDO_WIDTH                      32
#define GPIO_PDOR_PDO(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDOR_PDO_SHIFT))&GPIO_PDOR_PDO_MASK)
/* PSOR Bit Fields */
#define GPIO_PSOR_PTSO_MASK                      0xFFFFFFFFu
#define GPIO_PSOR_PTSO_SHIFT                     0
#define GPIO_PSOR_PTSO_WIDTH                     32
#define GPIO_PSOR_PTSO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PSOR_PTSO_SHIFT))&GPIO_PSOR_PTSO_MASK)
/* PCOR Bit Fields */
#define GPIO_PCOR_PTCO_MASK                      0xFFFFFFFFu
#define GPIO_PCOR_PTCO_SHIFT                     0
#define GPIO_PCOR_PTCO_WIDTH                     32
#define GPIO_PCOR_PTCO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PCOR_PTCO_SHIFT))&GPIO_PCOR_PTCO_MASK)
/* PTOR Bit Fields */
#define GPIO_PTOR_PTTO_MASK                      0xFFFFFFFFu
#define GPIO_PTOR_PTTO_SHIFT                     0
#define GPIO_PTOR_PTTO_WIDTH                     32
#define GPIO_PTOR_PTTO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PTOR_PTTO_SHIFT))&GPIO_PTOR_PTTO_MASK)
/* PDIR Bit Fields */
#define GPIO_PDIR_PDI_MASK                       0xFFFFFFFFu
#define GPIO_PDIR_PDI_SHIFT                      0
#define GPIO_PDIR_PDI_WIDTH                      32
#define GPIO_PDIR_PDI(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDIR_PDI_SHIFT))&GPIO_PDIR_PDI_MASK)
/* PDDR Bit Fields */
#define GPIO_PDDR_PDD_MASK                       0xFFFFFFFFu
#define GPIO_PDDR_PDD_SHIFT                      0
#define GPIO_PDDR_PDD_WIDTH                      32
#define GPIO_PDDR_PDD(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDDR_PDD_SHIFT))&GPIO_PDDR_PDD_MASK)

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIOA base address */
#define GPIOA_BASE                               (0x400FF000u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
#define GPIOA_BASE_PTR                           (GPIOA)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0x400FF040u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
#define GPIOB_BASE_PTR                           (GPIOB)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0x400FF080u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
#define GPIOC_BASE_PTR                           (GPIOC)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0x400FF0C0u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
#define GPIOD_BASE_PTR                           (GPIOD)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0x400FF100u)
/** Peripheral GPIOE base pointer */
#define GPIOE                                    ((GPIO_Type *)GPIOE_BASE)
#define GPIOE_BASE_PTR                           (GPIOE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE }

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register instance definitions */
/* GPIOA */
#define GPIOA_PDOR                               GPIO_PDOR_REG(GPIOA)
#define GPIOA_PSOR                               GPIO_PSOR_REG(GPIOA)
#define GPIOA_PCOR                               GPIO_PCOR_REG(GPIOA)
#define GPIOA_PTOR                               GPIO_PTOR_REG(GPIOA)
#define GPIOA_PDIR                               GPIO_PDIR_REG(GPIOA)
#define GPIOA_PDDR                               GPIO_PDDR_REG(GPIOA)
/* GPIOB */
#define GPIOB_PDOR                               GPIO_PDOR_REG(GPIOB)
#define GPIOB_PSOR                               GPIO_PSOR_REG(GPIOB)
#define GPIOB_PCOR                               GPIO_PCOR_REG(GPIOB)
#define GPIOB_PTOR                               GPIO_PTOR_REG(GPIOB)
#define GPIOB_PDIR                               GPIO_PDIR_REG(GPIOB)
#define GPIOB_PDDR                               GPIO_PDDR_REG(GPIOB)
/* GPIOC */
#define GPIOC_PDOR                               GPIO_PDOR_REG(GPIOC)
#define GPIOC_PSOR                               GPIO_PSOR_REG(GPIOC)
#define GPIOC_PCOR                               GPIO_PCOR_REG(GPIOC)
#define GPIOC_PTOR                               GPIO_PTOR_REG(GPIOC)
#define GPIOC_PDIR                               GPIO_PDIR_REG(GPIOC)
#define GPIOC_PDDR                               GPIO_PDDR_REG(GPIOC)
/* GPIOD */
#define GPIOD_PDOR                               GPIO_PDOR_REG(GPIOD)
#define GPIOD_PSOR                               GPIO_PSOR_REG(GPIOD)
#define GPIOD_PCOR                               GPIO_PCOR_REG(GPIOD)
#define GPIOD_PTOR                               GPIO_PTOR_REG(GPIOD)
#define GPIOD_PDIR                               GPIO_PDIR_REG(GPIOD)
#define GPIOD_PDDR                               GPIO_PDDR_REG(GPIOD)
/* GPIOE */
#define GPIOE_PDOR                               GPIO_PDOR_REG(GPIOE)
#define GPIOE_PSOR                               GPIO_PSOR_REG(GPIOE)
#define GPIOE_PCOR                               GPIO_PCOR_REG(GPIOE)
#define GPIOE_PTOR                               GPIO_PTOR_REG(GPIOE)
#define GPIOE_PDIR                               GPIO_PDIR_REG(GPIOE)
#define GPIOE_PDDR                               GPIO_PDDR_REG(GPIOE)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct {
  __IO uint8_t A1;                                 /**< I2C Address Register 1, offset: 0x0 */
  __IO uint8_t F;                                  /**< I2C Frequency Divider register, offset: 0x1 */
  __IO uint8_t C1;                                 /**< I2C Control Register 1, offset: 0x2 */
  __IO uint8_t S;                                  /**< I2C Status register, offset: 0x3 */
  __IO uint8_t D;                                  /**< I2C Data I/O register, offset: 0x4 */
  __IO uint8_t C2;                                 /**< I2C Control Register 2, offset: 0x5 */
  __IO uint8_t FLT;                                /**< I2C Programmable Input Glitch Filter Register, offset: 0x6 */
  __IO uint8_t RA;                                 /**< I2C Range Address register, offset: 0x7 */
  __IO uint8_t SMB;                                /**< I2C SMBus Control and Status register, offset: 0x8 */
  __IO uint8_t A2;                                 /**< I2C Address Register 2, offset: 0x9 */
  __IO uint8_t SLTH;                               /**< I2C SCL Low Timeout Register High, offset: 0xA */
  __IO uint8_t SLTL;                               /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_Type, *I2C_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register accessors */
#define I2C_A1_REG(base)                         ((base)->A1)
#define I2C_F_REG(base)                          ((base)->F)
#define I2C_C1_REG(base)                         ((base)->C1)
#define I2C_S_REG(base)                          ((base)->S)
#define I2C_D_REG(base)                          ((base)->D)
#define I2C_C2_REG(base)                         ((base)->C2)
#define I2C_FLT_REG(base)                        ((base)->FLT)
#define I2C_RA_REG(base)                         ((base)->RA)
#define I2C_SMB_REG(base)                        ((base)->SMB)
#define I2C_A2_REG(base)                         ((base)->A2)
#define I2C_SLTH_REG(base)                       ((base)->SLTH)
#define I2C_SLTL_REG(base)                       ((base)->SLTL)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/* A1 Bit Fields */
#define I2C_A1_AD_MASK                           0xFEu
#define I2C_A1_AD_SHIFT                          1
#define I2C_A1_AD_WIDTH                          7
#define I2C_A1_AD(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_A1_AD_SHIFT))&I2C_A1_AD_MASK)
/* F Bit Fields */
#define I2C_F_ICR_MASK                           0x3Fu
#define I2C_F_ICR_SHIFT                          0
#define I2C_F_ICR_WIDTH                          6
#define I2C_F_ICR(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_F_ICR_SHIFT))&I2C_F_ICR_MASK)
#define I2C_F_MULT_MASK                          0xC0u
#define I2C_F_MULT_SHIFT                         6
#define I2C_F_MULT_WIDTH                         2
#define I2C_F_MULT(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_F_MULT_SHIFT))&I2C_F_MULT_MASK)
/* C1 Bit Fields */
#define I2C_C1_DMAEN_MASK                        0x1u
#define I2C_C1_DMAEN_SHIFT                       0
#define I2C_C1_DMAEN_WIDTH                       1
#define I2C_C1_DMAEN(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_C1_DMAEN_SHIFT))&I2C_C1_DMAEN_MASK)
#define I2C_C1_WUEN_MASK                         0x2u
#define I2C_C1_WUEN_SHIFT                        1
#define I2C_C1_WUEN_WIDTH                        1
#define I2C_C1_WUEN(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C1_WUEN_SHIFT))&I2C_C1_WUEN_MASK)
#define I2C_C1_RSTA_MASK                         0x4u
#define I2C_C1_RSTA_SHIFT                        2
#define I2C_C1_RSTA_WIDTH                        1
#define I2C_C1_RSTA(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C1_RSTA_SHIFT))&I2C_C1_RSTA_MASK)
#define I2C_C1_TXAK_MASK                         0x8u
#define I2C_C1_TXAK_SHIFT                        3
#define I2C_C1_TXAK_WIDTH                        1
#define I2C_C1_TXAK(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C1_TXAK_SHIFT))&I2C_C1_TXAK_MASK)
#define I2C_C1_TX_MASK                           0x10u
#define I2C_C1_TX_SHIFT                          4
#define I2C_C1_TX_WIDTH                          1
#define I2C_C1_TX(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_C1_TX_SHIFT))&I2C_C1_TX_MASK)
#define I2C_C1_MST_MASK                          0x20u
#define I2C_C1_MST_SHIFT                         5
#define I2C_C1_MST_WIDTH                         1
#define I2C_C1_MST(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_C1_MST_SHIFT))&I2C_C1_MST_MASK)
#define I2C_C1_IICIE_MASK                        0x40u
#define I2C_C1_IICIE_SHIFT                       6
#define I2C_C1_IICIE_WIDTH                       1
#define I2C_C1_IICIE(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_C1_IICIE_SHIFT))&I2C_C1_IICIE_MASK)
#define I2C_C1_IICEN_MASK                        0x80u
#define I2C_C1_IICEN_SHIFT                       7
#define I2C_C1_IICEN_WIDTH                       1
#define I2C_C1_IICEN(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_C1_IICEN_SHIFT))&I2C_C1_IICEN_MASK)
/* S Bit Fields */
#define I2C_S_RXAK_MASK                          0x1u
#define I2C_S_RXAK_SHIFT                         0
#define I2C_S_RXAK_WIDTH                         1
#define I2C_S_RXAK(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_S_RXAK_SHIFT))&I2C_S_RXAK_MASK)
#define I2C_S_IICIF_MASK                         0x2u
#define I2C_S_IICIF_SHIFT                        1
#define I2C_S_IICIF_WIDTH                        1
#define I2C_S_IICIF(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_S_IICIF_SHIFT))&I2C_S_IICIF_MASK)
#define I2C_S_SRW_MASK                           0x4u
#define I2C_S_SRW_SHIFT                          2
#define I2C_S_SRW_WIDTH                          1
#define I2C_S_SRW(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_S_SRW_SHIFT))&I2C_S_SRW_MASK)
#define I2C_S_RAM_MASK                           0x8u
#define I2C_S_RAM_SHIFT                          3
#define I2C_S_RAM_WIDTH                          1
#define I2C_S_RAM(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_S_RAM_SHIFT))&I2C_S_RAM_MASK)
#define I2C_S_ARBL_MASK                          0x10u
#define I2C_S_ARBL_SHIFT                         4
#define I2C_S_ARBL_WIDTH                         1
#define I2C_S_ARBL(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_S_ARBL_SHIFT))&I2C_S_ARBL_MASK)
#define I2C_S_BUSY_MASK                          0x20u
#define I2C_S_BUSY_SHIFT                         5
#define I2C_S_BUSY_WIDTH                         1
#define I2C_S_BUSY(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_S_BUSY_SHIFT))&I2C_S_BUSY_MASK)
#define I2C_S_IAAS_MASK                          0x40u
#define I2C_S_IAAS_SHIFT                         6
#define I2C_S_IAAS_WIDTH                         1
#define I2C_S_IAAS(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_S_IAAS_SHIFT))&I2C_S_IAAS_MASK)
#define I2C_S_TCF_MASK                           0x80u
#define I2C_S_TCF_SHIFT                          7
#define I2C_S_TCF_WIDTH                          1
#define I2C_S_TCF(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_S_TCF_SHIFT))&I2C_S_TCF_MASK)
/* D Bit Fields */
#define I2C_D_DATA_MASK                          0xFFu
#define I2C_D_DATA_SHIFT                         0
#define I2C_D_DATA_WIDTH                         8
#define I2C_D_DATA(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_D_DATA_SHIFT))&I2C_D_DATA_MASK)
/* C2 Bit Fields */
#define I2C_C2_AD_MASK                           0x7u
#define I2C_C2_AD_SHIFT                          0
#define I2C_C2_AD_WIDTH                          3
#define I2C_C2_AD(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_C2_AD_SHIFT))&I2C_C2_AD_MASK)
#define I2C_C2_RMEN_MASK                         0x8u
#define I2C_C2_RMEN_SHIFT                        3
#define I2C_C2_RMEN_WIDTH                        1
#define I2C_C2_RMEN(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C2_RMEN_SHIFT))&I2C_C2_RMEN_MASK)
#define I2C_C2_SBRC_MASK                         0x10u
#define I2C_C2_SBRC_SHIFT                        4
#define I2C_C2_SBRC_WIDTH                        1
#define I2C_C2_SBRC(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C2_SBRC_SHIFT))&I2C_C2_SBRC_MASK)
#define I2C_C2_HDRS_MASK                         0x20u
#define I2C_C2_HDRS_SHIFT                        5
#define I2C_C2_HDRS_WIDTH                        1
#define I2C_C2_HDRS(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_C2_HDRS_SHIFT))&I2C_C2_HDRS_MASK)
#define I2C_C2_ADEXT_MASK                        0x40u
#define I2C_C2_ADEXT_SHIFT                       6
#define I2C_C2_ADEXT_WIDTH                       1
#define I2C_C2_ADEXT(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_C2_ADEXT_SHIFT))&I2C_C2_ADEXT_MASK)
#define I2C_C2_GCAEN_MASK                        0x80u
#define I2C_C2_GCAEN_SHIFT                       7
#define I2C_C2_GCAEN_WIDTH                       1
#define I2C_C2_GCAEN(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_C2_GCAEN_SHIFT))&I2C_C2_GCAEN_MASK)
/* FLT Bit Fields */
#define I2C_FLT_FLT_MASK                         0xFu
#define I2C_FLT_FLT_SHIFT                        0
#define I2C_FLT_FLT_WIDTH                        4
#define I2C_FLT_FLT(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_FLT_FLT_SHIFT))&I2C_FLT_FLT_MASK)
#define I2C_FLT_STARTF_MASK                      0x10u
#define I2C_FLT_STARTF_SHIFT                     4
#define I2C_FLT_STARTF_WIDTH                     1
#define I2C_FLT_STARTF(x)                        (((uint8_t)(((uint8_t)(x))<<I2C_FLT_STARTF_SHIFT))&I2C_FLT_STARTF_MASK)
#define I2C_FLT_SSIE_MASK                        0x20u
#define I2C_FLT_SSIE_SHIFT                       5
#define I2C_FLT_SSIE_WIDTH                       1
#define I2C_FLT_SSIE(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_FLT_SSIE_SHIFT))&I2C_FLT_SSIE_MASK)
#define I2C_FLT_STOPF_MASK                       0x40u
#define I2C_FLT_STOPF_SHIFT                      6
#define I2C_FLT_STOPF_WIDTH                      1
#define I2C_FLT_STOPF(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_FLT_STOPF_SHIFT))&I2C_FLT_STOPF_MASK)
#define I2C_FLT_SHEN_MASK                        0x80u
#define I2C_FLT_SHEN_SHIFT                       7
#define I2C_FLT_SHEN_WIDTH                       1
#define I2C_FLT_SHEN(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_FLT_SHEN_SHIFT))&I2C_FLT_SHEN_MASK)
/* RA Bit Fields */
#define I2C_RA_RAD_MASK                          0xFEu
#define I2C_RA_RAD_SHIFT                         1
#define I2C_RA_RAD_WIDTH                         7
#define I2C_RA_RAD(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_RA_RAD_SHIFT))&I2C_RA_RAD_MASK)
/* SMB Bit Fields */
#define I2C_SMB_SHTF2IE_MASK                     0x1u
#define I2C_SMB_SHTF2IE_SHIFT                    0
#define I2C_SMB_SHTF2IE_WIDTH                    1
#define I2C_SMB_SHTF2IE(x)                       (((uint8_t)(((uint8_t)(x))<<I2C_SMB_SHTF2IE_SHIFT))&I2C_SMB_SHTF2IE_MASK)
#define I2C_SMB_SHTF2_MASK                       0x2u
#define I2C_SMB_SHTF2_SHIFT                      1
#define I2C_SMB_SHTF2_WIDTH                      1
#define I2C_SMB_SHTF2(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SMB_SHTF2_SHIFT))&I2C_SMB_SHTF2_MASK)
#define I2C_SMB_SHTF1_MASK                       0x4u
#define I2C_SMB_SHTF1_SHIFT                      2
#define I2C_SMB_SHTF1_WIDTH                      1
#define I2C_SMB_SHTF1(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SMB_SHTF1_SHIFT))&I2C_SMB_SHTF1_MASK)
#define I2C_SMB_SLTF_MASK                        0x8u
#define I2C_SMB_SLTF_SHIFT                       3
#define I2C_SMB_SLTF_WIDTH                       1
#define I2C_SMB_SLTF(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_SMB_SLTF_SHIFT))&I2C_SMB_SLTF_MASK)
#define I2C_SMB_TCKSEL_MASK                      0x10u
#define I2C_SMB_TCKSEL_SHIFT                     4
#define I2C_SMB_TCKSEL_WIDTH                     1
#define I2C_SMB_TCKSEL(x)                        (((uint8_t)(((uint8_t)(x))<<I2C_SMB_TCKSEL_SHIFT))&I2C_SMB_TCKSEL_MASK)
#define I2C_SMB_SIICAEN_MASK                     0x20u
#define I2C_SMB_SIICAEN_SHIFT                    5
#define I2C_SMB_SIICAEN_WIDTH                    1
#define I2C_SMB_SIICAEN(x)                       (((uint8_t)(((uint8_t)(x))<<I2C_SMB_SIICAEN_SHIFT))&I2C_SMB_SIICAEN_MASK)
#define I2C_SMB_ALERTEN_MASK                     0x40u
#define I2C_SMB_ALERTEN_SHIFT                    6
#define I2C_SMB_ALERTEN_WIDTH                    1
#define I2C_SMB_ALERTEN(x)                       (((uint8_t)(((uint8_t)(x))<<I2C_SMB_ALERTEN_SHIFT))&I2C_SMB_ALERTEN_MASK)
#define I2C_SMB_FACK_MASK                        0x80u
#define I2C_SMB_FACK_SHIFT                       7
#define I2C_SMB_FACK_WIDTH                       1
#define I2C_SMB_FACK(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_SMB_FACK_SHIFT))&I2C_SMB_FACK_MASK)
/* A2 Bit Fields */
#define I2C_A2_SAD_MASK                          0xFEu
#define I2C_A2_SAD_SHIFT                         1
#define I2C_A2_SAD_WIDTH                         7
#define I2C_A2_SAD(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_A2_SAD_SHIFT))&I2C_A2_SAD_MASK)
/* SLTH Bit Fields */
#define I2C_SLTH_SSLT_MASK                       0xFFu
#define I2C_SLTH_SSLT_SHIFT                      0
#define I2C_SLTH_SSLT_WIDTH                      8
#define I2C_SLTH_SSLT(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SLTH_SSLT_SHIFT))&I2C_SLTH_SSLT_MASK)
/* SLTL Bit Fields */
#define I2C_SLTL_SSLT_MASK                       0xFFu
#define I2C_SLTL_SSLT_SHIFT                      0
#define I2C_SLTL_SSLT_WIDTH                      8
#define I2C_SLTL_SSLT(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SLTL_SSLT_SHIFT))&I2C_SLTL_SSLT_MASK)

/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE                                (0x40066000u)
/** Peripheral I2C0 base pointer */
#define I2C0                                     ((I2C_Type *)I2C0_BASE)
#define I2C0_BASE_PTR                            (I2C0)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0 }

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register instance definitions */
/* I2C0 */
#define I2C0_A1                                  I2C_A1_REG(I2C0)
#define I2C0_F                                   I2C_F_REG(I2C0)
#define I2C0_C1                                  I2C_C1_REG(I2C0)
#define I2C0_S                                   I2C_S_REG(I2C0)
#define I2C0_D                                   I2C_D_REG(I2C0)
#define I2C0_C2                                  I2C_C2_REG(I2C0)
#define I2C0_FLT                                 I2C_FLT_REG(I2C0)
#define I2C0_RA                                  I2C_RA_REG(I2C0)
#define I2C0_SMB                                 I2C_SMB_REG(I2C0)
#define I2C0_A2                                  I2C_A2_REG(I2C0)
#define I2C0_SLTH                                I2C_SLTH_REG(I2C0)
#define I2C0_SLTL                                I2C_SLTL_REG(I2C0)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LLWU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Peripheral_Access_Layer LLWU Peripheral Access Layer
 * @{
 */

/** LLWU - Register Layout Typedef */
typedef struct {
  __IO uint8_t PE1;                                /**< LLWU Pin Enable 1 register, offset: 0x0 */
  __IO uint8_t PE2;                                /**< LLWU Pin Enable 2 register, offset: 0x1 */
  __IO uint8_t PE3;                                /**< LLWU Pin Enable 3 register, offset: 0x2 */
  __IO uint8_t PE4;                                /**< LLWU Pin Enable 4 register, offset: 0x3 */
  __IO uint8_t PE5;                                /**< LLWU Pin Enable 5 register, offset: 0x4 */
  __IO uint8_t PE6;                                /**< LLWU Pin Enable 6 register, offset: 0x5 */
  __IO uint8_t PE7;                                /**< LLWU Pin Enable 7 register, offset: 0x6 */
  __IO uint8_t PE8;                                /**< LLWU Pin Enable 8 register, offset: 0x7 */
  __IO uint8_t ME;                                 /**< LLWU Module Enable register, offset: 0x8 */
  __IO uint8_t PF1;                                /**< LLWU Pin Flag 1 register, offset: 0x9 */
  __IO uint8_t PF2;                                /**< LLWU Pin Flag 2 register, offset: 0xA */
  __IO uint8_t PF3;                                /**< LLWU Pin Flag 3 register, offset: 0xB */
  __IO uint8_t PF4;                                /**< LLWU Pin Flag 4 register, offset: 0xC */
  __I  uint8_t MF5;                                /**< LLWU Module Flag 5 register, offset: 0xD */
  __IO uint8_t FILT1;                              /**< LLWU Pin Filter 1 register, offset: 0xE */
  __IO uint8_t FILT2;                              /**< LLWU Pin Filter 2 register, offset: 0xF */
} LLWU_Type, *LLWU_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LLWU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Accessor_Macros LLWU - Register accessor macros
 * @{
 */


/* LLWU - Register accessors */
#define LLWU_PE1_REG(base)                       ((base)->PE1)
#define LLWU_PE2_REG(base)                       ((base)->PE2)
#define LLWU_PE3_REG(base)                       ((base)->PE3)
#define LLWU_PE4_REG(base)                       ((base)->PE4)
#define LLWU_PE5_REG(base)                       ((base)->PE5)
#define LLWU_PE6_REG(base)                       ((base)->PE6)
#define LLWU_PE7_REG(base)                       ((base)->PE7)
#define LLWU_PE8_REG(base)                       ((base)->PE8)
#define LLWU_ME_REG(base)                        ((base)->ME)
#define LLWU_PF1_REG(base)                       ((base)->PF1)
#define LLWU_PF2_REG(base)                       ((base)->PF2)
#define LLWU_PF3_REG(base)                       ((base)->PF3)
#define LLWU_PF4_REG(base)                       ((base)->PF4)
#define LLWU_MF5_REG(base)                       ((base)->MF5)
#define LLWU_FILT1_REG(base)                     ((base)->FILT1)
#define LLWU_FILT2_REG(base)                     ((base)->FILT2)

/*!
 * @}
 */ /* end of group LLWU_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LLWU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Masks LLWU Register Masks
 * @{
 */

/* PE1 Bit Fields */
#define LLWU_PE1_WUPE0_MASK                      0x3u
#define LLWU_PE1_WUPE0_SHIFT                     0
#define LLWU_PE1_WUPE0_WIDTH                     2
#define LLWU_PE1_WUPE0(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE0_SHIFT))&LLWU_PE1_WUPE0_MASK)
#define LLWU_PE1_WUPE1_MASK                      0xCu
#define LLWU_PE1_WUPE1_SHIFT                     2
#define LLWU_PE1_WUPE1_WIDTH                     2
#define LLWU_PE1_WUPE1(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE1_SHIFT))&LLWU_PE1_WUPE1_MASK)
#define LLWU_PE1_WUPE2_MASK                      0x30u
#define LLWU_PE1_WUPE2_SHIFT                     4
#define LLWU_PE1_WUPE2_WIDTH                     2
#define LLWU_PE1_WUPE2(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE2_SHIFT))&LLWU_PE1_WUPE2_MASK)
#define LLWU_PE1_WUPE3_MASK                      0xC0u
#define LLWU_PE1_WUPE3_SHIFT                     6
#define LLWU_PE1_WUPE3_WIDTH                     2
#define LLWU_PE1_WUPE3(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE3_SHIFT))&LLWU_PE1_WUPE3_MASK)
/* PE2 Bit Fields */
#define LLWU_PE2_WUPE4_MASK                      0x3u
#define LLWU_PE2_WUPE4_SHIFT                     0
#define LLWU_PE2_WUPE4_WIDTH                     2
#define LLWU_PE2_WUPE4(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE4_SHIFT))&LLWU_PE2_WUPE4_MASK)
#define LLWU_PE2_WUPE5_MASK                      0xCu
#define LLWU_PE2_WUPE5_SHIFT                     2
#define LLWU_PE2_WUPE5_WIDTH                     2
#define LLWU_PE2_WUPE5(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE5_SHIFT))&LLWU_PE2_WUPE5_MASK)
#define LLWU_PE2_WUPE6_MASK                      0x30u
#define LLWU_PE2_WUPE6_SHIFT                     4
#define LLWU_PE2_WUPE6_WIDTH                     2
#define LLWU_PE2_WUPE6(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE6_SHIFT))&LLWU_PE2_WUPE6_MASK)
#define LLWU_PE2_WUPE7_MASK                      0xC0u
#define LLWU_PE2_WUPE7_SHIFT                     6
#define LLWU_PE2_WUPE7_WIDTH                     2
#define LLWU_PE2_WUPE7(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE7_SHIFT))&LLWU_PE2_WUPE7_MASK)
/* PE3 Bit Fields */
#define LLWU_PE3_WUPE8_MASK                      0x3u
#define LLWU_PE3_WUPE8_SHIFT                     0
#define LLWU_PE3_WUPE8_WIDTH                     2
#define LLWU_PE3_WUPE8(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE8_SHIFT))&LLWU_PE3_WUPE8_MASK)
#define LLWU_PE3_WUPE9_MASK                      0xCu
#define LLWU_PE3_WUPE9_SHIFT                     2
#define LLWU_PE3_WUPE9_WIDTH                     2
#define LLWU_PE3_WUPE9(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE9_SHIFT))&LLWU_PE3_WUPE9_MASK)
#define LLWU_PE3_WUPE10_MASK                     0x30u
#define LLWU_PE3_WUPE10_SHIFT                    4
#define LLWU_PE3_WUPE10_WIDTH                    2
#define LLWU_PE3_WUPE10(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE10_SHIFT))&LLWU_PE3_WUPE10_MASK)
#define LLWU_PE3_WUPE11_MASK                     0xC0u
#define LLWU_PE3_WUPE11_SHIFT                    6
#define LLWU_PE3_WUPE11_WIDTH                    2
#define LLWU_PE3_WUPE11(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE11_SHIFT))&LLWU_PE3_WUPE11_MASK)
/* PE4 Bit Fields */
#define LLWU_PE4_WUPE12_MASK                     0x3u
#define LLWU_PE4_WUPE12_SHIFT                    0
#define LLWU_PE4_WUPE12_WIDTH                    2
#define LLWU_PE4_WUPE12(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE12_SHIFT))&LLWU_PE4_WUPE12_MASK)
#define LLWU_PE4_WUPE13_MASK                     0xCu
#define LLWU_PE4_WUPE13_SHIFT                    2
#define LLWU_PE4_WUPE13_WIDTH                    2
#define LLWU_PE4_WUPE13(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE13_SHIFT))&LLWU_PE4_WUPE13_MASK)
#define LLWU_PE4_WUPE14_MASK                     0x30u
#define LLWU_PE4_WUPE14_SHIFT                    4
#define LLWU_PE4_WUPE14_WIDTH                    2
#define LLWU_PE4_WUPE14(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE14_SHIFT))&LLWU_PE4_WUPE14_MASK)
#define LLWU_PE4_WUPE15_MASK                     0xC0u
#define LLWU_PE4_WUPE15_SHIFT                    6
#define LLWU_PE4_WUPE15_WIDTH                    2
#define LLWU_PE4_WUPE15(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE15_SHIFT))&LLWU_PE4_WUPE15_MASK)
/* PE5 Bit Fields */
#define LLWU_PE5_WUPE16_MASK                     0x3u
#define LLWU_PE5_WUPE16_SHIFT                    0
#define LLWU_PE5_WUPE16_WIDTH                    2
#define LLWU_PE5_WUPE16(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE5_WUPE16_SHIFT))&LLWU_PE5_WUPE16_MASK)
#define LLWU_PE5_WUPE17_MASK                     0xCu
#define LLWU_PE5_WUPE17_SHIFT                    2
#define LLWU_PE5_WUPE17_WIDTH                    2
#define LLWU_PE5_WUPE17(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE5_WUPE17_SHIFT))&LLWU_PE5_WUPE17_MASK)
#define LLWU_PE5_WUPE18_MASK                     0x30u
#define LLWU_PE5_WUPE18_SHIFT                    4
#define LLWU_PE5_WUPE18_WIDTH                    2
#define LLWU_PE5_WUPE18(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE5_WUPE18_SHIFT))&LLWU_PE5_WUPE18_MASK)
#define LLWU_PE5_WUPE19_MASK                     0xC0u
#define LLWU_PE5_WUPE19_SHIFT                    6
#define LLWU_PE5_WUPE19_WIDTH                    2
#define LLWU_PE5_WUPE19(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE5_WUPE19_SHIFT))&LLWU_PE5_WUPE19_MASK)
/* PE6 Bit Fields */
#define LLWU_PE6_WUPE20_MASK                     0x3u
#define LLWU_PE6_WUPE20_SHIFT                    0
#define LLWU_PE6_WUPE20_WIDTH                    2
#define LLWU_PE6_WUPE20(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE6_WUPE20_SHIFT))&LLWU_PE6_WUPE20_MASK)
#define LLWU_PE6_WUPE21_MASK                     0xCu
#define LLWU_PE6_WUPE21_SHIFT                    2
#define LLWU_PE6_WUPE21_WIDTH                    2
#define LLWU_PE6_WUPE21(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE6_WUPE21_SHIFT))&LLWU_PE6_WUPE21_MASK)
#define LLWU_PE6_WUPE22_MASK                     0x30u
#define LLWU_PE6_WUPE22_SHIFT                    4
#define LLWU_PE6_WUPE22_WIDTH                    2
#define LLWU_PE6_WUPE22(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE6_WUPE22_SHIFT))&LLWU_PE6_WUPE22_MASK)
#define LLWU_PE6_WUPE23_MASK                     0xC0u
#define LLWU_PE6_WUPE23_SHIFT                    6
#define LLWU_PE6_WUPE23_WIDTH                    2
#define LLWU_PE6_WUPE23(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE6_WUPE23_SHIFT))&LLWU_PE6_WUPE23_MASK)
/* PE7 Bit Fields */
#define LLWU_PE7_WUPE24_MASK                     0x3u
#define LLWU_PE7_WUPE24_SHIFT                    0
#define LLWU_PE7_WUPE24_WIDTH                    2
#define LLWU_PE7_WUPE24(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE7_WUPE24_SHIFT))&LLWU_PE7_WUPE24_MASK)
#define LLWU_PE7_WUPE25_MASK                     0xCu
#define LLWU_PE7_WUPE25_SHIFT                    2
#define LLWU_PE7_WUPE25_WIDTH                    2
#define LLWU_PE7_WUPE25(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE7_WUPE25_SHIFT))&LLWU_PE7_WUPE25_MASK)
#define LLWU_PE7_WUPE26_MASK                     0x30u
#define LLWU_PE7_WUPE26_SHIFT                    4
#define LLWU_PE7_WUPE26_WIDTH                    2
#define LLWU_PE7_WUPE26(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE7_WUPE26_SHIFT))&LLWU_PE7_WUPE26_MASK)
#define LLWU_PE7_WUPE27_MASK                     0xC0u
#define LLWU_PE7_WUPE27_SHIFT                    6
#define LLWU_PE7_WUPE27_WIDTH                    2
#define LLWU_PE7_WUPE27(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE7_WUPE27_SHIFT))&LLWU_PE7_WUPE27_MASK)
/* PE8 Bit Fields */
#define LLWU_PE8_WUPE28_MASK                     0x3u
#define LLWU_PE8_WUPE28_SHIFT                    0
#define LLWU_PE8_WUPE28_WIDTH                    2
#define LLWU_PE8_WUPE28(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE8_WUPE28_SHIFT))&LLWU_PE8_WUPE28_MASK)
#define LLWU_PE8_WUPE29_MASK                     0xCu
#define LLWU_PE8_WUPE29_SHIFT                    2
#define LLWU_PE8_WUPE29_WIDTH                    2
#define LLWU_PE8_WUPE29(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE8_WUPE29_SHIFT))&LLWU_PE8_WUPE29_MASK)
#define LLWU_PE8_WUPE30_MASK                     0x30u
#define LLWU_PE8_WUPE30_SHIFT                    4
#define LLWU_PE8_WUPE30_WIDTH                    2
#define LLWU_PE8_WUPE30(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE8_WUPE30_SHIFT))&LLWU_PE8_WUPE30_MASK)
#define LLWU_PE8_WUPE31_MASK                     0xC0u
#define LLWU_PE8_WUPE31_SHIFT                    6
#define LLWU_PE8_WUPE31_WIDTH                    2
#define LLWU_PE8_WUPE31(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE8_WUPE31_SHIFT))&LLWU_PE8_WUPE31_MASK)
/* ME Bit Fields */
#define LLWU_ME_WUME0_MASK                       0x1u
#define LLWU_ME_WUME0_SHIFT                      0
#define LLWU_ME_WUME0_WIDTH                      1
#define LLWU_ME_WUME0(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME0_SHIFT))&LLWU_ME_WUME0_MASK)
#define LLWU_ME_WUME1_MASK                       0x2u
#define LLWU_ME_WUME1_SHIFT                      1
#define LLWU_ME_WUME1_WIDTH                      1
#define LLWU_ME_WUME1(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME1_SHIFT))&LLWU_ME_WUME1_MASK)
#define LLWU_ME_WUME2_MASK                       0x4u
#define LLWU_ME_WUME2_SHIFT                      2
#define LLWU_ME_WUME2_WIDTH                      1
#define LLWU_ME_WUME2(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME2_SHIFT))&LLWU_ME_WUME2_MASK)
#define LLWU_ME_WUME3_MASK                       0x8u
#define LLWU_ME_WUME3_SHIFT                      3
#define LLWU_ME_WUME3_WIDTH                      1
#define LLWU_ME_WUME3(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME3_SHIFT))&LLWU_ME_WUME3_MASK)
#define LLWU_ME_WUME4_MASK                       0x10u
#define LLWU_ME_WUME4_SHIFT                      4
#define LLWU_ME_WUME4_WIDTH                      1
#define LLWU_ME_WUME4(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME4_SHIFT))&LLWU_ME_WUME4_MASK)
#define LLWU_ME_WUME5_MASK                       0x20u
#define LLWU_ME_WUME5_SHIFT                      5
#define LLWU_ME_WUME5_WIDTH                      1
#define LLWU_ME_WUME5(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME5_SHIFT))&LLWU_ME_WUME5_MASK)
#define LLWU_ME_WUME6_MASK                       0x40u
#define LLWU_ME_WUME6_SHIFT                      6
#define LLWU_ME_WUME6_WIDTH                      1
#define LLWU_ME_WUME6(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME6_SHIFT))&LLWU_ME_WUME6_MASK)
#define LLWU_ME_WUME7_MASK                       0x80u
#define LLWU_ME_WUME7_SHIFT                      7
#define LLWU_ME_WUME7_WIDTH                      1
#define LLWU_ME_WUME7(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_ME_WUME7_SHIFT))&LLWU_ME_WUME7_MASK)
/* PF1 Bit Fields */
#define LLWU_PF1_WUF0_MASK                       0x1u
#define LLWU_PF1_WUF0_SHIFT                      0
#define LLWU_PF1_WUF0_WIDTH                      1
#define LLWU_PF1_WUF0(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF0_SHIFT))&LLWU_PF1_WUF0_MASK)
#define LLWU_PF1_WUF1_MASK                       0x2u
#define LLWU_PF1_WUF1_SHIFT                      1
#define LLWU_PF1_WUF1_WIDTH                      1
#define LLWU_PF1_WUF1(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF1_SHIFT))&LLWU_PF1_WUF1_MASK)
#define LLWU_PF1_WUF2_MASK                       0x4u
#define LLWU_PF1_WUF2_SHIFT                      2
#define LLWU_PF1_WUF2_WIDTH                      1
#define LLWU_PF1_WUF2(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF2_SHIFT))&LLWU_PF1_WUF2_MASK)
#define LLWU_PF1_WUF3_MASK                       0x8u
#define LLWU_PF1_WUF3_SHIFT                      3
#define LLWU_PF1_WUF3_WIDTH                      1
#define LLWU_PF1_WUF3(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF3_SHIFT))&LLWU_PF1_WUF3_MASK)
#define LLWU_PF1_WUF4_MASK                       0x10u
#define LLWU_PF1_WUF4_SHIFT                      4
#define LLWU_PF1_WUF4_WIDTH                      1
#define LLWU_PF1_WUF4(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF4_SHIFT))&LLWU_PF1_WUF4_MASK)
#define LLWU_PF1_WUF5_MASK                       0x20u
#define LLWU_PF1_WUF5_SHIFT                      5
#define LLWU_PF1_WUF5_WIDTH                      1
#define LLWU_PF1_WUF5(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF5_SHIFT))&LLWU_PF1_WUF5_MASK)
#define LLWU_PF1_WUF6_MASK                       0x40u
#define LLWU_PF1_WUF6_SHIFT                      6
#define LLWU_PF1_WUF6_WIDTH                      1
#define LLWU_PF1_WUF6(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF6_SHIFT))&LLWU_PF1_WUF6_MASK)
#define LLWU_PF1_WUF7_MASK                       0x80u
#define LLWU_PF1_WUF7_SHIFT                      7
#define LLWU_PF1_WUF7_WIDTH                      1
#define LLWU_PF1_WUF7(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF1_WUF7_SHIFT))&LLWU_PF1_WUF7_MASK)
/* PF2 Bit Fields */
#define LLWU_PF2_WUF8_MASK                       0x1u
#define LLWU_PF2_WUF8_SHIFT                      0
#define LLWU_PF2_WUF8_WIDTH                      1
#define LLWU_PF2_WUF8(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF8_SHIFT))&LLWU_PF2_WUF8_MASK)
#define LLWU_PF2_WUF9_MASK                       0x2u
#define LLWU_PF2_WUF9_SHIFT                      1
#define LLWU_PF2_WUF9_WIDTH                      1
#define LLWU_PF2_WUF9(x)                         (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF9_SHIFT))&LLWU_PF2_WUF9_MASK)
#define LLWU_PF2_WUF10_MASK                      0x4u
#define LLWU_PF2_WUF10_SHIFT                     2
#define LLWU_PF2_WUF10_WIDTH                     1
#define LLWU_PF2_WUF10(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF10_SHIFT))&LLWU_PF2_WUF10_MASK)
#define LLWU_PF2_WUF11_MASK                      0x8u
#define LLWU_PF2_WUF11_SHIFT                     3
#define LLWU_PF2_WUF11_WIDTH                     1
#define LLWU_PF2_WUF11(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF11_SHIFT))&LLWU_PF2_WUF11_MASK)
#define LLWU_PF2_WUF12_MASK                      0x10u
#define LLWU_PF2_WUF12_SHIFT                     4
#define LLWU_PF2_WUF12_WIDTH                     1
#define LLWU_PF2_WUF12(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF12_SHIFT))&LLWU_PF2_WUF12_MASK)
#define LLWU_PF2_WUF13_MASK                      0x20u
#define LLWU_PF2_WUF13_SHIFT                     5
#define LLWU_PF2_WUF13_WIDTH                     1
#define LLWU_PF2_WUF13(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF13_SHIFT))&LLWU_PF2_WUF13_MASK)
#define LLWU_PF2_WUF14_MASK                      0x40u
#define LLWU_PF2_WUF14_SHIFT                     6
#define LLWU_PF2_WUF14_WIDTH                     1
#define LLWU_PF2_WUF14(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF14_SHIFT))&LLWU_PF2_WUF14_MASK)
#define LLWU_PF2_WUF15_MASK                      0x80u
#define LLWU_PF2_WUF15_SHIFT                     7
#define LLWU_PF2_WUF15_WIDTH                     1
#define LLWU_PF2_WUF15(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF2_WUF15_SHIFT))&LLWU_PF2_WUF15_MASK)
/* PF3 Bit Fields */
#define LLWU_PF3_WUF16_MASK                      0x1u
#define LLWU_PF3_WUF16_SHIFT                     0
#define LLWU_PF3_WUF16_WIDTH                     1
#define LLWU_PF3_WUF16(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF16_SHIFT))&LLWU_PF3_WUF16_MASK)
#define LLWU_PF3_WUF17_MASK                      0x2u
#define LLWU_PF3_WUF17_SHIFT                     1
#define LLWU_PF3_WUF17_WIDTH                     1
#define LLWU_PF3_WUF17(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF17_SHIFT))&LLWU_PF3_WUF17_MASK)
#define LLWU_PF3_WUF18_MASK                      0x4u
#define LLWU_PF3_WUF18_SHIFT                     2
#define LLWU_PF3_WUF18_WIDTH                     1
#define LLWU_PF3_WUF18(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF18_SHIFT))&LLWU_PF3_WUF18_MASK)
#define LLWU_PF3_WUF19_MASK                      0x8u
#define LLWU_PF3_WUF19_SHIFT                     3
#define LLWU_PF3_WUF19_WIDTH                     1
#define LLWU_PF3_WUF19(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF19_SHIFT))&LLWU_PF3_WUF19_MASK)
#define LLWU_PF3_WUF20_MASK                      0x10u
#define LLWU_PF3_WUF20_SHIFT                     4
#define LLWU_PF3_WUF20_WIDTH                     1
#define LLWU_PF3_WUF20(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF20_SHIFT))&LLWU_PF3_WUF20_MASK)
#define LLWU_PF3_WUF21_MASK                      0x20u
#define LLWU_PF3_WUF21_SHIFT                     5
#define LLWU_PF3_WUF21_WIDTH                     1
#define LLWU_PF3_WUF21(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF21_SHIFT))&LLWU_PF3_WUF21_MASK)
#define LLWU_PF3_WUF22_MASK                      0x40u
#define LLWU_PF3_WUF22_SHIFT                     6
#define LLWU_PF3_WUF22_WIDTH                     1
#define LLWU_PF3_WUF22(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF22_SHIFT))&LLWU_PF3_WUF22_MASK)
#define LLWU_PF3_WUF23_MASK                      0x80u
#define LLWU_PF3_WUF23_SHIFT                     7
#define LLWU_PF3_WUF23_WIDTH                     1
#define LLWU_PF3_WUF23(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF3_WUF23_SHIFT))&LLWU_PF3_WUF23_MASK)
/* PF4 Bit Fields */
#define LLWU_PF4_WUF24_MASK                      0x1u
#define LLWU_PF4_WUF24_SHIFT                     0
#define LLWU_PF4_WUF24_WIDTH                     1
#define LLWU_PF4_WUF24(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF24_SHIFT))&LLWU_PF4_WUF24_MASK)
#define LLWU_PF4_WUF25_MASK                      0x2u
#define LLWU_PF4_WUF25_SHIFT                     1
#define LLWU_PF4_WUF25_WIDTH                     1
#define LLWU_PF4_WUF25(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF25_SHIFT))&LLWU_PF4_WUF25_MASK)
#define LLWU_PF4_WUF26_MASK                      0x4u
#define LLWU_PF4_WUF26_SHIFT                     2
#define LLWU_PF4_WUF26_WIDTH                     1
#define LLWU_PF4_WUF26(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF26_SHIFT))&LLWU_PF4_WUF26_MASK)
#define LLWU_PF4_WUF27_MASK                      0x8u
#define LLWU_PF4_WUF27_SHIFT                     3
#define LLWU_PF4_WUF27_WIDTH                     1
#define LLWU_PF4_WUF27(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF27_SHIFT))&LLWU_PF4_WUF27_MASK)
#define LLWU_PF4_WUF28_MASK                      0x10u
#define LLWU_PF4_WUF28_SHIFT                     4
#define LLWU_PF4_WUF28_WIDTH                     1
#define LLWU_PF4_WUF28(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF28_SHIFT))&LLWU_PF4_WUF28_MASK)
#define LLWU_PF4_WUF29_MASK                      0x20u
#define LLWU_PF4_WUF29_SHIFT                     5
#define LLWU_PF4_WUF29_WIDTH                     1
#define LLWU_PF4_WUF29(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF29_SHIFT))&LLWU_PF4_WUF29_MASK)
#define LLWU_PF4_WUF30_MASK                      0x40u
#define LLWU_PF4_WUF30_SHIFT                     6
#define LLWU_PF4_WUF30_WIDTH                     1
#define LLWU_PF4_WUF30(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF30_SHIFT))&LLWU_PF4_WUF30_MASK)
#define LLWU_PF4_WUF31_MASK                      0x80u
#define LLWU_PF4_WUF31_SHIFT                     7
#define LLWU_PF4_WUF31_WIDTH                     1
#define LLWU_PF4_WUF31(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PF4_WUF31_SHIFT))&LLWU_PF4_WUF31_MASK)
/* MF5 Bit Fields */
#define LLWU_MF5_MWUF0_MASK                      0x1u
#define LLWU_MF5_MWUF0_SHIFT                     0
#define LLWU_MF5_MWUF0_WIDTH                     1
#define LLWU_MF5_MWUF0(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF0_SHIFT))&LLWU_MF5_MWUF0_MASK)
#define LLWU_MF5_MWUF1_MASK                      0x2u
#define LLWU_MF5_MWUF1_SHIFT                     1
#define LLWU_MF5_MWUF1_WIDTH                     1
#define LLWU_MF5_MWUF1(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF1_SHIFT))&LLWU_MF5_MWUF1_MASK)
#define LLWU_MF5_MWUF2_MASK                      0x4u
#define LLWU_MF5_MWUF2_SHIFT                     2
#define LLWU_MF5_MWUF2_WIDTH                     1
#define LLWU_MF5_MWUF2(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF2_SHIFT))&LLWU_MF5_MWUF2_MASK)
#define LLWU_MF5_MWUF3_MASK                      0x8u
#define LLWU_MF5_MWUF3_SHIFT                     3
#define LLWU_MF5_MWUF3_WIDTH                     1
#define LLWU_MF5_MWUF3(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF3_SHIFT))&LLWU_MF5_MWUF3_MASK)
#define LLWU_MF5_MWUF4_MASK                      0x10u
#define LLWU_MF5_MWUF4_SHIFT                     4
#define LLWU_MF5_MWUF4_WIDTH                     1
#define LLWU_MF5_MWUF4(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF4_SHIFT))&LLWU_MF5_MWUF4_MASK)
#define LLWU_MF5_MWUF5_MASK                      0x20u
#define LLWU_MF5_MWUF5_SHIFT                     5
#define LLWU_MF5_MWUF5_WIDTH                     1
#define LLWU_MF5_MWUF5(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF5_SHIFT))&LLWU_MF5_MWUF5_MASK)
#define LLWU_MF5_MWUF6_MASK                      0x40u
#define LLWU_MF5_MWUF6_SHIFT                     6
#define LLWU_MF5_MWUF6_WIDTH                     1
#define LLWU_MF5_MWUF6(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF6_SHIFT))&LLWU_MF5_MWUF6_MASK)
#define LLWU_MF5_MWUF7_MASK                      0x80u
#define LLWU_MF5_MWUF7_SHIFT                     7
#define LLWU_MF5_MWUF7_WIDTH                     1
#define LLWU_MF5_MWUF7(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_MF5_MWUF7_SHIFT))&LLWU_MF5_MWUF7_MASK)
/* FILT1 Bit Fields */
#define LLWU_FILT1_FILTSEL_MASK                  0x1Fu
#define LLWU_FILT1_FILTSEL_SHIFT                 0
#define LLWU_FILT1_FILTSEL_WIDTH                 5
#define LLWU_FILT1_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<LLWU_FILT1_FILTSEL_SHIFT))&LLWU_FILT1_FILTSEL_MASK)
#define LLWU_FILT1_FILTE_MASK                    0x60u
#define LLWU_FILT1_FILTE_SHIFT                   5
#define LLWU_FILT1_FILTE_WIDTH                   2
#define LLWU_FILT1_FILTE(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT1_FILTE_SHIFT))&LLWU_FILT1_FILTE_MASK)
#define LLWU_FILT1_FILTF_MASK                    0x80u
#define LLWU_FILT1_FILTF_SHIFT                   7
#define LLWU_FILT1_FILTF_WIDTH                   1
#define LLWU_FILT1_FILTF(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT1_FILTF_SHIFT))&LLWU_FILT1_FILTF_MASK)
/* FILT2 Bit Fields */
#define LLWU_FILT2_FILTSEL_MASK                  0x1Fu
#define LLWU_FILT2_FILTSEL_SHIFT                 0
#define LLWU_FILT2_FILTSEL_WIDTH                 5
#define LLWU_FILT2_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<LLWU_FILT2_FILTSEL_SHIFT))&LLWU_FILT2_FILTSEL_MASK)
#define LLWU_FILT2_FILTE_MASK                    0x60u
#define LLWU_FILT2_FILTE_SHIFT                   5
#define LLWU_FILT2_FILTE_WIDTH                   2
#define LLWU_FILT2_FILTE(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT2_FILTE_SHIFT))&LLWU_FILT2_FILTE_MASK)
#define LLWU_FILT2_FILTF_MASK                    0x80u
#define LLWU_FILT2_FILTF_SHIFT                   7
#define LLWU_FILT2_FILTF_WIDTH                   1
#define LLWU_FILT2_FILTF(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT2_FILTF_SHIFT))&LLWU_FILT2_FILTF_MASK)

/*!
 * @}
 */ /* end of group LLWU_Register_Masks */


/* LLWU - Peripheral instance base addresses */
/** Peripheral LLWU base address */
#define LLWU_BASE                                (0x4007C000u)
/** Peripheral LLWU base pointer */
#define LLWU                                     ((LLWU_Type *)LLWU_BASE)
#define LLWU_BASE_PTR                            (LLWU)
/** Array initializer of LLWU peripheral base addresses */
#define LLWU_BASE_ADDRS                          { LLWU_BASE }
/** Array initializer of LLWU peripheral base pointers */
#define LLWU_BASE_PTRS                           { LLWU }

/* ----------------------------------------------------------------------------
   -- LLWU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Accessor_Macros LLWU - Register accessor macros
 * @{
 */


/* LLWU - Register instance definitions */
/* LLWU */
#define LLWU_PE1                                 LLWU_PE1_REG(LLWU)
#define LLWU_PE2                                 LLWU_PE2_REG(LLWU)
#define LLWU_PE3                                 LLWU_PE3_REG(LLWU)
#define LLWU_PE4                                 LLWU_PE4_REG(LLWU)
#define LLWU_PE5                                 LLWU_PE5_REG(LLWU)
#define LLWU_PE6                                 LLWU_PE6_REG(LLWU)
#define LLWU_PE7                                 LLWU_PE7_REG(LLWU)
#define LLWU_PE8                                 LLWU_PE8_REG(LLWU)
#define LLWU_ME                                  LLWU_ME_REG(LLWU)
#define LLWU_PF1                                 LLWU_PF1_REG(LLWU)
#define LLWU_PF2                                 LLWU_PF2_REG(LLWU)
#define LLWU_PF3                                 LLWU_PF3_REG(LLWU)
#define LLWU_PF4                                 LLWU_PF4_REG(LLWU)
#define LLWU_MF5                                 LLWU_MF5_REG(LLWU)
#define LLWU_FILT1                               LLWU_FILT1_REG(LLWU)
#define LLWU_FILT2                               LLWU_FILT2_REG(LLWU)

/*!
 * @}
 */ /* end of group LLWU_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group LLWU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPTMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Peripheral_Access_Layer LPTMR Peripheral Access Layer
 * @{
 */

/** LPTMR - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSR;                               /**< Low Power Timer Control Status Register, offset: 0x0 */
  __IO uint32_t PSR;                               /**< Low Power Timer Prescale Register, offset: 0x4 */
  __IO uint32_t CMR;                               /**< Low Power Timer Compare Register, offset: 0x8 */
  __IO uint32_t CNR;                               /**< Low Power Timer Counter Register, offset: 0xC */
} LPTMR_Type, *LPTMR_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register accessors */
#define LPTMR_CSR_REG(base)                      ((base)->CSR)
#define LPTMR_PSR_REG(base)                      ((base)->PSR)
#define LPTMR_CMR_REG(base)                      ((base)->CMR)
#define LPTMR_CNR_REG(base)                      ((base)->CNR)

/*!
 * @}
 */ /* end of group LPTMR_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LPTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Masks LPTMR Register Masks
 * @{
 */

/* CSR Bit Fields */
#define LPTMR_CSR_TEN_MASK                       0x1u
#define LPTMR_CSR_TEN_SHIFT                      0
#define LPTMR_CSR_TEN_WIDTH                      1
#define LPTMR_CSR_TEN(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TEN_SHIFT))&LPTMR_CSR_TEN_MASK)
#define LPTMR_CSR_TMS_MASK                       0x2u
#define LPTMR_CSR_TMS_SHIFT                      1
#define LPTMR_CSR_TMS_WIDTH                      1
#define LPTMR_CSR_TMS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TMS_SHIFT))&LPTMR_CSR_TMS_MASK)
#define LPTMR_CSR_TFC_MASK                       0x4u
#define LPTMR_CSR_TFC_SHIFT                      2
#define LPTMR_CSR_TFC_WIDTH                      1
#define LPTMR_CSR_TFC(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TFC_SHIFT))&LPTMR_CSR_TFC_MASK)
#define LPTMR_CSR_TPP_MASK                       0x8u
#define LPTMR_CSR_TPP_SHIFT                      3
#define LPTMR_CSR_TPP_WIDTH                      1
#define LPTMR_CSR_TPP(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TPP_SHIFT))&LPTMR_CSR_TPP_MASK)
#define LPTMR_CSR_TPS_MASK                       0x30u
#define LPTMR_CSR_TPS_SHIFT                      4
#define LPTMR_CSR_TPS_WIDTH                      2
#define LPTMR_CSR_TPS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TPS_SHIFT))&LPTMR_CSR_TPS_MASK)
#define LPTMR_CSR_TIE_MASK                       0x40u
#define LPTMR_CSR_TIE_SHIFT                      6
#define LPTMR_CSR_TIE_WIDTH                      1
#define LPTMR_CSR_TIE(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TIE_SHIFT))&LPTMR_CSR_TIE_MASK)
#define LPTMR_CSR_TCF_MASK                       0x80u
#define LPTMR_CSR_TCF_SHIFT                      7
#define LPTMR_CSR_TCF_WIDTH                      1
#define LPTMR_CSR_TCF(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TCF_SHIFT))&LPTMR_CSR_TCF_MASK)
/* PSR Bit Fields */
#define LPTMR_PSR_PCS_MASK                       0x3u
#define LPTMR_PSR_PCS_SHIFT                      0
#define LPTMR_PSR_PCS_WIDTH                      2
#define LPTMR_PSR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PCS_SHIFT))&LPTMR_PSR_PCS_MASK)
#define LPTMR_PSR_PBYP_MASK                      0x4u
#define LPTMR_PSR_PBYP_SHIFT                     2
#define LPTMR_PSR_PBYP_WIDTH                     1
#define LPTMR_PSR_PBYP(x)                        (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PBYP_SHIFT))&LPTMR_PSR_PBYP_MASK)
#define LPTMR_PSR_PRESCALE_MASK                  0x78u
#define LPTMR_PSR_PRESCALE_SHIFT                 3
#define LPTMR_PSR_PRESCALE_WIDTH                 4
#define LPTMR_PSR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PRESCALE_SHIFT))&LPTMR_PSR_PRESCALE_MASK)
/* CMR Bit Fields */
#define LPTMR_CMR_COMPARE_MASK                   0xFFFFu
#define LPTMR_CMR_COMPARE_SHIFT                  0
#define LPTMR_CMR_COMPARE_WIDTH                  16
#define LPTMR_CMR_COMPARE(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CMR_COMPARE_SHIFT))&LPTMR_CMR_COMPARE_MASK)
/* CNR Bit Fields */
#define LPTMR_CNR_COUNTER_MASK                   0xFFFFu
#define LPTMR_CNR_COUNTER_SHIFT                  0
#define LPTMR_CNR_COUNTER_WIDTH                  16
#define LPTMR_CNR_COUNTER(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CNR_COUNTER_SHIFT))&LPTMR_CNR_COUNTER_MASK)

/*!
 * @}
 */ /* end of group LPTMR_Register_Masks */


/* LPTMR - Peripheral instance base addresses */
/** Peripheral LPTMR0 base address */
#define LPTMR0_BASE                              (0x40040000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_Type *)LPTMR0_BASE)
#define LPTMR0_BASE_PTR                          (LPTMR0)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0 }

/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register instance definitions */
/* LPTMR0 */
#define LPTMR0_CSR                               LPTMR_CSR_REG(LPTMR0)
#define LPTMR0_PSR                               LPTMR_PSR_REG(LPTMR0)
#define LPTMR0_CMR                               LPTMR_CMR_REG(LPTMR0)
#define LPTMR0_CNR                               LPTMR_CNR_REG(LPTMR0)

/*!
 * @}
 */ /* end of group LPTMR_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group LPTMR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Peripheral_Access_Layer MCG Peripheral Access Layer
 * @{
 */

/** MCG - Register Layout Typedef */
typedef struct {
  __IO uint8_t C1;                                 /**< MCG Control 1 Register, offset: 0x0 */
  __IO uint8_t C2;                                 /**< MCG Control 2 Register, offset: 0x1 */
  __IO uint8_t C3;                                 /**< MCG Control 3 Register, offset: 0x2 */
  __IO uint8_t C4;                                 /**< MCG Control 4 Register, offset: 0x3 */
  __IO uint8_t C5;                                 /**< MCG Control 5 Register, offset: 0x4 */
  __IO uint8_t C6;                                 /**< MCG Control 6 Register, offset: 0x5 */
  __IO uint8_t S;                                  /**< MCG Status Register, offset: 0x6 */
       uint8_t RESERVED_0[1];
  __IO uint8_t SC;                                 /**< MCG Status and Control Register, offset: 0x8 */
       uint8_t RESERVED_1[1];
  __IO uint8_t ATCVH;                              /**< MCG Auto Trim Compare Value High Register, offset: 0xA */
  __IO uint8_t ATCVL;                              /**< MCG Auto Trim Compare Value Low Register, offset: 0xB */
       uint8_t RESERVED_2[1];
  __IO uint8_t C8;                                 /**< MCG Control 8 Register, offset: 0xD */
} MCG_Type, *MCG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Accessor_Macros MCG - Register accessor macros
 * @{
 */


/* MCG - Register accessors */
#define MCG_C1_REG(base)                         ((base)->C1)
#define MCG_C2_REG(base)                         ((base)->C2)
#define MCG_C3_REG(base)                         ((base)->C3)
#define MCG_C4_REG(base)                         ((base)->C4)
#define MCG_C5_REG(base)                         ((base)->C5)
#define MCG_C6_REG(base)                         ((base)->C6)
#define MCG_S_REG(base)                          ((base)->S)
#define MCG_SC_REG(base)                         ((base)->SC)
#define MCG_ATCVH_REG(base)                      ((base)->ATCVH)
#define MCG_ATCVL_REG(base)                      ((base)->ATCVL)
#define MCG_C8_REG(base)                         ((base)->C8)

/*!
 * @}
 */ /* end of group MCG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Masks MCG Register Masks
 * @{
 */

/* C1 Bit Fields */
#define MCG_C1_IREFSTEN_MASK                     0x1u
#define MCG_C1_IREFSTEN_SHIFT                    0
#define MCG_C1_IREFSTEN_WIDTH                    1
#define MCG_C1_IREFSTEN(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_C1_IREFSTEN_SHIFT))&MCG_C1_IREFSTEN_MASK)
#define MCG_C1_IRCLKEN_MASK                      0x2u
#define MCG_C1_IRCLKEN_SHIFT                     1
#define MCG_C1_IRCLKEN_WIDTH                     1
#define MCG_C1_IRCLKEN(x)                        (((uint8_t)(((uint8_t)(x))<<MCG_C1_IRCLKEN_SHIFT))&MCG_C1_IRCLKEN_MASK)
#define MCG_C1_IREFS_MASK                        0x4u
#define MCG_C1_IREFS_SHIFT                       2
#define MCG_C1_IREFS_WIDTH                       1
#define MCG_C1_IREFS(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C1_IREFS_SHIFT))&MCG_C1_IREFS_MASK)
#define MCG_C1_FRDIV_MASK                        0x38u
#define MCG_C1_FRDIV_SHIFT                       3
#define MCG_C1_FRDIV_WIDTH                       3
#define MCG_C1_FRDIV(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C1_FRDIV_SHIFT))&MCG_C1_FRDIV_MASK)
#define MCG_C1_CLKS_MASK                         0xC0u
#define MCG_C1_CLKS_SHIFT                        6
#define MCG_C1_CLKS_WIDTH                        2
#define MCG_C1_CLKS(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C1_CLKS_SHIFT))&MCG_C1_CLKS_MASK)
/* C2 Bit Fields */
#define MCG_C2_IRCS_MASK                         0x1u
#define MCG_C2_IRCS_SHIFT                        0
#define MCG_C2_IRCS_WIDTH                        1
#define MCG_C2_IRCS(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C2_IRCS_SHIFT))&MCG_C2_IRCS_MASK)
#define MCG_C2_LP_MASK                           0x2u
#define MCG_C2_LP_SHIFT                          1
#define MCG_C2_LP_WIDTH                          1
#define MCG_C2_LP(x)                             (((uint8_t)(((uint8_t)(x))<<MCG_C2_LP_SHIFT))&MCG_C2_LP_MASK)
#define MCG_C2_EREFS_MASK                        0x4u
#define MCG_C2_EREFS_SHIFT                       2
#define MCG_C2_EREFS_WIDTH                       1
#define MCG_C2_EREFS(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C2_EREFS_SHIFT))&MCG_C2_EREFS_MASK)
#define MCG_C2_HGO_MASK                          0x8u
#define MCG_C2_HGO_SHIFT                         3
#define MCG_C2_HGO_WIDTH                         1
#define MCG_C2_HGO(x)                            (((uint8_t)(((uint8_t)(x))<<MCG_C2_HGO_SHIFT))&MCG_C2_HGO_MASK)
#define MCG_C2_RANGE_MASK                        0x30u
#define MCG_C2_RANGE_SHIFT                       4
#define MCG_C2_RANGE_WIDTH                       2
#define MCG_C2_RANGE(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C2_RANGE_SHIFT))&MCG_C2_RANGE_MASK)
#define MCG_C2_FCFTRIM_MASK                      0x40u
#define MCG_C2_FCFTRIM_SHIFT                     6
#define MCG_C2_FCFTRIM_WIDTH                     1
#define MCG_C2_FCFTRIM(x)                        (((uint8_t)(((uint8_t)(x))<<MCG_C2_FCFTRIM_SHIFT))&MCG_C2_FCFTRIM_MASK)
#define MCG_C2_LOCRE0_MASK                       0x80u
#define MCG_C2_LOCRE0_SHIFT                      7
#define MCG_C2_LOCRE0_WIDTH                      1
#define MCG_C2_LOCRE0(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C2_LOCRE0_SHIFT))&MCG_C2_LOCRE0_MASK)
/* C3 Bit Fields */
#define MCG_C3_SCTRIM_MASK                       0xFFu
#define MCG_C3_SCTRIM_SHIFT                      0
#define MCG_C3_SCTRIM_WIDTH                      8
#define MCG_C3_SCTRIM(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C3_SCTRIM_SHIFT))&MCG_C3_SCTRIM_MASK)
/* C4 Bit Fields */
#define MCG_C4_SCFTRIM_MASK                      0x1u
#define MCG_C4_SCFTRIM_SHIFT                     0
#define MCG_C4_SCFTRIM_WIDTH                     1
#define MCG_C4_SCFTRIM(x)                        (((uint8_t)(((uint8_t)(x))<<MCG_C4_SCFTRIM_SHIFT))&MCG_C4_SCFTRIM_MASK)
#define MCG_C4_FCTRIM_MASK                       0x1Eu
#define MCG_C4_FCTRIM_SHIFT                      1
#define MCG_C4_FCTRIM_WIDTH                      4
#define MCG_C4_FCTRIM(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C4_FCTRIM_SHIFT))&MCG_C4_FCTRIM_MASK)
#define MCG_C4_DRST_DRS_MASK                     0x60u
#define MCG_C4_DRST_DRS_SHIFT                    5
#define MCG_C4_DRST_DRS_WIDTH                    2
#define MCG_C4_DRST_DRS(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_C4_DRST_DRS_SHIFT))&MCG_C4_DRST_DRS_MASK)
#define MCG_C4_DMX32_MASK                        0x80u
#define MCG_C4_DMX32_SHIFT                       7
#define MCG_C4_DMX32_WIDTH                       1
#define MCG_C4_DMX32(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C4_DMX32_SHIFT))&MCG_C4_DMX32_MASK)
/* C5 Bit Fields */
#define MCG_C5_PRDIV_MASK                        0x7u
#define MCG_C5_PRDIV_SHIFT                       0
#define MCG_C5_PRDIV_WIDTH                       3
#define MCG_C5_PRDIV(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C5_PRDIV_SHIFT))&MCG_C5_PRDIV_MASK)
#define MCG_C5_PLLSTEN_MASK                      0x20u
#define MCG_C5_PLLSTEN_SHIFT                     5
#define MCG_C5_PLLSTEN_WIDTH                     1
#define MCG_C5_PLLSTEN(x)                        (((uint8_t)(((uint8_t)(x))<<MCG_C5_PLLSTEN_SHIFT))&MCG_C5_PLLSTEN_MASK)
#define MCG_C5_PLLCLKEN_MASK                     0x40u
#define MCG_C5_PLLCLKEN_SHIFT                    6
#define MCG_C5_PLLCLKEN_WIDTH                    1
#define MCG_C5_PLLCLKEN(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_C5_PLLCLKEN_SHIFT))&MCG_C5_PLLCLKEN_MASK)
/* C6 Bit Fields */
#define MCG_C6_VDIV_MASK                         0x1Fu
#define MCG_C6_VDIV_SHIFT                        0
#define MCG_C6_VDIV_WIDTH                        5
#define MCG_C6_VDIV(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C6_VDIV_SHIFT))&MCG_C6_VDIV_MASK)
#define MCG_C6_CME0_MASK                         0x20u
#define MCG_C6_CME0_SHIFT                        5
#define MCG_C6_CME0_WIDTH                        1
#define MCG_C6_CME0(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C6_CME0_SHIFT))&MCG_C6_CME0_MASK)
#define MCG_C6_PLLS_MASK                         0x40u
#define MCG_C6_PLLS_SHIFT                        6
#define MCG_C6_PLLS_WIDTH                        1
#define MCG_C6_PLLS(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C6_PLLS_SHIFT))&MCG_C6_PLLS_MASK)
#define MCG_C6_LOLIE0_MASK                       0x80u
#define MCG_C6_LOLIE0_SHIFT                      7
#define MCG_C6_LOLIE0_WIDTH                      1
#define MCG_C6_LOLIE0(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C6_LOLIE0_SHIFT))&MCG_C6_LOLIE0_MASK)
/* S Bit Fields */
#define MCG_S_IRCST_MASK                         0x1u
#define MCG_S_IRCST_SHIFT                        0
#define MCG_S_IRCST_WIDTH                        1
#define MCG_S_IRCST(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_IRCST_SHIFT))&MCG_S_IRCST_MASK)
#define MCG_S_OSCINIT0_MASK                      0x2u
#define MCG_S_OSCINIT0_SHIFT                     1
#define MCG_S_OSCINIT0_WIDTH                     1
#define MCG_S_OSCINIT0(x)                        (((uint8_t)(((uint8_t)(x))<<MCG_S_OSCINIT0_SHIFT))&MCG_S_OSCINIT0_MASK)
#define MCG_S_CLKST_MASK                         0xCu
#define MCG_S_CLKST_SHIFT                        2
#define MCG_S_CLKST_WIDTH                        2
#define MCG_S_CLKST(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_CLKST_SHIFT))&MCG_S_CLKST_MASK)
#define MCG_S_IREFST_MASK                        0x10u
#define MCG_S_IREFST_SHIFT                       4
#define MCG_S_IREFST_WIDTH                       1
#define MCG_S_IREFST(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_S_IREFST_SHIFT))&MCG_S_IREFST_MASK)
#define MCG_S_PLLST_MASK                         0x20u
#define MCG_S_PLLST_SHIFT                        5
#define MCG_S_PLLST_WIDTH                        1
#define MCG_S_PLLST(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_PLLST_SHIFT))&MCG_S_PLLST_MASK)
#define MCG_S_LOCK0_MASK                         0x40u
#define MCG_S_LOCK0_SHIFT                        6
#define MCG_S_LOCK0_WIDTH                        1
#define MCG_S_LOCK0(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_LOCK0_SHIFT))&MCG_S_LOCK0_MASK)
#define MCG_S_LOLS0_MASK                         0x80u
#define MCG_S_LOLS0_SHIFT                        7
#define MCG_S_LOLS0_WIDTH                        1
#define MCG_S_LOLS0(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_LOLS0_SHIFT))&MCG_S_LOLS0_MASK)
/* SC Bit Fields */
#define MCG_SC_LOCS0_MASK                        0x1u
#define MCG_SC_LOCS0_SHIFT                       0
#define MCG_SC_LOCS0_WIDTH                       1
#define MCG_SC_LOCS0(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_SC_LOCS0_SHIFT))&MCG_SC_LOCS0_MASK)
#define MCG_SC_FCRDIV_MASK                       0xEu
#define MCG_SC_FCRDIV_SHIFT                      1
#define MCG_SC_FCRDIV_WIDTH                      3
#define MCG_SC_FCRDIV(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_SC_FCRDIV_SHIFT))&MCG_SC_FCRDIV_MASK)
#define MCG_SC_FLTPRSRV_MASK                     0x10u
#define MCG_SC_FLTPRSRV_SHIFT                    4
#define MCG_SC_FLTPRSRV_WIDTH                    1
#define MCG_SC_FLTPRSRV(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_SC_FLTPRSRV_SHIFT))&MCG_SC_FLTPRSRV_MASK)
#define MCG_SC_ATMF_MASK                         0x20u
#define MCG_SC_ATMF_SHIFT                        5
#define MCG_SC_ATMF_WIDTH                        1
#define MCG_SC_ATMF(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_SC_ATMF_SHIFT))&MCG_SC_ATMF_MASK)
#define MCG_SC_ATMS_MASK                         0x40u
#define MCG_SC_ATMS_SHIFT                        6
#define MCG_SC_ATMS_WIDTH                        1
#define MCG_SC_ATMS(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_SC_ATMS_SHIFT))&MCG_SC_ATMS_MASK)
#define MCG_SC_ATME_MASK                         0x80u
#define MCG_SC_ATME_SHIFT                        7
#define MCG_SC_ATME_WIDTH                        1
#define MCG_SC_ATME(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_SC_ATME_SHIFT))&MCG_SC_ATME_MASK)
/* ATCVH Bit Fields */
#define MCG_ATCVH_ATCVH_MASK                     0xFFu
#define MCG_ATCVH_ATCVH_SHIFT                    0
#define MCG_ATCVH_ATCVH_WIDTH                    8
#define MCG_ATCVH_ATCVH(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_ATCVH_ATCVH_SHIFT))&MCG_ATCVH_ATCVH_MASK)
/* ATCVL Bit Fields */
#define MCG_ATCVL_ATCVL_MASK                     0xFFu
#define MCG_ATCVL_ATCVL_SHIFT                    0
#define MCG_ATCVL_ATCVL_WIDTH                    8
#define MCG_ATCVL_ATCVL(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_ATCVL_ATCVL_SHIFT))&MCG_ATCVL_ATCVL_MASK)
/* C8 Bit Fields */
#define MCG_C8_LOLRE_MASK                        0x40u
#define MCG_C8_LOLRE_SHIFT                       6
#define MCG_C8_LOLRE_WIDTH                       1
#define MCG_C8_LOLRE(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C8_LOLRE_SHIFT))&MCG_C8_LOLRE_MASK)

/*!
 * @}
 */ /* end of group MCG_Register_Masks */


/* MCG - Peripheral instance base addresses */
/** Peripheral MCG base address */
#define MCG_BASE                                 (0x40064000u)
/** Peripheral MCG base pointer */
#define MCG                                      ((MCG_Type *)MCG_BASE)
#define MCG_BASE_PTR                             (MCG)
/** Array initializer of MCG peripheral base addresses */
#define MCG_BASE_ADDRS                           { MCG_BASE }
/** Array initializer of MCG peripheral base pointers */
#define MCG_BASE_PTRS                            { MCG }

/* ----------------------------------------------------------------------------
   -- MCG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Accessor_Macros MCG - Register accessor macros
 * @{
 */


/* MCG - Register instance definitions */
/* MCG */
#define MCG_C1                                   MCG_C1_REG(MCG)
#define MCG_C2                                   MCG_C2_REG(MCG)
#define MCG_C3                                   MCG_C3_REG(MCG)
#define MCG_C4                                   MCG_C4_REG(MCG)
#define MCG_C5                                   MCG_C5_REG(MCG)
#define MCG_C6                                   MCG_C6_REG(MCG)
#define MCG_S                                    MCG_S_REG(MCG)
#define MCG_SC                                   MCG_SC_REG(MCG)
#define MCG_ATCVH                                MCG_ATCVH_REG(MCG)
#define MCG_ATCVL                                MCG_ATCVL_REG(MCG)
#define MCG_C8                                   MCG_C8_REG(MCG)

/*!
 * @}
 */ /* end of group MCG_Register_Accessor_Macros */

/* MCG C5[PLLCLKEN0] backward compatibility */
#define MCG_C5_PLLCLKEN0_MASK         (MCG_C5_PLLCLKEN_MASK)
#define MCG_C5_PLLCLKEN0_SHIFT        (MCG_C5_PLLCLKEN_SHIFT)
#define MCG_C5_PLLCLKEN0_WIDTH        (MCG_C5_PLLCLKEN_WIDTH)
#define MCG_C5_PLLCLKEN0(x)           (MCG_C5_PLLCLKEN(x))

/* MCG C5[PLLSTEN0] backward compatibility */
#define MCG_C5_PLLSTEN0_MASK         (MCG_C5_PLLSTEN_MASK)
#define MCG_C5_PLLSTEN0_SHIFT        (MCG_C5_PLLSTEN_SHIFT)
#define MCG_C5_PLLSTEN0_WIDTH        (MCG_C5_PLLSTEN_WIDTH)
#define MCG_C5_PLLSTEN0(x)           (MCG_C5_PLLSTEN(x))

/* MCG C5[PRDIV0] backward compatibility */
#define MCG_C5_PRDIV0_MASK         (MCG_C5_PRDIV_MASK)
#define MCG_C5_PRDIV0_SHIFT        (MCG_C5_PRDIV_SHIFT)
#define MCG_C5_PRDIV0_WIDTH        (MCG_C5_PRDIV_WIDTH)
#define MCG_C5_PRDIV0(x)           (MCG_C5_PRDIV(x))

/* MCG C6[VDIV0] backward compatibility */
#define MCG_C6_VDIV0_MASK         (MCG_C6_VDIV_MASK)
#define MCG_C6_VDIV0_SHIFT        (MCG_C6_VDIV_SHIFT)
#define MCG_C6_VDIV0_WIDTH        (MCG_C6_VDIV_WIDTH)
#define MCG_C6_VDIV0(x)           (MCG_C6_VDIV(x))


/*!
 * @}
 */ /* end of group MCG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[8];
  __I  uint16_t PLASC;                             /**< Crossbar Switch (AXBS) Slave Configuration, offset: 0x8 */
  __I  uint16_t PLAMC;                             /**< Crossbar Switch (AXBS) Master Configuration, offset: 0xA */
  __IO uint32_t CR;                                /**< Control Register, offset: 0xC */
  __IO uint32_t ISCR;                              /**< Interrupt Status and Control Register, offset: 0x10 */
       uint8_t RESERVED_1[44];
  __IO uint32_t CPO;                               /**< Compute Operation Control Register, offset: 0x40 */
} MCM_Type, *MCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register accessors */
#define MCM_PLASC_REG(base)                      ((base)->PLASC)
#define MCM_PLAMC_REG(base)                      ((base)->PLAMC)
#define MCM_CR_REG(base)                         ((base)->CR)
#define MCM_ISCR_REG(base)                       ((base)->ISCR)
#define MCM_CPO_REG(base)                        ((base)->CPO)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/* PLASC Bit Fields */
#define MCM_PLASC_ASC_MASK                       0xFFu
#define MCM_PLASC_ASC_SHIFT                      0
#define MCM_PLASC_ASC_WIDTH                      8
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLASC_ASC_SHIFT))&MCM_PLASC_ASC_MASK)
/* PLAMC Bit Fields */
#define MCM_PLAMC_AMC_MASK                       0xFFu
#define MCM_PLAMC_AMC_SHIFT                      0
#define MCM_PLAMC_AMC_WIDTH                      8
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLAMC_AMC_SHIFT))&MCM_PLAMC_AMC_MASK)
/* CR Bit Fields */
#define MCM_CR_SRAMUAP_MASK                      0x3000000u
#define MCM_CR_SRAMUAP_SHIFT                     24
#define MCM_CR_SRAMUAP_WIDTH                     2
#define MCM_CR_SRAMUAP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMUAP_SHIFT))&MCM_CR_SRAMUAP_MASK)
#define MCM_CR_SRAMUWP_MASK                      0x4000000u
#define MCM_CR_SRAMUWP_SHIFT                     26
#define MCM_CR_SRAMUWP_WIDTH                     1
#define MCM_CR_SRAMUWP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMUWP_SHIFT))&MCM_CR_SRAMUWP_MASK)
#define MCM_CR_SRAMLAP_MASK                      0x30000000u
#define MCM_CR_SRAMLAP_SHIFT                     28
#define MCM_CR_SRAMLAP_WIDTH                     2
#define MCM_CR_SRAMLAP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMLAP_SHIFT))&MCM_CR_SRAMLAP_MASK)
#define MCM_CR_SRAMLWP_MASK                      0x40000000u
#define MCM_CR_SRAMLWP_SHIFT                     30
#define MCM_CR_SRAMLWP_WIDTH                     1
#define MCM_CR_SRAMLWP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMLWP_SHIFT))&MCM_CR_SRAMLWP_MASK)
/* ISCR Bit Fields */
#define MCM_ISCR_FIOC_MASK                       0x100u
#define MCM_ISCR_FIOC_SHIFT                      8
#define MCM_ISCR_FIOC_WIDTH                      1
#define MCM_ISCR_FIOC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIOC_SHIFT))&MCM_ISCR_FIOC_MASK)
#define MCM_ISCR_FDZC_MASK                       0x200u
#define MCM_ISCR_FDZC_SHIFT                      9
#define MCM_ISCR_FDZC_WIDTH                      1
#define MCM_ISCR_FDZC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FDZC_SHIFT))&MCM_ISCR_FDZC_MASK)
#define MCM_ISCR_FOFC_MASK                       0x400u
#define MCM_ISCR_FOFC_SHIFT                      10
#define MCM_ISCR_FOFC_WIDTH                      1
#define MCM_ISCR_FOFC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FOFC_SHIFT))&MCM_ISCR_FOFC_MASK)
#define MCM_ISCR_FUFC_MASK                       0x800u
#define MCM_ISCR_FUFC_SHIFT                      11
#define MCM_ISCR_FUFC_WIDTH                      1
#define MCM_ISCR_FUFC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FUFC_SHIFT))&MCM_ISCR_FUFC_MASK)
#define MCM_ISCR_FIXC_MASK                       0x1000u
#define MCM_ISCR_FIXC_SHIFT                      12
#define MCM_ISCR_FIXC_WIDTH                      1
#define MCM_ISCR_FIXC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIXC_SHIFT))&MCM_ISCR_FIXC_MASK)
#define MCM_ISCR_FIDC_MASK                       0x8000u
#define MCM_ISCR_FIDC_SHIFT                      15
#define MCM_ISCR_FIDC_WIDTH                      1
#define MCM_ISCR_FIDC(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIDC_SHIFT))&MCM_ISCR_FIDC_MASK)
#define MCM_ISCR_FIOCE_MASK                      0x1000000u
#define MCM_ISCR_FIOCE_SHIFT                     24
#define MCM_ISCR_FIOCE_WIDTH                     1
#define MCM_ISCR_FIOCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIOCE_SHIFT))&MCM_ISCR_FIOCE_MASK)
#define MCM_ISCR_FDZCE_MASK                      0x2000000u
#define MCM_ISCR_FDZCE_SHIFT                     25
#define MCM_ISCR_FDZCE_WIDTH                     1
#define MCM_ISCR_FDZCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FDZCE_SHIFT))&MCM_ISCR_FDZCE_MASK)
#define MCM_ISCR_FOFCE_MASK                      0x4000000u
#define MCM_ISCR_FOFCE_SHIFT                     26
#define MCM_ISCR_FOFCE_WIDTH                     1
#define MCM_ISCR_FOFCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FOFCE_SHIFT))&MCM_ISCR_FOFCE_MASK)
#define MCM_ISCR_FUFCE_MASK                      0x8000000u
#define MCM_ISCR_FUFCE_SHIFT                     27
#define MCM_ISCR_FUFCE_WIDTH                     1
#define MCM_ISCR_FUFCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FUFCE_SHIFT))&MCM_ISCR_FUFCE_MASK)
#define MCM_ISCR_FIXCE_MASK                      0x10000000u
#define MCM_ISCR_FIXCE_SHIFT                     28
#define MCM_ISCR_FIXCE_WIDTH                     1
#define MCM_ISCR_FIXCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIXCE_SHIFT))&MCM_ISCR_FIXCE_MASK)
#define MCM_ISCR_FIDCE_MASK                      0x80000000u
#define MCM_ISCR_FIDCE_SHIFT                     31
#define MCM_ISCR_FIDCE_WIDTH                     1
#define MCM_ISCR_FIDCE(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_ISCR_FIDCE_SHIFT))&MCM_ISCR_FIDCE_MASK)
/* CPO Bit Fields */
#define MCM_CPO_CPOREQ_MASK                      0x1u
#define MCM_CPO_CPOREQ_SHIFT                     0
#define MCM_CPO_CPOREQ_WIDTH                     1
#define MCM_CPO_CPOREQ(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CPO_CPOREQ_SHIFT))&MCM_CPO_CPOREQ_MASK)
#define MCM_CPO_CPOACK_MASK                      0x2u
#define MCM_CPO_CPOACK_SHIFT                     1
#define MCM_CPO_CPOACK_WIDTH                     1
#define MCM_CPO_CPOACK(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CPO_CPOACK_SHIFT))&MCM_CPO_CPOACK_MASK)
#define MCM_CPO_CPOWOI_MASK                      0x4u
#define MCM_CPO_CPOWOI_SHIFT                     2
#define MCM_CPO_CPOWOI_WIDTH                     1
#define MCM_CPO_CPOWOI(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CPO_CPOWOI_SHIFT))&MCM_CPO_CPOWOI_MASK)

/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xE0080000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_Type *)MCM_BASE)
#define MCM_BASE_PTR                             (MCM)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register instance definitions */
/* MCM */
#define MCM_PLASC                                MCM_PLASC_REG(MCM)
#define MCM_PLAMC                                MCM_PLAMC_REG(MCM)
#define MCM_CR                                   MCM_CR_REG(MCM)
#define MCM_ISCR                                 MCM_ISCR_REG(MCM)
#define MCM_CPO                                  MCM_CPO_REG(MCM)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- NV Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Peripheral_Access_Layer NV Peripheral Access Layer
 * @{
 */

/** NV - Register Layout Typedef */
typedef struct {
  __I  uint8_t BACKKEY3;                           /**< Backdoor Comparison Key 3., offset: 0x0 */
  __I  uint8_t BACKKEY2;                           /**< Backdoor Comparison Key 2., offset: 0x1 */
  __I  uint8_t BACKKEY1;                           /**< Backdoor Comparison Key 1., offset: 0x2 */
  __I  uint8_t BACKKEY0;                           /**< Backdoor Comparison Key 0., offset: 0x3 */
  __I  uint8_t BACKKEY7;                           /**< Backdoor Comparison Key 7., offset: 0x4 */
  __I  uint8_t BACKKEY6;                           /**< Backdoor Comparison Key 6., offset: 0x5 */
  __I  uint8_t BACKKEY5;                           /**< Backdoor Comparison Key 5., offset: 0x6 */
  __I  uint8_t BACKKEY4;                           /**< Backdoor Comparison Key 4., offset: 0x7 */
  __I  uint8_t FPROT3;                             /**< Non-volatile P-Flash Protection 1 - Low Register, offset: 0x8 */
  __I  uint8_t FPROT2;                             /**< Non-volatile P-Flash Protection 1 - High Register, offset: 0x9 */
  __I  uint8_t FPROT1;                             /**< Non-volatile P-Flash Protection 0 - Low Register, offset: 0xA */
  __I  uint8_t FPROT0;                             /**< Non-volatile P-Flash Protection 0 - High Register, offset: 0xB */
  __I  uint8_t FSEC;                               /**< Non-volatile Flash Security Register, offset: 0xC */
  __I  uint8_t FOPT;                               /**< Non-volatile Flash Option Register, offset: 0xD */
  __I  uint8_t FEPROT;                             /**< Non-volatile EERAM Protection Register, offset: 0xE */
  __I  uint8_t FDPROT;                             /**< Non-volatile D-Flash Protection Register, offset: 0xF */
} NV_Type, *NV_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- NV - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Accessor_Macros NV - Register accessor macros
 * @{
 */


/* NV - Register accessors */
#define NV_BACKKEY3_REG(base)                    ((base)->BACKKEY3)
#define NV_BACKKEY2_REG(base)                    ((base)->BACKKEY2)
#define NV_BACKKEY1_REG(base)                    ((base)->BACKKEY1)
#define NV_BACKKEY0_REG(base)                    ((base)->BACKKEY0)
#define NV_BACKKEY7_REG(base)                    ((base)->BACKKEY7)
#define NV_BACKKEY6_REG(base)                    ((base)->BACKKEY6)
#define NV_BACKKEY5_REG(base)                    ((base)->BACKKEY5)
#define NV_BACKKEY4_REG(base)                    ((base)->BACKKEY4)
#define NV_FPROT3_REG(base)                      ((base)->FPROT3)
#define NV_FPROT2_REG(base)                      ((base)->FPROT2)
#define NV_FPROT1_REG(base)                      ((base)->FPROT1)
#define NV_FPROT0_REG(base)                      ((base)->FPROT0)
#define NV_FSEC_REG(base)                        ((base)->FSEC)
#define NV_FOPT_REG(base)                        ((base)->FOPT)
#define NV_FEPROT_REG(base)                      ((base)->FEPROT)
#define NV_FDPROT_REG(base)                      ((base)->FDPROT)

/*!
 * @}
 */ /* end of group NV_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- NV Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Masks NV Register Masks
 * @{
 */

/* BACKKEY3 Bit Fields */
#define NV_BACKKEY3_KEY_MASK                     0xFFu
#define NV_BACKKEY3_KEY_SHIFT                    0
#define NV_BACKKEY3_KEY_WIDTH                    8
#define NV_BACKKEY3_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY3_KEY_SHIFT))&NV_BACKKEY3_KEY_MASK)
/* BACKKEY2 Bit Fields */
#define NV_BACKKEY2_KEY_MASK                     0xFFu
#define NV_BACKKEY2_KEY_SHIFT                    0
#define NV_BACKKEY2_KEY_WIDTH                    8
#define NV_BACKKEY2_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY2_KEY_SHIFT))&NV_BACKKEY2_KEY_MASK)
/* BACKKEY1 Bit Fields */
#define NV_BACKKEY1_KEY_MASK                     0xFFu
#define NV_BACKKEY1_KEY_SHIFT                    0
#define NV_BACKKEY1_KEY_WIDTH                    8
#define NV_BACKKEY1_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY1_KEY_SHIFT))&NV_BACKKEY1_KEY_MASK)
/* BACKKEY0 Bit Fields */
#define NV_BACKKEY0_KEY_MASK                     0xFFu
#define NV_BACKKEY0_KEY_SHIFT                    0
#define NV_BACKKEY0_KEY_WIDTH                    8
#define NV_BACKKEY0_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY0_KEY_SHIFT))&NV_BACKKEY0_KEY_MASK)
/* BACKKEY7 Bit Fields */
#define NV_BACKKEY7_KEY_MASK                     0xFFu
#define NV_BACKKEY7_KEY_SHIFT                    0
#define NV_BACKKEY7_KEY_WIDTH                    8
#define NV_BACKKEY7_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY7_KEY_SHIFT))&NV_BACKKEY7_KEY_MASK)
/* BACKKEY6 Bit Fields */
#define NV_BACKKEY6_KEY_MASK                     0xFFu
#define NV_BACKKEY6_KEY_SHIFT                    0
#define NV_BACKKEY6_KEY_WIDTH                    8
#define NV_BACKKEY6_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY6_KEY_SHIFT))&NV_BACKKEY6_KEY_MASK)
/* BACKKEY5 Bit Fields */
#define NV_BACKKEY5_KEY_MASK                     0xFFu
#define NV_BACKKEY5_KEY_SHIFT                    0
#define NV_BACKKEY5_KEY_WIDTH                    8
#define NV_BACKKEY5_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY5_KEY_SHIFT))&NV_BACKKEY5_KEY_MASK)
/* BACKKEY4 Bit Fields */
#define NV_BACKKEY4_KEY_MASK                     0xFFu
#define NV_BACKKEY4_KEY_SHIFT                    0
#define NV_BACKKEY4_KEY_WIDTH                    8
#define NV_BACKKEY4_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY4_KEY_SHIFT))&NV_BACKKEY4_KEY_MASK)
/* FPROT3 Bit Fields */
#define NV_FPROT3_PROT_MASK                      0xFFu
#define NV_FPROT3_PROT_SHIFT                     0
#define NV_FPROT3_PROT_WIDTH                     8
#define NV_FPROT3_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT3_PROT_SHIFT))&NV_FPROT3_PROT_MASK)
/* FPROT2 Bit Fields */
#define NV_FPROT2_PROT_MASK                      0xFFu
#define NV_FPROT2_PROT_SHIFT                     0
#define NV_FPROT2_PROT_WIDTH                     8
#define NV_FPROT2_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT2_PROT_SHIFT))&NV_FPROT2_PROT_MASK)
/* FPROT1 Bit Fields */
#define NV_FPROT1_PROT_MASK                      0xFFu
#define NV_FPROT1_PROT_SHIFT                     0
#define NV_FPROT1_PROT_WIDTH                     8
#define NV_FPROT1_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT1_PROT_SHIFT))&NV_FPROT1_PROT_MASK)
/* FPROT0 Bit Fields */
#define NV_FPROT0_PROT_MASK                      0xFFu
#define NV_FPROT0_PROT_SHIFT                     0
#define NV_FPROT0_PROT_WIDTH                     8
#define NV_FPROT0_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT0_PROT_SHIFT))&NV_FPROT0_PROT_MASK)
/* FSEC Bit Fields */
#define NV_FSEC_SEC_MASK                         0x3u
#define NV_FSEC_SEC_SHIFT                        0
#define NV_FSEC_SEC_WIDTH                        2
#define NV_FSEC_SEC(x)                           (((uint8_t)(((uint8_t)(x))<<NV_FSEC_SEC_SHIFT))&NV_FSEC_SEC_MASK)
#define NV_FSEC_FSLACC_MASK                      0xCu
#define NV_FSEC_FSLACC_SHIFT                     2
#define NV_FSEC_FSLACC_WIDTH                     2
#define NV_FSEC_FSLACC(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FSEC_FSLACC_SHIFT))&NV_FSEC_FSLACC_MASK)
#define NV_FSEC_MEEN_MASK                        0x30u
#define NV_FSEC_MEEN_SHIFT                       4
#define NV_FSEC_MEEN_WIDTH                       2
#define NV_FSEC_MEEN(x)                          (((uint8_t)(((uint8_t)(x))<<NV_FSEC_MEEN_SHIFT))&NV_FSEC_MEEN_MASK)
#define NV_FSEC_KEYEN_MASK                       0xC0u
#define NV_FSEC_KEYEN_SHIFT                      6
#define NV_FSEC_KEYEN_WIDTH                      2
#define NV_FSEC_KEYEN(x)                         (((uint8_t)(((uint8_t)(x))<<NV_FSEC_KEYEN_SHIFT))&NV_FSEC_KEYEN_MASK)
/* FOPT Bit Fields */
#define NV_FOPT_LPBOOT_MASK                      0x1u
#define NV_FOPT_LPBOOT_SHIFT                     0
#define NV_FOPT_LPBOOT_WIDTH                     1
#define NV_FOPT_LPBOOT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FOPT_LPBOOT_SHIFT))&NV_FOPT_LPBOOT_MASK)
#define NV_FOPT_NMI_DIS_MASK                     0x4u
#define NV_FOPT_NMI_DIS_SHIFT                    2
#define NV_FOPT_NMI_DIS_WIDTH                    1
#define NV_FOPT_NMI_DIS(x)                       (((uint8_t)(((uint8_t)(x))<<NV_FOPT_NMI_DIS_SHIFT))&NV_FOPT_NMI_DIS_MASK)
#define NV_FOPT_FAST_INIT_MASK                   0x20u
#define NV_FOPT_FAST_INIT_SHIFT                  5
#define NV_FOPT_FAST_INIT_WIDTH                  1
#define NV_FOPT_FAST_INIT(x)                     (((uint8_t)(((uint8_t)(x))<<NV_FOPT_FAST_INIT_SHIFT))&NV_FOPT_FAST_INIT_MASK)
/* FEPROT Bit Fields */
#define NV_FEPROT_EPROT_MASK                     0xFFu
#define NV_FEPROT_EPROT_SHIFT                    0
#define NV_FEPROT_EPROT_WIDTH                    8
#define NV_FEPROT_EPROT(x)                       (((uint8_t)(((uint8_t)(x))<<NV_FEPROT_EPROT_SHIFT))&NV_FEPROT_EPROT_MASK)
/* FDPROT Bit Fields */
#define NV_FDPROT_DPROT_MASK                     0xFFu
#define NV_FDPROT_DPROT_SHIFT                    0
#define NV_FDPROT_DPROT_WIDTH                    8
#define NV_FDPROT_DPROT(x)                       (((uint8_t)(((uint8_t)(x))<<NV_FDPROT_DPROT_SHIFT))&NV_FDPROT_DPROT_MASK)

/*!
 * @}
 */ /* end of group NV_Register_Masks */


/* NV - Peripheral instance base addresses */
/** Peripheral FTFL_FlashConfig base address */
#define FTFL_FlashConfig_BASE                    (0x400u)
/** Peripheral FTFL_FlashConfig base pointer */
#define FTFL_FlashConfig                         ((NV_Type *)FTFL_FlashConfig_BASE)
#define FTFL_FlashConfig_BASE_PTR                (FTFL_FlashConfig)
/** Array initializer of NV peripheral base addresses */
#define NV_BASE_ADDRS                            { FTFL_FlashConfig_BASE }
/** Array initializer of NV peripheral base pointers */
#define NV_BASE_PTRS                             { FTFL_FlashConfig }

/* ----------------------------------------------------------------------------
   -- NV - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Accessor_Macros NV - Register accessor macros
 * @{
 */


/* NV - Register instance definitions */
/* FTFL_FlashConfig */
#define NV_BACKKEY3                              NV_BACKKEY3_REG(FTFL_FlashConfig)
#define NV_BACKKEY2                              NV_BACKKEY2_REG(FTFL_FlashConfig)
#define NV_BACKKEY1                              NV_BACKKEY1_REG(FTFL_FlashConfig)
#define NV_BACKKEY0                              NV_BACKKEY0_REG(FTFL_FlashConfig)
#define NV_BACKKEY7                              NV_BACKKEY7_REG(FTFL_FlashConfig)
#define NV_BACKKEY6                              NV_BACKKEY6_REG(FTFL_FlashConfig)
#define NV_BACKKEY5                              NV_BACKKEY5_REG(FTFL_FlashConfig)
#define NV_BACKKEY4                              NV_BACKKEY4_REG(FTFL_FlashConfig)
#define NV_FPROT3                                NV_FPROT3_REG(FTFL_FlashConfig)
#define NV_FPROT2                                NV_FPROT2_REG(FTFL_FlashConfig)
#define NV_FPROT1                                NV_FPROT1_REG(FTFL_FlashConfig)
#define NV_FPROT0                                NV_FPROT0_REG(FTFL_FlashConfig)
#define NV_FSEC                                  NV_FSEC_REG(FTFL_FlashConfig)
#define NV_FOPT                                  NV_FOPT_REG(FTFL_FlashConfig)
#define NV_FEPROT                                NV_FEPROT_REG(FTFL_FlashConfig)
#define NV_FDPROT                                NV_FDPROT_REG(FTFL_FlashConfig)

/*!
 * @}
 */ /* end of group NV_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group NV_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- OSC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Peripheral_Access_Layer OSC Peripheral Access Layer
 * @{
 */

/** OSC - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR;                                 /**< OSC Control Register, offset: 0x0 */
       uint8_t RESERVED_0[1];
  __IO uint8_t DIV;                                /**< OSC_DIV, offset: 0x2 */
} OSC_Type, *OSC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- OSC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Accessor_Macros OSC - Register accessor macros
 * @{
 */


/* OSC - Register accessors */
#define OSC_CR_REG(base)                         ((base)->CR)
#define OSC_DIV_REG(base)                        ((base)->DIV)

/*!
 * @}
 */ /* end of group OSC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- OSC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Masks OSC Register Masks
 * @{
 */

/* CR Bit Fields */
#define OSC_CR_SC16P_MASK                        0x1u
#define OSC_CR_SC16P_SHIFT                       0
#define OSC_CR_SC16P_WIDTH                       1
#define OSC_CR_SC16P(x)                          (((uint8_t)(((uint8_t)(x))<<OSC_CR_SC16P_SHIFT))&OSC_CR_SC16P_MASK)
#define OSC_CR_SC8P_MASK                         0x2u
#define OSC_CR_SC8P_SHIFT                        1
#define OSC_CR_SC8P_WIDTH                        1
#define OSC_CR_SC8P(x)                           (((uint8_t)(((uint8_t)(x))<<OSC_CR_SC8P_SHIFT))&OSC_CR_SC8P_MASK)
#define OSC_CR_SC4P_MASK                         0x4u
#define OSC_CR_SC4P_SHIFT                        2
#define OSC_CR_SC4P_WIDTH                        1
#define OSC_CR_SC4P(x)                           (((uint8_t)(((uint8_t)(x))<<OSC_CR_SC4P_SHIFT))&OSC_CR_SC4P_MASK)
#define OSC_CR_SC2P_MASK                         0x8u
#define OSC_CR_SC2P_SHIFT                        3
#define OSC_CR_SC2P_WIDTH                        1
#define OSC_CR_SC2P(x)                           (((uint8_t)(((uint8_t)(x))<<OSC_CR_SC2P_SHIFT))&OSC_CR_SC2P_MASK)
#define OSC_CR_EREFSTEN_MASK                     0x20u
#define OSC_CR_EREFSTEN_SHIFT                    5
#define OSC_CR_EREFSTEN_WIDTH                    1
#define OSC_CR_EREFSTEN(x)                       (((uint8_t)(((uint8_t)(x))<<OSC_CR_EREFSTEN_SHIFT))&OSC_CR_EREFSTEN_MASK)
#define OSC_CR_ERCLKEN_MASK                      0x80u
#define OSC_CR_ERCLKEN_SHIFT                     7
#define OSC_CR_ERCLKEN_WIDTH                     1
#define OSC_CR_ERCLKEN(x)                        (((uint8_t)(((uint8_t)(x))<<OSC_CR_ERCLKEN_SHIFT))&OSC_CR_ERCLKEN_MASK)
/* DIV Bit Fields */
#define OSC_DIV_ERPS_MASK                        0xC0u
#define OSC_DIV_ERPS_SHIFT                       6
#define OSC_DIV_ERPS_WIDTH                       2
#define OSC_DIV_ERPS(x)                          (((uint8_t)(((uint8_t)(x))<<OSC_DIV_ERPS_SHIFT))&OSC_DIV_ERPS_MASK)

/*!
 * @}
 */ /* end of group OSC_Register_Masks */


/* OSC - Peripheral instance base addresses */
/** Peripheral OSC base address */
#define OSC_BASE                                 (0x40065000u)
/** Peripheral OSC base pointer */
#define OSC                                      ((OSC_Type *)OSC_BASE)
#define OSC_BASE_PTR                             (OSC)
/** Array initializer of OSC peripheral base addresses */
#define OSC_BASE_ADDRS                           { OSC_BASE }
/** Array initializer of OSC peripheral base pointers */
#define OSC_BASE_PTRS                            { OSC }

/* ----------------------------------------------------------------------------
   -- OSC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Accessor_Macros OSC - Register accessor macros
 * @{
 */


/* OSC - Register instance definitions */
/* OSC */
#define OSC_CR                                   OSC_CR_REG(OSC)
#define OSC_DIV                                  OSC_DIV_REG(OSC)

/*!
 * @}
 */ /* end of group OSC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group OSC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PDB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Peripheral_Access_Layer PDB Peripheral Access Layer
 * @{
 */

/** PDB - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status and Control register, offset: 0x0 */
  __IO uint32_t MOD;                               /**< Modulus register, offset: 0x4 */
  __I  uint32_t CNT;                               /**< Counter register, offset: 0x8 */
  __IO uint32_t IDLY;                              /**< Interrupt Delay register, offset: 0xC */
  struct {                                         /* offset: 0x10, array step: 0x18 */
    __IO uint32_t C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x18 */
    __IO uint32_t S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x18 */
    __IO uint32_t DLY[4];                            /**< Channel n Delay 0 register..Channel n Delay 3 register, array offset: 0x18, array step: index*0x18, index2*0x4 */
  } CH[1];
       uint8_t RESERVED_0[296];
  struct {                                         /* offset: 0x150, array step: 0x8 */
    __IO uint32_t INTC;                              /**< DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8 */
    __IO uint32_t INT;                               /**< DAC Interval n register, array offset: 0x154, array step: 0x8 */
  } DAC[1];
       uint8_t RESERVED_1[56];
  __IO uint32_t POEN;                              /**< Pulse-Out n Enable register, offset: 0x190 */
  __IO uint32_t PODLY[4];                          /**< Pulse-Out n Delay register, array offset: 0x194, array step: 0x4 */
} PDB_Type, *PDB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register accessors */
#define PDB_SC_REG(base)                         ((base)->SC)
#define PDB_MOD_REG(base)                        ((base)->MOD)
#define PDB_CNT_REG(base)                        ((base)->CNT)
#define PDB_IDLY_REG(base)                       ((base)->IDLY)
#define PDB_C1_REG(base,index)                   ((base)->CH[index].C1)
#define PDB_C1_COUNT                             1
#define PDB_S_REG(base,index)                    ((base)->CH[index].S)
#define PDB_S_COUNT                              1
#define PDB_DLY_REG(base,index,index2)           ((base)->CH[index].DLY[index2])
#define PDB_DLY_COUNT                            1
#define PDB_DLY_COUNT2                           4
#define PDB_INTC_REG(base,index)                 ((base)->DAC[index].INTC)
#define PDB_INTC_COUNT                           1
#define PDB_INT_REG(base,index)                  ((base)->DAC[index].INT)
#define PDB_INT_COUNT                            1
#define PDB_POEN_REG(base)                       ((base)->POEN)
#define PDB_PODLY_REG(base,index)                ((base)->PODLY[index])
#define PDB_PODLY_COUNT                          4

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Masks PDB Register Masks
 * @{
 */

/* SC Bit Fields */
#define PDB_SC_LDOK_MASK                         0x1u
#define PDB_SC_LDOK_SHIFT                        0
#define PDB_SC_LDOK_WIDTH                        1
#define PDB_SC_LDOK(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_SC_LDOK_SHIFT))&PDB_SC_LDOK_MASK)
#define PDB_SC_CONT_MASK                         0x2u
#define PDB_SC_CONT_SHIFT                        1
#define PDB_SC_CONT_WIDTH                        1
#define PDB_SC_CONT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_SC_CONT_SHIFT))&PDB_SC_CONT_MASK)
#define PDB_SC_MULT_MASK                         0xCu
#define PDB_SC_MULT_SHIFT                        2
#define PDB_SC_MULT_WIDTH                        2
#define PDB_SC_MULT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_SC_MULT_SHIFT))&PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE_MASK                        0x20u
#define PDB_SC_PDBIE_SHIFT                       5
#define PDB_SC_PDBIE_WIDTH                       1
#define PDB_SC_PDBIE(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_PDBIE_SHIFT))&PDB_SC_PDBIE_MASK)
#define PDB_SC_PDBIF_MASK                        0x40u
#define PDB_SC_PDBIF_SHIFT                       6
#define PDB_SC_PDBIF_WIDTH                       1
#define PDB_SC_PDBIF(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_PDBIF_SHIFT))&PDB_SC_PDBIF_MASK)
#define PDB_SC_PDBEN_MASK                        0x80u
#define PDB_SC_PDBEN_SHIFT                       7
#define PDB_SC_PDBEN_WIDTH                       1
#define PDB_SC_PDBEN(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_PDBEN_SHIFT))&PDB_SC_PDBEN_MASK)
#define PDB_SC_TRGSEL_MASK                       0xF00u
#define PDB_SC_TRGSEL_SHIFT                      8
#define PDB_SC_TRGSEL_WIDTH                      4
#define PDB_SC_TRGSEL(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_SC_TRGSEL_SHIFT))&PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    0x7000u
#define PDB_SC_PRESCALER_SHIFT                   12
#define PDB_SC_PRESCALER_WIDTH                   3
#define PDB_SC_PRESCALER(x)                      (((uint32_t)(((uint32_t)(x))<<PDB_SC_PRESCALER_SHIFT))&PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN_MASK                        0x8000u
#define PDB_SC_DMAEN_SHIFT                       15
#define PDB_SC_DMAEN_WIDTH                       1
#define PDB_SC_DMAEN(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_DMAEN_SHIFT))&PDB_SC_DMAEN_MASK)
#define PDB_SC_SWTRIG_MASK                       0x10000u
#define PDB_SC_SWTRIG_SHIFT                      16
#define PDB_SC_SWTRIG_WIDTH                      1
#define PDB_SC_SWTRIG(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_SC_SWTRIG_SHIFT))&PDB_SC_SWTRIG_MASK)
#define PDB_SC_PDBEIE_MASK                       0x20000u
#define PDB_SC_PDBEIE_SHIFT                      17
#define PDB_SC_PDBEIE_WIDTH                      1
#define PDB_SC_PDBEIE(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_SC_PDBEIE_SHIFT))&PDB_SC_PDBEIE_MASK)
#define PDB_SC_LDMOD_MASK                        0xC0000u
#define PDB_SC_LDMOD_SHIFT                       18
#define PDB_SC_LDMOD_WIDTH                       2
#define PDB_SC_LDMOD(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_LDMOD_SHIFT))&PDB_SC_LDMOD_MASK)
/* MOD Bit Fields */
#define PDB_MOD_MOD_MASK                         0xFFFFu
#define PDB_MOD_MOD_SHIFT                        0
#define PDB_MOD_MOD_WIDTH                        16
#define PDB_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_MOD_MOD_SHIFT))&PDB_MOD_MOD_MASK)
/* CNT Bit Fields */
#define PDB_CNT_CNT_MASK                         0xFFFFu
#define PDB_CNT_CNT_SHIFT                        0
#define PDB_CNT_CNT_WIDTH                        16
#define PDB_CNT_CNT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_CNT_CNT_SHIFT))&PDB_CNT_CNT_MASK)
/* IDLY Bit Fields */
#define PDB_IDLY_IDLY_MASK                       0xFFFFu
#define PDB_IDLY_IDLY_SHIFT                      0
#define PDB_IDLY_IDLY_WIDTH                      16
#define PDB_IDLY_IDLY(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_IDLY_IDLY_SHIFT))&PDB_IDLY_IDLY_MASK)
/* C1 Bit Fields */
#define PDB_C1_EN_MASK                           0xFFu
#define PDB_C1_EN_SHIFT                          0
#define PDB_C1_EN_WIDTH                          8
#define PDB_C1_EN(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_EN_SHIFT))&PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          0xFF00u
#define PDB_C1_TOS_SHIFT                         8
#define PDB_C1_TOS_WIDTH                         8
#define PDB_C1_TOS(x)                            (((uint32_t)(((uint32_t)(x))<<PDB_C1_TOS_SHIFT))&PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           0xFF0000u
#define PDB_C1_BB_SHIFT                          16
#define PDB_C1_BB_WIDTH                          8
#define PDB_C1_BB(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_BB_SHIFT))&PDB_C1_BB_MASK)
/* S Bit Fields */
#define PDB_S_ERR_MASK                           0xFFu
#define PDB_S_ERR_SHIFT                          0
#define PDB_S_ERR_WIDTH                          8
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_S_ERR_SHIFT))&PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            0xFF0000u
#define PDB_S_CF_SHIFT                           16
#define PDB_S_CF_WIDTH                           8
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x))<<PDB_S_CF_SHIFT))&PDB_S_CF_MASK)
/* DLY Bit Fields */
#define PDB_DLY_DLY_MASK                         0xFFFFu
#define PDB_DLY_DLY_SHIFT                        0
#define PDB_DLY_DLY_WIDTH                        16
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_DLY_DLY_SHIFT))&PDB_DLY_DLY_MASK)
/* INTC Bit Fields */
#define PDB_INTC_TOE_MASK                        0x1u
#define PDB_INTC_TOE_SHIFT                       0
#define PDB_INTC_TOE_WIDTH                       1
#define PDB_INTC_TOE(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_INTC_TOE_SHIFT))&PDB_INTC_TOE_MASK)
#define PDB_INTC_EXT_MASK                        0x2u
#define PDB_INTC_EXT_SHIFT                       1
#define PDB_INTC_EXT_WIDTH                       1
#define PDB_INTC_EXT(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_INTC_EXT_SHIFT))&PDB_INTC_EXT_MASK)
/* INT Bit Fields */
#define PDB_INT_INT_MASK                         0xFFFFu
#define PDB_INT_INT_SHIFT                        0
#define PDB_INT_INT_WIDTH                        16
#define PDB_INT_INT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_INT_INT_SHIFT))&PDB_INT_INT_MASK)
/* POEN Bit Fields */
#define PDB_POEN_POEN_MASK                       0xFFu
#define PDB_POEN_POEN_SHIFT                      0
#define PDB_POEN_POEN_WIDTH                      8
#define PDB_POEN_POEN(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_POEN_POEN_SHIFT))&PDB_POEN_POEN_MASK)
/* PODLY Bit Fields */
#define PDB_PODLY_DLY2_MASK                      0xFFFFu
#define PDB_PODLY_DLY2_SHIFT                     0
#define PDB_PODLY_DLY2_WIDTH                     16
#define PDB_PODLY_DLY2(x)                        (((uint32_t)(((uint32_t)(x))<<PDB_PODLY_DLY2_SHIFT))&PDB_PODLY_DLY2_MASK)
#define PDB_PODLY_DLY1_MASK                      0xFFFF0000u
#define PDB_PODLY_DLY1_SHIFT                     16
#define PDB_PODLY_DLY1_WIDTH                     16
#define PDB_PODLY_DLY1(x)                        (((uint32_t)(((uint32_t)(x))<<PDB_PODLY_DLY1_SHIFT))&PDB_PODLY_DLY1_MASK)

/*!
 * @}
 */ /* end of group PDB_Register_Masks */


/* PDB - Peripheral instance base addresses */
/** Peripheral PDB0 base address */
#define PDB0_BASE                                (0x40036000u)
/** Peripheral PDB0 base pointer */
#define PDB0                                     ((PDB_Type *)PDB0_BASE)
#define PDB0_BASE_PTR                            (PDB0)
/** Peripheral PDB1 base address */
#define PDB1_BASE                                (0x40031000u)
/** Peripheral PDB1 base pointer */
#define PDB1                                     ((PDB_Type *)PDB1_BASE)
#define PDB1_BASE_PTR                            (PDB1)
/** Array initializer of PDB peripheral base addresses */
#define PDB_BASE_ADDRS                           { PDB0_BASE, PDB1_BASE }
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB0, PDB1 }

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register instance definitions */
/* PDB0 */
#define PDB0_SC                                  PDB_SC_REG(PDB0)
#define PDB0_MOD                                 PDB_MOD_REG(PDB0)
#define PDB0_CNT                                 PDB_CNT_REG(PDB0)
#define PDB0_IDLY                                PDB_IDLY_REG(PDB0)
#define PDB0_CH0C1                               PDB_C1_REG(PDB0,0)
#define PDB0_CH0S                                PDB_S_REG(PDB0,0)
#define PDB0_CH0DLY0                             PDB_DLY_REG(PDB0,0,0)
#define PDB0_CH0DLY1                             PDB_DLY_REG(PDB0,0,1)
#define PDB0_CH0DLY2                             PDB_DLY_REG(PDB0,0,2)
#define PDB0_CH0DLY3                             PDB_DLY_REG(PDB0,0,3)
#define PDB0_DACINTC0                            PDB_INTC_REG(PDB0,0)
#define PDB0_DACINT0                             PDB_INT_REG(PDB0,0)
#define PDB0_POEN                                PDB_POEN_REG(PDB0)
#define PDB0_PO0DLY                              PDB_PODLY_REG(PDB0,0)
#define PDB0_PO1DLY                              PDB_PODLY_REG(PDB0,1)
#define PDB0_PO2DLY                              PDB_PODLY_REG(PDB0,2)
#define PDB0_PO3DLY                              PDB_PODLY_REG(PDB0,3)
/* PDB1 */
#define PDB1_SC                                  PDB_SC_REG(PDB1)
#define PDB1_MOD                                 PDB_MOD_REG(PDB1)
#define PDB1_CNT                                 PDB_CNT_REG(PDB1)
#define PDB1_IDLY                                PDB_IDLY_REG(PDB1)
#define PDB1_CH0C1                               PDB_C1_REG(PDB1,0)
#define PDB1_CH0S                                PDB_S_REG(PDB1,0)
#define PDB1_CH0DLY0                             PDB_DLY_REG(PDB1,0,0)
#define PDB1_CH0DLY1                             PDB_DLY_REG(PDB1,0,1)
#define PDB1_CH0DLY2                             PDB_DLY_REG(PDB1,0,2)
#define PDB1_CH0DLY3                             PDB_DLY_REG(PDB1,0,3)
#define PDB1_DACINTC0                            PDB_INTC_REG(PDB1,0)
#define PDB1_DACINT0                             PDB_INT_REG(PDB1,0)
#define PDB1_POEN                                PDB_POEN_REG(PDB1)
#define PDB1_PO0DLY                              PDB_PODLY_REG(PDB1,0)
#define PDB1_PO1DLY                              PDB_PODLY_REG(PDB1,1)
#define PDB1_PO2DLY                              PDB_PODLY_REG(PDB1,2)
#define PDB1_PO3DLY                              PDB_PODLY_REG(PDB1,3)

/* PDB - Register array accessors */
#define PDB0_C1(index)                           PDB_C1_REG(PDB0,index)
#define PDB1_C1(index)                           PDB_C1_REG(PDB1,index)
#define PDB0_S(index)                            PDB_S_REG(PDB0,index)
#define PDB1_S(index)                            PDB_S_REG(PDB1,index)
#define PDB0_DLY(index,index2)                   PDB_DLY_REG(PDB0,index,index2)
#define PDB1_DLY(index,index2)                   PDB_DLY_REG(PDB1,index,index2)
#define PDB0_INTC(index)                         PDB_INTC_REG(PDB0,index)
#define PDB1_INTC(index)                         PDB_INTC_REG(PDB1,index)
#define PDB0_INT(index)                          PDB_INT_REG(PDB0,index)
#define PDB1_INT(index)                          PDB_INT_REG(PDB1,index)
#define PDB0_PODLY(index)                        PDB_PODLY_REG(PDB0,index)
#define PDB1_PODLY(index)                        PDB_PODLY_REG(PDB1,index)

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PDB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< PIT Module Control Register, offset: 0x0 */
       uint8_t RESERVED_0[252];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __IO uint32_t LDVAL;                             /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    __IO uint32_t TFLG;                              /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[4];
} PIT_Type, *PIT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register accessors */
#define PIT_MCR_REG(base)                        ((base)->MCR)
#define PIT_LDVAL_REG(base,index)                ((base)->CHANNEL[index].LDVAL)
#define PIT_LDVAL_COUNT                          4
#define PIT_CVAL_REG(base,index)                 ((base)->CHANNEL[index].CVAL)
#define PIT_CVAL_COUNT                           4
#define PIT_TCTRL_REG(base,index)                ((base)->CHANNEL[index].TCTRL)
#define PIT_TCTRL_COUNT                          4
#define PIT_TFLG_REG(base,index)                 ((base)->CHANNEL[index].TFLG)
#define PIT_TFLG_COUNT                           4

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/* MCR Bit Fields */
#define PIT_MCR_FRZ_MASK                         0x1u
#define PIT_MCR_FRZ_SHIFT                        0
#define PIT_MCR_FRZ_WIDTH                        1
#define PIT_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x))<<PIT_MCR_FRZ_SHIFT))&PIT_MCR_FRZ_MASK)
#define PIT_MCR_MDIS_MASK                        0x2u
#define PIT_MCR_MDIS_SHIFT                       1
#define PIT_MCR_MDIS_WIDTH                       1
#define PIT_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x))<<PIT_MCR_MDIS_SHIFT))&PIT_MCR_MDIS_MASK)
/* LDVAL Bit Fields */
#define PIT_LDVAL_TSV_MASK                       0xFFFFFFFFu
#define PIT_LDVAL_TSV_SHIFT                      0
#define PIT_LDVAL_TSV_WIDTH                      32
#define PIT_LDVAL_TSV(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_LDVAL_TSV_SHIFT))&PIT_LDVAL_TSV_MASK)
/* CVAL Bit Fields */
#define PIT_CVAL_TVL_MASK                        0xFFFFFFFFu
#define PIT_CVAL_TVL_SHIFT                       0
#define PIT_CVAL_TVL_WIDTH                       32
#define PIT_CVAL_TVL(x)                          (((uint32_t)(((uint32_t)(x))<<PIT_CVAL_TVL_SHIFT))&PIT_CVAL_TVL_MASK)
/* TCTRL Bit Fields */
#define PIT_TCTRL_TEN_MASK                       0x1u
#define PIT_TCTRL_TEN_SHIFT                      0
#define PIT_TCTRL_TEN_WIDTH                      1
#define PIT_TCTRL_TEN(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_TCTRL_TEN_SHIFT))&PIT_TCTRL_TEN_MASK)
#define PIT_TCTRL_TIE_MASK                       0x2u
#define PIT_TCTRL_TIE_SHIFT                      1
#define PIT_TCTRL_TIE_WIDTH                      1
#define PIT_TCTRL_TIE(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_TCTRL_TIE_SHIFT))&PIT_TCTRL_TIE_MASK)
#define PIT_TCTRL_CHN_MASK                       0x4u
#define PIT_TCTRL_CHN_SHIFT                      2
#define PIT_TCTRL_CHN_WIDTH                      1
#define PIT_TCTRL_CHN(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_TCTRL_CHN_SHIFT))&PIT_TCTRL_CHN_MASK)
/* TFLG Bit Fields */
#define PIT_TFLG_TIF_MASK                        0x1u
#define PIT_TFLG_TIF_SHIFT                       0
#define PIT_TFLG_TIF_WIDTH                       1
#define PIT_TFLG_TIF(x)                          (((uint32_t)(((uint32_t)(x))<<PIT_TFLG_TIF_SHIFT))&PIT_TFLG_TIF_MASK)

/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT base address */
#define PIT_BASE                                 (0x40037000u)
/** Peripheral PIT base pointer */
#define PIT                                      ((PIT_Type *)PIT_BASE)
#define PIT_BASE_PTR                             (PIT)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS                           { PIT_BASE }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT }

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register instance definitions */
/* PIT */
#define PIT_MCR                                  PIT_MCR_REG(PIT)
#define PIT_LDVAL0                               PIT_LDVAL_REG(PIT,0)
#define PIT_CVAL0                                PIT_CVAL_REG(PIT,0)
#define PIT_TCTRL0                               PIT_TCTRL_REG(PIT,0)
#define PIT_TFLG0                                PIT_TFLG_REG(PIT,0)
#define PIT_LDVAL1                               PIT_LDVAL_REG(PIT,1)
#define PIT_CVAL1                                PIT_CVAL_REG(PIT,1)
#define PIT_TCTRL1                               PIT_TCTRL_REG(PIT,1)
#define PIT_TFLG1                                PIT_TFLG_REG(PIT,1)
#define PIT_LDVAL2                               PIT_LDVAL_REG(PIT,2)
#define PIT_CVAL2                                PIT_CVAL_REG(PIT,2)
#define PIT_TCTRL2                               PIT_TCTRL_REG(PIT,2)
#define PIT_TFLG2                                PIT_TFLG_REG(PIT,2)
#define PIT_LDVAL3                               PIT_LDVAL_REG(PIT,3)
#define PIT_CVAL3                                PIT_CVAL_REG(PIT,3)
#define PIT_TCTRL3                               PIT_TCTRL_REG(PIT,3)
#define PIT_TFLG3                                PIT_TFLG_REG(PIT,3)

/* PIT - Register array accessors */
#define PIT_LDVAL(index)                         PIT_LDVAL_REG(PIT,index)
#define PIT_CVAL(index)                          PIT_CVAL_REG(PIT,index)
#define PIT_TCTRL(index)                         PIT_TCTRL_REG(PIT,index)
#define PIT_TFLG(index)                          PIT_TFLG_REG(PIT,index)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t LVDSC1;                             /**< Low Voltage Detect Status And Control 1 register, offset: 0x0 */
  __IO uint8_t LVDSC2;                             /**< Low Voltage Detect Status And Control 2 register, offset: 0x1 */
  __IO uint8_t REGSC;                              /**< Regulator Status And Control register, offset: 0x2 */
} PMC_Type, *PMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register accessors */
#define PMC_LVDSC1_REG(base)                     ((base)->LVDSC1)
#define PMC_LVDSC2_REG(base)                     ((base)->LVDSC2)
#define PMC_REGSC_REG(base)                      ((base)->REGSC)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/* LVDSC1 Bit Fields */
#define PMC_LVDSC1_LVDV_MASK                     0x3u
#define PMC_LVDSC1_LVDV_SHIFT                    0
#define PMC_LVDSC1_LVDV_WIDTH                    2
#define PMC_LVDSC1_LVDV(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDV_SHIFT))&PMC_LVDSC1_LVDV_MASK)
#define PMC_LVDSC1_LVDRE_MASK                    0x10u
#define PMC_LVDSC1_LVDRE_SHIFT                   4
#define PMC_LVDSC1_LVDRE_WIDTH                   1
#define PMC_LVDSC1_LVDRE(x)                      (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDRE_SHIFT))&PMC_LVDSC1_LVDRE_MASK)
#define PMC_LVDSC1_LVDIE_MASK                    0x20u
#define PMC_LVDSC1_LVDIE_SHIFT                   5
#define PMC_LVDSC1_LVDIE_WIDTH                   1
#define PMC_LVDSC1_LVDIE(x)                      (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDIE_SHIFT))&PMC_LVDSC1_LVDIE_MASK)
#define PMC_LVDSC1_LVDACK_MASK                   0x40u
#define PMC_LVDSC1_LVDACK_SHIFT                  6
#define PMC_LVDSC1_LVDACK_WIDTH                  1
#define PMC_LVDSC1_LVDACK(x)                     (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDACK_SHIFT))&PMC_LVDSC1_LVDACK_MASK)
#define PMC_LVDSC1_LVDF_MASK                     0x80u
#define PMC_LVDSC1_LVDF_SHIFT                    7
#define PMC_LVDSC1_LVDF_WIDTH                    1
#define PMC_LVDSC1_LVDF(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDF_SHIFT))&PMC_LVDSC1_LVDF_MASK)
/* LVDSC2 Bit Fields */
#define PMC_LVDSC2_LVWV_MASK                     0x3u
#define PMC_LVDSC2_LVWV_SHIFT                    0
#define PMC_LVDSC2_LVWV_WIDTH                    2
#define PMC_LVDSC2_LVWV(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWV_SHIFT))&PMC_LVDSC2_LVWV_MASK)
#define PMC_LVDSC2_LVWIE_MASK                    0x20u
#define PMC_LVDSC2_LVWIE_SHIFT                   5
#define PMC_LVDSC2_LVWIE_WIDTH                   1
#define PMC_LVDSC2_LVWIE(x)                      (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWIE_SHIFT))&PMC_LVDSC2_LVWIE_MASK)
#define PMC_LVDSC2_LVWACK_MASK                   0x40u
#define PMC_LVDSC2_LVWACK_SHIFT                  6
#define PMC_LVDSC2_LVWACK_WIDTH                  1
#define PMC_LVDSC2_LVWACK(x)                     (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWACK_SHIFT))&PMC_LVDSC2_LVWACK_MASK)
#define PMC_LVDSC2_LVWF_MASK                     0x80u
#define PMC_LVDSC2_LVWF_SHIFT                    7
#define PMC_LVDSC2_LVWF_WIDTH                    1
#define PMC_LVDSC2_LVWF(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWF_SHIFT))&PMC_LVDSC2_LVWF_MASK)
/* REGSC Bit Fields */
#define PMC_REGSC_BGBE_MASK                      0x1u
#define PMC_REGSC_BGBE_SHIFT                     0
#define PMC_REGSC_BGBE_WIDTH                     1
#define PMC_REGSC_BGBE(x)                        (((uint8_t)(((uint8_t)(x))<<PMC_REGSC_BGBE_SHIFT))&PMC_REGSC_BGBE_MASK)
#define PMC_REGSC_BGBDS_MASK                     0x2u
#define PMC_REGSC_BGBDS_SHIFT                    1
#define PMC_REGSC_BGBDS_WIDTH                    1
#define PMC_REGSC_BGBDS(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_REGSC_BGBDS_SHIFT))&PMC_REGSC_BGBDS_MASK)
#define PMC_REGSC_REGONS_MASK                    0x4u
#define PMC_REGSC_REGONS_SHIFT                   2
#define PMC_REGSC_REGONS_WIDTH                   1
#define PMC_REGSC_REGONS(x)                      (((uint8_t)(((uint8_t)(x))<<PMC_REGSC_REGONS_SHIFT))&PMC_REGSC_REGONS_MASK)
#define PMC_REGSC_ACKISO_MASK                    0x8u
#define PMC_REGSC_ACKISO_SHIFT                   3
#define PMC_REGSC_ACKISO_WIDTH                   1
#define PMC_REGSC_ACKISO(x)                      (((uint8_t)(((uint8_t)(x))<<PMC_REGSC_ACKISO_SHIFT))&PMC_REGSC_ACKISO_MASK)
#define PMC_REGSC_BGEN_MASK                      0x10u
#define PMC_REGSC_BGEN_SHIFT                     4
#define PMC_REGSC_BGEN_WIDTH                     1
#define PMC_REGSC_BGEN(x)                        (((uint8_t)(((uint8_t)(x))<<PMC_REGSC_BGEN_SHIFT))&PMC_REGSC_BGEN_MASK)

/*!
 * @}
 */ /* end of group PMC_Register_Masks */


/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE                                 (0x4007D000u)
/** Peripheral PMC base pointer */
#define PMC                                      ((PMC_Type *)PMC_BASE)
#define PMC_BASE_PTR                             (PMC)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS                           { PMC_BASE }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC }

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register instance definitions */
/* PMC */
#define PMC_LVDSC1                               PMC_LVDSC1_REG(PMC)
#define PMC_LVDSC2                               PMC_LVDSC2_REG(PMC)
#define PMC_REGSC                                PMC_REGSC_REG(PMC)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCR[32];                           /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  __O  uint32_t GPCLR;                             /**< Global Pin Control Low Register, offset: 0x80 */
  __O  uint32_t GPCHR;                             /**< Global Pin Control High Register, offset: 0x84 */
       uint8_t RESERVED_0[24];
  __IO uint32_t ISFR;                              /**< Interrupt Status Flag Register, offset: 0xA0 */
       uint8_t RESERVED_1[28];
  __IO uint32_t DFER;                              /**< Digital Filter Enable Register, offset: 0xC0 */
  __IO uint32_t DFCR;                              /**< Digital Filter Clock Register, offset: 0xC4 */
  __IO uint32_t DFWR;                              /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_Type, *PORT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register accessors */
#define PORT_PCR_REG(base,index)                 ((base)->PCR[index])
#define PORT_PCR_COUNT                           32
#define PORT_GPCLR_REG(base)                     ((base)->GPCLR)
#define PORT_GPCHR_REG(base)                     ((base)->GPCHR)
#define PORT_ISFR_REG(base)                      ((base)->ISFR)
#define PORT_DFER_REG(base)                      ((base)->DFER)
#define PORT_DFCR_REG(base)                      ((base)->DFCR)
#define PORT_DFWR_REG(base)                      ((base)->DFWR)

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/* PCR Bit Fields */
#define PORT_PCR_PS_MASK                         0x1u
#define PORT_PCR_PS_SHIFT                        0
#define PORT_PCR_PS_WIDTH                        1
#define PORT_PCR_PS(x)                           (((uint32_t)(((uint32_t)(x))<<PORT_PCR_PS_SHIFT))&PORT_PCR_PS_MASK)
#define PORT_PCR_PE_MASK                         0x2u
#define PORT_PCR_PE_SHIFT                        1
#define PORT_PCR_PE_WIDTH                        1
#define PORT_PCR_PE(x)                           (((uint32_t)(((uint32_t)(x))<<PORT_PCR_PE_SHIFT))&PORT_PCR_PE_MASK)
#define PORT_PCR_SRE_MASK                        0x4u
#define PORT_PCR_SRE_SHIFT                       2
#define PORT_PCR_SRE_WIDTH                       1
#define PORT_PCR_SRE(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_SRE_SHIFT))&PORT_PCR_SRE_MASK)
#define PORT_PCR_PFE_MASK                        0x10u
#define PORT_PCR_PFE_SHIFT                       4
#define PORT_PCR_PFE_WIDTH                       1
#define PORT_PCR_PFE(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_PFE_SHIFT))&PORT_PCR_PFE_MASK)
#define PORT_PCR_ODE_MASK                        0x20u
#define PORT_PCR_ODE_SHIFT                       5
#define PORT_PCR_ODE_WIDTH                       1
#define PORT_PCR_ODE(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_ODE_SHIFT))&PORT_PCR_ODE_MASK)
#define PORT_PCR_DSE_MASK                        0x40u
#define PORT_PCR_DSE_SHIFT                       6
#define PORT_PCR_DSE_WIDTH                       1
#define PORT_PCR_DSE(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_DSE_SHIFT))&PORT_PCR_DSE_MASK)
#define PORT_PCR_MUX_MASK                        0x700u
#define PORT_PCR_MUX_SHIFT                       8
#define PORT_PCR_MUX_WIDTH                       3
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_MUX_SHIFT))&PORT_PCR_MUX_MASK)
#define PORT_PCR_LK_MASK                         0x8000u
#define PORT_PCR_LK_SHIFT                        15
#define PORT_PCR_LK_WIDTH                        1
#define PORT_PCR_LK(x)                           (((uint32_t)(((uint32_t)(x))<<PORT_PCR_LK_SHIFT))&PORT_PCR_LK_MASK)
#define PORT_PCR_IRQC_MASK                       0xF0000u
#define PORT_PCR_IRQC_SHIFT                      16
#define PORT_PCR_IRQC_WIDTH                      4
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_PCR_IRQC_SHIFT))&PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF_MASK                        0x1000000u
#define PORT_PCR_ISF_SHIFT                       24
#define PORT_PCR_ISF_WIDTH                       1
#define PORT_PCR_ISF(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_ISF_SHIFT))&PORT_PCR_ISF_MASK)
/* GPCLR Bit Fields */
#define PORT_GPCLR_GPWD_MASK                     0xFFFFu
#define PORT_GPCLR_GPWD_SHIFT                    0
#define PORT_GPCLR_GPWD_WIDTH                    16
#define PORT_GPCLR_GPWD(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCLR_GPWD_SHIFT))&PORT_GPCLR_GPWD_MASK)
#define PORT_GPCLR_GPWE_MASK                     0xFFFF0000u
#define PORT_GPCLR_GPWE_SHIFT                    16
#define PORT_GPCLR_GPWE_WIDTH                    16
#define PORT_GPCLR_GPWE(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCLR_GPWE_SHIFT))&PORT_GPCLR_GPWE_MASK)
/* GPCHR Bit Fields */
#define PORT_GPCHR_GPWD_MASK                     0xFFFFu
#define PORT_GPCHR_GPWD_SHIFT                    0
#define PORT_GPCHR_GPWD_WIDTH                    16
#define PORT_GPCHR_GPWD(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCHR_GPWD_SHIFT))&PORT_GPCHR_GPWD_MASK)
#define PORT_GPCHR_GPWE_MASK                     0xFFFF0000u
#define PORT_GPCHR_GPWE_SHIFT                    16
#define PORT_GPCHR_GPWE_WIDTH                    16
#define PORT_GPCHR_GPWE(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCHR_GPWE_SHIFT))&PORT_GPCHR_GPWE_MASK)
/* ISFR Bit Fields */
#define PORT_ISFR_ISF_MASK                       0xFFFFFFFFu
#define PORT_ISFR_ISF_SHIFT                      0
#define PORT_ISFR_ISF_WIDTH                      32
#define PORT_ISFR_ISF(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_ISFR_ISF_SHIFT))&PORT_ISFR_ISF_MASK)
/* DFER Bit Fields */
#define PORT_DFER_DFE_MASK                       0xFFFFFFFFu
#define PORT_DFER_DFE_SHIFT                      0
#define PORT_DFER_DFE_WIDTH                      32
#define PORT_DFER_DFE(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_DFER_DFE_SHIFT))&PORT_DFER_DFE_MASK)
/* DFCR Bit Fields */
#define PORT_DFCR_CS_MASK                        0x1u
#define PORT_DFCR_CS_SHIFT                       0
#define PORT_DFCR_CS_WIDTH                       1
#define PORT_DFCR_CS(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_DFCR_CS_SHIFT))&PORT_DFCR_CS_MASK)
/* DFWR Bit Fields */
#define PORT_DFWR_FILT_MASK                      0x1Fu
#define PORT_DFWR_FILT_SHIFT                     0
#define PORT_DFWR_FILT_WIDTH                     5
#define PORT_DFWR_FILT(x)                        (((uint32_t)(((uint32_t)(x))<<PORT_DFWR_FILT_SHIFT))&PORT_DFWR_FILT_MASK)

/*!
 * @}
 */ /* end of group PORT_Register_Masks */


/* PORT - Peripheral instance base addresses */
/** Peripheral PORTA base address */
#define PORTA_BASE                               (0x40049000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_Type *)PORTA_BASE)
#define PORTA_BASE_PTR                           (PORTA)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4004A000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_Type *)PORTB_BASE)
#define PORTB_BASE_PTR                           (PORTB)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_Type *)PORTC_BASE)
#define PORTC_BASE_PTR                           (PORTC)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4004C000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_Type *)PORTD_BASE)
#define PORTD_BASE_PTR                           (PORTD)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4004D000u)
/** Peripheral PORTE base pointer */
#define PORTE                                    ((PORT_Type *)PORTE_BASE)
#define PORTE_BASE_PTR                           (PORTE)
/** Array initializer of PORT peripheral base addresses */
#define PORT_BASE_ADDRS                          { PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE }
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORTA, PORTB, PORTC, PORTD, PORTE }

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register instance definitions */
/* PORTA */
#define PORTA_PCR0                               PORT_PCR_REG(PORTA,0)
#define PORTA_PCR1                               PORT_PCR_REG(PORTA,1)
#define PORTA_PCR2                               PORT_PCR_REG(PORTA,2)
#define PORTA_PCR3                               PORT_PCR_REG(PORTA,3)
#define PORTA_PCR4                               PORT_PCR_REG(PORTA,4)
#define PORTA_PCR5                               PORT_PCR_REG(PORTA,5)
#define PORTA_PCR6                               PORT_PCR_REG(PORTA,6)
#define PORTA_PCR7                               PORT_PCR_REG(PORTA,7)
#define PORTA_PCR8                               PORT_PCR_REG(PORTA,8)
#define PORTA_PCR9                               PORT_PCR_REG(PORTA,9)
#define PORTA_PCR10                              PORT_PCR_REG(PORTA,10)
#define PORTA_PCR11                              PORT_PCR_REG(PORTA,11)
#define PORTA_PCR12                              PORT_PCR_REG(PORTA,12)
#define PORTA_PCR13                              PORT_PCR_REG(PORTA,13)
#define PORTA_PCR14                              PORT_PCR_REG(PORTA,14)
#define PORTA_PCR15                              PORT_PCR_REG(PORTA,15)
#define PORTA_PCR16                              PORT_PCR_REG(PORTA,16)
#define PORTA_PCR17                              PORT_PCR_REG(PORTA,17)
#define PORTA_PCR18                              PORT_PCR_REG(PORTA,18)
#define PORTA_PCR19                              PORT_PCR_REG(PORTA,19)
#define PORTA_PCR20                              PORT_PCR_REG(PORTA,20)
#define PORTA_PCR21                              PORT_PCR_REG(PORTA,21)
#define PORTA_PCR22                              PORT_PCR_REG(PORTA,22)
#define PORTA_PCR23                              PORT_PCR_REG(PORTA,23)
#define PORTA_PCR24                              PORT_PCR_REG(PORTA,24)
#define PORTA_PCR25                              PORT_PCR_REG(PORTA,25)
#define PORTA_PCR26                              PORT_PCR_REG(PORTA,26)
#define PORTA_PCR27                              PORT_PCR_REG(PORTA,27)
#define PORTA_PCR28                              PORT_PCR_REG(PORTA,28)
#define PORTA_PCR29                              PORT_PCR_REG(PORTA,29)
#define PORTA_PCR30                              PORT_PCR_REG(PORTA,30)
#define PORTA_PCR31                              PORT_PCR_REG(PORTA,31)
#define PORTA_GPCLR                              PORT_GPCLR_REG(PORTA)
#define PORTA_GPCHR                              PORT_GPCHR_REG(PORTA)
#define PORTA_ISFR                               PORT_ISFR_REG(PORTA)
/* PORTB */
#define PORTB_PCR0                               PORT_PCR_REG(PORTB,0)
#define PORTB_PCR1                               PORT_PCR_REG(PORTB,1)
#define PORTB_PCR2                               PORT_PCR_REG(PORTB,2)
#define PORTB_PCR3                               PORT_PCR_REG(PORTB,3)
#define PORTB_PCR4                               PORT_PCR_REG(PORTB,4)
#define PORTB_PCR5                               PORT_PCR_REG(PORTB,5)
#define PORTB_PCR6                               PORT_PCR_REG(PORTB,6)
#define PORTB_PCR7                               PORT_PCR_REG(PORTB,7)
#define PORTB_PCR8                               PORT_PCR_REG(PORTB,8)
#define PORTB_PCR9                               PORT_PCR_REG(PORTB,9)
#define PORTB_PCR10                              PORT_PCR_REG(PORTB,10)
#define PORTB_PCR11                              PORT_PCR_REG(PORTB,11)
#define PORTB_PCR12                              PORT_PCR_REG(PORTB,12)
#define PORTB_PCR13                              PORT_PCR_REG(PORTB,13)
#define PORTB_PCR14                              PORT_PCR_REG(PORTB,14)
#define PORTB_PCR15                              PORT_PCR_REG(PORTB,15)
#define PORTB_PCR16                              PORT_PCR_REG(PORTB,16)
#define PORTB_PCR17                              PORT_PCR_REG(PORTB,17)
#define PORTB_PCR18                              PORT_PCR_REG(PORTB,18)
#define PORTB_PCR19                              PORT_PCR_REG(PORTB,19)
#define PORTB_PCR20                              PORT_PCR_REG(PORTB,20)
#define PORTB_PCR21                              PORT_PCR_REG(PORTB,21)
#define PORTB_PCR22                              PORT_PCR_REG(PORTB,22)
#define PORTB_PCR23                              PORT_PCR_REG(PORTB,23)
#define PORTB_PCR24                              PORT_PCR_REG(PORTB,24)
#define PORTB_PCR25                              PORT_PCR_REG(PORTB,25)
#define PORTB_PCR26                              PORT_PCR_REG(PORTB,26)
#define PORTB_PCR27                              PORT_PCR_REG(PORTB,27)
#define PORTB_PCR28                              PORT_PCR_REG(PORTB,28)
#define PORTB_PCR29                              PORT_PCR_REG(PORTB,29)
#define PORTB_PCR30                              PORT_PCR_REG(PORTB,30)
#define PORTB_PCR31                              PORT_PCR_REG(PORTB,31)
#define PORTB_GPCLR                              PORT_GPCLR_REG(PORTB)
#define PORTB_GPCHR                              PORT_GPCHR_REG(PORTB)
#define PORTB_ISFR                               PORT_ISFR_REG(PORTB)
/* PORTC */
#define PORTC_PCR0                               PORT_PCR_REG(PORTC,0)
#define PORTC_PCR1                               PORT_PCR_REG(PORTC,1)
#define PORTC_PCR2                               PORT_PCR_REG(PORTC,2)
#define PORTC_PCR3                               PORT_PCR_REG(PORTC,3)
#define PORTC_PCR4                               PORT_PCR_REG(PORTC,4)
#define PORTC_PCR5                               PORT_PCR_REG(PORTC,5)
#define PORTC_PCR6                               PORT_PCR_REG(PORTC,6)
#define PORTC_PCR7                               PORT_PCR_REG(PORTC,7)
#define PORTC_PCR8                               PORT_PCR_REG(PORTC,8)
#define PORTC_PCR9                               PORT_PCR_REG(PORTC,9)
#define PORTC_PCR10                              PORT_PCR_REG(PORTC,10)
#define PORTC_PCR11                              PORT_PCR_REG(PORTC,11)
#define PORTC_PCR12                              PORT_PCR_REG(PORTC,12)
#define PORTC_PCR13                              PORT_PCR_REG(PORTC,13)
#define PORTC_PCR14                              PORT_PCR_REG(PORTC,14)
#define PORTC_PCR15                              PORT_PCR_REG(PORTC,15)
#define PORTC_PCR16                              PORT_PCR_REG(PORTC,16)
#define PORTC_PCR17                              PORT_PCR_REG(PORTC,17)
#define PORTC_PCR18                              PORT_PCR_REG(PORTC,18)
#define PORTC_PCR19                              PORT_PCR_REG(PORTC,19)
#define PORTC_PCR20                              PORT_PCR_REG(PORTC,20)
#define PORTC_PCR21                              PORT_PCR_REG(PORTC,21)
#define PORTC_PCR22                              PORT_PCR_REG(PORTC,22)
#define PORTC_PCR23                              PORT_PCR_REG(PORTC,23)
#define PORTC_PCR24                              PORT_PCR_REG(PORTC,24)
#define PORTC_PCR25                              PORT_PCR_REG(PORTC,25)
#define PORTC_PCR26                              PORT_PCR_REG(PORTC,26)
#define PORTC_PCR27                              PORT_PCR_REG(PORTC,27)
#define PORTC_PCR28                              PORT_PCR_REG(PORTC,28)
#define PORTC_PCR29                              PORT_PCR_REG(PORTC,29)
#define PORTC_PCR30                              PORT_PCR_REG(PORTC,30)
#define PORTC_PCR31                              PORT_PCR_REG(PORTC,31)
#define PORTC_GPCLR                              PORT_GPCLR_REG(PORTC)
#define PORTC_GPCHR                              PORT_GPCHR_REG(PORTC)
#define PORTC_ISFR                               PORT_ISFR_REG(PORTC)
/* PORTD */
#define PORTD_PCR0                               PORT_PCR_REG(PORTD,0)
#define PORTD_PCR1                               PORT_PCR_REG(PORTD,1)
#define PORTD_PCR2                               PORT_PCR_REG(PORTD,2)
#define PORTD_PCR3                               PORT_PCR_REG(PORTD,3)
#define PORTD_PCR4                               PORT_PCR_REG(PORTD,4)
#define PORTD_PCR5                               PORT_PCR_REG(PORTD,5)
#define PORTD_PCR6                               PORT_PCR_REG(PORTD,6)
#define PORTD_PCR7                               PORT_PCR_REG(PORTD,7)
#define PORTD_PCR8                               PORT_PCR_REG(PORTD,8)
#define PORTD_PCR9                               PORT_PCR_REG(PORTD,9)
#define PORTD_PCR10                              PORT_PCR_REG(PORTD,10)
#define PORTD_PCR11                              PORT_PCR_REG(PORTD,11)
#define PORTD_PCR12                              PORT_PCR_REG(PORTD,12)
#define PORTD_PCR13                              PORT_PCR_REG(PORTD,13)
#define PORTD_PCR14                              PORT_PCR_REG(PORTD,14)
#define PORTD_PCR15                              PORT_PCR_REG(PORTD,15)
#define PORTD_PCR16                              PORT_PCR_REG(PORTD,16)
#define PORTD_PCR17                              PORT_PCR_REG(PORTD,17)
#define PORTD_PCR18                              PORT_PCR_REG(PORTD,18)
#define PORTD_PCR19                              PORT_PCR_REG(PORTD,19)
#define PORTD_PCR20                              PORT_PCR_REG(PORTD,20)
#define PORTD_PCR21                              PORT_PCR_REG(PORTD,21)
#define PORTD_PCR22                              PORT_PCR_REG(PORTD,22)
#define PORTD_PCR23                              PORT_PCR_REG(PORTD,23)
#define PORTD_PCR24                              PORT_PCR_REG(PORTD,24)
#define PORTD_PCR25                              PORT_PCR_REG(PORTD,25)
#define PORTD_PCR26                              PORT_PCR_REG(PORTD,26)
#define PORTD_PCR27                              PORT_PCR_REG(PORTD,27)
#define PORTD_PCR28                              PORT_PCR_REG(PORTD,28)
#define PORTD_PCR29                              PORT_PCR_REG(PORTD,29)
#define PORTD_PCR30                              PORT_PCR_REG(PORTD,30)
#define PORTD_PCR31                              PORT_PCR_REG(PORTD,31)
#define PORTD_GPCLR                              PORT_GPCLR_REG(PORTD)
#define PORTD_GPCHR                              PORT_GPCHR_REG(PORTD)
#define PORTD_ISFR                               PORT_ISFR_REG(PORTD)
#define PORTD_DFER                               PORT_DFER_REG(PORTD)
#define PORTD_DFCR                               PORT_DFCR_REG(PORTD)
#define PORTD_DFWR                               PORT_DFWR_REG(PORTD)
/* PORTE */
#define PORTE_PCR0                               PORT_PCR_REG(PORTE,0)
#define PORTE_PCR1                               PORT_PCR_REG(PORTE,1)
#define PORTE_PCR2                               PORT_PCR_REG(PORTE,2)
#define PORTE_PCR3                               PORT_PCR_REG(PORTE,3)
#define PORTE_PCR4                               PORT_PCR_REG(PORTE,4)
#define PORTE_PCR5                               PORT_PCR_REG(PORTE,5)
#define PORTE_PCR6                               PORT_PCR_REG(PORTE,6)
#define PORTE_PCR7                               PORT_PCR_REG(PORTE,7)
#define PORTE_PCR8                               PORT_PCR_REG(PORTE,8)
#define PORTE_PCR9                               PORT_PCR_REG(PORTE,9)
#define PORTE_PCR10                              PORT_PCR_REG(PORTE,10)
#define PORTE_PCR11                              PORT_PCR_REG(PORTE,11)
#define PORTE_PCR12                              PORT_PCR_REG(PORTE,12)
#define PORTE_PCR13                              PORT_PCR_REG(PORTE,13)
#define PORTE_PCR14                              PORT_PCR_REG(PORTE,14)
#define PORTE_PCR15                              PORT_PCR_REG(PORTE,15)
#define PORTE_PCR16                              PORT_PCR_REG(PORTE,16)
#define PORTE_PCR17                              PORT_PCR_REG(PORTE,17)
#define PORTE_PCR18                              PORT_PCR_REG(PORTE,18)
#define PORTE_PCR19                              PORT_PCR_REG(PORTE,19)
#define PORTE_PCR20                              PORT_PCR_REG(PORTE,20)
#define PORTE_PCR21                              PORT_PCR_REG(PORTE,21)
#define PORTE_PCR22                              PORT_PCR_REG(PORTE,22)
#define PORTE_PCR23                              PORT_PCR_REG(PORTE,23)
#define PORTE_PCR24                              PORT_PCR_REG(PORTE,24)
#define PORTE_PCR25                              PORT_PCR_REG(PORTE,25)
#define PORTE_PCR26                              PORT_PCR_REG(PORTE,26)
#define PORTE_PCR27                              PORT_PCR_REG(PORTE,27)
#define PORTE_PCR28                              PORT_PCR_REG(PORTE,28)
#define PORTE_PCR29                              PORT_PCR_REG(PORTE,29)
#define PORTE_PCR30                              PORT_PCR_REG(PORTE,30)
#define PORTE_PCR31                              PORT_PCR_REG(PORTE,31)
#define PORTE_GPCLR                              PORT_GPCLR_REG(PORTE)
#define PORTE_GPCHR                              PORT_GPCHR_REG(PORTE)
#define PORTE_ISFR                               PORT_ISFR_REG(PORTE)

/* PORT - Register array accessors */
#define PORTA_PCR(index)                         PORT_PCR_REG(PORTA,index)
#define PORTB_PCR(index)                         PORT_PCR_REG(PORTB,index)
#define PORTC_PCR(index)                         PORT_PCR_REG(PORTC,index)
#define PORTD_PCR(index)                         PORT_PCR_REG(PORTD,index)
#define PORTE_PCR(index)                         PORT_PCR_REG(PORTE,index)

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Peripheral_Access_Layer PWM Peripheral Access Layer
 * @{
 */

/** PWM - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x60 */
    __I  uint16_t CNT;                               /**< Counter Register, array offset: 0x0, array step: 0x60 */
    __IO uint16_t INIT;                              /**< Initial Count Register, array offset: 0x2, array step: 0x60 */
    __IO uint16_t CTRL2;                             /**< Control 2 Register, array offset: 0x4, array step: 0x60 */
    __IO uint16_t CTRL;                              /**< Control Register, array offset: 0x6, array step: 0x60 */
         uint8_t RESERVED_0[2];
    __IO uint16_t VAL0;                              /**< Value Register 0, array offset: 0xA, array step: 0x60 */
    __IO uint16_t FRACVAL1;                          /**< Fractional Value Register 1, array offset: 0xC, array step: 0x60 */
    __IO uint16_t VAL1;                              /**< Value Register 1, array offset: 0xE, array step: 0x60 */
    __IO uint16_t FRACVAL2;                          /**< Fractional Value Register 2, array offset: 0x10, array step: 0x60 */
    __IO uint16_t VAL2;                              /**< Value Register 2, array offset: 0x12, array step: 0x60 */
    __IO uint16_t FRACVAL3;                          /**< Fractional Value Register 3, array offset: 0x14, array step: 0x60 */
    __IO uint16_t VAL3;                              /**< Value Register 3, array offset: 0x16, array step: 0x60 */
    __IO uint16_t FRACVAL4;                          /**< Fractional Value Register 4, array offset: 0x18, array step: 0x60 */
    __IO uint16_t VAL4;                              /**< Value Register 4, array offset: 0x1A, array step: 0x60 */
    __IO uint16_t FRACVAL5;                          /**< Fractional Value Register 5, array offset: 0x1C, array step: 0x60 */
    __IO uint16_t VAL5;                              /**< Value Register 5, array offset: 0x1E, array step: 0x60 */
    __IO uint16_t FRCTRL;                            /**< Fractional Control Register, array offset: 0x20, array step: 0x60 */
    __IO uint16_t OCTRL;                             /**< Output Control Register, array offset: 0x22, array step: 0x60 */
    __IO uint16_t STS;                               /**< Status Register, array offset: 0x24, array step: 0x60 */
    __IO uint16_t INTEN;                             /**< Interrupt Enable Register, array offset: 0x26, array step: 0x60 */
    __IO uint16_t DMAEN;                             /**< DMA Enable Register, array offset: 0x28, array step: 0x60 */
    __IO uint16_t TCTRL;                             /**< Output Trigger Control Register, array offset: 0x2A, array step: 0x60 */
    __IO uint16_t DISMAP[2];                         /**< Fault Disable Mapping Register 0..Fault Disable Mapping Register 1, array offset: 0x2C, array step: index*0x60, index2*0x2 */
    __IO uint16_t DTCNT0;                            /**< Deadtime Count Register 0, array offset: 0x30, array step: 0x60 */
    __IO uint16_t DTCNT1;                            /**< Deadtime Count Register 1, array offset: 0x32, array step: 0x60 */
    __IO uint16_t CAPTCTRLA;                         /**< Capture Control A Register, array offset: 0x34, array step: 0x60 */
    __IO uint16_t CAPTCOMPA;                         /**< Capture Compare A Register, array offset: 0x36, array step: 0x60 */
    __IO uint16_t CAPTCTRLB;                         /**< Capture Control B Register, array offset: 0x38, array step: 0x60 */
    __IO uint16_t CAPTCOMPB;                         /**< Capture Compare B Register, array offset: 0x3A, array step: 0x60 */
    __IO uint16_t CAPTCTRLX;                         /**< Capture Control X Register, array offset: 0x3C, array step: 0x60 */
    __IO uint16_t CAPTCOMPX;                         /**< Capture Compare X Register, array offset: 0x3E, array step: 0x60 */
    __I  uint16_t CVAL0;                             /**< Capture Value 0 Register, array offset: 0x40, array step: 0x60 */
    __I  uint16_t CVAL0CYC;                          /**< Capture Value 0 Cycle Register, array offset: 0x42, array step: 0x60 */
    __I  uint16_t CVAL1;                             /**< Capture Value 1 Register, array offset: 0x44, array step: 0x60 */
    __I  uint16_t CVAL1CYC;                          /**< Capture Value 1 Cycle Register, array offset: 0x46, array step: 0x60 */
    __I  uint16_t CVAL2;                             /**< Capture Value 2 Register, array offset: 0x48, array step: 0x60 */
    __I  uint16_t CVAL2CYC;                          /**< Capture Value 2 Cycle Register, array offset: 0x4A, array step: 0x60 */
    __I  uint16_t CVAL3;                             /**< Capture Value 3 Register, array offset: 0x4C, array step: 0x60 */
    __I  uint16_t CVAL3CYC;                          /**< Capture Value 3 Cycle Register, array offset: 0x4E, array step: 0x60 */
    __I  uint16_t CVAL4;                             /**< Capture Value 4 Register, array offset: 0x50, array step: 0x60 */
    __I  uint16_t CVAL4CYC;                          /**< Capture Value 4 Cycle Register, array offset: 0x52, array step: 0x60 */
    __I  uint16_t CVAL5;                             /**< Capture Value 5 Register, array offset: 0x54, array step: 0x60 */
    __I  uint16_t CVAL5CYC;                          /**< Capture Value 5 Cycle Register, array offset: 0x56, array step: 0x60 */
         uint8_t RESERVED_1[8];
  } SM[4];
  __IO uint16_t OUTEN;                             /**< Output Enable Register, offset: 0x180 */
  __IO uint16_t MASK;                              /**< Mask Register, offset: 0x182 */
  __IO uint16_t SWCOUT;                            /**< Software Controlled Output Register, offset: 0x184 */
  __IO uint16_t DTSRCSEL;                          /**< PWM Source Select Register, offset: 0x186 */
  __IO uint16_t MCTRL;                             /**< Master Control Register, offset: 0x188 */
  __IO uint16_t MCTRL2;                            /**< Master Control 2 Register, offset: 0x18A */
  __IO uint16_t FCTRL;                             /**< Fault Control Register, offset: 0x18C */
  __IO uint16_t FSTS;                              /**< Fault Status Register, offset: 0x18E */
  __IO uint16_t FFILT;                             /**< Fault Filter Register, offset: 0x190 */
  __IO uint16_t FTST;                              /**< Fault Test Register, offset: 0x192 */
  __IO uint16_t FCTRL2;                            /**< Fault Control 2 Register, offset: 0x194 */
} PWM_Type, *PWM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Accessor_Macros PWM - Register accessor macros
 * @{
 */


/* PWM - Register accessors */
#define PWM_CNT_REG(base,index)                  ((base)->SM[index].CNT)
#define PWM_CNT_COUNT                            4
#define PWM_INIT_REG(base,index)                 ((base)->SM[index].INIT)
#define PWM_INIT_COUNT                           4
#define PWM_CTRL2_REG(base,index)                ((base)->SM[index].CTRL2)
#define PWM_CTRL2_COUNT                          4
#define PWM_CTRL_REG(base,index)                 ((base)->SM[index].CTRL)
#define PWM_CTRL_COUNT                           4
#define PWM_VAL0_REG(base,index)                 ((base)->SM[index].VAL0)
#define PWM_VAL0_COUNT                           4
#define PWM_FRACVAL1_REG(base,index)             ((base)->SM[index].FRACVAL1)
#define PWM_FRACVAL1_COUNT                       4
#define PWM_VAL1_REG(base,index)                 ((base)->SM[index].VAL1)
#define PWM_VAL1_COUNT                           4
#define PWM_FRACVAL2_REG(base,index)             ((base)->SM[index].FRACVAL2)
#define PWM_FRACVAL2_COUNT                       4
#define PWM_VAL2_REG(base,index)                 ((base)->SM[index].VAL2)
#define PWM_VAL2_COUNT                           4
#define PWM_FRACVAL3_REG(base,index)             ((base)->SM[index].FRACVAL3)
#define PWM_FRACVAL3_COUNT                       4
#define PWM_VAL3_REG(base,index)                 ((base)->SM[index].VAL3)
#define PWM_VAL3_COUNT                           4
#define PWM_FRACVAL4_REG(base,index)             ((base)->SM[index].FRACVAL4)
#define PWM_FRACVAL4_COUNT                       4
#define PWM_VAL4_REG(base,index)                 ((base)->SM[index].VAL4)
#define PWM_VAL4_COUNT                           4
#define PWM_FRACVAL5_REG(base,index)             ((base)->SM[index].FRACVAL5)
#define PWM_FRACVAL5_COUNT                       4
#define PWM_VAL5_REG(base,index)                 ((base)->SM[index].VAL5)
#define PWM_VAL5_COUNT                           4
#define PWM_FRCTRL_REG(base,index)               ((base)->SM[index].FRCTRL)
#define PWM_FRCTRL_COUNT                         4
#define PWM_OCTRL_REG(base,index)                ((base)->SM[index].OCTRL)
#define PWM_OCTRL_COUNT                          4
#define PWM_STS_REG(base,index)                  ((base)->SM[index].STS)
#define PWM_STS_COUNT                            4
#define PWM_INTEN_REG(base,index)                ((base)->SM[index].INTEN)
#define PWM_INTEN_COUNT                          4
#define PWM_DMAEN_REG(base,index)                ((base)->SM[index].DMAEN)
#define PWM_DMAEN_COUNT                          4
#define PWM_TCTRL_REG(base,index)                ((base)->SM[index].TCTRL)
#define PWM_TCTRL_COUNT                          4
#define PWM_DISMAP_REG(base,index,index2)        ((base)->SM[index].DISMAP[index2])
#define PWM_DISMAP_COUNT                         4
#define PWM_DISMAP_COUNT2                        2
#define PWM_DTCNT0_REG(base,index)               ((base)->SM[index].DTCNT0)
#define PWM_DTCNT0_COUNT                         4
#define PWM_DTCNT1_REG(base,index)               ((base)->SM[index].DTCNT1)
#define PWM_DTCNT1_COUNT                         4
#define PWM_CAPTCTRLA_REG(base,index)            ((base)->SM[index].CAPTCTRLA)
#define PWM_CAPTCTRLA_COUNT                      4
#define PWM_CAPTCOMPA_REG(base,index)            ((base)->SM[index].CAPTCOMPA)
#define PWM_CAPTCOMPA_COUNT                      4
#define PWM_CAPTCTRLB_REG(base,index)            ((base)->SM[index].CAPTCTRLB)
#define PWM_CAPTCTRLB_COUNT                      4
#define PWM_CAPTCOMPB_REG(base,index)            ((base)->SM[index].CAPTCOMPB)
#define PWM_CAPTCOMPB_COUNT                      4
#define PWM_CAPTCTRLX_REG(base,index)            ((base)->SM[index].CAPTCTRLX)
#define PWM_CAPTCTRLX_COUNT                      4
#define PWM_CAPTCOMPX_REG(base,index)            ((base)->SM[index].CAPTCOMPX)
#define PWM_CAPTCOMPX_COUNT                      4
#define PWM_CVAL0_REG(base,index)                ((base)->SM[index].CVAL0)
#define PWM_CVAL0_COUNT                          4
#define PWM_CVAL0CYC_REG(base,index)             ((base)->SM[index].CVAL0CYC)
#define PWM_CVAL0CYC_COUNT                       4
#define PWM_CVAL1_REG(base,index)                ((base)->SM[index].CVAL1)
#define PWM_CVAL1_COUNT                          4
#define PWM_CVAL1CYC_REG(base,index)             ((base)->SM[index].CVAL1CYC)
#define PWM_CVAL1CYC_COUNT                       4
#define PWM_CVAL2_REG(base,index)                ((base)->SM[index].CVAL2)
#define PWM_CVAL2_COUNT                          4
#define PWM_CVAL2CYC_REG(base,index)             ((base)->SM[index].CVAL2CYC)
#define PWM_CVAL2CYC_COUNT                       4
#define PWM_CVAL3_REG(base,index)                ((base)->SM[index].CVAL3)
#define PWM_CVAL3_COUNT                          4
#define PWM_CVAL3CYC_REG(base,index)             ((base)->SM[index].CVAL3CYC)
#define PWM_CVAL3CYC_COUNT                       4
#define PWM_CVAL4_REG(base,index)                ((base)->SM[index].CVAL4)
#define PWM_CVAL4_COUNT                          4
#define PWM_CVAL4CYC_REG(base,index)             ((base)->SM[index].CVAL4CYC)
#define PWM_CVAL4CYC_COUNT                       4
#define PWM_CVAL5_REG(base,index)                ((base)->SM[index].CVAL5)
#define PWM_CVAL5_COUNT                          4
#define PWM_CVAL5CYC_REG(base,index)             ((base)->SM[index].CVAL5CYC)
#define PWM_CVAL5CYC_COUNT                       4
#define PWM_OUTEN_REG(base)                      ((base)->OUTEN)
#define PWM_MASK_REG(base)                       ((base)->MASK)
#define PWM_SWCOUT_REG(base)                     ((base)->SWCOUT)
#define PWM_DTSRCSEL_REG(base)                   ((base)->DTSRCSEL)
#define PWM_MCTRL_REG(base)                      ((base)->MCTRL)
#define PWM_MCTRL2_REG(base)                     ((base)->MCTRL2)
#define PWM_FCTRL_REG(base)                      ((base)->FCTRL)
#define PWM_FSTS_REG(base)                       ((base)->FSTS)
#define PWM_FFILT_REG(base)                      ((base)->FFILT)
#define PWM_FTST_REG(base)                       ((base)->FTST)
#define PWM_FCTRL2_REG(base)                     ((base)->FCTRL2)

/*!
 * @}
 */ /* end of group PWM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Masks PWM Register Masks
 * @{
 */

/* CNT Bit Fields */
#define PWM_CNT_CNT_MASK                         0xFFFFu
#define PWM_CNT_CNT_SHIFT                        0
#define PWM_CNT_CNT_WIDTH                        16
#define PWM_CNT_CNT(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_CNT_CNT_SHIFT))&PWM_CNT_CNT_MASK)
/* INIT Bit Fields */
#define PWM_INIT_INIT_MASK                       0xFFFFu
#define PWM_INIT_INIT_SHIFT                      0
#define PWM_INIT_INIT_WIDTH                      16
#define PWM_INIT_INIT(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_INIT_INIT_SHIFT))&PWM_INIT_INIT_MASK)
/* CTRL2 Bit Fields */
#define PWM_CTRL2_CLK_SEL_MASK                   0x3u
#define PWM_CTRL2_CLK_SEL_SHIFT                  0
#define PWM_CTRL2_CLK_SEL_WIDTH                  2
#define PWM_CTRL2_CLK_SEL(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_CLK_SEL_SHIFT))&PWM_CTRL2_CLK_SEL_MASK)
#define PWM_CTRL2_RELOAD_SEL_MASK                0x4u
#define PWM_CTRL2_RELOAD_SEL_SHIFT               2
#define PWM_CTRL2_RELOAD_SEL_WIDTH               1
#define PWM_CTRL2_RELOAD_SEL(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_RELOAD_SEL_SHIFT))&PWM_CTRL2_RELOAD_SEL_MASK)
#define PWM_CTRL2_FORCE_SEL_MASK                 0x38u
#define PWM_CTRL2_FORCE_SEL_SHIFT                3
#define PWM_CTRL2_FORCE_SEL_WIDTH                3
#define PWM_CTRL2_FORCE_SEL(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_FORCE_SEL_SHIFT))&PWM_CTRL2_FORCE_SEL_MASK)
#define PWM_CTRL2_FORCE_MASK                     0x40u
#define PWM_CTRL2_FORCE_SHIFT                    6
#define PWM_CTRL2_FORCE_WIDTH                    1
#define PWM_CTRL2_FORCE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_FORCE_SHIFT))&PWM_CTRL2_FORCE_MASK)
#define PWM_CTRL2_FRCEN_MASK                     0x80u
#define PWM_CTRL2_FRCEN_SHIFT                    7
#define PWM_CTRL2_FRCEN_WIDTH                    1
#define PWM_CTRL2_FRCEN(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_FRCEN_SHIFT))&PWM_CTRL2_FRCEN_MASK)
#define PWM_CTRL2_INIT_SEL_MASK                  0x300u
#define PWM_CTRL2_INIT_SEL_SHIFT                 8
#define PWM_CTRL2_INIT_SEL_WIDTH                 2
#define PWM_CTRL2_INIT_SEL(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_INIT_SEL_SHIFT))&PWM_CTRL2_INIT_SEL_MASK)
#define PWM_CTRL2_PWMX_INIT_MASK                 0x400u
#define PWM_CTRL2_PWMX_INIT_SHIFT                10
#define PWM_CTRL2_PWMX_INIT_WIDTH                1
#define PWM_CTRL2_PWMX_INIT(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_PWMX_INIT_SHIFT))&PWM_CTRL2_PWMX_INIT_MASK)
#define PWM_CTRL2_PWM45_INIT_MASK                0x800u
#define PWM_CTRL2_PWM45_INIT_SHIFT               11
#define PWM_CTRL2_PWM45_INIT_WIDTH               1
#define PWM_CTRL2_PWM45_INIT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_PWM45_INIT_SHIFT))&PWM_CTRL2_PWM45_INIT_MASK)
#define PWM_CTRL2_PWM23_INIT_MASK                0x1000u
#define PWM_CTRL2_PWM23_INIT_SHIFT               12
#define PWM_CTRL2_PWM23_INIT_WIDTH               1
#define PWM_CTRL2_PWM23_INIT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_PWM23_INIT_SHIFT))&PWM_CTRL2_PWM23_INIT_MASK)
#define PWM_CTRL2_INDEP_MASK                     0x2000u
#define PWM_CTRL2_INDEP_SHIFT                    13
#define PWM_CTRL2_INDEP_WIDTH                    1
#define PWM_CTRL2_INDEP(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_INDEP_SHIFT))&PWM_CTRL2_INDEP_MASK)
#define PWM_CTRL2_WAITEN_MASK                    0x4000u
#define PWM_CTRL2_WAITEN_SHIFT                   14
#define PWM_CTRL2_WAITEN_WIDTH                   1
#define PWM_CTRL2_WAITEN(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_WAITEN_SHIFT))&PWM_CTRL2_WAITEN_MASK)
#define PWM_CTRL2_DBGEN_MASK                     0x8000u
#define PWM_CTRL2_DBGEN_SHIFT                    15
#define PWM_CTRL2_DBGEN_WIDTH                    1
#define PWM_CTRL2_DBGEN(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_DBGEN_SHIFT))&PWM_CTRL2_DBGEN_MASK)
/* CTRL Bit Fields */
#define PWM_CTRL_DBLEN_MASK                      0x1u
#define PWM_CTRL_DBLEN_SHIFT                     0
#define PWM_CTRL_DBLEN_WIDTH                     1
#define PWM_CTRL_DBLEN(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_DBLEN_SHIFT))&PWM_CTRL_DBLEN_MASK)
#define PWM_CTRL_DBLX_MASK                       0x2u
#define PWM_CTRL_DBLX_SHIFT                      1
#define PWM_CTRL_DBLX_WIDTH                      1
#define PWM_CTRL_DBLX(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_DBLX_SHIFT))&PWM_CTRL_DBLX_MASK)
#define PWM_CTRL_LDMOD_MASK                      0x4u
#define PWM_CTRL_LDMOD_SHIFT                     2
#define PWM_CTRL_LDMOD_WIDTH                     1
#define PWM_CTRL_LDMOD(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_LDMOD_SHIFT))&PWM_CTRL_LDMOD_MASK)
#define PWM_CTRL_SPLIT_MASK                      0x8u
#define PWM_CTRL_SPLIT_SHIFT                     3
#define PWM_CTRL_SPLIT_WIDTH                     1
#define PWM_CTRL_SPLIT(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_SPLIT_SHIFT))&PWM_CTRL_SPLIT_MASK)
#define PWM_CTRL_PRSC_MASK                       0x70u
#define PWM_CTRL_PRSC_SHIFT                      4
#define PWM_CTRL_PRSC_WIDTH                      3
#define PWM_CTRL_PRSC(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_PRSC_SHIFT))&PWM_CTRL_PRSC_MASK)
#define PWM_CTRL_COMPMODE_MASK                   0x80u
#define PWM_CTRL_COMPMODE_SHIFT                  7
#define PWM_CTRL_COMPMODE_WIDTH                  1
#define PWM_CTRL_COMPMODE(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_COMPMODE_SHIFT))&PWM_CTRL_COMPMODE_MASK)
#define PWM_CTRL_DT_MASK                         0x300u
#define PWM_CTRL_DT_SHIFT                        8
#define PWM_CTRL_DT_WIDTH                        2
#define PWM_CTRL_DT(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_DT_SHIFT))&PWM_CTRL_DT_MASK)
#define PWM_CTRL_FULL_MASK                       0x400u
#define PWM_CTRL_FULL_SHIFT                      10
#define PWM_CTRL_FULL_WIDTH                      1
#define PWM_CTRL_FULL(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_FULL_SHIFT))&PWM_CTRL_FULL_MASK)
#define PWM_CTRL_HALF_MASK                       0x800u
#define PWM_CTRL_HALF_SHIFT                      11
#define PWM_CTRL_HALF_WIDTH                      1
#define PWM_CTRL_HALF(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_HALF_SHIFT))&PWM_CTRL_HALF_MASK)
#define PWM_CTRL_LDFQ_MASK                       0xF000u
#define PWM_CTRL_LDFQ_SHIFT                      12
#define PWM_CTRL_LDFQ_WIDTH                      4
#define PWM_CTRL_LDFQ(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_LDFQ_SHIFT))&PWM_CTRL_LDFQ_MASK)
/* VAL0 Bit Fields */
#define PWM_VAL0_VAL0_MASK                       0xFFFFu
#define PWM_VAL0_VAL0_SHIFT                      0
#define PWM_VAL0_VAL0_WIDTH                      16
#define PWM_VAL0_VAL0(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL0_VAL0_SHIFT))&PWM_VAL0_VAL0_MASK)
/* FRACVAL1 Bit Fields */
#define PWM_FRACVAL1_FRACVAL1_MASK               0xF800u
#define PWM_FRACVAL1_FRACVAL1_SHIFT              11
#define PWM_FRACVAL1_FRACVAL1_WIDTH              5
#define PWM_FRACVAL1_FRACVAL1(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL1_FRACVAL1_SHIFT))&PWM_FRACVAL1_FRACVAL1_MASK)
/* VAL1 Bit Fields */
#define PWM_VAL1_VAL1_MASK                       0xFFFFu
#define PWM_VAL1_VAL1_SHIFT                      0
#define PWM_VAL1_VAL1_WIDTH                      16
#define PWM_VAL1_VAL1(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL1_VAL1_SHIFT))&PWM_VAL1_VAL1_MASK)
/* FRACVAL2 Bit Fields */
#define PWM_FRACVAL2_FRACVAL2_MASK               0xF800u
#define PWM_FRACVAL2_FRACVAL2_SHIFT              11
#define PWM_FRACVAL2_FRACVAL2_WIDTH              5
#define PWM_FRACVAL2_FRACVAL2(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL2_FRACVAL2_SHIFT))&PWM_FRACVAL2_FRACVAL2_MASK)
/* VAL2 Bit Fields */
#define PWM_VAL2_VAL2_MASK                       0xFFFFu
#define PWM_VAL2_VAL2_SHIFT                      0
#define PWM_VAL2_VAL2_WIDTH                      16
#define PWM_VAL2_VAL2(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL2_VAL2_SHIFT))&PWM_VAL2_VAL2_MASK)
/* FRACVAL3 Bit Fields */
#define PWM_FRACVAL3_FRACVAL3_MASK               0xF800u
#define PWM_FRACVAL3_FRACVAL3_SHIFT              11
#define PWM_FRACVAL3_FRACVAL3_WIDTH              5
#define PWM_FRACVAL3_FRACVAL3(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL3_FRACVAL3_SHIFT))&PWM_FRACVAL3_FRACVAL3_MASK)
/* VAL3 Bit Fields */
#define PWM_VAL3_VAL3_MASK                       0xFFFFu
#define PWM_VAL3_VAL3_SHIFT                      0
#define PWM_VAL3_VAL3_WIDTH                      16
#define PWM_VAL3_VAL3(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL3_VAL3_SHIFT))&PWM_VAL3_VAL3_MASK)
/* FRACVAL4 Bit Fields */
#define PWM_FRACVAL4_FRACVAL4_MASK               0xF800u
#define PWM_FRACVAL4_FRACVAL4_SHIFT              11
#define PWM_FRACVAL4_FRACVAL4_WIDTH              5
#define PWM_FRACVAL4_FRACVAL4(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL4_FRACVAL4_SHIFT))&PWM_FRACVAL4_FRACVAL4_MASK)
/* VAL4 Bit Fields */
#define PWM_VAL4_VAL4_MASK                       0xFFFFu
#define PWM_VAL4_VAL4_SHIFT                      0
#define PWM_VAL4_VAL4_WIDTH                      16
#define PWM_VAL4_VAL4(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL4_VAL4_SHIFT))&PWM_VAL4_VAL4_MASK)
/* FRACVAL5 Bit Fields */
#define PWM_FRACVAL5_FRACVAL5_MASK               0xF800u
#define PWM_FRACVAL5_FRACVAL5_SHIFT              11
#define PWM_FRACVAL5_FRACVAL5_WIDTH              5
#define PWM_FRACVAL5_FRACVAL5(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL5_FRACVAL5_SHIFT))&PWM_FRACVAL5_FRACVAL5_MASK)
/* VAL5 Bit Fields */
#define PWM_VAL5_VAL5_MASK                       0xFFFFu
#define PWM_VAL5_VAL5_SHIFT                      0
#define PWM_VAL5_VAL5_WIDTH                      16
#define PWM_VAL5_VAL5(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL5_VAL5_SHIFT))&PWM_VAL5_VAL5_MASK)
/* FRCTRL Bit Fields */
#define PWM_FRCTRL_FRAC1_EN_MASK                 0x2u
#define PWM_FRCTRL_FRAC1_EN_SHIFT                1
#define PWM_FRCTRL_FRAC1_EN_WIDTH                1
#define PWM_FRCTRL_FRAC1_EN(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_FRCTRL_FRAC1_EN_SHIFT))&PWM_FRCTRL_FRAC1_EN_MASK)
#define PWM_FRCTRL_FRAC23_EN_MASK                0x4u
#define PWM_FRCTRL_FRAC23_EN_SHIFT               2
#define PWM_FRCTRL_FRAC23_EN_WIDTH               1
#define PWM_FRCTRL_FRAC23_EN(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_FRCTRL_FRAC23_EN_SHIFT))&PWM_FRCTRL_FRAC23_EN_MASK)
#define PWM_FRCTRL_FRAC45_EN_MASK                0x10u
#define PWM_FRCTRL_FRAC45_EN_SHIFT               4
#define PWM_FRCTRL_FRAC45_EN_WIDTH               1
#define PWM_FRCTRL_FRAC45_EN(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_FRCTRL_FRAC45_EN_SHIFT))&PWM_FRCTRL_FRAC45_EN_MASK)
#define PWM_FRCTRL_FRAC_PU_MASK                  0x100u
#define PWM_FRCTRL_FRAC_PU_SHIFT                 8
#define PWM_FRCTRL_FRAC_PU_WIDTH                 1
#define PWM_FRCTRL_FRAC_PU(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_FRCTRL_FRAC_PU_SHIFT))&PWM_FRCTRL_FRAC_PU_MASK)
#define PWM_FRCTRL_TEST_MASK                     0x8000u
#define PWM_FRCTRL_TEST_SHIFT                    15
#define PWM_FRCTRL_TEST_WIDTH                    1
#define PWM_FRCTRL_TEST(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_FRCTRL_TEST_SHIFT))&PWM_FRCTRL_TEST_MASK)
/* OCTRL Bit Fields */
#define PWM_OCTRL_PWMXFS_MASK                    0x3u
#define PWM_OCTRL_PWMXFS_SHIFT                   0
#define PWM_OCTRL_PWMXFS_WIDTH                   2
#define PWM_OCTRL_PWMXFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMXFS_SHIFT))&PWM_OCTRL_PWMXFS_MASK)
#define PWM_OCTRL_PWMBFS_MASK                    0xCu
#define PWM_OCTRL_PWMBFS_SHIFT                   2
#define PWM_OCTRL_PWMBFS_WIDTH                   2
#define PWM_OCTRL_PWMBFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMBFS_SHIFT))&PWM_OCTRL_PWMBFS_MASK)
#define PWM_OCTRL_PWMAFS_MASK                    0x30u
#define PWM_OCTRL_PWMAFS_SHIFT                   4
#define PWM_OCTRL_PWMAFS_WIDTH                   2
#define PWM_OCTRL_PWMAFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMAFS_SHIFT))&PWM_OCTRL_PWMAFS_MASK)
#define PWM_OCTRL_POLX_MASK                      0x100u
#define PWM_OCTRL_POLX_SHIFT                     8
#define PWM_OCTRL_POLX_WIDTH                     1
#define PWM_OCTRL_POLX(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_POLX_SHIFT))&PWM_OCTRL_POLX_MASK)
#define PWM_OCTRL_POLB_MASK                      0x200u
#define PWM_OCTRL_POLB_SHIFT                     9
#define PWM_OCTRL_POLB_WIDTH                     1
#define PWM_OCTRL_POLB(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_POLB_SHIFT))&PWM_OCTRL_POLB_MASK)
#define PWM_OCTRL_POLA_MASK                      0x400u
#define PWM_OCTRL_POLA_SHIFT                     10
#define PWM_OCTRL_POLA_WIDTH                     1
#define PWM_OCTRL_POLA(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_POLA_SHIFT))&PWM_OCTRL_POLA_MASK)
#define PWM_OCTRL_PWMX_IN_MASK                   0x2000u
#define PWM_OCTRL_PWMX_IN_SHIFT                  13
#define PWM_OCTRL_PWMX_IN_WIDTH                  1
#define PWM_OCTRL_PWMX_IN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMX_IN_SHIFT))&PWM_OCTRL_PWMX_IN_MASK)
#define PWM_OCTRL_PWMB_IN_MASK                   0x4000u
#define PWM_OCTRL_PWMB_IN_SHIFT                  14
#define PWM_OCTRL_PWMB_IN_WIDTH                  1
#define PWM_OCTRL_PWMB_IN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMB_IN_SHIFT))&PWM_OCTRL_PWMB_IN_MASK)
#define PWM_OCTRL_PWMA_IN_MASK                   0x8000u
#define PWM_OCTRL_PWMA_IN_SHIFT                  15
#define PWM_OCTRL_PWMA_IN_WIDTH                  1
#define PWM_OCTRL_PWMA_IN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMA_IN_SHIFT))&PWM_OCTRL_PWMA_IN_MASK)
/* STS Bit Fields */
#define PWM_STS_CMPF_MASK                        0x3Fu
#define PWM_STS_CMPF_SHIFT                       0
#define PWM_STS_CMPF_WIDTH                       6
#define PWM_STS_CMPF(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CMPF_SHIFT))&PWM_STS_CMPF_MASK)
#define PWM_STS_CFX0_MASK                        0x40u
#define PWM_STS_CFX0_SHIFT                       6
#define PWM_STS_CFX0_WIDTH                       1
#define PWM_STS_CFX0(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFX0_SHIFT))&PWM_STS_CFX0_MASK)
#define PWM_STS_CFX1_MASK                        0x80u
#define PWM_STS_CFX1_SHIFT                       7
#define PWM_STS_CFX1_WIDTH                       1
#define PWM_STS_CFX1(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFX1_SHIFT))&PWM_STS_CFX1_MASK)
#define PWM_STS_CFB0_MASK                        0x100u
#define PWM_STS_CFB0_SHIFT                       8
#define PWM_STS_CFB0_WIDTH                       1
#define PWM_STS_CFB0(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFB0_SHIFT))&PWM_STS_CFB0_MASK)
#define PWM_STS_CFB1_MASK                        0x200u
#define PWM_STS_CFB1_SHIFT                       9
#define PWM_STS_CFB1_WIDTH                       1
#define PWM_STS_CFB1(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFB1_SHIFT))&PWM_STS_CFB1_MASK)
#define PWM_STS_CFA0_MASK                        0x400u
#define PWM_STS_CFA0_SHIFT                       10
#define PWM_STS_CFA0_WIDTH                       1
#define PWM_STS_CFA0(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFA0_SHIFT))&PWM_STS_CFA0_MASK)
#define PWM_STS_CFA1_MASK                        0x800u
#define PWM_STS_CFA1_SHIFT                       11
#define PWM_STS_CFA1_WIDTH                       1
#define PWM_STS_CFA1(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CFA1_SHIFT))&PWM_STS_CFA1_MASK)
#define PWM_STS_RF_MASK                          0x1000u
#define PWM_STS_RF_SHIFT                         12
#define PWM_STS_RF_WIDTH                         1
#define PWM_STS_RF(x)                            (((uint16_t)(((uint16_t)(x))<<PWM_STS_RF_SHIFT))&PWM_STS_RF_MASK)
#define PWM_STS_REF_MASK                         0x2000u
#define PWM_STS_REF_SHIFT                        13
#define PWM_STS_REF_WIDTH                        1
#define PWM_STS_REF(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_STS_REF_SHIFT))&PWM_STS_REF_MASK)
#define PWM_STS_RUF_MASK                         0x4000u
#define PWM_STS_RUF_SHIFT                        14
#define PWM_STS_RUF_WIDTH                        1
#define PWM_STS_RUF(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_STS_RUF_SHIFT))&PWM_STS_RUF_MASK)
/* INTEN Bit Fields */
#define PWM_INTEN_CMPIE_MASK                     0x3Fu
#define PWM_INTEN_CMPIE_SHIFT                    0
#define PWM_INTEN_CMPIE_WIDTH                    6
#define PWM_INTEN_CMPIE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CMPIE_SHIFT))&PWM_INTEN_CMPIE_MASK)
#define PWM_INTEN_CX0IE_MASK                     0x40u
#define PWM_INTEN_CX0IE_SHIFT                    6
#define PWM_INTEN_CX0IE_WIDTH                    1
#define PWM_INTEN_CX0IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CX0IE_SHIFT))&PWM_INTEN_CX0IE_MASK)
#define PWM_INTEN_CX1IE_MASK                     0x80u
#define PWM_INTEN_CX1IE_SHIFT                    7
#define PWM_INTEN_CX1IE_WIDTH                    1
#define PWM_INTEN_CX1IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CX1IE_SHIFT))&PWM_INTEN_CX1IE_MASK)
#define PWM_INTEN_CB0IE_MASK                     0x100u
#define PWM_INTEN_CB0IE_SHIFT                    8
#define PWM_INTEN_CB0IE_WIDTH                    1
#define PWM_INTEN_CB0IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CB0IE_SHIFT))&PWM_INTEN_CB0IE_MASK)
#define PWM_INTEN_CB1IE_MASK                     0x200u
#define PWM_INTEN_CB1IE_SHIFT                    9
#define PWM_INTEN_CB1IE_WIDTH                    1
#define PWM_INTEN_CB1IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CB1IE_SHIFT))&PWM_INTEN_CB1IE_MASK)
#define PWM_INTEN_CA0IE_MASK                     0x400u
#define PWM_INTEN_CA0IE_SHIFT                    10
#define PWM_INTEN_CA0IE_WIDTH                    1
#define PWM_INTEN_CA0IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CA0IE_SHIFT))&PWM_INTEN_CA0IE_MASK)
#define PWM_INTEN_CA1IE_MASK                     0x800u
#define PWM_INTEN_CA1IE_SHIFT                    11
#define PWM_INTEN_CA1IE_WIDTH                    1
#define PWM_INTEN_CA1IE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CA1IE_SHIFT))&PWM_INTEN_CA1IE_MASK)
#define PWM_INTEN_RIE_MASK                       0x1000u
#define PWM_INTEN_RIE_SHIFT                      12
#define PWM_INTEN_RIE_WIDTH                      1
#define PWM_INTEN_RIE(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_RIE_SHIFT))&PWM_INTEN_RIE_MASK)
#define PWM_INTEN_REIE_MASK                      0x2000u
#define PWM_INTEN_REIE_SHIFT                     13
#define PWM_INTEN_REIE_WIDTH                     1
#define PWM_INTEN_REIE(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_REIE_SHIFT))&PWM_INTEN_REIE_MASK)
/* DMAEN Bit Fields */
#define PWM_DMAEN_CX0DE_MASK                     0x1u
#define PWM_DMAEN_CX0DE_SHIFT                    0
#define PWM_DMAEN_CX0DE_WIDTH                    1
#define PWM_DMAEN_CX0DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CX0DE_SHIFT))&PWM_DMAEN_CX0DE_MASK)
#define PWM_DMAEN_CX1DE_MASK                     0x2u
#define PWM_DMAEN_CX1DE_SHIFT                    1
#define PWM_DMAEN_CX1DE_WIDTH                    1
#define PWM_DMAEN_CX1DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CX1DE_SHIFT))&PWM_DMAEN_CX1DE_MASK)
#define PWM_DMAEN_CB0DE_MASK                     0x4u
#define PWM_DMAEN_CB0DE_SHIFT                    2
#define PWM_DMAEN_CB0DE_WIDTH                    1
#define PWM_DMAEN_CB0DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CB0DE_SHIFT))&PWM_DMAEN_CB0DE_MASK)
#define PWM_DMAEN_CB1DE_MASK                     0x8u
#define PWM_DMAEN_CB1DE_SHIFT                    3
#define PWM_DMAEN_CB1DE_WIDTH                    1
#define PWM_DMAEN_CB1DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CB1DE_SHIFT))&PWM_DMAEN_CB1DE_MASK)
#define PWM_DMAEN_CA0DE_MASK                     0x10u
#define PWM_DMAEN_CA0DE_SHIFT                    4
#define PWM_DMAEN_CA0DE_WIDTH                    1
#define PWM_DMAEN_CA0DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CA0DE_SHIFT))&PWM_DMAEN_CA0DE_MASK)
#define PWM_DMAEN_CA1DE_MASK                     0x20u
#define PWM_DMAEN_CA1DE_SHIFT                    5
#define PWM_DMAEN_CA1DE_WIDTH                    1
#define PWM_DMAEN_CA1DE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CA1DE_SHIFT))&PWM_DMAEN_CA1DE_MASK)
#define PWM_DMAEN_CAPTDE_MASK                    0xC0u
#define PWM_DMAEN_CAPTDE_SHIFT                   6
#define PWM_DMAEN_CAPTDE_WIDTH                   2
#define PWM_DMAEN_CAPTDE(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CAPTDE_SHIFT))&PWM_DMAEN_CAPTDE_MASK)
#define PWM_DMAEN_FAND_MASK                      0x100u
#define PWM_DMAEN_FAND_SHIFT                     8
#define PWM_DMAEN_FAND_WIDTH                     1
#define PWM_DMAEN_FAND(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_FAND_SHIFT))&PWM_DMAEN_FAND_MASK)
#define PWM_DMAEN_VALDE_MASK                     0x200u
#define PWM_DMAEN_VALDE_SHIFT                    9
#define PWM_DMAEN_VALDE_WIDTH                    1
#define PWM_DMAEN_VALDE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_VALDE_SHIFT))&PWM_DMAEN_VALDE_MASK)
/* TCTRL Bit Fields */
#define PWM_TCTRL_OUT_TRIG_EN_MASK               0x3Fu
#define PWM_TCTRL_OUT_TRIG_EN_SHIFT              0
#define PWM_TCTRL_OUT_TRIG_EN_WIDTH              6
#define PWM_TCTRL_OUT_TRIG_EN(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_TCTRL_OUT_TRIG_EN_SHIFT))&PWM_TCTRL_OUT_TRIG_EN_MASK)
#define PWM_TCTRL_PWBOT1_MASK                    0x4000u
#define PWM_TCTRL_PWBOT1_SHIFT                   14
#define PWM_TCTRL_PWBOT1_WIDTH                   1
#define PWM_TCTRL_PWBOT1(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_TCTRL_PWBOT1_SHIFT))&PWM_TCTRL_PWBOT1_MASK)
#define PWM_TCTRL_PWAOT0_MASK                    0x8000u
#define PWM_TCTRL_PWAOT0_SHIFT                   15
#define PWM_TCTRL_PWAOT0_WIDTH                   1
#define PWM_TCTRL_PWAOT0(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_TCTRL_PWAOT0_SHIFT))&PWM_TCTRL_PWAOT0_MASK)
/* DISMAP Bit Fields */
#define PWM_DISMAP_DIS0A_MASK                    0xFu
#define PWM_DISMAP_DIS0A_SHIFT                   0
#define PWM_DISMAP_DIS0A_WIDTH                   4
#define PWM_DISMAP_DIS0A(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0A_SHIFT))&PWM_DISMAP_DIS0A_MASK)
#define PWM_DISMAP_DIS1A_MASK                    0xFu
#define PWM_DISMAP_DIS1A_SHIFT                   0
#define PWM_DISMAP_DIS1A_WIDTH                   4
#define PWM_DISMAP_DIS1A(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1A_SHIFT))&PWM_DISMAP_DIS1A_MASK)
#define PWM_DISMAP_DIS0B_MASK                    0xF0u
#define PWM_DISMAP_DIS0B_SHIFT                   4
#define PWM_DISMAP_DIS0B_WIDTH                   4
#define PWM_DISMAP_DIS0B(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0B_SHIFT))&PWM_DISMAP_DIS0B_MASK)
#define PWM_DISMAP_DIS1B_MASK                    0xF0u
#define PWM_DISMAP_DIS1B_SHIFT                   4
#define PWM_DISMAP_DIS1B_WIDTH                   4
#define PWM_DISMAP_DIS1B(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1B_SHIFT))&PWM_DISMAP_DIS1B_MASK)
#define PWM_DISMAP_DIS1X_MASK                    0xF00u
#define PWM_DISMAP_DIS1X_SHIFT                   8
#define PWM_DISMAP_DIS1X_WIDTH                   4
#define PWM_DISMAP_DIS1X(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1X_SHIFT))&PWM_DISMAP_DIS1X_MASK)
#define PWM_DISMAP_DIS0X_MASK                    0xF00u
#define PWM_DISMAP_DIS0X_SHIFT                   8
#define PWM_DISMAP_DIS0X_WIDTH                   4
#define PWM_DISMAP_DIS0X(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0X_SHIFT))&PWM_DISMAP_DIS0X_MASK)
/* DTCNT0 Bit Fields */
#define PWM_DTCNT0_DTCNT0_MASK                   0xFFFFu
#define PWM_DTCNT0_DTCNT0_SHIFT                  0
#define PWM_DTCNT0_DTCNT0_WIDTH                  16
#define PWM_DTCNT0_DTCNT0(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_DTCNT0_DTCNT0_SHIFT))&PWM_DTCNT0_DTCNT0_MASK)
/* DTCNT1 Bit Fields */
#define PWM_DTCNT1_DTCNT1_MASK                   0xFFFFu
#define PWM_DTCNT1_DTCNT1_SHIFT                  0
#define PWM_DTCNT1_DTCNT1_WIDTH                  16
#define PWM_DTCNT1_DTCNT1(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_DTCNT1_DTCNT1_SHIFT))&PWM_DTCNT1_DTCNT1_MASK)
/* CAPTCTRLA Bit Fields */
#define PWM_CAPTCTRLA_ARMA_MASK                  0x1u
#define PWM_CAPTCTRLA_ARMA_SHIFT                 0
#define PWM_CAPTCTRLA_ARMA_WIDTH                 1
#define PWM_CAPTCTRLA_ARMA(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_ARMA_SHIFT))&PWM_CAPTCTRLA_ARMA_MASK)
#define PWM_CAPTCTRLA_ONESHOTA_MASK              0x2u
#define PWM_CAPTCTRLA_ONESHOTA_SHIFT             1
#define PWM_CAPTCTRLA_ONESHOTA_WIDTH             1
#define PWM_CAPTCTRLA_ONESHOTA(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_ONESHOTA_SHIFT))&PWM_CAPTCTRLA_ONESHOTA_MASK)
#define PWM_CAPTCTRLA_EDGA0_MASK                 0xCu
#define PWM_CAPTCTRLA_EDGA0_SHIFT                2
#define PWM_CAPTCTRLA_EDGA0_WIDTH                2
#define PWM_CAPTCTRLA_EDGA0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_EDGA0_SHIFT))&PWM_CAPTCTRLA_EDGA0_MASK)
#define PWM_CAPTCTRLA_EDGA1_MASK                 0x30u
#define PWM_CAPTCTRLA_EDGA1_SHIFT                4
#define PWM_CAPTCTRLA_EDGA1_WIDTH                2
#define PWM_CAPTCTRLA_EDGA1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_EDGA1_SHIFT))&PWM_CAPTCTRLA_EDGA1_MASK)
#define PWM_CAPTCTRLA_INP_SELA_MASK              0x40u
#define PWM_CAPTCTRLA_INP_SELA_SHIFT             6
#define PWM_CAPTCTRLA_INP_SELA_WIDTH             1
#define PWM_CAPTCTRLA_INP_SELA(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_INP_SELA_SHIFT))&PWM_CAPTCTRLA_INP_SELA_MASK)
#define PWM_CAPTCTRLA_EDGCNTA_EN_MASK            0x80u
#define PWM_CAPTCTRLA_EDGCNTA_EN_SHIFT           7
#define PWM_CAPTCTRLA_EDGCNTA_EN_WIDTH           1
#define PWM_CAPTCTRLA_EDGCNTA_EN(x)              (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_EDGCNTA_EN_SHIFT))&PWM_CAPTCTRLA_EDGCNTA_EN_MASK)
#define PWM_CAPTCTRLA_CFAWM_MASK                 0x300u
#define PWM_CAPTCTRLA_CFAWM_SHIFT                8
#define PWM_CAPTCTRLA_CFAWM_WIDTH                2
#define PWM_CAPTCTRLA_CFAWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CFAWM_SHIFT))&PWM_CAPTCTRLA_CFAWM_MASK)
#define PWM_CAPTCTRLA_CA0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLA_CA0CNT_SHIFT               10
#define PWM_CAPTCTRLA_CA0CNT_WIDTH               3
#define PWM_CAPTCTRLA_CA0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CA0CNT_SHIFT))&PWM_CAPTCTRLA_CA0CNT_MASK)
#define PWM_CAPTCTRLA_CA1CNT_MASK                0xE000u
#define PWM_CAPTCTRLA_CA1CNT_SHIFT               13
#define PWM_CAPTCTRLA_CA1CNT_WIDTH               3
#define PWM_CAPTCTRLA_CA1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CA1CNT_SHIFT))&PWM_CAPTCTRLA_CA1CNT_MASK)
/* CAPTCOMPA Bit Fields */
#define PWM_CAPTCOMPA_EDGCMPA_MASK               0xFFu
#define PWM_CAPTCOMPA_EDGCMPA_SHIFT              0
#define PWM_CAPTCOMPA_EDGCMPA_WIDTH              8
#define PWM_CAPTCOMPA_EDGCMPA(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPA_EDGCMPA_SHIFT))&PWM_CAPTCOMPA_EDGCMPA_MASK)
#define PWM_CAPTCOMPA_EDGCNTA_MASK               0xFF00u
#define PWM_CAPTCOMPA_EDGCNTA_SHIFT              8
#define PWM_CAPTCOMPA_EDGCNTA_WIDTH              8
#define PWM_CAPTCOMPA_EDGCNTA(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPA_EDGCNTA_SHIFT))&PWM_CAPTCOMPA_EDGCNTA_MASK)
/* CAPTCTRLB Bit Fields */
#define PWM_CAPTCTRLB_ARMB_MASK                  0x1u
#define PWM_CAPTCTRLB_ARMB_SHIFT                 0
#define PWM_CAPTCTRLB_ARMB_WIDTH                 1
#define PWM_CAPTCTRLB_ARMB(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_ARMB_SHIFT))&PWM_CAPTCTRLB_ARMB_MASK)
#define PWM_CAPTCTRLB_ONESHOTB_MASK              0x2u
#define PWM_CAPTCTRLB_ONESHOTB_SHIFT             1
#define PWM_CAPTCTRLB_ONESHOTB_WIDTH             1
#define PWM_CAPTCTRLB_ONESHOTB(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_ONESHOTB_SHIFT))&PWM_CAPTCTRLB_ONESHOTB_MASK)
#define PWM_CAPTCTRLB_EDGB0_MASK                 0xCu
#define PWM_CAPTCTRLB_EDGB0_SHIFT                2
#define PWM_CAPTCTRLB_EDGB0_WIDTH                2
#define PWM_CAPTCTRLB_EDGB0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_EDGB0_SHIFT))&PWM_CAPTCTRLB_EDGB0_MASK)
#define PWM_CAPTCTRLB_EDGB1_MASK                 0x30u
#define PWM_CAPTCTRLB_EDGB1_SHIFT                4
#define PWM_CAPTCTRLB_EDGB1_WIDTH                2
#define PWM_CAPTCTRLB_EDGB1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_EDGB1_SHIFT))&PWM_CAPTCTRLB_EDGB1_MASK)
#define PWM_CAPTCTRLB_INP_SELB_MASK              0x40u
#define PWM_CAPTCTRLB_INP_SELB_SHIFT             6
#define PWM_CAPTCTRLB_INP_SELB_WIDTH             1
#define PWM_CAPTCTRLB_INP_SELB(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_INP_SELB_SHIFT))&PWM_CAPTCTRLB_INP_SELB_MASK)
#define PWM_CAPTCTRLB_EDGCNTB_EN_MASK            0x80u
#define PWM_CAPTCTRLB_EDGCNTB_EN_SHIFT           7
#define PWM_CAPTCTRLB_EDGCNTB_EN_WIDTH           1
#define PWM_CAPTCTRLB_EDGCNTB_EN(x)              (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_EDGCNTB_EN_SHIFT))&PWM_CAPTCTRLB_EDGCNTB_EN_MASK)
#define PWM_CAPTCTRLB_CFBWM_MASK                 0x300u
#define PWM_CAPTCTRLB_CFBWM_SHIFT                8
#define PWM_CAPTCTRLB_CFBWM_WIDTH                2
#define PWM_CAPTCTRLB_CFBWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CFBWM_SHIFT))&PWM_CAPTCTRLB_CFBWM_MASK)
#define PWM_CAPTCTRLB_CB0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLB_CB0CNT_SHIFT               10
#define PWM_CAPTCTRLB_CB0CNT_WIDTH               3
#define PWM_CAPTCTRLB_CB0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CB0CNT_SHIFT))&PWM_CAPTCTRLB_CB0CNT_MASK)
#define PWM_CAPTCTRLB_CB1CNT_MASK                0xE000u
#define PWM_CAPTCTRLB_CB1CNT_SHIFT               13
#define PWM_CAPTCTRLB_CB1CNT_WIDTH               3
#define PWM_CAPTCTRLB_CB1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CB1CNT_SHIFT))&PWM_CAPTCTRLB_CB1CNT_MASK)
/* CAPTCOMPB Bit Fields */
#define PWM_CAPTCOMPB_EDGCMPB_MASK               0xFFu
#define PWM_CAPTCOMPB_EDGCMPB_SHIFT              0
#define PWM_CAPTCOMPB_EDGCMPB_WIDTH              8
#define PWM_CAPTCOMPB_EDGCMPB(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPB_EDGCMPB_SHIFT))&PWM_CAPTCOMPB_EDGCMPB_MASK)
#define PWM_CAPTCOMPB_EDGCNTB_MASK               0xFF00u
#define PWM_CAPTCOMPB_EDGCNTB_SHIFT              8
#define PWM_CAPTCOMPB_EDGCNTB_WIDTH              8
#define PWM_CAPTCOMPB_EDGCNTB(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPB_EDGCNTB_SHIFT))&PWM_CAPTCOMPB_EDGCNTB_MASK)
/* CAPTCTRLX Bit Fields */
#define PWM_CAPTCTRLX_ARMX_MASK                  0x1u
#define PWM_CAPTCTRLX_ARMX_SHIFT                 0
#define PWM_CAPTCTRLX_ARMX_WIDTH                 1
#define PWM_CAPTCTRLX_ARMX(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_ARMX_SHIFT))&PWM_CAPTCTRLX_ARMX_MASK)
#define PWM_CAPTCTRLX_ONESHOTX_MASK              0x2u
#define PWM_CAPTCTRLX_ONESHOTX_SHIFT             1
#define PWM_CAPTCTRLX_ONESHOTX_WIDTH             1
#define PWM_CAPTCTRLX_ONESHOTX(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_ONESHOTX_SHIFT))&PWM_CAPTCTRLX_ONESHOTX_MASK)
#define PWM_CAPTCTRLX_EDGX0_MASK                 0xCu
#define PWM_CAPTCTRLX_EDGX0_SHIFT                2
#define PWM_CAPTCTRLX_EDGX0_WIDTH                2
#define PWM_CAPTCTRLX_EDGX0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_EDGX0_SHIFT))&PWM_CAPTCTRLX_EDGX0_MASK)
#define PWM_CAPTCTRLX_EDGX1_MASK                 0x30u
#define PWM_CAPTCTRLX_EDGX1_SHIFT                4
#define PWM_CAPTCTRLX_EDGX1_WIDTH                2
#define PWM_CAPTCTRLX_EDGX1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_EDGX1_SHIFT))&PWM_CAPTCTRLX_EDGX1_MASK)
#define PWM_CAPTCTRLX_INP_SELX_MASK              0x40u
#define PWM_CAPTCTRLX_INP_SELX_SHIFT             6
#define PWM_CAPTCTRLX_INP_SELX_WIDTH             1
#define PWM_CAPTCTRLX_INP_SELX(x)                (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_INP_SELX_SHIFT))&PWM_CAPTCTRLX_INP_SELX_MASK)
#define PWM_CAPTCTRLX_EDGCNTX_EN_MASK            0x80u
#define PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT           7
#define PWM_CAPTCTRLX_EDGCNTX_EN_WIDTH           1
#define PWM_CAPTCTRLX_EDGCNTX_EN(x)              (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT))&PWM_CAPTCTRLX_EDGCNTX_EN_MASK)
#define PWM_CAPTCTRLX_CFXWM_MASK                 0x300u
#define PWM_CAPTCTRLX_CFXWM_SHIFT                8
#define PWM_CAPTCTRLX_CFXWM_WIDTH                2
#define PWM_CAPTCTRLX_CFXWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CFXWM_SHIFT))&PWM_CAPTCTRLX_CFXWM_MASK)
#define PWM_CAPTCTRLX_CX0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLX_CX0CNT_SHIFT               10
#define PWM_CAPTCTRLX_CX0CNT_WIDTH               3
#define PWM_CAPTCTRLX_CX0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CX0CNT_SHIFT))&PWM_CAPTCTRLX_CX0CNT_MASK)
#define PWM_CAPTCTRLX_CX1CNT_MASK                0xE000u
#define PWM_CAPTCTRLX_CX1CNT_SHIFT               13
#define PWM_CAPTCTRLX_CX1CNT_WIDTH               3
#define PWM_CAPTCTRLX_CX1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CX1CNT_SHIFT))&PWM_CAPTCTRLX_CX1CNT_MASK)
/* CAPTCOMPX Bit Fields */
#define PWM_CAPTCOMPX_EDGCMPX_MASK               0xFFu
#define PWM_CAPTCOMPX_EDGCMPX_SHIFT              0
#define PWM_CAPTCOMPX_EDGCMPX_WIDTH              8
#define PWM_CAPTCOMPX_EDGCMPX(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPX_EDGCMPX_SHIFT))&PWM_CAPTCOMPX_EDGCMPX_MASK)
#define PWM_CAPTCOMPX_EDGCNTX_MASK               0xFF00u
#define PWM_CAPTCOMPX_EDGCNTX_SHIFT              8
#define PWM_CAPTCOMPX_EDGCNTX_WIDTH              8
#define PWM_CAPTCOMPX_EDGCNTX(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPX_EDGCNTX_SHIFT))&PWM_CAPTCOMPX_EDGCNTX_MASK)
/* CVAL0 Bit Fields */
#define PWM_CVAL0_CAPTVAL0_MASK                  0xFFFFu
#define PWM_CVAL0_CAPTVAL0_SHIFT                 0
#define PWM_CVAL0_CAPTVAL0_WIDTH                 16
#define PWM_CVAL0_CAPTVAL0(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL0_CAPTVAL0_SHIFT))&PWM_CVAL0_CAPTVAL0_MASK)
/* CVAL0CYC Bit Fields */
#define PWM_CVAL0CYC_CVAL0CYC_MASK               0xFu
#define PWM_CVAL0CYC_CVAL0CYC_SHIFT              0
#define PWM_CVAL0CYC_CVAL0CYC_WIDTH              4
#define PWM_CVAL0CYC_CVAL0CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL0CYC_CVAL0CYC_SHIFT))&PWM_CVAL0CYC_CVAL0CYC_MASK)
/* CVAL1 Bit Fields */
#define PWM_CVAL1_CAPTVAL1_MASK                  0xFFFFu
#define PWM_CVAL1_CAPTVAL1_SHIFT                 0
#define PWM_CVAL1_CAPTVAL1_WIDTH                 16
#define PWM_CVAL1_CAPTVAL1(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL1_CAPTVAL1_SHIFT))&PWM_CVAL1_CAPTVAL1_MASK)
/* CVAL1CYC Bit Fields */
#define PWM_CVAL1CYC_CVAL1CYC_MASK               0xFu
#define PWM_CVAL1CYC_CVAL1CYC_SHIFT              0
#define PWM_CVAL1CYC_CVAL1CYC_WIDTH              4
#define PWM_CVAL1CYC_CVAL1CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL1CYC_CVAL1CYC_SHIFT))&PWM_CVAL1CYC_CVAL1CYC_MASK)
/* CVAL2 Bit Fields */
#define PWM_CVAL2_CAPTVAL2_MASK                  0xFFFFu
#define PWM_CVAL2_CAPTVAL2_SHIFT                 0
#define PWM_CVAL2_CAPTVAL2_WIDTH                 16
#define PWM_CVAL2_CAPTVAL2(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL2_CAPTVAL2_SHIFT))&PWM_CVAL2_CAPTVAL2_MASK)
/* CVAL2CYC Bit Fields */
#define PWM_CVAL2CYC_CVAL2CYC_MASK               0xFu
#define PWM_CVAL2CYC_CVAL2CYC_SHIFT              0
#define PWM_CVAL2CYC_CVAL2CYC_WIDTH              4
#define PWM_CVAL2CYC_CVAL2CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL2CYC_CVAL2CYC_SHIFT))&PWM_CVAL2CYC_CVAL2CYC_MASK)
/* CVAL3 Bit Fields */
#define PWM_CVAL3_CAPTVAL3_MASK                  0xFFFFu
#define PWM_CVAL3_CAPTVAL3_SHIFT                 0
#define PWM_CVAL3_CAPTVAL3_WIDTH                 16
#define PWM_CVAL3_CAPTVAL3(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL3_CAPTVAL3_SHIFT))&PWM_CVAL3_CAPTVAL3_MASK)
/* CVAL3CYC Bit Fields */
#define PWM_CVAL3CYC_CVAL3CYC_MASK               0xFu
#define PWM_CVAL3CYC_CVAL3CYC_SHIFT              0
#define PWM_CVAL3CYC_CVAL3CYC_WIDTH              4
#define PWM_CVAL3CYC_CVAL3CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL3CYC_CVAL3CYC_SHIFT))&PWM_CVAL3CYC_CVAL3CYC_MASK)
/* CVAL4 Bit Fields */
#define PWM_CVAL4_CAPTVAL4_MASK                  0xFFFFu
#define PWM_CVAL4_CAPTVAL4_SHIFT                 0
#define PWM_CVAL4_CAPTVAL4_WIDTH                 16
#define PWM_CVAL4_CAPTVAL4(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL4_CAPTVAL4_SHIFT))&PWM_CVAL4_CAPTVAL4_MASK)
/* CVAL4CYC Bit Fields */
#define PWM_CVAL4CYC_CVAL4CYC_MASK               0xFu
#define PWM_CVAL4CYC_CVAL4CYC_SHIFT              0
#define PWM_CVAL4CYC_CVAL4CYC_WIDTH              4
#define PWM_CVAL4CYC_CVAL4CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL4CYC_CVAL4CYC_SHIFT))&PWM_CVAL4CYC_CVAL4CYC_MASK)
/* CVAL5 Bit Fields */
#define PWM_CVAL5_CAPTVAL5_MASK                  0xFFFFu
#define PWM_CVAL5_CAPTVAL5_SHIFT                 0
#define PWM_CVAL5_CAPTVAL5_WIDTH                 16
#define PWM_CVAL5_CAPTVAL5(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL5_CAPTVAL5_SHIFT))&PWM_CVAL5_CAPTVAL5_MASK)
/* CVAL5CYC Bit Fields */
#define PWM_CVAL5CYC_CVAL5CYC_MASK               0xFu
#define PWM_CVAL5CYC_CVAL5CYC_SHIFT              0
#define PWM_CVAL5CYC_CVAL5CYC_WIDTH              4
#define PWM_CVAL5CYC_CVAL5CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL5CYC_CVAL5CYC_SHIFT))&PWM_CVAL5CYC_CVAL5CYC_MASK)
/* OUTEN Bit Fields */
#define PWM_OUTEN_PWMX_EN_MASK                   0xFu
#define PWM_OUTEN_PWMX_EN_SHIFT                  0
#define PWM_OUTEN_PWMX_EN_WIDTH                  4
#define PWM_OUTEN_PWMX_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMX_EN_SHIFT))&PWM_OUTEN_PWMX_EN_MASK)
#define PWM_OUTEN_PWMB_EN_MASK                   0xF0u
#define PWM_OUTEN_PWMB_EN_SHIFT                  4
#define PWM_OUTEN_PWMB_EN_WIDTH                  4
#define PWM_OUTEN_PWMB_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMB_EN_SHIFT))&PWM_OUTEN_PWMB_EN_MASK)
#define PWM_OUTEN_PWMA_EN_MASK                   0xF00u
#define PWM_OUTEN_PWMA_EN_SHIFT                  8
#define PWM_OUTEN_PWMA_EN_WIDTH                  4
#define PWM_OUTEN_PWMA_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMA_EN_SHIFT))&PWM_OUTEN_PWMA_EN_MASK)
/* MASK Bit Fields */
#define PWM_MASK_MASKX_MASK                      0xFu
#define PWM_MASK_MASKX_SHIFT                     0
#define PWM_MASK_MASKX_WIDTH                     4
#define PWM_MASK_MASKX(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKX_SHIFT))&PWM_MASK_MASKX_MASK)
#define PWM_MASK_MASKB_MASK                      0xF0u
#define PWM_MASK_MASKB_SHIFT                     4
#define PWM_MASK_MASKB_WIDTH                     4
#define PWM_MASK_MASKB(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKB_SHIFT))&PWM_MASK_MASKB_MASK)
#define PWM_MASK_MASKA_MASK                      0xF00u
#define PWM_MASK_MASKA_SHIFT                     8
#define PWM_MASK_MASKA_WIDTH                     4
#define PWM_MASK_MASKA(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKA_SHIFT))&PWM_MASK_MASKA_MASK)
#define PWM_MASK_UPDATE_MASK_MASK                0xF000u
#define PWM_MASK_UPDATE_MASK_SHIFT               12
#define PWM_MASK_UPDATE_MASK_WIDTH               4
#define PWM_MASK_UPDATE_MASK(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_MASK_UPDATE_MASK_SHIFT))&PWM_MASK_UPDATE_MASK_MASK)
/* SWCOUT Bit Fields */
#define PWM_SWCOUT_SM0OUT45_MASK                 0x1u
#define PWM_SWCOUT_SM0OUT45_SHIFT                0
#define PWM_SWCOUT_SM0OUT45_WIDTH                1
#define PWM_SWCOUT_SM0OUT45(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM0OUT45_SHIFT))&PWM_SWCOUT_SM0OUT45_MASK)
#define PWM_SWCOUT_SM0OUT23_MASK                 0x2u
#define PWM_SWCOUT_SM0OUT23_SHIFT                1
#define PWM_SWCOUT_SM0OUT23_WIDTH                1
#define PWM_SWCOUT_SM0OUT23(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM0OUT23_SHIFT))&PWM_SWCOUT_SM0OUT23_MASK)
#define PWM_SWCOUT_SM1OUT45_MASK                 0x4u
#define PWM_SWCOUT_SM1OUT45_SHIFT                2
#define PWM_SWCOUT_SM1OUT45_WIDTH                1
#define PWM_SWCOUT_SM1OUT45(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM1OUT45_SHIFT))&PWM_SWCOUT_SM1OUT45_MASK)
#define PWM_SWCOUT_SM1OUT23_MASK                 0x8u
#define PWM_SWCOUT_SM1OUT23_SHIFT                3
#define PWM_SWCOUT_SM1OUT23_WIDTH                1
#define PWM_SWCOUT_SM1OUT23(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM1OUT23_SHIFT))&PWM_SWCOUT_SM1OUT23_MASK)
#define PWM_SWCOUT_SM2OUT45_MASK                 0x10u
#define PWM_SWCOUT_SM2OUT45_SHIFT                4
#define PWM_SWCOUT_SM2OUT45_WIDTH                1
#define PWM_SWCOUT_SM2OUT45(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM2OUT45_SHIFT))&PWM_SWCOUT_SM2OUT45_MASK)
#define PWM_SWCOUT_SM2OUT23_MASK                 0x20u
#define PWM_SWCOUT_SM2OUT23_SHIFT                5
#define PWM_SWCOUT_SM2OUT23_WIDTH                1
#define PWM_SWCOUT_SM2OUT23(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM2OUT23_SHIFT))&PWM_SWCOUT_SM2OUT23_MASK)
#define PWM_SWCOUT_SM3OUT45_MASK                 0x40u
#define PWM_SWCOUT_SM3OUT45_SHIFT                6
#define PWM_SWCOUT_SM3OUT45_WIDTH                1
#define PWM_SWCOUT_SM3OUT45(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM3OUT45_SHIFT))&PWM_SWCOUT_SM3OUT45_MASK)
#define PWM_SWCOUT_SM3OUT23_MASK                 0x80u
#define PWM_SWCOUT_SM3OUT23_SHIFT                7
#define PWM_SWCOUT_SM3OUT23_WIDTH                1
#define PWM_SWCOUT_SM3OUT23(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_SWCOUT_SM3OUT23_SHIFT))&PWM_SWCOUT_SM3OUT23_MASK)
/* DTSRCSEL Bit Fields */
#define PWM_DTSRCSEL_SM0SEL45_MASK               0x3u
#define PWM_DTSRCSEL_SM0SEL45_SHIFT              0
#define PWM_DTSRCSEL_SM0SEL45_WIDTH              2
#define PWM_DTSRCSEL_SM0SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM0SEL45_SHIFT))&PWM_DTSRCSEL_SM0SEL45_MASK)
#define PWM_DTSRCSEL_SM0SEL23_MASK               0xCu
#define PWM_DTSRCSEL_SM0SEL23_SHIFT              2
#define PWM_DTSRCSEL_SM0SEL23_WIDTH              2
#define PWM_DTSRCSEL_SM0SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM0SEL23_SHIFT))&PWM_DTSRCSEL_SM0SEL23_MASK)
#define PWM_DTSRCSEL_SM1SEL45_MASK               0x30u
#define PWM_DTSRCSEL_SM1SEL45_SHIFT              4
#define PWM_DTSRCSEL_SM1SEL45_WIDTH              2
#define PWM_DTSRCSEL_SM1SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM1SEL45_SHIFT))&PWM_DTSRCSEL_SM1SEL45_MASK)
#define PWM_DTSRCSEL_SM1SEL23_MASK               0xC0u
#define PWM_DTSRCSEL_SM1SEL23_SHIFT              6
#define PWM_DTSRCSEL_SM1SEL23_WIDTH              2
#define PWM_DTSRCSEL_SM1SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM1SEL23_SHIFT))&PWM_DTSRCSEL_SM1SEL23_MASK)
#define PWM_DTSRCSEL_SM2SEL45_MASK               0x300u
#define PWM_DTSRCSEL_SM2SEL45_SHIFT              8
#define PWM_DTSRCSEL_SM2SEL45_WIDTH              2
#define PWM_DTSRCSEL_SM2SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM2SEL45_SHIFT))&PWM_DTSRCSEL_SM2SEL45_MASK)
#define PWM_DTSRCSEL_SM2SEL23_MASK               0xC00u
#define PWM_DTSRCSEL_SM2SEL23_SHIFT              10
#define PWM_DTSRCSEL_SM2SEL23_WIDTH              2
#define PWM_DTSRCSEL_SM2SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM2SEL23_SHIFT))&PWM_DTSRCSEL_SM2SEL23_MASK)
#define PWM_DTSRCSEL_SM3SEL45_MASK               0x3000u
#define PWM_DTSRCSEL_SM3SEL45_SHIFT              12
#define PWM_DTSRCSEL_SM3SEL45_WIDTH              2
#define PWM_DTSRCSEL_SM3SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM3SEL45_SHIFT))&PWM_DTSRCSEL_SM3SEL45_MASK)
#define PWM_DTSRCSEL_SM3SEL23_MASK               0xC000u
#define PWM_DTSRCSEL_SM3SEL23_SHIFT              14
#define PWM_DTSRCSEL_SM3SEL23_WIDTH              2
#define PWM_DTSRCSEL_SM3SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM3SEL23_SHIFT))&PWM_DTSRCSEL_SM3SEL23_MASK)
/* MCTRL Bit Fields */
#define PWM_MCTRL_LDOK_MASK                      0xFu
#define PWM_MCTRL_LDOK_SHIFT                     0
#define PWM_MCTRL_LDOK_WIDTH                     4
#define PWM_MCTRL_LDOK(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_LDOK_SHIFT))&PWM_MCTRL_LDOK_MASK)
#define PWM_MCTRL_CLDOK_MASK                     0xF0u
#define PWM_MCTRL_CLDOK_SHIFT                    4
#define PWM_MCTRL_CLDOK_WIDTH                    4
#define PWM_MCTRL_CLDOK(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_CLDOK_SHIFT))&PWM_MCTRL_CLDOK_MASK)
#define PWM_MCTRL_RUN_MASK                       0xF00u
#define PWM_MCTRL_RUN_SHIFT                      8
#define PWM_MCTRL_RUN_WIDTH                      4
#define PWM_MCTRL_RUN(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_RUN_SHIFT))&PWM_MCTRL_RUN_MASK)
#define PWM_MCTRL_IPOL_MASK                      0xF000u
#define PWM_MCTRL_IPOL_SHIFT                     12
#define PWM_MCTRL_IPOL_WIDTH                     4
#define PWM_MCTRL_IPOL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_IPOL_SHIFT))&PWM_MCTRL_IPOL_MASK)
/* MCTRL2 Bit Fields */
#define PWM_MCTRL2_MONPLL_MASK                   0x3u
#define PWM_MCTRL2_MONPLL_SHIFT                  0
#define PWM_MCTRL2_MONPLL_WIDTH                  2
#define PWM_MCTRL2_MONPLL(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL2_MONPLL_SHIFT))&PWM_MCTRL2_MONPLL_MASK)
/* FCTRL Bit Fields */
#define PWM_FCTRL_FIE_MASK                       0xFu
#define PWM_FCTRL_FIE_SHIFT                      0
#define PWM_FCTRL_FIE_WIDTH                      4
#define PWM_FCTRL_FIE(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FIE_SHIFT))&PWM_FCTRL_FIE_MASK)
#define PWM_FCTRL_FSAFE_MASK                     0xF0u
#define PWM_FCTRL_FSAFE_SHIFT                    4
#define PWM_FCTRL_FSAFE_WIDTH                    4
#define PWM_FCTRL_FSAFE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FSAFE_SHIFT))&PWM_FCTRL_FSAFE_MASK)
#define PWM_FCTRL_FAUTO_MASK                     0xF00u
#define PWM_FCTRL_FAUTO_SHIFT                    8
#define PWM_FCTRL_FAUTO_WIDTH                    4
#define PWM_FCTRL_FAUTO(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FAUTO_SHIFT))&PWM_FCTRL_FAUTO_MASK)
#define PWM_FCTRL_FLVL_MASK                      0xF000u
#define PWM_FCTRL_FLVL_SHIFT                     12
#define PWM_FCTRL_FLVL_WIDTH                     4
#define PWM_FCTRL_FLVL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FLVL_SHIFT))&PWM_FCTRL_FLVL_MASK)
/* FSTS Bit Fields */
#define PWM_FSTS_FFLAG_MASK                      0xFu
#define PWM_FSTS_FFLAG_SHIFT                     0
#define PWM_FSTS_FFLAG_WIDTH                     4
#define PWM_FSTS_FFLAG(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFLAG_SHIFT))&PWM_FSTS_FFLAG_MASK)
#define PWM_FSTS_FFULL_MASK                      0xF0u
#define PWM_FSTS_FFULL_SHIFT                     4
#define PWM_FSTS_FFULL_WIDTH                     4
#define PWM_FSTS_FFULL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFULL_SHIFT))&PWM_FSTS_FFULL_MASK)
#define PWM_FSTS_FFPIN_MASK                      0xF00u
#define PWM_FSTS_FFPIN_SHIFT                     8
#define PWM_FSTS_FFPIN_WIDTH                     4
#define PWM_FSTS_FFPIN(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFPIN_SHIFT))&PWM_FSTS_FFPIN_MASK)
#define PWM_FSTS_FHALF_MASK                      0xF000u
#define PWM_FSTS_FHALF_SHIFT                     12
#define PWM_FSTS_FHALF_WIDTH                     4
#define PWM_FSTS_FHALF(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FHALF_SHIFT))&PWM_FSTS_FHALF_MASK)
/* FFILT Bit Fields */
#define PWM_FFILT_FILT_PER_MASK                  0xFFu
#define PWM_FFILT_FILT_PER_SHIFT                 0
#define PWM_FFILT_FILT_PER_WIDTH                 8
#define PWM_FFILT_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_FFILT_FILT_PER_SHIFT))&PWM_FFILT_FILT_PER_MASK)
#define PWM_FFILT_FILT_CNT_MASK                  0x700u
#define PWM_FFILT_FILT_CNT_SHIFT                 8
#define PWM_FFILT_FILT_CNT_WIDTH                 3
#define PWM_FFILT_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_FFILT_FILT_CNT_SHIFT))&PWM_FFILT_FILT_CNT_MASK)
#define PWM_FFILT_GSTR_MASK                      0x8000u
#define PWM_FFILT_GSTR_SHIFT                     15
#define PWM_FFILT_GSTR_WIDTH                     1
#define PWM_FFILT_GSTR(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FFILT_GSTR_SHIFT))&PWM_FFILT_GSTR_MASK)
/* FTST Bit Fields */
#define PWM_FTST_FTEST_MASK                      0x1u
#define PWM_FTST_FTEST_SHIFT                     0
#define PWM_FTST_FTEST_WIDTH                     1
#define PWM_FTST_FTEST(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FTST_FTEST_SHIFT))&PWM_FTST_FTEST_MASK)
/* FCTRL2 Bit Fields */
#define PWM_FCTRL2_NOCOMB_MASK                   0xFu
#define PWM_FCTRL2_NOCOMB_SHIFT                  0
#define PWM_FCTRL2_NOCOMB_WIDTH                  4
#define PWM_FCTRL2_NOCOMB(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL2_NOCOMB_SHIFT))&PWM_FCTRL2_NOCOMB_MASK)

/*!
 * @}
 */ /* end of group PWM_Register_Masks */


/* PWM - Peripheral instance base addresses */
/** Peripheral PWMA base address */
#define PWMA_BASE                                (0x40033000u)
/** Peripheral PWMA base pointer */
#define PWMA                                     ((PWM_Type *)PWMA_BASE)
#define PWMA_BASE_PTR                            (PWMA)
/** Array initializer of PWM peripheral base addresses */
#define PWM_BASE_ADDRS                           { PWMA_BASE }
/** Array initializer of PWM peripheral base pointers */
#define PWM_BASE_PTRS                            { PWMA }

/* ----------------------------------------------------------------------------
   -- PWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Accessor_Macros PWM - Register accessor macros
 * @{
 */


/* PWM - Register instance definitions */
/* PWMA */
#define PWMA_SM0CNT                              PWM_CNT_REG(PWMA,0)
#define PWMA_SM0INIT                             PWM_INIT_REG(PWMA,0)
#define PWMA_SM0CTRL2                            PWM_CTRL2_REG(PWMA,0)
#define PWMA_SM0CTRL                             PWM_CTRL_REG(PWMA,0)
#define PWMA_SM0VAL0                             PWM_VAL0_REG(PWMA,0)
#define PWMA_SM0FRACVAL1                         PWM_FRACVAL1_REG(PWMA,0)
#define PWMA_SM0VAL1                             PWM_VAL1_REG(PWMA,0)
#define PWMA_SM0FRACVAL2                         PWM_FRACVAL2_REG(PWMA,0)
#define PWMA_SM0VAL2                             PWM_VAL2_REG(PWMA,0)
#define PWMA_SM0FRACVAL3                         PWM_FRACVAL3_REG(PWMA,0)
#define PWMA_SM0VAL3                             PWM_VAL3_REG(PWMA,0)
#define PWMA_SM0FRACVAL4                         PWM_FRACVAL4_REG(PWMA,0)
#define PWMA_SM0VAL4                             PWM_VAL4_REG(PWMA,0)
#define PWMA_SM0FRACVAL5                         PWM_FRACVAL5_REG(PWMA,0)
#define PWMA_SM0VAL5                             PWM_VAL5_REG(PWMA,0)
#define PWMA_SM0FRCTRL                           PWM_FRCTRL_REG(PWMA,0)
#define PWMA_SM0OCTRL                            PWM_OCTRL_REG(PWMA,0)
#define PWMA_SM0STS                              PWM_STS_REG(PWMA,0)
#define PWMA_SM0INTEN                            PWM_INTEN_REG(PWMA,0)
#define PWMA_SM0DMAEN                            PWM_DMAEN_REG(PWMA,0)
#define PWMA_SM0TCTRL                            PWM_TCTRL_REG(PWMA,0)
#define PWMA_SM0DISMAP0                          PWM_DISMAP_REG(PWMA,0,0)
#define PWMA_SM0DISMAP1                          PWM_DISMAP_REG(PWMA,0,1)
#define PWMA_SM0DTCNT0                           PWM_DTCNT0_REG(PWMA,0)
#define PWMA_SM0DTCNT1                           PWM_DTCNT1_REG(PWMA,0)
#define PWMA_SM0CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA,0)
#define PWMA_SM0CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA,0)
#define PWMA_SM0CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA,0)
#define PWMA_SM0CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA,0)
#define PWMA_SM0CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA,0)
#define PWMA_SM0CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA,0)
#define PWMA_SM0CVAL0                            PWM_CVAL0_REG(PWMA,0)
#define PWMA_SM0CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA,0)
#define PWMA_SM0CVAL1                            PWM_CVAL1_REG(PWMA,0)
#define PWMA_SM0CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA,0)
#define PWMA_SM0CVAL2                            PWM_CVAL2_REG(PWMA,0)
#define PWMA_SM0CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA,0)
#define PWMA_SM0CVAL3                            PWM_CVAL3_REG(PWMA,0)
#define PWMA_SM0CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA,0)
#define PWMA_SM0CVAL4                            PWM_CVAL4_REG(PWMA,0)
#define PWMA_SM0CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA,0)
#define PWMA_SM0CVAL5                            PWM_CVAL5_REG(PWMA,0)
#define PWMA_SM0CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA,0)
#define PWMA_SM1CNT                              PWM_CNT_REG(PWMA,1)
#define PWMA_SM1INIT                             PWM_INIT_REG(PWMA,1)
#define PWMA_SM1CTRL2                            PWM_CTRL2_REG(PWMA,1)
#define PWMA_SM1CTRL                             PWM_CTRL_REG(PWMA,1)
#define PWMA_SM1VAL0                             PWM_VAL0_REG(PWMA,1)
#define PWMA_SM1FRACVAL1                         PWM_FRACVAL1_REG(PWMA,1)
#define PWMA_SM1VAL1                             PWM_VAL1_REG(PWMA,1)
#define PWMA_SM1FRACVAL2                         PWM_FRACVAL2_REG(PWMA,1)
#define PWMA_SM1VAL2                             PWM_VAL2_REG(PWMA,1)
#define PWMA_SM1FRACVAL3                         PWM_FRACVAL3_REG(PWMA,1)
#define PWMA_SM1VAL3                             PWM_VAL3_REG(PWMA,1)
#define PWMA_SM1FRACVAL4                         PWM_FRACVAL4_REG(PWMA,1)
#define PWMA_SM1VAL4                             PWM_VAL4_REG(PWMA,1)
#define PWMA_SM1FRACVAL5                         PWM_FRACVAL5_REG(PWMA,1)
#define PWMA_SM1VAL5                             PWM_VAL5_REG(PWMA,1)
#define PWMA_SM1FRCTRL                           PWM_FRCTRL_REG(PWMA,1)
#define PWMA_SM1OCTRL                            PWM_OCTRL_REG(PWMA,1)
#define PWMA_SM1STS                              PWM_STS_REG(PWMA,1)
#define PWMA_SM1INTEN                            PWM_INTEN_REG(PWMA,1)
#define PWMA_SM1DMAEN                            PWM_DMAEN_REG(PWMA,1)
#define PWMA_SM1TCTRL                            PWM_TCTRL_REG(PWMA,1)
#define PWMA_SM1DISMAP0                          PWM_DISMAP_REG(PWMA,1,0)
#define PWMA_SM1DISMAP1                          PWM_DISMAP_REG(PWMA,1,1)
#define PWMA_SM1DTCNT0                           PWM_DTCNT0_REG(PWMA,1)
#define PWMA_SM1DTCNT1                           PWM_DTCNT1_REG(PWMA,1)
#define PWMA_SM1CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA,1)
#define PWMA_SM1CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA,1)
#define PWMA_SM1CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA,1)
#define PWMA_SM1CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA,1)
#define PWMA_SM1CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA,1)
#define PWMA_SM1CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA,1)
#define PWMA_SM1CVAL0                            PWM_CVAL0_REG(PWMA,1)
#define PWMA_SM1CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA,1)
#define PWMA_SM1CVAL1                            PWM_CVAL1_REG(PWMA,1)
#define PWMA_SM1CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA,1)
#define PWMA_SM1CVAL2                            PWM_CVAL2_REG(PWMA,1)
#define PWMA_SM1CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA,1)
#define PWMA_SM1CVAL3                            PWM_CVAL3_REG(PWMA,1)
#define PWMA_SM1CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA,1)
#define PWMA_SM1CVAL4                            PWM_CVAL4_REG(PWMA,1)
#define PWMA_SM1CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA,1)
#define PWMA_SM1CVAL5                            PWM_CVAL5_REG(PWMA,1)
#define PWMA_SM1CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA,1)
#define PWMA_SM2CNT                              PWM_CNT_REG(PWMA,2)
#define PWMA_SM2INIT                             PWM_INIT_REG(PWMA,2)
#define PWMA_SM2CTRL2                            PWM_CTRL2_REG(PWMA,2)
#define PWMA_SM2CTRL                             PWM_CTRL_REG(PWMA,2)
#define PWMA_SM2VAL0                             PWM_VAL0_REG(PWMA,2)
#define PWMA_SM2FRACVAL1                         PWM_FRACVAL1_REG(PWMA,2)
#define PWMA_SM2VAL1                             PWM_VAL1_REG(PWMA,2)
#define PWMA_SM2FRACVAL2                         PWM_FRACVAL2_REG(PWMA,2)
#define PWMA_SM2VAL2                             PWM_VAL2_REG(PWMA,2)
#define PWMA_SM2FRACVAL3                         PWM_FRACVAL3_REG(PWMA,2)
#define PWMA_SM2VAL3                             PWM_VAL3_REG(PWMA,2)
#define PWMA_SM2FRACVAL4                         PWM_FRACVAL4_REG(PWMA,2)
#define PWMA_SM2VAL4                             PWM_VAL4_REG(PWMA,2)
#define PWMA_SM2FRACVAL5                         PWM_FRACVAL5_REG(PWMA,2)
#define PWMA_SM2VAL5                             PWM_VAL5_REG(PWMA,2)
#define PWMA_SM2FRCTRL                           PWM_FRCTRL_REG(PWMA,2)
#define PWMA_SM2OCTRL                            PWM_OCTRL_REG(PWMA,2)
#define PWMA_SM2STS                              PWM_STS_REG(PWMA,2)
#define PWMA_SM2INTEN                            PWM_INTEN_REG(PWMA,2)
#define PWMA_SM2DMAEN                            PWM_DMAEN_REG(PWMA,2)
#define PWMA_SM2TCTRL                            PWM_TCTRL_REG(PWMA,2)
#define PWMA_SM2DISMAP0                          PWM_DISMAP_REG(PWMA,2,0)
#define PWMA_SM2DISMAP1                          PWM_DISMAP_REG(PWMA,2,1)
#define PWMA_SM2DTCNT0                           PWM_DTCNT0_REG(PWMA,2)
#define PWMA_SM2DTCNT1                           PWM_DTCNT1_REG(PWMA,2)
#define PWMA_SM2CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA,2)
#define PWMA_SM2CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA,2)
#define PWMA_SM2CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA,2)
#define PWMA_SM2CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA,2)
#define PWMA_SM2CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA,2)
#define PWMA_SM2CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA,2)
#define PWMA_SM2CVAL0                            PWM_CVAL0_REG(PWMA,2)
#define PWMA_SM2CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA,2)
#define PWMA_SM2CVAL1                            PWM_CVAL1_REG(PWMA,2)
#define PWMA_SM2CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA,2)
#define PWMA_SM2CVAL2                            PWM_CVAL2_REG(PWMA,2)
#define PWMA_SM2CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA,2)
#define PWMA_SM2CVAL3                            PWM_CVAL3_REG(PWMA,2)
#define PWMA_SM2CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA,2)
#define PWMA_SM2CVAL4                            PWM_CVAL4_REG(PWMA,2)
#define PWMA_SM2CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA,2)
#define PWMA_SM2CVAL5                            PWM_CVAL5_REG(PWMA,2)
#define PWMA_SM2CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA,2)
#define PWMA_SM3CNT                              PWM_CNT_REG(PWMA,3)
#define PWMA_SM3INIT                             PWM_INIT_REG(PWMA,3)
#define PWMA_SM3CTRL2                            PWM_CTRL2_REG(PWMA,3)
#define PWMA_SM3CTRL                             PWM_CTRL_REG(PWMA,3)
#define PWMA_SM3VAL0                             PWM_VAL0_REG(PWMA,3)
#define PWMA_SM3FRACVAL1                         PWM_FRACVAL1_REG(PWMA,3)
#define PWMA_SM3VAL1                             PWM_VAL1_REG(PWMA,3)
#define PWMA_SM3FRACVAL2                         PWM_FRACVAL2_REG(PWMA,3)
#define PWMA_SM3VAL2                             PWM_VAL2_REG(PWMA,3)
#define PWMA_SM3FRACVAL3                         PWM_FRACVAL3_REG(PWMA,3)
#define PWMA_SM3VAL3                             PWM_VAL3_REG(PWMA,3)
#define PWMA_SM3FRACVAL4                         PWM_FRACVAL4_REG(PWMA,3)
#define PWMA_SM3VAL4                             PWM_VAL4_REG(PWMA,3)
#define PWMA_SM3FRACVAL5                         PWM_FRACVAL5_REG(PWMA,3)
#define PWMA_SM3VAL5                             PWM_VAL5_REG(PWMA,3)
#define PWMA_SM3FRCTRL                           PWM_FRCTRL_REG(PWMA,3)
#define PWMA_SM3OCTRL                            PWM_OCTRL_REG(PWMA,3)
#define PWMA_SM3STS                              PWM_STS_REG(PWMA,3)
#define PWMA_SM3INTEN                            PWM_INTEN_REG(PWMA,3)
#define PWMA_SM3DMAEN                            PWM_DMAEN_REG(PWMA,3)
#define PWMA_SM3TCTRL                            PWM_TCTRL_REG(PWMA,3)
#define PWMA_SM3DISMAP0                          PWM_DISMAP_REG(PWMA,3,0)
#define PWMA_SM3DISMAP1                          PWM_DISMAP_REG(PWMA,3,1)
#define PWMA_SM3DTCNT0                           PWM_DTCNT0_REG(PWMA,3)
#define PWMA_SM3DTCNT1                           PWM_DTCNT1_REG(PWMA,3)
#define PWMA_SM3CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA,3)
#define PWMA_SM3CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA,3)
#define PWMA_SM3CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA,3)
#define PWMA_SM3CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA,3)
#define PWMA_SM3CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA,3)
#define PWMA_SM3CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA,3)
#define PWMA_SM3CVAL0                            PWM_CVAL0_REG(PWMA,3)
#define PWMA_SM3CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA,3)
#define PWMA_SM3CVAL1                            PWM_CVAL1_REG(PWMA,3)
#define PWMA_SM3CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA,3)
#define PWMA_SM3CVAL2                            PWM_CVAL2_REG(PWMA,3)
#define PWMA_SM3CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA,3)
#define PWMA_SM3CVAL3                            PWM_CVAL3_REG(PWMA,3)
#define PWMA_SM3CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA,3)
#define PWMA_SM3CVAL4                            PWM_CVAL4_REG(PWMA,3)
#define PWMA_SM3CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA,3)
#define PWMA_SM3CVAL5                            PWM_CVAL5_REG(PWMA,3)
#define PWMA_SM3CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA,3)
#define PWMA_OUTEN                               PWM_OUTEN_REG(PWMA)
#define PWMA_MASK                                PWM_MASK_REG(PWMA)
#define PWMA_SWCOUT                              PWM_SWCOUT_REG(PWMA)
#define PWMA_DTSRCSEL                            PWM_DTSRCSEL_REG(PWMA)
#define PWMA_MCTRL                               PWM_MCTRL_REG(PWMA)
#define PWMA_MCTRL2                              PWM_MCTRL2_REG(PWMA)
#define PWMA_FCTRL                               PWM_FCTRL_REG(PWMA)
#define PWMA_FSTS                                PWM_FSTS_REG(PWMA)
#define PWMA_FFILT                               PWM_FFILT_REG(PWMA)
#define PWMA_FTST                                PWM_FTST_REG(PWMA)
#define PWMA_FCTRL2                              PWM_FCTRL2_REG(PWMA)

/* PWM - Register array accessors */
#define PWMA_CNT(index)                          PWM_CNT_REG(PWMA,index)
#define PWMA_INIT(index)                         PWM_INIT_REG(PWMA,index)
#define PWMA_CTRL2(index)                        PWM_CTRL2_REG(PWMA,index)
#define PWMA_CTRL(index)                         PWM_CTRL_REG(PWMA,index)
#define PWMA_VAL0(index)                         PWM_VAL0_REG(PWMA,index)
#define PWMA_FRACVAL1(index)                     PWM_FRACVAL1_REG(PWMA,index)
#define PWMA_VAL1(index)                         PWM_VAL1_REG(PWMA,index)
#define PWMA_FRACVAL2(index)                     PWM_FRACVAL2_REG(PWMA,index)
#define PWMA_VAL2(index)                         PWM_VAL2_REG(PWMA,index)
#define PWMA_FRACVAL3(index)                     PWM_FRACVAL3_REG(PWMA,index)
#define PWMA_VAL3(index)                         PWM_VAL3_REG(PWMA,index)
#define PWMA_FRACVAL4(index)                     PWM_FRACVAL4_REG(PWMA,index)
#define PWMA_VAL4(index)                         PWM_VAL4_REG(PWMA,index)
#define PWMA_FRACVAL5(index)                     PWM_FRACVAL5_REG(PWMA,index)
#define PWMA_VAL5(index)                         PWM_VAL5_REG(PWMA,index)
#define PWMA_FRCTRL(index)                       PWM_FRCTRL_REG(PWMA,index)
#define PWMA_OCTRL(index)                        PWM_OCTRL_REG(PWMA,index)
#define PWMA_STS(index)                          PWM_STS_REG(PWMA,index)
#define PWMA_INTEN(index)                        PWM_INTEN_REG(PWMA,index)
#define PWMA_DMAEN(index)                        PWM_DMAEN_REG(PWMA,index)
#define PWMA_TCTRL(index)                        PWM_TCTRL_REG(PWMA,index)
#define PWMA_DISMAP(index,index2)                PWM_DISMAP_REG(PWMA,index,index2)
#define PWMA_DTCNT0(index)                       PWM_DTCNT0_REG(PWMA,index)
#define PWMA_DTCNT1(index)                       PWM_DTCNT1_REG(PWMA,index)
#define PWMA_CAPTCTRLA(index)                    PWM_CAPTCTRLA_REG(PWMA,index)
#define PWMA_CAPTCOMPA(index)                    PWM_CAPTCOMPA_REG(PWMA,index)
#define PWMA_CAPTCTRLB(index)                    PWM_CAPTCTRLB_REG(PWMA,index)
#define PWMA_CAPTCOMPB(index)                    PWM_CAPTCOMPB_REG(PWMA,index)
#define PWMA_CAPTCTRLX(index)                    PWM_CAPTCTRLX_REG(PWMA,index)
#define PWMA_CAPTCOMPX(index)                    PWM_CAPTCOMPX_REG(PWMA,index)
#define PWMA_CVAL0(index)                        PWM_CVAL0_REG(PWMA,index)
#define PWMA_CVAL0CYC(index)                     PWM_CVAL0CYC_REG(PWMA,index)
#define PWMA_CVAL1(index)                        PWM_CVAL1_REG(PWMA,index)
#define PWMA_CVAL1CYC(index)                     PWM_CVAL1CYC_REG(PWMA,index)
#define PWMA_CVAL2(index)                        PWM_CVAL2_REG(PWMA,index)
#define PWMA_CVAL2CYC(index)                     PWM_CVAL2CYC_REG(PWMA,index)
#define PWMA_CVAL3(index)                        PWM_CVAL3_REG(PWMA,index)
#define PWMA_CVAL3CYC(index)                     PWM_CVAL3CYC_REG(PWMA,index)
#define PWMA_CVAL4(index)                        PWM_CVAL4_REG(PWMA,index)
#define PWMA_CVAL4CYC(index)                     PWM_CVAL4CYC_REG(PWMA,index)
#define PWMA_CVAL5(index)                        PWM_CVAL5_REG(PWMA,index)
#define PWMA_CVAL5CYC(index)                     PWM_CVAL5CYC_REG(PWMA,index)

/*!
 * @}
 */ /* end of group PWM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Peripheral_Access_Layer RCM Peripheral Access Layer
 * @{
 */

/** RCM - Register Layout Typedef */
typedef struct {
  __I  uint8_t SRS0;                               /**< System Reset Status Register 0, offset: 0x0 */
  __I  uint8_t SRS1;                               /**< System Reset Status Register 1, offset: 0x1 */
       uint8_t RESERVED_0[2];
  __IO uint8_t RPFC;                               /**< Reset Pin Filter Control register, offset: 0x4 */
  __IO uint8_t RPFW;                               /**< Reset Pin Filter Width register, offset: 0x5 */
       uint8_t RESERVED_1[2];
  __IO uint8_t SSRS0;                              /**< Sticky System Reset Status Register 0, offset: 0x8 */
  __IO uint8_t SSRS1;                              /**< Sticky System Reset Status Register 1, offset: 0x9 */
} RCM_Type, *RCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Accessor_Macros RCM - Register accessor macros
 * @{
 */


/* RCM - Register accessors */
#define RCM_SRS0_REG(base)                       ((base)->SRS0)
#define RCM_SRS1_REG(base)                       ((base)->SRS1)
#define RCM_RPFC_REG(base)                       ((base)->RPFC)
#define RCM_RPFW_REG(base)                       ((base)->RPFW)
#define RCM_SSRS0_REG(base)                      ((base)->SSRS0)
#define RCM_SSRS1_REG(base)                      ((base)->SSRS1)

/*!
 * @}
 */ /* end of group RCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Masks RCM Register Masks
 * @{
 */

/* SRS0 Bit Fields */
#define RCM_SRS0_WAKEUP_MASK                     0x1u
#define RCM_SRS0_WAKEUP_SHIFT                    0
#define RCM_SRS0_WAKEUP_WIDTH                    1
#define RCM_SRS0_WAKEUP(x)                       (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_WAKEUP_SHIFT))&RCM_SRS0_WAKEUP_MASK)
#define RCM_SRS0_LVD_MASK                        0x2u
#define RCM_SRS0_LVD_SHIFT                       1
#define RCM_SRS0_LVD_WIDTH                       1
#define RCM_SRS0_LVD(x)                          (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_LVD_SHIFT))&RCM_SRS0_LVD_MASK)
#define RCM_SRS0_LOC_MASK                        0x4u
#define RCM_SRS0_LOC_SHIFT                       2
#define RCM_SRS0_LOC_WIDTH                       1
#define RCM_SRS0_LOC(x)                          (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_LOC_SHIFT))&RCM_SRS0_LOC_MASK)
#define RCM_SRS0_LOL_MASK                        0x8u
#define RCM_SRS0_LOL_SHIFT                       3
#define RCM_SRS0_LOL_WIDTH                       1
#define RCM_SRS0_LOL(x)                          (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_LOL_SHIFT))&RCM_SRS0_LOL_MASK)
#define RCM_SRS0_WDOG_MASK                       0x20u
#define RCM_SRS0_WDOG_SHIFT                      5
#define RCM_SRS0_WDOG_WIDTH                      1
#define RCM_SRS0_WDOG(x)                         (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_WDOG_SHIFT))&RCM_SRS0_WDOG_MASK)
#define RCM_SRS0_PIN_MASK                        0x40u
#define RCM_SRS0_PIN_SHIFT                       6
#define RCM_SRS0_PIN_WIDTH                       1
#define RCM_SRS0_PIN(x)                          (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_PIN_SHIFT))&RCM_SRS0_PIN_MASK)
#define RCM_SRS0_POR_MASK                        0x80u
#define RCM_SRS0_POR_SHIFT                       7
#define RCM_SRS0_POR_WIDTH                       1
#define RCM_SRS0_POR(x)                          (((uint8_t)(((uint8_t)(x))<<RCM_SRS0_POR_SHIFT))&RCM_SRS0_POR_MASK)
/* SRS1 Bit Fields */
#define RCM_SRS1_LOCKUP_MASK                     0x2u
#define RCM_SRS1_LOCKUP_SHIFT                    1
#define RCM_SRS1_LOCKUP_WIDTH                    1
#define RCM_SRS1_LOCKUP(x)                       (((uint8_t)(((uint8_t)(x))<<RCM_SRS1_LOCKUP_SHIFT))&RCM_SRS1_LOCKUP_MASK)
#define RCM_SRS1_SW_MASK                         0x4u
#define RCM_SRS1_SW_SHIFT                        2
#define RCM_SRS1_SW_WIDTH                        1
#define RCM_SRS1_SW(x)                           (((uint8_t)(((uint8_t)(x))<<RCM_SRS1_SW_SHIFT))&RCM_SRS1_SW_MASK)
#define RCM_SRS1_MDM_AP_MASK                     0x8u
#define RCM_SRS1_MDM_AP_SHIFT                    3
#define RCM_SRS1_MDM_AP_WIDTH                    1
#define RCM_SRS1_MDM_AP(x)                       (((uint8_t)(((uint8_t)(x))<<RCM_SRS1_MDM_AP_SHIFT))&RCM_SRS1_MDM_AP_MASK)
#define RCM_SRS1_SACKERR_MASK                    0x20u
#define RCM_SRS1_SACKERR_SHIFT                   5
#define RCM_SRS1_SACKERR_WIDTH                   1
#define RCM_SRS1_SACKERR(x)                      (((uint8_t)(((uint8_t)(x))<<RCM_SRS1_SACKERR_SHIFT))&RCM_SRS1_SACKERR_MASK)
/* RPFC Bit Fields */
#define RCM_RPFC_RSTFLTSRW_MASK                  0x3u
#define RCM_RPFC_RSTFLTSRW_SHIFT                 0
#define RCM_RPFC_RSTFLTSRW_WIDTH                 2
#define RCM_RPFC_RSTFLTSRW(x)                    (((uint8_t)(((uint8_t)(x))<<RCM_RPFC_RSTFLTSRW_SHIFT))&RCM_RPFC_RSTFLTSRW_MASK)
#define RCM_RPFC_RSTFLTSS_MASK                   0x4u
#define RCM_RPFC_RSTFLTSS_SHIFT                  2
#define RCM_RPFC_RSTFLTSS_WIDTH                  1
#define RCM_RPFC_RSTFLTSS(x)                     (((uint8_t)(((uint8_t)(x))<<RCM_RPFC_RSTFLTSS_SHIFT))&RCM_RPFC_RSTFLTSS_MASK)
/* RPFW Bit Fields */
#define RCM_RPFW_RSTFLTSEL_MASK                  0x1Fu
#define RCM_RPFW_RSTFLTSEL_SHIFT                 0
#define RCM_RPFW_RSTFLTSEL_WIDTH                 5
#define RCM_RPFW_RSTFLTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<RCM_RPFW_RSTFLTSEL_SHIFT))&RCM_RPFW_RSTFLTSEL_MASK)
/* SSRS0 Bit Fields */
#define RCM_SSRS0_SWAKEUP_MASK                   0x1u
#define RCM_SSRS0_SWAKEUP_SHIFT                  0
#define RCM_SSRS0_SWAKEUP_WIDTH                  1
#define RCM_SSRS0_SWAKEUP(x)                     (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SWAKEUP_SHIFT))&RCM_SSRS0_SWAKEUP_MASK)
#define RCM_SSRS0_SLVD_MASK                      0x2u
#define RCM_SSRS0_SLVD_SHIFT                     1
#define RCM_SSRS0_SLVD_WIDTH                     1
#define RCM_SSRS0_SLVD(x)                        (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SLVD_SHIFT))&RCM_SSRS0_SLVD_MASK)
#define RCM_SSRS0_SLOC_MASK                      0x4u
#define RCM_SSRS0_SLOC_SHIFT                     2
#define RCM_SSRS0_SLOC_WIDTH                     1
#define RCM_SSRS0_SLOC(x)                        (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SLOC_SHIFT))&RCM_SSRS0_SLOC_MASK)
#define RCM_SSRS0_SLOL_MASK                      0x8u
#define RCM_SSRS0_SLOL_SHIFT                     3
#define RCM_SSRS0_SLOL_WIDTH                     1
#define RCM_SSRS0_SLOL(x)                        (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SLOL_SHIFT))&RCM_SSRS0_SLOL_MASK)
#define RCM_SSRS0_SWDOG_MASK                     0x20u
#define RCM_SSRS0_SWDOG_SHIFT                    5
#define RCM_SSRS0_SWDOG_WIDTH                    1
#define RCM_SSRS0_SWDOG(x)                       (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SWDOG_SHIFT))&RCM_SSRS0_SWDOG_MASK)
#define RCM_SSRS0_SPIN_MASK                      0x40u
#define RCM_SSRS0_SPIN_SHIFT                     6
#define RCM_SSRS0_SPIN_WIDTH                     1
#define RCM_SSRS0_SPIN(x)                        (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SPIN_SHIFT))&RCM_SSRS0_SPIN_MASK)
#define RCM_SSRS0_SPOR_MASK                      0x80u
#define RCM_SSRS0_SPOR_SHIFT                     7
#define RCM_SSRS0_SPOR_WIDTH                     1
#define RCM_SSRS0_SPOR(x)                        (((uint8_t)(((uint8_t)(x))<<RCM_SSRS0_SPOR_SHIFT))&RCM_SSRS0_SPOR_MASK)
/* SSRS1 Bit Fields */
#define RCM_SSRS1_SLOCKUP_MASK                   0x2u
#define RCM_SSRS1_SLOCKUP_SHIFT                  1
#define RCM_SSRS1_SLOCKUP_WIDTH                  1
#define RCM_SSRS1_SLOCKUP(x)                     (((uint8_t)(((uint8_t)(x))<<RCM_SSRS1_SLOCKUP_SHIFT))&RCM_SSRS1_SLOCKUP_MASK)
#define RCM_SSRS1_SSW_MASK                       0x4u
#define RCM_SSRS1_SSW_SHIFT                      2
#define RCM_SSRS1_SSW_WIDTH                      1
#define RCM_SSRS1_SSW(x)                         (((uint8_t)(((uint8_t)(x))<<RCM_SSRS1_SSW_SHIFT))&RCM_SSRS1_SSW_MASK)
#define RCM_SSRS1_SMDM_AP_MASK                   0x8u
#define RCM_SSRS1_SMDM_AP_SHIFT                  3
#define RCM_SSRS1_SMDM_AP_WIDTH                  1
#define RCM_SSRS1_SMDM_AP(x)                     (((uint8_t)(((uint8_t)(x))<<RCM_SSRS1_SMDM_AP_SHIFT))&RCM_SSRS1_SMDM_AP_MASK)
#define RCM_SSRS1_SSACKERR_MASK                  0x20u
#define RCM_SSRS1_SSACKERR_SHIFT                 5
#define RCM_SSRS1_SSACKERR_WIDTH                 1
#define RCM_SSRS1_SSACKERR(x)                    (((uint8_t)(((uint8_t)(x))<<RCM_SSRS1_SSACKERR_SHIFT))&RCM_SSRS1_SSACKERR_MASK)

/*!
 * @}
 */ /* end of group RCM_Register_Masks */


/* RCM - Peripheral instance base addresses */
/** Peripheral RCM base address */
#define RCM_BASE                                 (0x4007F000u)
/** Peripheral RCM base pointer */
#define RCM                                      ((RCM_Type *)RCM_BASE)
#define RCM_BASE_PTR                             (RCM)
/** Array initializer of RCM peripheral base addresses */
#define RCM_BASE_ADDRS                           { RCM_BASE }
/** Array initializer of RCM peripheral base pointers */
#define RCM_BASE_PTRS                            { RCM }

/* ----------------------------------------------------------------------------
   -- RCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Accessor_Macros RCM - Register accessor macros
 * @{
 */


/* RCM - Register instance definitions */
/* RCM */
#define RCM_SRS0                                 RCM_SRS0_REG(RCM)
#define RCM_SRS1                                 RCM_SRS1_REG(RCM)
#define RCM_RPFC                                 RCM_RPFC_REG(RCM)
#define RCM_RPFW                                 RCM_RPFW_REG(RCM)
#define RCM_SSRS0                                RCM_SSRS0_REG(RCM)
#define RCM_SSRS1                                RCM_SSRS1_REG(RCM)

/*!
 * @}
 */ /* end of group RCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SOPT1;                             /**< System Options Register 1, offset: 0x0 */
       uint8_t RESERVED_0[4096];
  __IO uint32_t SOPT2;                             /**< System Options Register 2, offset: 0x1004 */
       uint8_t RESERVED_1[4];
  __IO uint32_t SOPT4;                             /**< System Options Register 4, offset: 0x100C */
  __IO uint32_t SOPT5;                             /**< System Options Register 5, offset: 0x1010 */
       uint8_t RESERVED_2[4];
  __IO uint32_t SOPT7;                             /**< System Options Register 7, offset: 0x1018 */
  __IO uint32_t SOPT8;                             /**< System Options Register 8, offset: 0x101C */
  __IO uint32_t SOPT9;                             /**< System Options Register 9, offset: 0x1020 */
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x1024 */
       uint8_t RESERVED_3[12];
  __IO uint32_t SCGC4;                             /**< System Clock Gating Control Register 4, offset: 0x1034 */
  __IO uint32_t SCGC5;                             /**< System Clock Gating Control Register 5, offset: 0x1038 */
  __IO uint32_t SCGC6;                             /**< System Clock Gating Control Register 6, offset: 0x103C */
  __IO uint32_t SCGC7;                             /**< System Clock Gating Control Register 7, offset: 0x1040 */
  __IO uint32_t CLKDIV1;                           /**< System Clock Divider Register 1, offset: 0x1044 */
       uint8_t RESERVED_4[4];
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x104C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x1050 */
  __I  uint32_t UIDH;                              /**< Unique Identification Register High, offset: 0x1054 */
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x1058 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x105C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x1060 */
       uint8_t RESERVED_5[4];
  __IO uint32_t CLKDIV4;                           /**< System Clock Divider Register 4, offset: 0x1068 */
  __IO uint32_t MISCTRL0;                          /**< Miscellaneous Control Register 0, offset: 0x106C */
  __IO uint32_t MISCTRL1;                          /**< Miscellaneous Control Register 1, offset: 0x1070 */
       uint8_t RESERVED_6[140];
  __IO uint32_t WDOGC;                             /**< WDOG Control Register, offset: 0x1100 */
  __IO uint32_t PWRC;                              /**< Power Control Register, offset: 0x1104 */
  __IO uint32_t ADCOPT;                            /**< ADC Channel 6/7 Mux Control Register, offset: 0x1108 */
} SIM_Type, *SIM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register accessors */
#define SIM_SOPT1_REG(base)                      ((base)->SOPT1)
#define SIM_SOPT2_REG(base)                      ((base)->SOPT2)
#define SIM_SOPT4_REG(base)                      ((base)->SOPT4)
#define SIM_SOPT5_REG(base)                      ((base)->SOPT5)
#define SIM_SOPT7_REG(base)                      ((base)->SOPT7)
#define SIM_SOPT8_REG(base)                      ((base)->SOPT8)
#define SIM_SOPT9_REG(base)                      ((base)->SOPT9)
#define SIM_SDID_REG(base)                       ((base)->SDID)
#define SIM_SCGC4_REG(base)                      ((base)->SCGC4)
#define SIM_SCGC5_REG(base)                      ((base)->SCGC5)
#define SIM_SCGC6_REG(base)                      ((base)->SCGC6)
#define SIM_SCGC7_REG(base)                      ((base)->SCGC7)
#define SIM_CLKDIV1_REG(base)                    ((base)->CLKDIV1)
#define SIM_FCFG1_REG(base)                      ((base)->FCFG1)
#define SIM_FCFG2_REG(base)                      ((base)->FCFG2)
#define SIM_UIDH_REG(base)                       ((base)->UIDH)
#define SIM_UIDMH_REG(base)                      ((base)->UIDMH)
#define SIM_UIDML_REG(base)                      ((base)->UIDML)
#define SIM_UIDL_REG(base)                       ((base)->UIDL)
#define SIM_CLKDIV4_REG(base)                    ((base)->CLKDIV4)
#define SIM_MISCTRL0_REG(base)                   ((base)->MISCTRL0)
#define SIM_MISCTRL1_REG(base)                   ((base)->MISCTRL1)
#define SIM_WDOGC_REG(base)                      ((base)->WDOGC)
#define SIM_PWRC_REG(base)                       ((base)->PWRC)
#define SIM_ADCOPT_REG(base)                     ((base)->ADCOPT)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/* SOPT1 Bit Fields */
#define SIM_SOPT1_RAMSIZE_MASK                   0xF000u
#define SIM_SOPT1_RAMSIZE_SHIFT                  12
#define SIM_SOPT1_RAMSIZE_WIDTH                  4
#define SIM_SOPT1_RAMSIZE(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SOPT1_RAMSIZE_SHIFT))&SIM_SOPT1_RAMSIZE_MASK)
#define SIM_SOPT1_OSC32KSEL_MASK                 0xC0000u
#define SIM_SOPT1_OSC32KSEL_SHIFT                18
#define SIM_SOPT1_OSC32KSEL_WIDTH                2
#define SIM_SOPT1_OSC32KSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT1_OSC32KSEL_SHIFT))&SIM_SOPT1_OSC32KSEL_MASK)
/* SOPT2 Bit Fields */
#define SIM_SOPT2_CLKOUTSEL_MASK                 0xE0u
#define SIM_SOPT2_CLKOUTSEL_SHIFT                5
#define SIM_SOPT2_CLKOUTSEL_WIDTH                3
#define SIM_SOPT2_CLKOUTSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_CLKOUTSEL_SHIFT))&SIM_SOPT2_CLKOUTSEL_MASK)
#define SIM_SOPT2_TRACECLKSEL_MASK               0x1000u
#define SIM_SOPT2_TRACECLKSEL_SHIFT              12
#define SIM_SOPT2_TRACECLKSEL_WIDTH              1
#define SIM_SOPT2_TRACECLKSEL(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_TRACECLKSEL_SHIFT))&SIM_SOPT2_TRACECLKSEL_MASK)
/* SOPT4 Bit Fields */
#define SIM_SOPT4_FTM0FLT0_MASK                  0x1u
#define SIM_SOPT4_FTM0FLT0_SHIFT                 0
#define SIM_SOPT4_FTM0FLT0_WIDTH                 1
#define SIM_SOPT4_FTM0FLT0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0FLT0_SHIFT))&SIM_SOPT4_FTM0FLT0_MASK)
#define SIM_SOPT4_FTM0FLT1_MASK                  0x2u
#define SIM_SOPT4_FTM0FLT1_SHIFT                 1
#define SIM_SOPT4_FTM0FLT1_WIDTH                 1
#define SIM_SOPT4_FTM0FLT1(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0FLT1_SHIFT))&SIM_SOPT4_FTM0FLT1_MASK)
#define SIM_SOPT4_FTM0FLT2_MASK                  0x4u
#define SIM_SOPT4_FTM0FLT2_SHIFT                 2
#define SIM_SOPT4_FTM0FLT2_WIDTH                 1
#define SIM_SOPT4_FTM0FLT2(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0FLT2_SHIFT))&SIM_SOPT4_FTM0FLT2_MASK)
#define SIM_SOPT4_FTM0FLT3_MASK                  0x8u
#define SIM_SOPT4_FTM0FLT3_SHIFT                 3
#define SIM_SOPT4_FTM0FLT3_WIDTH                 1
#define SIM_SOPT4_FTM0FLT3(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0FLT3_SHIFT))&SIM_SOPT4_FTM0FLT3_MASK)
#define SIM_SOPT4_FTM1FLT0_MASK                  0x10u
#define SIM_SOPT4_FTM1FLT0_SHIFT                 4
#define SIM_SOPT4_FTM1FLT0_WIDTH                 1
#define SIM_SOPT4_FTM1FLT0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM1FLT0_SHIFT))&SIM_SOPT4_FTM1FLT0_MASK)
#define SIM_SOPT4_FTM3FLT0_MASK                  0x1000u
#define SIM_SOPT4_FTM3FLT0_SHIFT                 12
#define SIM_SOPT4_FTM3FLT0_WIDTH                 1
#define SIM_SOPT4_FTM3FLT0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM3FLT0_SHIFT))&SIM_SOPT4_FTM3FLT0_MASK)
#define SIM_SOPT4_FTM0TRG0SRC_MASK               0x10000u
#define SIM_SOPT4_FTM0TRG0SRC_SHIFT              16
#define SIM_SOPT4_FTM0TRG0SRC_WIDTH              1
#define SIM_SOPT4_FTM0TRG0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0TRG0SRC_SHIFT))&SIM_SOPT4_FTM0TRG0SRC_MASK)
#define SIM_SOPT4_FTM0TRG1SRC_MASK               0x20000u
#define SIM_SOPT4_FTM0TRG1SRC_SHIFT              17
#define SIM_SOPT4_FTM0TRG1SRC_WIDTH              1
#define SIM_SOPT4_FTM0TRG1SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0TRG1SRC_SHIFT))&SIM_SOPT4_FTM0TRG1SRC_MASK)
#define SIM_SOPT4_FTM0TRG2SRC_MASK               0x40000u
#define SIM_SOPT4_FTM0TRG2SRC_SHIFT              18
#define SIM_SOPT4_FTM0TRG2SRC_WIDTH              1
#define SIM_SOPT4_FTM0TRG2SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM0TRG2SRC_SHIFT))&SIM_SOPT4_FTM0TRG2SRC_MASK)
#define SIM_SOPT4_FTM1TRG0SRC_MASK               0x100000u
#define SIM_SOPT4_FTM1TRG0SRC_SHIFT              20
#define SIM_SOPT4_FTM1TRG0SRC_WIDTH              1
#define SIM_SOPT4_FTM1TRG0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM1TRG0SRC_SHIFT))&SIM_SOPT4_FTM1TRG0SRC_MASK)
#define SIM_SOPT4_FTM1TRG2SRC_MASK               0x400000u
#define SIM_SOPT4_FTM1TRG2SRC_SHIFT              22
#define SIM_SOPT4_FTM1TRG2SRC_WIDTH              1
#define SIM_SOPT4_FTM1TRG2SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM1TRG2SRC_SHIFT))&SIM_SOPT4_FTM1TRG2SRC_MASK)
#define SIM_SOPT4_FTM3TRG0SRC_MASK               0x10000000u
#define SIM_SOPT4_FTM3TRG0SRC_SHIFT              28
#define SIM_SOPT4_FTM3TRG0SRC_WIDTH              1
#define SIM_SOPT4_FTM3TRG0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM3TRG0SRC_SHIFT))&SIM_SOPT4_FTM3TRG0SRC_MASK)
#define SIM_SOPT4_FTM3TRG1SRC_MASK               0x20000000u
#define SIM_SOPT4_FTM3TRG1SRC_SHIFT              29
#define SIM_SOPT4_FTM3TRG1SRC_WIDTH              1
#define SIM_SOPT4_FTM3TRG1SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM3TRG1SRC_SHIFT))&SIM_SOPT4_FTM3TRG1SRC_MASK)
#define SIM_SOPT4_FTM3TRG2SRC_MASK               0x40000000u
#define SIM_SOPT4_FTM3TRG2SRC_SHIFT              30
#define SIM_SOPT4_FTM3TRG2SRC_WIDTH              1
#define SIM_SOPT4_FTM3TRG2SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM3TRG2SRC_SHIFT))&SIM_SOPT4_FTM3TRG2SRC_MASK)
/* SOPT5 Bit Fields */
#define SIM_SOPT5_UART0TXSRC_MASK                0x1u
#define SIM_SOPT5_UART0TXSRC_SHIFT               0
#define SIM_SOPT5_UART0TXSRC_WIDTH               1
#define SIM_SOPT5_UART0TXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART0TXSRC_SHIFT))&SIM_SOPT5_UART0TXSRC_MASK)
#define SIM_SOPT5_UART0RXSRC_MASK                0xCu
#define SIM_SOPT5_UART0RXSRC_SHIFT               2
#define SIM_SOPT5_UART0RXSRC_WIDTH               2
#define SIM_SOPT5_UART0RXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART0RXSRC_SHIFT))&SIM_SOPT5_UART0RXSRC_MASK)
#define SIM_SOPT5_UART1TXSRC_MASK                0x10u
#define SIM_SOPT5_UART1TXSRC_SHIFT               4
#define SIM_SOPT5_UART1TXSRC_WIDTH               1
#define SIM_SOPT5_UART1TXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART1TXSRC_SHIFT))&SIM_SOPT5_UART1TXSRC_MASK)
#define SIM_SOPT5_UART1RXSRC_MASK                0xC0u
#define SIM_SOPT5_UART1RXSRC_SHIFT               6
#define SIM_SOPT5_UART1RXSRC_WIDTH               2
#define SIM_SOPT5_UART1RXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART1RXSRC_SHIFT))&SIM_SOPT5_UART1RXSRC_MASK)
/* SOPT7 Bit Fields */
#define SIM_SOPT7_ADCATRGSEL_MASK                0xFu
#define SIM_SOPT7_ADCATRGSEL_SHIFT               0
#define SIM_SOPT7_ADCATRGSEL_WIDTH               4
#define SIM_SOPT7_ADCATRGSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADCATRGSEL_SHIFT))&SIM_SOPT7_ADCATRGSEL_MASK)
#define SIM_SOPT7_ADCAALTTRGEN_MASK              0xC0u
#define SIM_SOPT7_ADCAALTTRGEN_SHIFT             6
#define SIM_SOPT7_ADCAALTTRGEN_WIDTH             2
#define SIM_SOPT7_ADCAALTTRGEN(x)                (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADCAALTTRGEN_SHIFT))&SIM_SOPT7_ADCAALTTRGEN_MASK)
#define SIM_SOPT7_ADCBTRGSEL_MASK                0xF00u
#define SIM_SOPT7_ADCBTRGSEL_SHIFT               8
#define SIM_SOPT7_ADCBTRGSEL_WIDTH               4
#define SIM_SOPT7_ADCBTRGSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADCBTRGSEL_SHIFT))&SIM_SOPT7_ADCBTRGSEL_MASK)
#define SIM_SOPT7_ADCBALTTRGEN_MASK              0xC000u
#define SIM_SOPT7_ADCBALTTRGEN_SHIFT             14
#define SIM_SOPT7_ADCBALTTRGEN_WIDTH             2
#define SIM_SOPT7_ADCBALTTRGEN(x)                (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADCBALTTRGEN_SHIFT))&SIM_SOPT7_ADCBALTTRGEN_MASK)
/* SOPT8 Bit Fields */
#define SIM_SOPT8_FTM0SYNCBIT_MASK               0x1u
#define SIM_SOPT8_FTM0SYNCBIT_SHIFT              0
#define SIM_SOPT8_FTM0SYNCBIT_WIDTH              1
#define SIM_SOPT8_FTM0SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0SYNCBIT_SHIFT))&SIM_SOPT8_FTM0SYNCBIT_MASK)
#define SIM_SOPT8_FTM1SYNCBIT_MASK               0x2u
#define SIM_SOPT8_FTM1SYNCBIT_SHIFT              1
#define SIM_SOPT8_FTM1SYNCBIT_WIDTH              1
#define SIM_SOPT8_FTM1SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM1SYNCBIT_SHIFT))&SIM_SOPT8_FTM1SYNCBIT_MASK)
#define SIM_SOPT8_FTM3SYNCBIT_MASK               0x8u
#define SIM_SOPT8_FTM3SYNCBIT_SHIFT              3
#define SIM_SOPT8_FTM3SYNCBIT_WIDTH              1
#define SIM_SOPT8_FTM3SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3SYNCBIT_SHIFT))&SIM_SOPT8_FTM3SYNCBIT_MASK)
#define SIM_SOPT8_FTM0CFSEL_MASK                 0x100u
#define SIM_SOPT8_FTM0CFSEL_SHIFT                8
#define SIM_SOPT8_FTM0CFSEL_WIDTH                1
#define SIM_SOPT8_FTM0CFSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0CFSEL_SHIFT))&SIM_SOPT8_FTM0CFSEL_MASK)
#define SIM_SOPT8_FTM3CFSEL_MASK                 0x200u
#define SIM_SOPT8_FTM3CFSEL_SHIFT                9
#define SIM_SOPT8_FTM3CFSEL_WIDTH                1
#define SIM_SOPT8_FTM3CFSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3CFSEL_SHIFT))&SIM_SOPT8_FTM3CFSEL_MASK)
#define SIM_SOPT8_FTM0OCH0SRC_MASK               0x10000u
#define SIM_SOPT8_FTM0OCH0SRC_SHIFT              16
#define SIM_SOPT8_FTM0OCH0SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH0SRC_SHIFT))&SIM_SOPT8_FTM0OCH0SRC_MASK)
#define SIM_SOPT8_FTM0OCH1SRC_MASK               0x20000u
#define SIM_SOPT8_FTM0OCH1SRC_SHIFT              17
#define SIM_SOPT8_FTM0OCH1SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH1SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH1SRC_SHIFT))&SIM_SOPT8_FTM0OCH1SRC_MASK)
#define SIM_SOPT8_FTM0OCH2SRC_MASK               0x40000u
#define SIM_SOPT8_FTM0OCH2SRC_SHIFT              18
#define SIM_SOPT8_FTM0OCH2SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH2SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH2SRC_SHIFT))&SIM_SOPT8_FTM0OCH2SRC_MASK)
#define SIM_SOPT8_FTM0OCH3SRC_MASK               0x80000u
#define SIM_SOPT8_FTM0OCH3SRC_SHIFT              19
#define SIM_SOPT8_FTM0OCH3SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH3SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH3SRC_SHIFT))&SIM_SOPT8_FTM0OCH3SRC_MASK)
#define SIM_SOPT8_FTM0OCH4SRC_MASK               0x100000u
#define SIM_SOPT8_FTM0OCH4SRC_SHIFT              20
#define SIM_SOPT8_FTM0OCH4SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH4SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH4SRC_SHIFT))&SIM_SOPT8_FTM0OCH4SRC_MASK)
#define SIM_SOPT8_FTM0OCH5SRC_MASK               0x200000u
#define SIM_SOPT8_FTM0OCH5SRC_SHIFT              21
#define SIM_SOPT8_FTM0OCH5SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH5SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH5SRC_SHIFT))&SIM_SOPT8_FTM0OCH5SRC_MASK)
#define SIM_SOPT8_FTM0OCH6SRC_MASK               0x400000u
#define SIM_SOPT8_FTM0OCH6SRC_SHIFT              22
#define SIM_SOPT8_FTM0OCH6SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH6SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH6SRC_SHIFT))&SIM_SOPT8_FTM0OCH6SRC_MASK)
#define SIM_SOPT8_FTM0OCH7SRC_MASK               0x800000u
#define SIM_SOPT8_FTM0OCH7SRC_SHIFT              23
#define SIM_SOPT8_FTM0OCH7SRC_WIDTH              1
#define SIM_SOPT8_FTM0OCH7SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM0OCH7SRC_SHIFT))&SIM_SOPT8_FTM0OCH7SRC_MASK)
#define SIM_SOPT8_FTM3OCH0SRC_MASK               0x1000000u
#define SIM_SOPT8_FTM3OCH0SRC_SHIFT              24
#define SIM_SOPT8_FTM3OCH0SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH0SRC_SHIFT))&SIM_SOPT8_FTM3OCH0SRC_MASK)
#define SIM_SOPT8_FTM3OCH1SRC_MASK               0x2000000u
#define SIM_SOPT8_FTM3OCH1SRC_SHIFT              25
#define SIM_SOPT8_FTM3OCH1SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH1SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH1SRC_SHIFT))&SIM_SOPT8_FTM3OCH1SRC_MASK)
#define SIM_SOPT8_FTM3OCH2SRC_MASK               0x4000000u
#define SIM_SOPT8_FTM3OCH2SRC_SHIFT              26
#define SIM_SOPT8_FTM3OCH2SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH2SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH2SRC_SHIFT))&SIM_SOPT8_FTM3OCH2SRC_MASK)
#define SIM_SOPT8_FTM3OCH3SRC_MASK               0x8000000u
#define SIM_SOPT8_FTM3OCH3SRC_SHIFT              27
#define SIM_SOPT8_FTM3OCH3SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH3SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH3SRC_SHIFT))&SIM_SOPT8_FTM3OCH3SRC_MASK)
#define SIM_SOPT8_FTM3OCH4SRC_MASK               0x10000000u
#define SIM_SOPT8_FTM3OCH4SRC_SHIFT              28
#define SIM_SOPT8_FTM3OCH4SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH4SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH4SRC_SHIFT))&SIM_SOPT8_FTM3OCH4SRC_MASK)
#define SIM_SOPT8_FTM3OCH5SRC_MASK               0x20000000u
#define SIM_SOPT8_FTM3OCH5SRC_SHIFT              29
#define SIM_SOPT8_FTM3OCH5SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH5SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH5SRC_SHIFT))&SIM_SOPT8_FTM3OCH5SRC_MASK)
#define SIM_SOPT8_FTM3OCH6SRC_MASK               0x40000000u
#define SIM_SOPT8_FTM3OCH6SRC_SHIFT              30
#define SIM_SOPT8_FTM3OCH6SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH6SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH6SRC_SHIFT))&SIM_SOPT8_FTM3OCH6SRC_MASK)
#define SIM_SOPT8_FTM3OCH7SRC_MASK               0x80000000u
#define SIM_SOPT8_FTM3OCH7SRC_SHIFT              31
#define SIM_SOPT8_FTM3OCH7SRC_WIDTH              1
#define SIM_SOPT8_FTM3OCH7SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT8_FTM3OCH7SRC_SHIFT))&SIM_SOPT8_FTM3OCH7SRC_MASK)
/* SOPT9 Bit Fields */
#define SIM_SOPT9_FTM1ICH0SRC_MASK               0x30u
#define SIM_SOPT9_FTM1ICH0SRC_SHIFT              4
#define SIM_SOPT9_FTM1ICH0SRC_WIDTH              2
#define SIM_SOPT9_FTM1ICH0SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT9_FTM1ICH0SRC_SHIFT))&SIM_SOPT9_FTM1ICH0SRC_MASK)
#define SIM_SOPT9_FTM1ICH1SRC_MASK               0x40u
#define SIM_SOPT9_FTM1ICH1SRC_SHIFT              6
#define SIM_SOPT9_FTM1ICH1SRC_WIDTH              1
#define SIM_SOPT9_FTM1ICH1SRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_SOPT9_FTM1ICH1SRC_SHIFT))&SIM_SOPT9_FTM1ICH1SRC_MASK)
#define SIM_SOPT9_FTM0CLKSEL_MASK                0x3000000u
#define SIM_SOPT9_FTM0CLKSEL_SHIFT               24
#define SIM_SOPT9_FTM0CLKSEL_WIDTH               2
#define SIM_SOPT9_FTM0CLKSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT9_FTM0CLKSEL_SHIFT))&SIM_SOPT9_FTM0CLKSEL_MASK)
#define SIM_SOPT9_FTM1CLKSEL_MASK                0xC000000u
#define SIM_SOPT9_FTM1CLKSEL_SHIFT               26
#define SIM_SOPT9_FTM1CLKSEL_WIDTH               2
#define SIM_SOPT9_FTM1CLKSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT9_FTM1CLKSEL_SHIFT))&SIM_SOPT9_FTM1CLKSEL_MASK)
#define SIM_SOPT9_FTM3CLKSEL_MASK                0xC0000000u
#define SIM_SOPT9_FTM3CLKSEL_SHIFT               30
#define SIM_SOPT9_FTM3CLKSEL_WIDTH               2
#define SIM_SOPT9_FTM3CLKSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT9_FTM3CLKSEL_SHIFT))&SIM_SOPT9_FTM3CLKSEL_MASK)
/* SDID Bit Fields */
#define SIM_SDID_PINID_MASK                      0xFu
#define SIM_SDID_PINID_SHIFT                     0
#define SIM_SDID_PINID_WIDTH                     4
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_PINID_SHIFT))&SIM_SDID_PINID_MASK)
#define SIM_SDID_DIEID_MASK                      0xF80u
#define SIM_SDID_DIEID_SHIFT                     7
#define SIM_SDID_DIEID_WIDTH                     5
#define SIM_SDID_DIEID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_DIEID_SHIFT))&SIM_SDID_DIEID_MASK)
#define SIM_SDID_REVID_MASK                      0xF000u
#define SIM_SDID_REVID_SHIFT                     12
#define SIM_SDID_REVID_WIDTH                     4
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_REVID_SHIFT))&SIM_SDID_REVID_MASK)
#define SIM_SDID_SERIESID_MASK                   0xF00000u
#define SIM_SDID_SERIESID_SHIFT                  20
#define SIM_SDID_SERIESID_WIDTH                  4
#define SIM_SDID_SERIESID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_SERIESID_SHIFT))&SIM_SDID_SERIESID_MASK)
#define SIM_SDID_SUBFAMID_MASK                   0xF000000u
#define SIM_SDID_SUBFAMID_SHIFT                  24
#define SIM_SDID_SUBFAMID_WIDTH                  4
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_SUBFAMID_SHIFT))&SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMILYID_MASK                   0xF0000000u
#define SIM_SDID_FAMILYID_SHIFT                  28
#define SIM_SDID_FAMILYID_WIDTH                  4
#define SIM_SDID_FAMILYID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_FAMILYID_SHIFT))&SIM_SDID_FAMILYID_MASK)
/* SCGC4 Bit Fields */
#define SIM_SCGC4_EWM_MASK                       0x2u
#define SIM_SCGC4_EWM_SHIFT                      1
#define SIM_SCGC4_EWM_WIDTH                      1
#define SIM_SCGC4_EWM(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_EWM_SHIFT))&SIM_SCGC4_EWM_MASK)
#define SIM_SCGC4_I2C0_MASK                      0x40u
#define SIM_SCGC4_I2C0_SHIFT                     6
#define SIM_SCGC4_I2C0_WIDTH                     1
#define SIM_SCGC4_I2C0(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_I2C0_SHIFT))&SIM_SCGC4_I2C0_MASK)
#define SIM_SCGC4_UART0_MASK                     0x400u
#define SIM_SCGC4_UART0_SHIFT                    10
#define SIM_SCGC4_UART0_WIDTH                    1
#define SIM_SCGC4_UART0(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_UART0_SHIFT))&SIM_SCGC4_UART0_MASK)
#define SIM_SCGC4_UART1_MASK                     0x800u
#define SIM_SCGC4_UART1_SHIFT                    11
#define SIM_SCGC4_UART1_WIDTH                    1
#define SIM_SCGC4_UART1(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_UART1_SHIFT))&SIM_SCGC4_UART1_MASK)
#define SIM_SCGC4_CMP_MASK                       0x80000u
#define SIM_SCGC4_CMP_SHIFT                      19
#define SIM_SCGC4_CMP_WIDTH                      1
#define SIM_SCGC4_CMP(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_CMP_SHIFT))&SIM_SCGC4_CMP_MASK)
#define SIM_SCGC4_eFlexPWM0_MASK                 0x1000000u
#define SIM_SCGC4_eFlexPWM0_SHIFT                24
#define SIM_SCGC4_eFlexPWM0_WIDTH                1
#define SIM_SCGC4_eFlexPWM0(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_eFlexPWM0_SHIFT))&SIM_SCGC4_eFlexPWM0_MASK)
#define SIM_SCGC4_eFlexPWM1_MASK                 0x2000000u
#define SIM_SCGC4_eFlexPWM1_SHIFT                25
#define SIM_SCGC4_eFlexPWM1_WIDTH                1
#define SIM_SCGC4_eFlexPWM1(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_eFlexPWM1_SHIFT))&SIM_SCGC4_eFlexPWM1_MASK)
#define SIM_SCGC4_eFlexPWM2_MASK                 0x4000000u
#define SIM_SCGC4_eFlexPWM2_SHIFT                26
#define SIM_SCGC4_eFlexPWM2_WIDTH                1
#define SIM_SCGC4_eFlexPWM2(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_eFlexPWM2_SHIFT))&SIM_SCGC4_eFlexPWM2_MASK)
#define SIM_SCGC4_eFlexPWM3_MASK                 0x8000000u
#define SIM_SCGC4_eFlexPWM3_SHIFT                27
#define SIM_SCGC4_eFlexPWM3_WIDTH                1
#define SIM_SCGC4_eFlexPWM3(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SCGC4_eFlexPWM3_SHIFT))&SIM_SCGC4_eFlexPWM3_MASK)
/* SCGC5 Bit Fields */
#define SIM_SCGC5_LPTMR_MASK                     0x1u
#define SIM_SCGC5_LPTMR_SHIFT                    0
#define SIM_SCGC5_LPTMR_WIDTH                    1
#define SIM_SCGC5_LPTMR(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_LPTMR_SHIFT))&SIM_SCGC5_LPTMR_MASK)
#define SIM_SCGC5_PORTA_MASK                     0x200u
#define SIM_SCGC5_PORTA_SHIFT                    9
#define SIM_SCGC5_PORTA_WIDTH                    1
#define SIM_SCGC5_PORTA(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_PORTA_SHIFT))&SIM_SCGC5_PORTA_MASK)
#define SIM_SCGC5_PORTB_MASK                     0x400u
#define SIM_SCGC5_PORTB_SHIFT                    10
#define SIM_SCGC5_PORTB_WIDTH                    1
#define SIM_SCGC5_PORTB(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_PORTB_SHIFT))&SIM_SCGC5_PORTB_MASK)
#define SIM_SCGC5_PORTC_MASK                     0x800u
#define SIM_SCGC5_PORTC_SHIFT                    11
#define SIM_SCGC5_PORTC_WIDTH                    1
#define SIM_SCGC5_PORTC(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_PORTC_SHIFT))&SIM_SCGC5_PORTC_MASK)
#define SIM_SCGC5_PORTD_MASK                     0x1000u
#define SIM_SCGC5_PORTD_SHIFT                    12
#define SIM_SCGC5_PORTD_WIDTH                    1
#define SIM_SCGC5_PORTD(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_PORTD_SHIFT))&SIM_SCGC5_PORTD_MASK)
#define SIM_SCGC5_PORTE_MASK                     0x2000u
#define SIM_SCGC5_PORTE_SHIFT                    13
#define SIM_SCGC5_PORTE_WIDTH                    1
#define SIM_SCGC5_PORTE(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_PORTE_SHIFT))&SIM_SCGC5_PORTE_MASK)
#define SIM_SCGC5_ENC_MASK                       0x200000u
#define SIM_SCGC5_ENC_SHIFT                      21
#define SIM_SCGC5_ENC_WIDTH                      1
#define SIM_SCGC5_ENC(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_ENC_SHIFT))&SIM_SCGC5_ENC_MASK)
#define SIM_SCGC5_XBARA_MASK                     0x2000000u
#define SIM_SCGC5_XBARA_SHIFT                    25
#define SIM_SCGC5_XBARA_WIDTH                    1
#define SIM_SCGC5_XBARA(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_XBARA_SHIFT))&SIM_SCGC5_XBARA_MASK)
#define SIM_SCGC5_XBARB_MASK                     0x4000000u
#define SIM_SCGC5_XBARB_SHIFT                    26
#define SIM_SCGC5_XBARB_WIDTH                    1
#define SIM_SCGC5_XBARB(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_XBARB_SHIFT))&SIM_SCGC5_XBARB_MASK)
#define SIM_SCGC5_AOI_MASK                       0x8000000u
#define SIM_SCGC5_AOI_SHIFT                      27
#define SIM_SCGC5_AOI_WIDTH                      1
#define SIM_SCGC5_AOI(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_AOI_SHIFT))&SIM_SCGC5_AOI_MASK)
#define SIM_SCGC5_ADC_MASK                       0x10000000u
#define SIM_SCGC5_ADC_SHIFT                      28
#define SIM_SCGC5_ADC_WIDTH                      1
#define SIM_SCGC5_ADC(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC5_ADC_SHIFT))&SIM_SCGC5_ADC_MASK)
/* SCGC6 Bit Fields */
#define SIM_SCGC6_FTF_MASK                       0x1u
#define SIM_SCGC6_FTF_SHIFT                      0
#define SIM_SCGC6_FTF_WIDTH                      1
#define SIM_SCGC6_FTF(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FTF_SHIFT))&SIM_SCGC6_FTF_MASK)
#define SIM_SCGC6_DMAMUX_MASK                    0x2u
#define SIM_SCGC6_DMAMUX_SHIFT                   1
#define SIM_SCGC6_DMAMUX_WIDTH                   1
#define SIM_SCGC6_DMAMUX(x)                      (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_DMAMUX_SHIFT))&SIM_SCGC6_DMAMUX_MASK)
#define SIM_SCGC6_FLEXCAN0_MASK                  0x10u
#define SIM_SCGC6_FLEXCAN0_SHIFT                 4
#define SIM_SCGC6_FLEXCAN0_WIDTH                 1
#define SIM_SCGC6_FLEXCAN0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FLEXCAN0_SHIFT))&SIM_SCGC6_FLEXCAN0_MASK)
#define SIM_SCGC6_FLEXCAN1_MASK                  0x20u
#define SIM_SCGC6_FLEXCAN1_SHIFT                 5
#define SIM_SCGC6_FLEXCAN1_WIDTH                 1
#define SIM_SCGC6_FLEXCAN1(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FLEXCAN1_SHIFT))&SIM_SCGC6_FLEXCAN1_MASK)
#define SIM_SCGC6_FTM3_MASK                      0x40u
#define SIM_SCGC6_FTM3_SHIFT                     6
#define SIM_SCGC6_FTM3_WIDTH                     1
#define SIM_SCGC6_FTM3(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FTM3_SHIFT))&SIM_SCGC6_FTM3_MASK)
#define SIM_SCGC6_SPI0_MASK                      0x1000u
#define SIM_SCGC6_SPI0_SHIFT                     12
#define SIM_SCGC6_SPI0_WIDTH                     1
#define SIM_SCGC6_SPI0(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_SPI0_SHIFT))&SIM_SCGC6_SPI0_MASK)
#define SIM_SCGC6_PDB1_MASK                      0x20000u
#define SIM_SCGC6_PDB1_SHIFT                     17
#define SIM_SCGC6_PDB1_WIDTH                     1
#define SIM_SCGC6_PDB1(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_PDB1_SHIFT))&SIM_SCGC6_PDB1_MASK)
#define SIM_SCGC6_CRC_MASK                       0x40000u
#define SIM_SCGC6_CRC_SHIFT                      18
#define SIM_SCGC6_CRC_WIDTH                      1
#define SIM_SCGC6_CRC(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_CRC_SHIFT))&SIM_SCGC6_CRC_MASK)
#define SIM_SCGC6_PDB0_MASK                      0x400000u
#define SIM_SCGC6_PDB0_SHIFT                     22
#define SIM_SCGC6_PDB0_WIDTH                     1
#define SIM_SCGC6_PDB0(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_PDB0_SHIFT))&SIM_SCGC6_PDB0_MASK)
#define SIM_SCGC6_PIT_MASK                       0x800000u
#define SIM_SCGC6_PIT_SHIFT                      23
#define SIM_SCGC6_PIT_WIDTH                      1
#define SIM_SCGC6_PIT(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_PIT_SHIFT))&SIM_SCGC6_PIT_MASK)
#define SIM_SCGC6_FTM0_MASK                      0x1000000u
#define SIM_SCGC6_FTM0_SHIFT                     24
#define SIM_SCGC6_FTM0_WIDTH                     1
#define SIM_SCGC6_FTM0(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FTM0_SHIFT))&SIM_SCGC6_FTM0_MASK)
#define SIM_SCGC6_FTM1_MASK                      0x2000000u
#define SIM_SCGC6_FTM1_SHIFT                     25
#define SIM_SCGC6_FTM1_WIDTH                     1
#define SIM_SCGC6_FTM1(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_FTM1_SHIFT))&SIM_SCGC6_FTM1_MASK)
#define SIM_SCGC6_DAC0_MASK                      0x80000000u
#define SIM_SCGC6_DAC0_SHIFT                     31
#define SIM_SCGC6_DAC0_WIDTH                     1
#define SIM_SCGC6_DAC0(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SCGC6_DAC0_SHIFT))&SIM_SCGC6_DAC0_MASK)
/* SCGC7 Bit Fields */
#define SIM_SCGC7_DMA_MASK                       0x100u
#define SIM_SCGC7_DMA_SHIFT                      8
#define SIM_SCGC7_DMA_WIDTH                      1
#define SIM_SCGC7_DMA(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_SCGC7_DMA_SHIFT))&SIM_SCGC7_DMA_MASK)
/* CLKDIV1 Bit Fields */
#define SIM_CLKDIV1_OUTDIV4_MASK                 0xF0000u
#define SIM_CLKDIV1_OUTDIV4_SHIFT                16
#define SIM_CLKDIV1_OUTDIV4_WIDTH                4
#define SIM_CLKDIV1_OUTDIV4(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV4_SHIFT))&SIM_CLKDIV1_OUTDIV4_MASK)
#define SIM_CLKDIV1_OUTDIV2_MASK                 0xF000000u
#define SIM_CLKDIV1_OUTDIV2_SHIFT                24
#define SIM_CLKDIV1_OUTDIV2_WIDTH                4
#define SIM_CLKDIV1_OUTDIV2(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV2_SHIFT))&SIM_CLKDIV1_OUTDIV2_MASK)
#define SIM_CLKDIV1_OUTDIV1_MASK                 0xF0000000u
#define SIM_CLKDIV1_OUTDIV1_SHIFT                28
#define SIM_CLKDIV1_OUTDIV1_WIDTH                4
#define SIM_CLKDIV1_OUTDIV1(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV1_SHIFT))&SIM_CLKDIV1_OUTDIV1_MASK)
/* FCFG1 Bit Fields */
#define SIM_FCFG1_FLASHDIS_MASK                  0x1u
#define SIM_FCFG1_FLASHDIS_SHIFT                 0
#define SIM_FCFG1_FLASHDIS_WIDTH                 1
#define SIM_FCFG1_FLASHDIS(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_FCFG1_FLASHDIS_SHIFT))&SIM_FCFG1_FLASHDIS_MASK)
#define SIM_FCFG1_FLASHDOZE_MASK                 0x2u
#define SIM_FCFG1_FLASHDOZE_SHIFT                1
#define SIM_FCFG1_FLASHDOZE_WIDTH                1
#define SIM_FCFG1_FLASHDOZE(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_FCFG1_FLASHDOZE_SHIFT))&SIM_FCFG1_FLASHDOZE_MASK)
#define SIM_FCFG1_PFSIZE_MASK                    0xF000000u
#define SIM_FCFG1_PFSIZE_SHIFT                   24
#define SIM_FCFG1_PFSIZE_WIDTH                   4
#define SIM_FCFG1_PFSIZE(x)                      (((uint32_t)(((uint32_t)(x))<<SIM_FCFG1_PFSIZE_SHIFT))&SIM_FCFG1_PFSIZE_MASK)
/* FCFG2 Bit Fields */
#define SIM_FCFG2_MAXADDR0_MASK                  0x7F000000u
#define SIM_FCFG2_MAXADDR0_SHIFT                 24
#define SIM_FCFG2_MAXADDR0_WIDTH                 7
#define SIM_FCFG2_MAXADDR0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_FCFG2_MAXADDR0_SHIFT))&SIM_FCFG2_MAXADDR0_MASK)
/* UIDH Bit Fields */
#define SIM_UIDH_UID_MASK                        0xFFFFFFFFu
#define SIM_UIDH_UID_SHIFT                       0
#define SIM_UIDH_UID_WIDTH                       32
#define SIM_UIDH_UID(x)                          (((uint32_t)(((uint32_t)(x))<<SIM_UIDH_UID_SHIFT))&SIM_UIDH_UID_MASK)
/* UIDMH Bit Fields */
#define SIM_UIDMH_UID_MASK                       0xFFFFFFFFu
#define SIM_UIDMH_UID_SHIFT                      0
#define SIM_UIDMH_UID_WIDTH                      32
#define SIM_UIDMH_UID(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_UIDMH_UID_SHIFT))&SIM_UIDMH_UID_MASK)
/* UIDML Bit Fields */
#define SIM_UIDML_UID_MASK                       0xFFFFFFFFu
#define SIM_UIDML_UID_SHIFT                      0
#define SIM_UIDML_UID_WIDTH                      32
#define SIM_UIDML_UID(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_UIDML_UID_SHIFT))&SIM_UIDML_UID_MASK)
/* UIDL Bit Fields */
#define SIM_UIDL_UID_MASK                        0xFFFFFFFFu
#define SIM_UIDL_UID_SHIFT                       0
#define SIM_UIDL_UID_WIDTH                       32
#define SIM_UIDL_UID(x)                          (((uint32_t)(((uint32_t)(x))<<SIM_UIDL_UID_SHIFT))&SIM_UIDL_UID_MASK)
/* CLKDIV4 Bit Fields */
#define SIM_CLKDIV4_TRACEFRAC_MASK               0x1u
#define SIM_CLKDIV4_TRACEFRAC_SHIFT              0
#define SIM_CLKDIV4_TRACEFRAC_WIDTH              1
#define SIM_CLKDIV4_TRACEFRAC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV4_TRACEFRAC_SHIFT))&SIM_CLKDIV4_TRACEFRAC_MASK)
#define SIM_CLKDIV4_TRACEDIV_MASK                0xEu
#define SIM_CLKDIV4_TRACEDIV_SHIFT               1
#define SIM_CLKDIV4_TRACEDIV_WIDTH               3
#define SIM_CLKDIV4_TRACEDIV(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV4_TRACEDIV_SHIFT))&SIM_CLKDIV4_TRACEDIV_MASK)
#define SIM_CLKDIV4_TRACEDIVEN_MASK              0x10000000u
#define SIM_CLKDIV4_TRACEDIVEN_SHIFT             28
#define SIM_CLKDIV4_TRACEDIVEN_WIDTH             1
#define SIM_CLKDIV4_TRACEDIVEN(x)                (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV4_TRACEDIVEN_SHIFT))&SIM_CLKDIV4_TRACEDIVEN_MASK)
/* MISCTRL0 Bit Fields */
#define SIM_MISCTRL0_CMPWIN0SRC_MASK             0x300u
#define SIM_MISCTRL0_CMPWIN0SRC_SHIFT            8
#define SIM_MISCTRL0_CMPWIN0SRC_WIDTH            2
#define SIM_MISCTRL0_CMPWIN0SRC(x)               (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_CMPWIN0SRC_SHIFT))&SIM_MISCTRL0_CMPWIN0SRC_MASK)
#define SIM_MISCTRL0_CMPWIN1SRC_MASK             0xC00u
#define SIM_MISCTRL0_CMPWIN1SRC_SHIFT            10
#define SIM_MISCTRL0_CMPWIN1SRC_WIDTH            2
#define SIM_MISCTRL0_CMPWIN1SRC(x)               (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_CMPWIN1SRC_SHIFT))&SIM_MISCTRL0_CMPWIN1SRC_MASK)
#define SIM_MISCTRL0_CMPWIN2SRC_MASK             0x3000u
#define SIM_MISCTRL0_CMPWIN2SRC_SHIFT            12
#define SIM_MISCTRL0_CMPWIN2SRC_WIDTH            2
#define SIM_MISCTRL0_CMPWIN2SRC(x)               (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_CMPWIN2SRC_SHIFT))&SIM_MISCTRL0_CMPWIN2SRC_MASK)
#define SIM_MISCTRL0_CMPWIN3SRC_MASK             0xC000u
#define SIM_MISCTRL0_CMPWIN3SRC_SHIFT            14
#define SIM_MISCTRL0_CMPWIN3SRC_WIDTH            2
#define SIM_MISCTRL0_CMPWIN3SRC(x)               (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_CMPWIN3SRC_SHIFT))&SIM_MISCTRL0_CMPWIN3SRC_MASK)
#define SIM_MISCTRL0_EWMINSRC_MASK               0x10000u
#define SIM_MISCTRL0_EWMINSRC_SHIFT              16
#define SIM_MISCTRL0_EWMINSRC_WIDTH              1
#define SIM_MISCTRL0_EWMINSRC(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_EWMINSRC_SHIFT))&SIM_MISCTRL0_EWMINSRC_MASK)
#define SIM_MISCTRL0_DACTRIGSRC_MASK             0xC0000u
#define SIM_MISCTRL0_DACTRIGSRC_SHIFT            18
#define SIM_MISCTRL0_DACTRIGSRC_WIDTH            2
#define SIM_MISCTRL0_DACTRIGSRC(x)               (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL0_DACTRIGSRC_SHIFT))&SIM_MISCTRL0_DACTRIGSRC_MASK)
/* MISCTRL1 Bit Fields */
#define SIM_MISCTRL1_SYNCXBARAPITTRIG0_MASK      0x100u
#define SIM_MISCTRL1_SYNCXBARAPITTRIG0_SHIFT     8
#define SIM_MISCTRL1_SYNCXBARAPITTRIG0_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARAPITTRIG0(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARAPITTRIG0_SHIFT))&SIM_MISCTRL1_SYNCXBARAPITTRIG0_MASK)
#define SIM_MISCTRL1_SYNCXBARAPITTRIG1_MASK      0x200u
#define SIM_MISCTRL1_SYNCXBARAPITTRIG1_SHIFT     9
#define SIM_MISCTRL1_SYNCXBARAPITTRIG1_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARAPITTRIG1(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARAPITTRIG1_SHIFT))&SIM_MISCTRL1_SYNCXBARAPITTRIG1_MASK)
#define SIM_MISCTRL1_SYNCXBARAPITTRIG2_MASK      0x400u
#define SIM_MISCTRL1_SYNCXBARAPITTRIG2_SHIFT     10
#define SIM_MISCTRL1_SYNCXBARAPITTRIG2_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARAPITTRIG2(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARAPITTRIG2_SHIFT))&SIM_MISCTRL1_SYNCXBARAPITTRIG2_MASK)
#define SIM_MISCTRL1_SYNCXBARAPITTRIG3_MASK      0x800u
#define SIM_MISCTRL1_SYNCXBARAPITTRIG3_SHIFT     11
#define SIM_MISCTRL1_SYNCXBARAPITTRIG3_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARAPITTRIG3(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARAPITTRIG3_SHIFT))&SIM_MISCTRL1_SYNCXBARAPITTRIG3_MASK)
#define SIM_MISCTRL1_SYNCXBARBPITTRIG0_MASK      0x1000u
#define SIM_MISCTRL1_SYNCXBARBPITTRIG0_SHIFT     12
#define SIM_MISCTRL1_SYNCXBARBPITTRIG0_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARBPITTRIG0(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARBPITTRIG0_SHIFT))&SIM_MISCTRL1_SYNCXBARBPITTRIG0_MASK)
#define SIM_MISCTRL1_SYNCXBARBPITTRIG1_MASK      0x2000u
#define SIM_MISCTRL1_SYNCXBARBPITTRIG1_SHIFT     13
#define SIM_MISCTRL1_SYNCXBARBPITTRIG1_WIDTH     1
#define SIM_MISCTRL1_SYNCXBARBPITTRIG1(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCXBARBPITTRIG1_SHIFT))&SIM_MISCTRL1_SYNCXBARBPITTRIG1_MASK)
#define SIM_MISCTRL1_SYNCDACHWTRIG_MASK          0x10000u
#define SIM_MISCTRL1_SYNCDACHWTRIG_SHIFT         16
#define SIM_MISCTRL1_SYNCDACHWTRIG_WIDTH         1
#define SIM_MISCTRL1_SYNCDACHWTRIG(x)            (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCDACHWTRIG_SHIFT))&SIM_MISCTRL1_SYNCDACHWTRIG_MASK)
#define SIM_MISCTRL1_SYNCEWMIN_MASK              0x20000u
#define SIM_MISCTRL1_SYNCEWMIN_SHIFT             17
#define SIM_MISCTRL1_SYNCEWMIN_WIDTH             1
#define SIM_MISCTRL1_SYNCEWMIN(x)                (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCEWMIN_SHIFT))&SIM_MISCTRL1_SYNCEWMIN_MASK)
#define SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_MASK      0x100000u
#define SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_SHIFT     20
#define SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_WIDTH     1
#define SIM_MISCTRL1_SYNCCMP0SAMPLEWIN(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_SHIFT))&SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_MASK)
#define SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_MASK      0x200000u
#define SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_SHIFT     21
#define SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_WIDTH     1
#define SIM_MISCTRL1_SYNCCMP1SAMPLEWIN(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_SHIFT))&SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_MASK)
#define SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_MASK      0x400000u
#define SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_SHIFT     22
#define SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_WIDTH     1
#define SIM_MISCTRL1_SYNCCMP2SAMPLEWIN(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_SHIFT))&SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_MASK)
#define SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_MASK      0x800000u
#define SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_SHIFT     23
#define SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_WIDTH     1
#define SIM_MISCTRL1_SYNCCMP3SAMPLEWIN(x)        (((uint32_t)(((uint32_t)(x))<<SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_SHIFT))&SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_MASK)
/* WDOGC Bit Fields */
#define SIM_WDOGC_WDOGCLKS_MASK                  0x2u
#define SIM_WDOGC_WDOGCLKS_SHIFT                 1
#define SIM_WDOGC_WDOGCLKS_WIDTH                 1
#define SIM_WDOGC_WDOGCLKS(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_WDOGC_WDOGCLKS_SHIFT))&SIM_WDOGC_WDOGCLKS_MASK)
/* PWRC Bit Fields */
#define SIM_PWRC_SRPDN_MASK                      0x3u
#define SIM_PWRC_SRPDN_SHIFT                     0
#define SIM_PWRC_SRPDN_WIDTH                     2
#define SIM_PWRC_SRPDN(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SRPDN_SHIFT))&SIM_PWRC_SRPDN_MASK)
#define SIM_PWRC_SR27STDBY_MASK                  0xCu
#define SIM_PWRC_SR27STDBY_SHIFT                 2
#define SIM_PWRC_SR27STDBY_WIDTH                 2
#define SIM_PWRC_SR27STDBY(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SR27STDBY_SHIFT))&SIM_PWRC_SR27STDBY_MASK)
#define SIM_PWRC_SR12STDBY_MASK                  0xC0u
#define SIM_PWRC_SR12STDBY_SHIFT                 6
#define SIM_PWRC_SR12STDBY_WIDTH                 2
#define SIM_PWRC_SR12STDBY(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SR12STDBY_SHIFT))&SIM_PWRC_SR12STDBY_MASK)
#define SIM_PWRC_SRPWRDETEN_MASK                 0x100u
#define SIM_PWRC_SRPWRDETEN_SHIFT                8
#define SIM_PWRC_SRPWRDETEN_WIDTH                1
#define SIM_PWRC_SRPWRDETEN(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SRPWRDETEN_SHIFT))&SIM_PWRC_SRPWRDETEN_MASK)
#define SIM_PWRC_SRPWRRDY_MASK                   0x200u
#define SIM_PWRC_SRPWRRDY_SHIFT                  9
#define SIM_PWRC_SRPWRRDY_WIDTH                  1
#define SIM_PWRC_SRPWRRDY(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SRPWRRDY_SHIFT))&SIM_PWRC_SRPWRRDY_MASK)
#define SIM_PWRC_SRPWROK_MASK                    0x10000u
#define SIM_PWRC_SRPWROK_SHIFT                   16
#define SIM_PWRC_SRPWROK_WIDTH                   1
#define SIM_PWRC_SRPWROK(x)                      (((uint32_t)(((uint32_t)(x))<<SIM_PWRC_SRPWROK_SHIFT))&SIM_PWRC_SRPWROK_MASK)
/* ADCOPT Bit Fields */
#define SIM_ADCOPT_ADCACH6SEL_MASK               0x7u
#define SIM_ADCOPT_ADCACH6SEL_SHIFT              0
#define SIM_ADCOPT_ADCACH6SEL_WIDTH              3
#define SIM_ADCOPT_ADCACH6SEL(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ADCACH6SEL_SHIFT))&SIM_ADCOPT_ADCACH6SEL_MASK)
#define SIM_ADCOPT_ADCACH7SEL_MASK               0x70u
#define SIM_ADCOPT_ADCACH7SEL_SHIFT              4
#define SIM_ADCOPT_ADCACH7SEL_WIDTH              3
#define SIM_ADCOPT_ADCACH7SEL(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ADCACH7SEL_SHIFT))&SIM_ADCOPT_ADCACH7SEL_MASK)
#define SIM_ADCOPT_ADCBCH6SEL_MASK               0x700u
#define SIM_ADCOPT_ADCBCH6SEL_SHIFT              8
#define SIM_ADCOPT_ADCBCH6SEL_WIDTH              3
#define SIM_ADCOPT_ADCBCH6SEL(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ADCBCH6SEL_SHIFT))&SIM_ADCOPT_ADCBCH6SEL_MASK)
#define SIM_ADCOPT_ADCBCH7SEL_MASK               0x7000u
#define SIM_ADCOPT_ADCBCH7SEL_SHIFT              12
#define SIM_ADCOPT_ADCBCH7SEL_WIDTH              3
#define SIM_ADCOPT_ADCBCH7SEL(x)                 (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ADCBCH7SEL_SHIFT))&SIM_ADCOPT_ADCBCH7SEL_MASK)
#define SIM_ADCOPT_ROSB_MASK                     0x1000000u
#define SIM_ADCOPT_ROSB_SHIFT                    24
#define SIM_ADCOPT_ROSB_WIDTH                    1
#define SIM_ADCOPT_ROSB(x)                       (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ROSB_SHIFT))&SIM_ADCOPT_ROSB_MASK)
#define SIM_ADCOPT_ADCIRCLK_MASK                 0x2000000u
#define SIM_ADCOPT_ADCIRCLK_SHIFT                25
#define SIM_ADCOPT_ADCIRCLK_WIDTH                1
#define SIM_ADCOPT_ADCIRCLK(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_ADCOPT_ADCIRCLK_SHIFT))&SIM_ADCOPT_ADCIRCLK_MASK)

/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40047000u)
/** Peripheral SIM base pointer */
#define SIM                                      ((SIM_Type *)SIM_BASE)
#define SIM_BASE_PTR                             (SIM)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS                           { SIM_BASE }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM }

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register instance definitions */
/* SIM */
#define SIM_SOPT1                                SIM_SOPT1_REG(SIM)
#define SIM_SOPT2                                SIM_SOPT2_REG(SIM)
#define SIM_SOPT4                                SIM_SOPT4_REG(SIM)
#define SIM_SOPT5                                SIM_SOPT5_REG(SIM)
#define SIM_SOPT7                                SIM_SOPT7_REG(SIM)
#define SIM_SOPT8                                SIM_SOPT8_REG(SIM)
#define SIM_SOPT9                                SIM_SOPT9_REG(SIM)
#define SIM_SDID                                 SIM_SDID_REG(SIM)
#define SIM_SCGC4                                SIM_SCGC4_REG(SIM)
#define SIM_SCGC5                                SIM_SCGC5_REG(SIM)
#define SIM_SCGC6                                SIM_SCGC6_REG(SIM)
#define SIM_SCGC7                                SIM_SCGC7_REG(SIM)
#define SIM_CLKDIV1                              SIM_CLKDIV1_REG(SIM)
#define SIM_FCFG1                                SIM_FCFG1_REG(SIM)
#define SIM_FCFG2                                SIM_FCFG2_REG(SIM)
#define SIM_UIDH                                 SIM_UIDH_REG(SIM)
#define SIM_UIDMH                                SIM_UIDMH_REG(SIM)
#define SIM_UIDML                                SIM_UIDML_REG(SIM)
#define SIM_UIDL                                 SIM_UIDL_REG(SIM)
#define SIM_CLKDIV4                              SIM_CLKDIV4_REG(SIM)
#define SIM_MISCTRL0                             SIM_MISCTRL0_REG(SIM)
#define SIM_MISCTRL1                             SIM_MISCTRL1_REG(SIM)
#define SIM_WDOGC                                SIM_WDOGC_REG(SIM)
#define SIM_PWRC                                 SIM_PWRC_REG(SIM)
#define SIM_ADCOPT                               SIM_ADCOPT_REG(SIM)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Peripheral_Access_Layer SMC Peripheral Access Layer
 * @{
 */

/** SMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t PMPROT;                             /**< Power Mode Protection register, offset: 0x0 */
  __IO uint8_t PMCTRL;                             /**< Power Mode Control register, offset: 0x1 */
  __IO uint8_t STOPCTRL;                           /**< Stop Control Register, offset: 0x2 */
  __I  uint8_t PMSTAT;                             /**< Power Mode Status register, offset: 0x3 */
} SMC_Type, *SMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Accessor_Macros SMC - Register accessor macros
 * @{
 */


/* SMC - Register accessors */
#define SMC_PMPROT_REG(base)                     ((base)->PMPROT)
#define SMC_PMCTRL_REG(base)                     ((base)->PMCTRL)
#define SMC_STOPCTRL_REG(base)                   ((base)->STOPCTRL)
#define SMC_PMSTAT_REG(base)                     ((base)->PMSTAT)

/*!
 * @}
 */ /* end of group SMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Masks SMC Register Masks
 * @{
 */

/* PMPROT Bit Fields */
#define SMC_PMPROT_AVLLS_MASK                    0x2u
#define SMC_PMPROT_AVLLS_SHIFT                   1
#define SMC_PMPROT_AVLLS_WIDTH                   1
#define SMC_PMPROT_AVLLS(x)                      (((uint8_t)(((uint8_t)(x))<<SMC_PMPROT_AVLLS_SHIFT))&SMC_PMPROT_AVLLS_MASK)
#define SMC_PMPROT_AVLP_MASK                     0x20u
#define SMC_PMPROT_AVLP_SHIFT                    5
#define SMC_PMPROT_AVLP_WIDTH                    1
#define SMC_PMPROT_AVLP(x)                       (((uint8_t)(((uint8_t)(x))<<SMC_PMPROT_AVLP_SHIFT))&SMC_PMPROT_AVLP_MASK)
#define SMC_PMPROT_AHSRUN_MASK                   0x80u
#define SMC_PMPROT_AHSRUN_SHIFT                  7
#define SMC_PMPROT_AHSRUN_WIDTH                  1
#define SMC_PMPROT_AHSRUN(x)                     (((uint8_t)(((uint8_t)(x))<<SMC_PMPROT_AHSRUN_SHIFT))&SMC_PMPROT_AHSRUN_MASK)
/* PMCTRL Bit Fields */
#define SMC_PMCTRL_STOPM_MASK                    0x7u
#define SMC_PMCTRL_STOPM_SHIFT                   0
#define SMC_PMCTRL_STOPM_WIDTH                   3
#define SMC_PMCTRL_STOPM(x)                      (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_STOPM_SHIFT))&SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_STOPA_MASK                    0x8u
#define SMC_PMCTRL_STOPA_SHIFT                   3
#define SMC_PMCTRL_STOPA_WIDTH                   1
#define SMC_PMCTRL_STOPA(x)                      (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_STOPA_SHIFT))&SMC_PMCTRL_STOPA_MASK)
#define SMC_PMCTRL_RUNM_MASK                     0x60u
#define SMC_PMCTRL_RUNM_SHIFT                    5
#define SMC_PMCTRL_RUNM_WIDTH                    2
#define SMC_PMCTRL_RUNM(x)                       (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_RUNM_SHIFT))&SMC_PMCTRL_RUNM_MASK)
/* STOPCTRL Bit Fields */
#define SMC_STOPCTRL_VLLSM_MASK                  0x7u
#define SMC_STOPCTRL_VLLSM_SHIFT                 0
#define SMC_STOPCTRL_VLLSM_WIDTH                 3
#define SMC_STOPCTRL_VLLSM(x)                    (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_VLLSM_SHIFT))&SMC_STOPCTRL_VLLSM_MASK)
#define SMC_STOPCTRL_LPOPO_MASK                  0x8u
#define SMC_STOPCTRL_LPOPO_SHIFT                 3
#define SMC_STOPCTRL_LPOPO_WIDTH                 1
#define SMC_STOPCTRL_LPOPO(x)                    (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_LPOPO_SHIFT))&SMC_STOPCTRL_LPOPO_MASK)
#define SMC_STOPCTRL_RAM2PO_MASK                 0x10u
#define SMC_STOPCTRL_RAM2PO_SHIFT                4
#define SMC_STOPCTRL_RAM2PO_WIDTH                1
#define SMC_STOPCTRL_RAM2PO(x)                   (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_RAM2PO_SHIFT))&SMC_STOPCTRL_RAM2PO_MASK)
#define SMC_STOPCTRL_PORPO_MASK                  0x20u
#define SMC_STOPCTRL_PORPO_SHIFT                 5
#define SMC_STOPCTRL_PORPO_WIDTH                 1
#define SMC_STOPCTRL_PORPO(x)                    (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_PORPO_SHIFT))&SMC_STOPCTRL_PORPO_MASK)
#define SMC_STOPCTRL_PSTOPO_MASK                 0xC0u
#define SMC_STOPCTRL_PSTOPO_SHIFT                6
#define SMC_STOPCTRL_PSTOPO_WIDTH                2
#define SMC_STOPCTRL_PSTOPO(x)                   (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_PSTOPO_SHIFT))&SMC_STOPCTRL_PSTOPO_MASK)
/* PMSTAT Bit Fields */
#define SMC_PMSTAT_PMSTAT_MASK                   0xFFu
#define SMC_PMSTAT_PMSTAT_SHIFT                  0
#define SMC_PMSTAT_PMSTAT_WIDTH                  8
#define SMC_PMSTAT_PMSTAT(x)                     (((uint8_t)(((uint8_t)(x))<<SMC_PMSTAT_PMSTAT_SHIFT))&SMC_PMSTAT_PMSTAT_MASK)

/*!
 * @}
 */ /* end of group SMC_Register_Masks */


/* SMC - Peripheral instance base addresses */
/** Peripheral SMC base address */
#define SMC_BASE                                 (0x4007E000u)
/** Peripheral SMC base pointer */
#define SMC                                      ((SMC_Type *)SMC_BASE)
#define SMC_BASE_PTR                             (SMC)
/** Array initializer of SMC peripheral base addresses */
#define SMC_BASE_ADDRS                           { SMC_BASE }
/** Array initializer of SMC peripheral base pointers */
#define SMC_BASE_PTRS                            { SMC }

/* ----------------------------------------------------------------------------
   -- SMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Accessor_Macros SMC - Register accessor macros
 * @{
 */


/* SMC - Register instance definitions */
/* SMC */
#define SMC_PMPROT                               SMC_PMPROT_REG(SMC)
#define SMC_PMCTRL                               SMC_PMCTRL_REG(SMC)
#define SMC_STOPCTRL                             SMC_STOPCTRL_REG(SMC)
#define SMC_PMSTAT                               SMC_PMSTAT_REG(SMC)

/*!
 * @}
 */ /* end of group SMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 * @{
 */

/** SPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t TCR;                               /**< Transfer Count Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t CTAR[2];                           /**< Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];                     /**< Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint8_t RESERVED_1[24];
  __IO uint32_t SR;                                /**< Status Register, offset: 0x2C */
  __IO uint32_t RSER;                              /**< DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                                          /* offset: 0x34 */
    __IO uint32_t PUSHR;                             /**< PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;                       /**< PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;                              /**< POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR0;                             /**< Transmit FIFO Registers, offset: 0x3C */
  __I  uint32_t TXFR1;                             /**< Transmit FIFO Registers, offset: 0x40 */
  __I  uint32_t TXFR2;                             /**< Transmit FIFO Registers, offset: 0x44 */
  __I  uint32_t TXFR3;                             /**< Transmit FIFO Registers, offset: 0x48 */
       uint8_t RESERVED_2[48];
  __I  uint32_t RXFR0;                             /**< Receive FIFO Registers, offset: 0x7C */
  __I  uint32_t RXFR1;                             /**< Receive FIFO Registers, offset: 0x80 */
  __I  uint32_t RXFR2;                             /**< Receive FIFO Registers, offset: 0x84 */
  __I  uint32_t RXFR3;                             /**< Receive FIFO Registers, offset: 0x88 */
} SPI_Type, *SPI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register accessors */
#define SPI_MCR_REG(base)                        ((base)->MCR)
#define SPI_TCR_REG(base)                        ((base)->TCR)
#define SPI_CTAR_REG(base,index2)                ((base)->CTAR[index2])
#define SPI_CTAR_COUNT                           2
#define SPI_CTAR_SLAVE_REG(base,index2)          ((base)->CTAR_SLAVE[index2])
#define SPI_CTAR_SLAVE_COUNT                     1
#define SPI_SR_REG(base)                         ((base)->SR)
#define SPI_RSER_REG(base)                       ((base)->RSER)
#define SPI_PUSHR_REG(base)                      ((base)->PUSHR)
#define SPI_PUSHR_SLAVE_REG(base)                ((base)->PUSHR_SLAVE)
#define SPI_POPR_REG(base)                       ((base)->POPR)
#define SPI_TXFR0_REG(base)                      ((base)->TXFR0)
#define SPI_TXFR1_REG(base)                      ((base)->TXFR1)
#define SPI_TXFR2_REG(base)                      ((base)->TXFR2)
#define SPI_TXFR3_REG(base)                      ((base)->TXFR3)
#define SPI_RXFR0_REG(base)                      ((base)->RXFR0)
#define SPI_RXFR1_REG(base)                      ((base)->RXFR1)
#define SPI_RXFR2_REG(base)                      ((base)->RXFR2)
#define SPI_RXFR3_REG(base)                      ((base)->RXFR3)

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Masks SPI Register Masks
 * @{
 */

/* MCR Bit Fields */
#define SPI_MCR_HALT_MASK                        0x1u
#define SPI_MCR_HALT_SHIFT                       0
#define SPI_MCR_HALT_WIDTH                       1
#define SPI_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_HALT_SHIFT))&SPI_MCR_HALT_MASK)
#define SPI_MCR_SMPL_PT_MASK                     0x300u
#define SPI_MCR_SMPL_PT_SHIFT                    8
#define SPI_MCR_SMPL_PT_WIDTH                    2
#define SPI_MCR_SMPL_PT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_SMPL_PT_SHIFT))&SPI_MCR_SMPL_PT_MASK)
#define SPI_MCR_CLR_RXF_MASK                     0x400u
#define SPI_MCR_CLR_RXF_SHIFT                    10
#define SPI_MCR_CLR_RXF_WIDTH                    1
#define SPI_MCR_CLR_RXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CLR_RXF_SHIFT))&SPI_MCR_CLR_RXF_MASK)
#define SPI_MCR_CLR_TXF_MASK                     0x800u
#define SPI_MCR_CLR_TXF_SHIFT                    11
#define SPI_MCR_CLR_TXF_WIDTH                    1
#define SPI_MCR_CLR_TXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CLR_TXF_SHIFT))&SPI_MCR_CLR_TXF_MASK)
#define SPI_MCR_DIS_RXF_MASK                     0x1000u
#define SPI_MCR_DIS_RXF_SHIFT                    12
#define SPI_MCR_DIS_RXF_WIDTH                    1
#define SPI_MCR_DIS_RXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DIS_RXF_SHIFT))&SPI_MCR_DIS_RXF_MASK)
#define SPI_MCR_DIS_TXF_MASK                     0x2000u
#define SPI_MCR_DIS_TXF_SHIFT                    13
#define SPI_MCR_DIS_TXF_WIDTH                    1
#define SPI_MCR_DIS_TXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DIS_TXF_SHIFT))&SPI_MCR_DIS_TXF_MASK)
#define SPI_MCR_MDIS_MASK                        0x4000u
#define SPI_MCR_MDIS_SHIFT                       14
#define SPI_MCR_MDIS_WIDTH                       1
#define SPI_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MDIS_SHIFT))&SPI_MCR_MDIS_MASK)
#define SPI_MCR_DOZE_MASK                        0x8000u
#define SPI_MCR_DOZE_SHIFT                       15
#define SPI_MCR_DOZE_WIDTH                       1
#define SPI_MCR_DOZE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DOZE_SHIFT))&SPI_MCR_DOZE_MASK)
#define SPI_MCR_PCSIS_MASK                       0x3F0000u
#define SPI_MCR_PCSIS_SHIFT                      16
#define SPI_MCR_PCSIS_WIDTH                      6
#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSIS_SHIFT))&SPI_MCR_PCSIS_MASK)
#define SPI_MCR_ROOE_MASK                        0x1000000u
#define SPI_MCR_ROOE_SHIFT                       24
#define SPI_MCR_ROOE_WIDTH                       1
#define SPI_MCR_ROOE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_ROOE_SHIFT))&SPI_MCR_ROOE_MASK)
#define SPI_MCR_PCSSE_MASK                       0x2000000u
#define SPI_MCR_PCSSE_SHIFT                      25
#define SPI_MCR_PCSSE_WIDTH                      1
#define SPI_MCR_PCSSE(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSSE_SHIFT))&SPI_MCR_PCSSE_MASK)
#define SPI_MCR_MTFE_MASK                        0x4000000u
#define SPI_MCR_MTFE_SHIFT                       26
#define SPI_MCR_MTFE_WIDTH                       1
#define SPI_MCR_MTFE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MTFE_SHIFT))&SPI_MCR_MTFE_MASK)
#define SPI_MCR_FRZ_MASK                         0x8000000u
#define SPI_MCR_FRZ_SHIFT                        27
#define SPI_MCR_FRZ_WIDTH                        1
#define SPI_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_MCR_FRZ_SHIFT))&SPI_MCR_FRZ_MASK)
#define SPI_MCR_DCONF_MASK                       0x30000000u
#define SPI_MCR_DCONF_SHIFT                      28
#define SPI_MCR_DCONF_WIDTH                      2
#define SPI_MCR_DCONF(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DCONF_SHIFT))&SPI_MCR_DCONF_MASK)
#define SPI_MCR_CONT_SCKE_MASK                   0x40000000u
#define SPI_MCR_CONT_SCKE_SHIFT                  30
#define SPI_MCR_CONT_SCKE_WIDTH                  1
#define SPI_MCR_CONT_SCKE(x)                     (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CONT_SCKE_SHIFT))&SPI_MCR_CONT_SCKE_MASK)
#define SPI_MCR_MSTR_MASK                        0x80000000u
#define SPI_MCR_MSTR_SHIFT                       31
#define SPI_MCR_MSTR_WIDTH                       1
#define SPI_MCR_MSTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MSTR_SHIFT))&SPI_MCR_MSTR_MASK)
/* TCR Bit Fields */
#define SPI_TCR_SPI_TCNT_MASK                    0xFFFF0000u
#define SPI_TCR_SPI_TCNT_SHIFT                   16
#define SPI_TCR_SPI_TCNT_WIDTH                   16
#define SPI_TCR_SPI_TCNT(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TCR_SPI_TCNT_SHIFT))&SPI_TCR_SPI_TCNT_MASK)
/* CTAR Bit Fields */
#define SPI_CTAR_BR_MASK                         0xFu
#define SPI_CTAR_BR_SHIFT                        0
#define SPI_CTAR_BR_WIDTH                        4
#define SPI_CTAR_BR(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_BR_SHIFT))&SPI_CTAR_BR_MASK)
#define SPI_CTAR_DT_MASK                         0xF0u
#define SPI_CTAR_DT_SHIFT                        4
#define SPI_CTAR_DT_WIDTH                        4
#define SPI_CTAR_DT(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DT_SHIFT))&SPI_CTAR_DT_MASK)
#define SPI_CTAR_ASC_MASK                        0xF00u
#define SPI_CTAR_ASC_SHIFT                       8
#define SPI_CTAR_ASC_WIDTH                       4
#define SPI_CTAR_ASC(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_ASC_SHIFT))&SPI_CTAR_ASC_MASK)
#define SPI_CTAR_CSSCK_MASK                      0xF000u
#define SPI_CTAR_CSSCK_SHIFT                     12
#define SPI_CTAR_CSSCK_WIDTH                     4
#define SPI_CTAR_CSSCK(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CSSCK_SHIFT))&SPI_CTAR_CSSCK_MASK)
#define SPI_CTAR_PBR_MASK                        0x30000u
#define SPI_CTAR_PBR_SHIFT                       16
#define SPI_CTAR_PBR_WIDTH                       2
#define SPI_CTAR_PBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PBR_SHIFT))&SPI_CTAR_PBR_MASK)
#define SPI_CTAR_PDT_MASK                        0xC0000u
#define SPI_CTAR_PDT_SHIFT                       18
#define SPI_CTAR_PDT_WIDTH                       2
#define SPI_CTAR_PDT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PDT_SHIFT))&SPI_CTAR_PDT_MASK)
#define SPI_CTAR_PASC_MASK                       0x300000u
#define SPI_CTAR_PASC_SHIFT                      20
#define SPI_CTAR_PASC_WIDTH                      2
#define SPI_CTAR_PASC(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PASC_SHIFT))&SPI_CTAR_PASC_MASK)
#define SPI_CTAR_PCSSCK_MASK                     0xC00000u
#define SPI_CTAR_PCSSCK_SHIFT                    22
#define SPI_CTAR_PCSSCK_WIDTH                    2
#define SPI_CTAR_PCSSCK(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PCSSCK_SHIFT))&SPI_CTAR_PCSSCK_MASK)
#define SPI_CTAR_LSBFE_MASK                      0x1000000u
#define SPI_CTAR_LSBFE_SHIFT                     24
#define SPI_CTAR_LSBFE_WIDTH                     1
#define SPI_CTAR_LSBFE(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_LSBFE_SHIFT))&SPI_CTAR_LSBFE_MASK)
#define SPI_CTAR_CPHA_MASK                       0x2000000u
#define SPI_CTAR_CPHA_SHIFT                      25
#define SPI_CTAR_CPHA_WIDTH                      1
#define SPI_CTAR_CPHA(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CPHA_SHIFT))&SPI_CTAR_CPHA_MASK)
#define SPI_CTAR_CPOL_MASK                       0x4000000u
#define SPI_CTAR_CPOL_SHIFT                      26
#define SPI_CTAR_CPOL_WIDTH                      1
#define SPI_CTAR_CPOL(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CPOL_SHIFT))&SPI_CTAR_CPOL_MASK)
#define SPI_CTAR_FMSZ_MASK                       0x78000000u
#define SPI_CTAR_FMSZ_SHIFT                      27
#define SPI_CTAR_FMSZ_WIDTH                      4
#define SPI_CTAR_FMSZ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_FMSZ_SHIFT))&SPI_CTAR_FMSZ_MASK)
#define SPI_CTAR_DBR_MASK                        0x80000000u
#define SPI_CTAR_DBR_SHIFT                       31
#define SPI_CTAR_DBR_WIDTH                       1
#define SPI_CTAR_DBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DBR_SHIFT))&SPI_CTAR_DBR_MASK)
/* CTAR_SLAVE Bit Fields */
#define SPI_CTAR_SLAVE_CPHA_MASK                 0x2000000u
#define SPI_CTAR_SLAVE_CPHA_SHIFT                25
#define SPI_CTAR_SLAVE_CPHA_WIDTH                1
#define SPI_CTAR_SLAVE_CPHA(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_CPHA_SHIFT))&SPI_CTAR_SLAVE_CPHA_MASK)
#define SPI_CTAR_SLAVE_CPOL_MASK                 0x4000000u
#define SPI_CTAR_SLAVE_CPOL_SHIFT                26
#define SPI_CTAR_SLAVE_CPOL_WIDTH                1
#define SPI_CTAR_SLAVE_CPOL(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_CPOL_SHIFT))&SPI_CTAR_SLAVE_CPOL_MASK)
#define SPI_CTAR_SLAVE_FMSZ_MASK                 0x78000000u
#define SPI_CTAR_SLAVE_FMSZ_SHIFT                27
#define SPI_CTAR_SLAVE_FMSZ_WIDTH                4
#define SPI_CTAR_SLAVE_FMSZ(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_FMSZ_SHIFT))&SPI_CTAR_SLAVE_FMSZ_MASK)
/* SR Bit Fields */
#define SPI_SR_POPNXTPTR_MASK                    0xFu
#define SPI_SR_POPNXTPTR_SHIFT                   0
#define SPI_SR_POPNXTPTR_WIDTH                   4
#define SPI_SR_POPNXTPTR(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_SR_POPNXTPTR_SHIFT))&SPI_SR_POPNXTPTR_MASK)
#define SPI_SR_RXCTR_MASK                        0xF0u
#define SPI_SR_RXCTR_SHIFT                       4
#define SPI_SR_RXCTR_WIDTH                       4
#define SPI_SR_RXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_RXCTR_SHIFT))&SPI_SR_RXCTR_MASK)
#define SPI_SR_TXNXTPTR_MASK                     0xF00u
#define SPI_SR_TXNXTPTR_SHIFT                    8
#define SPI_SR_TXNXTPTR_WIDTH                    4
#define SPI_SR_TXNXTPTR(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXNXTPTR_SHIFT))&SPI_SR_TXNXTPTR_MASK)
#define SPI_SR_TXCTR_MASK                        0xF000u
#define SPI_SR_TXCTR_SHIFT                       12
#define SPI_SR_TXCTR_WIDTH                       4
#define SPI_SR_TXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXCTR_SHIFT))&SPI_SR_TXCTR_MASK)
#define SPI_SR_RFDF_MASK                         0x20000u
#define SPI_SR_RFDF_SHIFT                        17
#define SPI_SR_RFDF_WIDTH                        1
#define SPI_SR_RFDF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_RFDF_SHIFT))&SPI_SR_RFDF_MASK)
#define SPI_SR_RFOF_MASK                         0x80000u
#define SPI_SR_RFOF_SHIFT                        19
#define SPI_SR_RFOF_WIDTH                        1
#define SPI_SR_RFOF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_RFOF_SHIFT))&SPI_SR_RFOF_MASK)
#define SPI_SR_TFFF_MASK                         0x2000000u
#define SPI_SR_TFFF_SHIFT                        25
#define SPI_SR_TFFF_WIDTH                        1
#define SPI_SR_TFFF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_TFFF_SHIFT))&SPI_SR_TFFF_MASK)
#define SPI_SR_TFUF_MASK                         0x8000000u
#define SPI_SR_TFUF_SHIFT                        27
#define SPI_SR_TFUF_WIDTH                        1
#define SPI_SR_TFUF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_TFUF_SHIFT))&SPI_SR_TFUF_MASK)
#define SPI_SR_EOQF_MASK                         0x10000000u
#define SPI_SR_EOQF_SHIFT                        28
#define SPI_SR_EOQF_WIDTH                        1
#define SPI_SR_EOQF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_EOQF_SHIFT))&SPI_SR_EOQF_MASK)
#define SPI_SR_TXRXS_MASK                        0x40000000u
#define SPI_SR_TXRXS_SHIFT                       30
#define SPI_SR_TXRXS_WIDTH                       1
#define SPI_SR_TXRXS(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXRXS_SHIFT))&SPI_SR_TXRXS_MASK)
#define SPI_SR_TCF_MASK                          0x80000000u
#define SPI_SR_TCF_SHIFT                         31
#define SPI_SR_TCF_WIDTH                         1
#define SPI_SR_TCF(x)                            (((uint32_t)(((uint32_t)(x))<<SPI_SR_TCF_SHIFT))&SPI_SR_TCF_MASK)
/* RSER Bit Fields */
#define SPI_RSER_RFDF_DIRS_MASK                  0x10000u
#define SPI_RSER_RFDF_DIRS_SHIFT                 16
#define SPI_RSER_RFDF_DIRS_WIDTH                 1
#define SPI_RSER_RFDF_DIRS(x)                    (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFDF_DIRS_SHIFT))&SPI_RSER_RFDF_DIRS_MASK)
#define SPI_RSER_RFDF_RE_MASK                    0x20000u
#define SPI_RSER_RFDF_RE_SHIFT                   17
#define SPI_RSER_RFDF_RE_WIDTH                   1
#define SPI_RSER_RFDF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFDF_RE_SHIFT))&SPI_RSER_RFDF_RE_MASK)
#define SPI_RSER_RFOF_RE_MASK                    0x80000u
#define SPI_RSER_RFOF_RE_SHIFT                   19
#define SPI_RSER_RFOF_RE_WIDTH                   1
#define SPI_RSER_RFOF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFOF_RE_SHIFT))&SPI_RSER_RFOF_RE_MASK)
#define SPI_RSER_TFFF_DIRS_MASK                  0x1000000u
#define SPI_RSER_TFFF_DIRS_SHIFT                 24
#define SPI_RSER_TFFF_DIRS_WIDTH                 1
#define SPI_RSER_TFFF_DIRS(x)                    (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFFF_DIRS_SHIFT))&SPI_RSER_TFFF_DIRS_MASK)
#define SPI_RSER_TFFF_RE_MASK                    0x2000000u
#define SPI_RSER_TFFF_RE_SHIFT                   25
#define SPI_RSER_TFFF_RE_WIDTH                   1
#define SPI_RSER_TFFF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFFF_RE_SHIFT))&SPI_RSER_TFFF_RE_MASK)
#define SPI_RSER_TFUF_RE_MASK                    0x8000000u
#define SPI_RSER_TFUF_RE_SHIFT                   27
#define SPI_RSER_TFUF_RE_WIDTH                   1
#define SPI_RSER_TFUF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFUF_RE_SHIFT))&SPI_RSER_TFUF_RE_MASK)
#define SPI_RSER_EOQF_RE_MASK                    0x10000000u
#define SPI_RSER_EOQF_RE_SHIFT                   28
#define SPI_RSER_EOQF_RE_WIDTH                   1
#define SPI_RSER_EOQF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_EOQF_RE_SHIFT))&SPI_RSER_EOQF_RE_MASK)
#define SPI_RSER_TCF_RE_MASK                     0x80000000u
#define SPI_RSER_TCF_RE_SHIFT                    31
#define SPI_RSER_TCF_RE_WIDTH                    1
#define SPI_RSER_TCF_RE(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TCF_RE_SHIFT))&SPI_RSER_TCF_RE_MASK)
/* PUSHR Bit Fields */
#define SPI_PUSHR_TXDATA_MASK                    0xFFFFu
#define SPI_PUSHR_TXDATA_SHIFT                   0
#define SPI_PUSHR_TXDATA_WIDTH                   16
#define SPI_PUSHR_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_TXDATA_SHIFT))&SPI_PUSHR_TXDATA_MASK)
#define SPI_PUSHR_PCS_MASK                       0x3F0000u
#define SPI_PUSHR_PCS_SHIFT                      16
#define SPI_PUSHR_PCS_WIDTH                      6
#define SPI_PUSHR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_PCS_SHIFT))&SPI_PUSHR_PCS_MASK)
#define SPI_PUSHR_CTCNT_MASK                     0x4000000u
#define SPI_PUSHR_CTCNT_SHIFT                    26
#define SPI_PUSHR_CTCNT_WIDTH                    1
#define SPI_PUSHR_CTCNT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTCNT_SHIFT))&SPI_PUSHR_CTCNT_MASK)
#define SPI_PUSHR_EOQ_MASK                       0x8000000u
#define SPI_PUSHR_EOQ_SHIFT                      27
#define SPI_PUSHR_EOQ_WIDTH                      1
#define SPI_PUSHR_EOQ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_EOQ_SHIFT))&SPI_PUSHR_EOQ_MASK)
#define SPI_PUSHR_CTAS_MASK                      0x70000000u
#define SPI_PUSHR_CTAS_SHIFT                     28
#define SPI_PUSHR_CTAS_WIDTH                     3
#define SPI_PUSHR_CTAS(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTAS_SHIFT))&SPI_PUSHR_CTAS_MASK)
#define SPI_PUSHR_CONT_MASK                      0x80000000u
#define SPI_PUSHR_CONT_SHIFT                     31
#define SPI_PUSHR_CONT_WIDTH                     1
#define SPI_PUSHR_CONT(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CONT_SHIFT))&SPI_PUSHR_CONT_MASK)
/* PUSHR_SLAVE Bit Fields */
#define SPI_PUSHR_SLAVE_TXDATA_MASK              0xFFFFu
#define SPI_PUSHR_SLAVE_TXDATA_SHIFT             0
#define SPI_PUSHR_SLAVE_TXDATA_WIDTH             16
#define SPI_PUSHR_SLAVE_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_SLAVE_TXDATA_SHIFT))&SPI_PUSHR_SLAVE_TXDATA_MASK)
/* POPR Bit Fields */
#define SPI_POPR_RXDATA_MASK                     0xFFFFFFFFu
#define SPI_POPR_RXDATA_SHIFT                    0
#define SPI_POPR_RXDATA_WIDTH                    32
#define SPI_POPR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_POPR_RXDATA_SHIFT))&SPI_POPR_RXDATA_MASK)
/* TXFR0 Bit Fields */
#define SPI_TXFR0_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR0_TXDATA_SHIFT                   0
#define SPI_TXFR0_TXDATA_WIDTH                   16
#define SPI_TXFR0_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXDATA_SHIFT))&SPI_TXFR0_TXDATA_MASK)
#define SPI_TXFR0_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR0_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR0_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR0_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXCMD_TXDATA_SHIFT))&SPI_TXFR0_TXCMD_TXDATA_MASK)
/* TXFR1 Bit Fields */
#define SPI_TXFR1_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR1_TXDATA_SHIFT                   0
#define SPI_TXFR1_TXDATA_WIDTH                   16
#define SPI_TXFR1_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXDATA_SHIFT))&SPI_TXFR1_TXDATA_MASK)
#define SPI_TXFR1_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR1_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR1_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR1_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXCMD_TXDATA_SHIFT))&SPI_TXFR1_TXCMD_TXDATA_MASK)
/* TXFR2 Bit Fields */
#define SPI_TXFR2_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR2_TXDATA_SHIFT                   0
#define SPI_TXFR2_TXDATA_WIDTH                   16
#define SPI_TXFR2_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXDATA_SHIFT))&SPI_TXFR2_TXDATA_MASK)
#define SPI_TXFR2_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR2_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR2_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR2_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXCMD_TXDATA_SHIFT))&SPI_TXFR2_TXCMD_TXDATA_MASK)
/* TXFR3 Bit Fields */
#define SPI_TXFR3_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR3_TXDATA_SHIFT                   0
#define SPI_TXFR3_TXDATA_WIDTH                   16
#define SPI_TXFR3_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXDATA_SHIFT))&SPI_TXFR3_TXDATA_MASK)
#define SPI_TXFR3_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR3_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR3_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR3_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXCMD_TXDATA_SHIFT))&SPI_TXFR3_TXCMD_TXDATA_MASK)
/* RXFR0 Bit Fields */
#define SPI_RXFR0_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR0_RXDATA_SHIFT                   0
#define SPI_RXFR0_RXDATA_WIDTH                   32
#define SPI_RXFR0_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR0_RXDATA_SHIFT))&SPI_RXFR0_RXDATA_MASK)
/* RXFR1 Bit Fields */
#define SPI_RXFR1_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR1_RXDATA_SHIFT                   0
#define SPI_RXFR1_RXDATA_WIDTH                   32
#define SPI_RXFR1_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR1_RXDATA_SHIFT))&SPI_RXFR1_RXDATA_MASK)
/* RXFR2 Bit Fields */
#define SPI_RXFR2_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR2_RXDATA_SHIFT                   0
#define SPI_RXFR2_RXDATA_WIDTH                   32
#define SPI_RXFR2_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR2_RXDATA_SHIFT))&SPI_RXFR2_RXDATA_MASK)
/* RXFR3 Bit Fields */
#define SPI_RXFR3_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR3_RXDATA_SHIFT                   0
#define SPI_RXFR3_RXDATA_WIDTH                   32
#define SPI_RXFR3_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR3_RXDATA_SHIFT))&SPI_RXFR3_RXDATA_MASK)

/*!
 * @}
 */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base address */
#define SPI0_BASE                                (0x4002C000u)
/** Peripheral SPI0 base pointer */
#define SPI0                                     ((SPI_Type *)SPI0_BASE)
#define SPI0_BASE_PTR                            (SPI0)
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE }
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0 }

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register instance definitions */
/* SPI0 */
#define SPI0_MCR                                 SPI_MCR_REG(SPI0)
#define SPI0_TCR                                 SPI_TCR_REG(SPI0)
#define SPI0_CTAR0                               SPI_CTAR_REG(SPI0,0)
#define SPI0_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI0,0)
#define SPI0_CTAR1                               SPI_CTAR_REG(SPI0,1)
#define SPI0_SR                                  SPI_SR_REG(SPI0)
#define SPI0_RSER                                SPI_RSER_REG(SPI0)
#define SPI0_PUSHR                               SPI_PUSHR_REG(SPI0)
#define SPI0_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI0)
#define SPI0_POPR                                SPI_POPR_REG(SPI0)
#define SPI0_TXFR0                               SPI_TXFR0_REG(SPI0)
#define SPI0_TXFR1                               SPI_TXFR1_REG(SPI0)
#define SPI0_TXFR2                               SPI_TXFR2_REG(SPI0)
#define SPI0_TXFR3                               SPI_TXFR3_REG(SPI0)
#define SPI0_RXFR0                               SPI_RXFR0_REG(SPI0)
#define SPI0_RXFR1                               SPI_RXFR1_REG(SPI0)
#define SPI0_RXFR2                               SPI_RXFR2_REG(SPI0)
#define SPI0_RXFR3                               SPI_RXFR3_REG(SPI0)

/* SPI - Register array accessors */
#define SPI0_CTAR(index2)                        SPI_CTAR_REG(SPI0,index2)
#define SPI0_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI0,index2)

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- UART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Peripheral_Access_Layer UART Peripheral Access Layer
 * @{
 */

/** UART - Register Layout Typedef */
typedef struct {
  __IO uint8_t BDH;                                /**< UART Baud Rate Registers: High, offset: 0x0 */
  __IO uint8_t BDL;                                /**< UART Baud Rate Registers: Low, offset: 0x1 */
  __IO uint8_t C1;                                 /**< UART Control Register 1, offset: 0x2 */
  __IO uint8_t C2;                                 /**< UART Control Register 2, offset: 0x3 */
  __I  uint8_t S1;                                 /**< UART Status Register 1, offset: 0x4 */
  __IO uint8_t S2;                                 /**< UART Status Register 2, offset: 0x5 */
  __IO uint8_t C3;                                 /**< UART Control Register 3, offset: 0x6 */
  __IO uint8_t D;                                  /**< UART Data Register, offset: 0x7 */
  __IO uint8_t MA1;                                /**< UART Match Address Registers 1, offset: 0x8 */
  __IO uint8_t MA2;                                /**< UART Match Address Registers 2, offset: 0x9 */
  __IO uint8_t C4;                                 /**< UART Control Register 4, offset: 0xA */
  __IO uint8_t C5;                                 /**< UART Control Register 5, offset: 0xB */
  __I  uint8_t ED;                                 /**< UART Extended Data Register, offset: 0xC */
  __IO uint8_t MODEM;                              /**< UART Modem Register, offset: 0xD */
       uint8_t RESERVED_0[2];
  __IO uint8_t PFIFO;                              /**< UART FIFO Parameters, offset: 0x10 */
  __IO uint8_t CFIFO;                              /**< UART FIFO Control Register, offset: 0x11 */
  __IO uint8_t SFIFO;                              /**< UART FIFO Status Register, offset: 0x12 */
  __IO uint8_t TWFIFO;                             /**< UART FIFO Transmit Watermark, offset: 0x13 */
  __I  uint8_t TCFIFO;                             /**< UART FIFO Transmit Count, offset: 0x14 */
  __IO uint8_t RWFIFO;                             /**< UART FIFO Receive Watermark, offset: 0x15 */
  __I  uint8_t RCFIFO;                             /**< UART FIFO Receive Count, offset: 0x16 */
} UART_Type, *UART_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */


/* UART - Register accessors */
#define UART_BDH_REG(base)                       ((base)->BDH)
#define UART_BDL_REG(base)                       ((base)->BDL)
#define UART_C1_REG(base)                        ((base)->C1)
#define UART_C2_REG(base)                        ((base)->C2)
#define UART_S1_REG(base)                        ((base)->S1)
#define UART_S2_REG(base)                        ((base)->S2)
#define UART_C3_REG(base)                        ((base)->C3)
#define UART_D_REG(base)                         ((base)->D)
#define UART_MA1_REG(base)                       ((base)->MA1)
#define UART_MA2_REG(base)                       ((base)->MA2)
#define UART_C4_REG(base)                        ((base)->C4)
#define UART_C5_REG(base)                        ((base)->C5)
#define UART_ED_REG(base)                        ((base)->ED)
#define UART_MODEM_REG(base)                     ((base)->MODEM)
#define UART_PFIFO_REG(base)                     ((base)->PFIFO)
#define UART_CFIFO_REG(base)                     ((base)->CFIFO)
#define UART_SFIFO_REG(base)                     ((base)->SFIFO)
#define UART_TWFIFO_REG(base)                    ((base)->TWFIFO)
#define UART_TCFIFO_REG(base)                    ((base)->TCFIFO)
#define UART_RWFIFO_REG(base)                    ((base)->RWFIFO)
#define UART_RCFIFO_REG(base)                    ((base)->RCFIFO)

/*!
 * @}
 */ /* end of group UART_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- UART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Masks UART Register Masks
 * @{
 */

/* BDH Bit Fields */
#define UART_BDH_SBR_MASK                        0x1Fu
#define UART_BDH_SBR_SHIFT                       0
#define UART_BDH_SBR_WIDTH                       5
#define UART_BDH_SBR(x)                          (((uint8_t)(((uint8_t)(x))<<UART_BDH_SBR_SHIFT))&UART_BDH_SBR_MASK)
#define UART_BDH_SBNS_MASK                       0x20u
#define UART_BDH_SBNS_SHIFT                      5
#define UART_BDH_SBNS_WIDTH                      1
#define UART_BDH_SBNS(x)                         (((uint8_t)(((uint8_t)(x))<<UART_BDH_SBNS_SHIFT))&UART_BDH_SBNS_MASK)
#define UART_BDH_RXEDGIE_MASK                    0x40u
#define UART_BDH_RXEDGIE_SHIFT                   6
#define UART_BDH_RXEDGIE_WIDTH                   1
#define UART_BDH_RXEDGIE(x)                      (((uint8_t)(((uint8_t)(x))<<UART_BDH_RXEDGIE_SHIFT))&UART_BDH_RXEDGIE_MASK)
#define UART_BDH_LBKDIE_MASK                     0x80u
#define UART_BDH_LBKDIE_SHIFT                    7
#define UART_BDH_LBKDIE_WIDTH                    1
#define UART_BDH_LBKDIE(x)                       (((uint8_t)(((uint8_t)(x))<<UART_BDH_LBKDIE_SHIFT))&UART_BDH_LBKDIE_MASK)
/* BDL Bit Fields */
#define UART_BDL_SBR_MASK                        0xFFu
#define UART_BDL_SBR_SHIFT                       0
#define UART_BDL_SBR_WIDTH                       8
#define UART_BDL_SBR(x)                          (((uint8_t)(((uint8_t)(x))<<UART_BDL_SBR_SHIFT))&UART_BDL_SBR_MASK)
/* C1 Bit Fields */
#define UART_C1_PT_MASK                          0x1u
#define UART_C1_PT_SHIFT                         0
#define UART_C1_PT_WIDTH                         1
#define UART_C1_PT(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C1_PT_SHIFT))&UART_C1_PT_MASK)
#define UART_C1_PE_MASK                          0x2u
#define UART_C1_PE_SHIFT                         1
#define UART_C1_PE_WIDTH                         1
#define UART_C1_PE(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C1_PE_SHIFT))&UART_C1_PE_MASK)
#define UART_C1_ILT_MASK                         0x4u
#define UART_C1_ILT_SHIFT                        2
#define UART_C1_ILT_WIDTH                        1
#define UART_C1_ILT(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C1_ILT_SHIFT))&UART_C1_ILT_MASK)
#define UART_C1_WAKE_MASK                        0x8u
#define UART_C1_WAKE_SHIFT                       3
#define UART_C1_WAKE_WIDTH                       1
#define UART_C1_WAKE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C1_WAKE_SHIFT))&UART_C1_WAKE_MASK)
#define UART_C1_M_MASK                           0x10u
#define UART_C1_M_SHIFT                          4
#define UART_C1_M_WIDTH                          1
#define UART_C1_M(x)                             (((uint8_t)(((uint8_t)(x))<<UART_C1_M_SHIFT))&UART_C1_M_MASK)
#define UART_C1_RSRC_MASK                        0x20u
#define UART_C1_RSRC_SHIFT                       5
#define UART_C1_RSRC_WIDTH                       1
#define UART_C1_RSRC(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C1_RSRC_SHIFT))&UART_C1_RSRC_MASK)
#define UART_C1_UARTSWAI_MASK                    0x40u
#define UART_C1_UARTSWAI_SHIFT                   6
#define UART_C1_UARTSWAI_WIDTH                   1
#define UART_C1_UARTSWAI(x)                      (((uint8_t)(((uint8_t)(x))<<UART_C1_UARTSWAI_SHIFT))&UART_C1_UARTSWAI_MASK)
#define UART_C1_LOOPS_MASK                       0x80u
#define UART_C1_LOOPS_SHIFT                      7
#define UART_C1_LOOPS_WIDTH                      1
#define UART_C1_LOOPS(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C1_LOOPS_SHIFT))&UART_C1_LOOPS_MASK)
/* C2 Bit Fields */
#define UART_C2_SBK_MASK                         0x1u
#define UART_C2_SBK_SHIFT                        0
#define UART_C2_SBK_WIDTH                        1
#define UART_C2_SBK(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C2_SBK_SHIFT))&UART_C2_SBK_MASK)
#define UART_C2_RWU_MASK                         0x2u
#define UART_C2_RWU_SHIFT                        1
#define UART_C2_RWU_WIDTH                        1
#define UART_C2_RWU(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C2_RWU_SHIFT))&UART_C2_RWU_MASK)
#define UART_C2_RE_MASK                          0x4u
#define UART_C2_RE_SHIFT                         2
#define UART_C2_RE_WIDTH                         1
#define UART_C2_RE(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C2_RE_SHIFT))&UART_C2_RE_MASK)
#define UART_C2_TE_MASK                          0x8u
#define UART_C2_TE_SHIFT                         3
#define UART_C2_TE_WIDTH                         1
#define UART_C2_TE(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C2_TE_SHIFT))&UART_C2_TE_MASK)
#define UART_C2_ILIE_MASK                        0x10u
#define UART_C2_ILIE_SHIFT                       4
#define UART_C2_ILIE_WIDTH                       1
#define UART_C2_ILIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C2_ILIE_SHIFT))&UART_C2_ILIE_MASK)
#define UART_C2_RIE_MASK                         0x20u
#define UART_C2_RIE_SHIFT                        5
#define UART_C2_RIE_WIDTH                        1
#define UART_C2_RIE(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C2_RIE_SHIFT))&UART_C2_RIE_MASK)
#define UART_C2_TCIE_MASK                        0x40u
#define UART_C2_TCIE_SHIFT                       6
#define UART_C2_TCIE_WIDTH                       1
#define UART_C2_TCIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C2_TCIE_SHIFT))&UART_C2_TCIE_MASK)
#define UART_C2_TIE_MASK                         0x80u
#define UART_C2_TIE_SHIFT                        7
#define UART_C2_TIE_WIDTH                        1
#define UART_C2_TIE(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C2_TIE_SHIFT))&UART_C2_TIE_MASK)
/* S1 Bit Fields */
#define UART_S1_PF_MASK                          0x1u
#define UART_S1_PF_SHIFT                         0
#define UART_S1_PF_WIDTH                         1
#define UART_S1_PF(x)                            (((uint8_t)(((uint8_t)(x))<<UART_S1_PF_SHIFT))&UART_S1_PF_MASK)
#define UART_S1_FE_MASK                          0x2u
#define UART_S1_FE_SHIFT                         1
#define UART_S1_FE_WIDTH                         1
#define UART_S1_FE(x)                            (((uint8_t)(((uint8_t)(x))<<UART_S1_FE_SHIFT))&UART_S1_FE_MASK)
#define UART_S1_NF_MASK                          0x4u
#define UART_S1_NF_SHIFT                         2
#define UART_S1_NF_WIDTH                         1
#define UART_S1_NF(x)                            (((uint8_t)(((uint8_t)(x))<<UART_S1_NF_SHIFT))&UART_S1_NF_MASK)
#define UART_S1_OR_MASK                          0x8u
#define UART_S1_OR_SHIFT                         3
#define UART_S1_OR_WIDTH                         1
#define UART_S1_OR(x)                            (((uint8_t)(((uint8_t)(x))<<UART_S1_OR_SHIFT))&UART_S1_OR_MASK)
#define UART_S1_IDLE_MASK                        0x10u
#define UART_S1_IDLE_SHIFT                       4
#define UART_S1_IDLE_WIDTH                       1
#define UART_S1_IDLE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_S1_IDLE_SHIFT))&UART_S1_IDLE_MASK)
#define UART_S1_RDRF_MASK                        0x20u
#define UART_S1_RDRF_SHIFT                       5
#define UART_S1_RDRF_WIDTH                       1
#define UART_S1_RDRF(x)                          (((uint8_t)(((uint8_t)(x))<<UART_S1_RDRF_SHIFT))&UART_S1_RDRF_MASK)
#define UART_S1_TC_MASK                          0x40u
#define UART_S1_TC_SHIFT                         6
#define UART_S1_TC_WIDTH                         1
#define UART_S1_TC(x)                            (((uint8_t)(((uint8_t)(x))<<UART_S1_TC_SHIFT))&UART_S1_TC_MASK)
#define UART_S1_TDRE_MASK                        0x80u
#define UART_S1_TDRE_SHIFT                       7
#define UART_S1_TDRE_WIDTH                       1
#define UART_S1_TDRE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_S1_TDRE_SHIFT))&UART_S1_TDRE_MASK)
/* S2 Bit Fields */
#define UART_S2_RAF_MASK                         0x1u
#define UART_S2_RAF_SHIFT                        0
#define UART_S2_RAF_WIDTH                        1
#define UART_S2_RAF(x)                           (((uint8_t)(((uint8_t)(x))<<UART_S2_RAF_SHIFT))&UART_S2_RAF_MASK)
#define UART_S2_LBKDE_MASK                       0x2u
#define UART_S2_LBKDE_SHIFT                      1
#define UART_S2_LBKDE_WIDTH                      1
#define UART_S2_LBKDE(x)                         (((uint8_t)(((uint8_t)(x))<<UART_S2_LBKDE_SHIFT))&UART_S2_LBKDE_MASK)
#define UART_S2_BRK13_MASK                       0x4u
#define UART_S2_BRK13_SHIFT                      2
#define UART_S2_BRK13_WIDTH                      1
#define UART_S2_BRK13(x)                         (((uint8_t)(((uint8_t)(x))<<UART_S2_BRK13_SHIFT))&UART_S2_BRK13_MASK)
#define UART_S2_RWUID_MASK                       0x8u
#define UART_S2_RWUID_SHIFT                      3
#define UART_S2_RWUID_WIDTH                      1
#define UART_S2_RWUID(x)                         (((uint8_t)(((uint8_t)(x))<<UART_S2_RWUID_SHIFT))&UART_S2_RWUID_MASK)
#define UART_S2_RXINV_MASK                       0x10u
#define UART_S2_RXINV_SHIFT                      4
#define UART_S2_RXINV_WIDTH                      1
#define UART_S2_RXINV(x)                         (((uint8_t)(((uint8_t)(x))<<UART_S2_RXINV_SHIFT))&UART_S2_RXINV_MASK)
#define UART_S2_MSBF_MASK                        0x20u
#define UART_S2_MSBF_SHIFT                       5
#define UART_S2_MSBF_WIDTH                       1
#define UART_S2_MSBF(x)                          (((uint8_t)(((uint8_t)(x))<<UART_S2_MSBF_SHIFT))&UART_S2_MSBF_MASK)
#define UART_S2_RXEDGIF_MASK                     0x40u
#define UART_S2_RXEDGIF_SHIFT                    6
#define UART_S2_RXEDGIF_WIDTH                    1
#define UART_S2_RXEDGIF(x)                       (((uint8_t)(((uint8_t)(x))<<UART_S2_RXEDGIF_SHIFT))&UART_S2_RXEDGIF_MASK)
#define UART_S2_LBKDIF_MASK                      0x80u
#define UART_S2_LBKDIF_SHIFT                     7
#define UART_S2_LBKDIF_WIDTH                     1
#define UART_S2_LBKDIF(x)                        (((uint8_t)(((uint8_t)(x))<<UART_S2_LBKDIF_SHIFT))&UART_S2_LBKDIF_MASK)
/* C3 Bit Fields */
#define UART_C3_PEIE_MASK                        0x1u
#define UART_C3_PEIE_SHIFT                       0
#define UART_C3_PEIE_WIDTH                       1
#define UART_C3_PEIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C3_PEIE_SHIFT))&UART_C3_PEIE_MASK)
#define UART_C3_FEIE_MASK                        0x2u
#define UART_C3_FEIE_SHIFT                       1
#define UART_C3_FEIE_WIDTH                       1
#define UART_C3_FEIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C3_FEIE_SHIFT))&UART_C3_FEIE_MASK)
#define UART_C3_NEIE_MASK                        0x4u
#define UART_C3_NEIE_SHIFT                       2
#define UART_C3_NEIE_WIDTH                       1
#define UART_C3_NEIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C3_NEIE_SHIFT))&UART_C3_NEIE_MASK)
#define UART_C3_ORIE_MASK                        0x8u
#define UART_C3_ORIE_SHIFT                       3
#define UART_C3_ORIE_WIDTH                       1
#define UART_C3_ORIE(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C3_ORIE_SHIFT))&UART_C3_ORIE_MASK)
#define UART_C3_TXINV_MASK                       0x10u
#define UART_C3_TXINV_SHIFT                      4
#define UART_C3_TXINV_WIDTH                      1
#define UART_C3_TXINV(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C3_TXINV_SHIFT))&UART_C3_TXINV_MASK)
#define UART_C3_TXDIR_MASK                       0x20u
#define UART_C3_TXDIR_SHIFT                      5
#define UART_C3_TXDIR_WIDTH                      1
#define UART_C3_TXDIR(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C3_TXDIR_SHIFT))&UART_C3_TXDIR_MASK)
#define UART_C3_T8_MASK                          0x40u
#define UART_C3_T8_SHIFT                         6
#define UART_C3_T8_WIDTH                         1
#define UART_C3_T8(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C3_T8_SHIFT))&UART_C3_T8_MASK)
#define UART_C3_R8_MASK                          0x80u
#define UART_C3_R8_SHIFT                         7
#define UART_C3_R8_WIDTH                         1
#define UART_C3_R8(x)                            (((uint8_t)(((uint8_t)(x))<<UART_C3_R8_SHIFT))&UART_C3_R8_MASK)
/* D Bit Fields */
#define UART_D_RT_MASK                           0xFFu
#define UART_D_RT_SHIFT                          0
#define UART_D_RT_WIDTH                          8
#define UART_D_RT(x)                             (((uint8_t)(((uint8_t)(x))<<UART_D_RT_SHIFT))&UART_D_RT_MASK)
/* MA1 Bit Fields */
#define UART_MA1_MA_MASK                         0xFFu
#define UART_MA1_MA_SHIFT                        0
#define UART_MA1_MA_WIDTH                        8
#define UART_MA1_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA1_MA_SHIFT))&UART_MA1_MA_MASK)
/* MA2 Bit Fields */
#define UART_MA2_MA_MASK                         0xFFu
#define UART_MA2_MA_SHIFT                        0
#define UART_MA2_MA_WIDTH                        8
#define UART_MA2_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA2_MA_SHIFT))&UART_MA2_MA_MASK)
/* C4 Bit Fields */
#define UART_C4_BRFA_MASK                        0x1Fu
#define UART_C4_BRFA_SHIFT                       0
#define UART_C4_BRFA_WIDTH                       5
#define UART_C4_BRFA(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C4_BRFA_SHIFT))&UART_C4_BRFA_MASK)
#define UART_C4_M10_MASK                         0x20u
#define UART_C4_M10_SHIFT                        5
#define UART_C4_M10_WIDTH                        1
#define UART_C4_M10(x)                           (((uint8_t)(((uint8_t)(x))<<UART_C4_M10_SHIFT))&UART_C4_M10_MASK)
#define UART_C4_MAEN2_MASK                       0x40u
#define UART_C4_MAEN2_SHIFT                      6
#define UART_C4_MAEN2_WIDTH                      1
#define UART_C4_MAEN2(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C4_MAEN2_SHIFT))&UART_C4_MAEN2_MASK)
#define UART_C4_MAEN1_MASK                       0x80u
#define UART_C4_MAEN1_SHIFT                      7
#define UART_C4_MAEN1_WIDTH                      1
#define UART_C4_MAEN1(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C4_MAEN1_SHIFT))&UART_C4_MAEN1_MASK)
/* C5 Bit Fields */
#define UART_C5_LBKDDMAS_MASK                    0x8u
#define UART_C5_LBKDDMAS_SHIFT                   3
#define UART_C5_LBKDDMAS_WIDTH                   1
#define UART_C5_LBKDDMAS(x)                      (((uint8_t)(((uint8_t)(x))<<UART_C5_LBKDDMAS_SHIFT))&UART_C5_LBKDDMAS_MASK)
#define UART_C5_RDMAS_MASK                       0x20u
#define UART_C5_RDMAS_SHIFT                      5
#define UART_C5_RDMAS_WIDTH                      1
#define UART_C5_RDMAS(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C5_RDMAS_SHIFT))&UART_C5_RDMAS_MASK)
#define UART_C5_TDMAS_MASK                       0x80u
#define UART_C5_TDMAS_SHIFT                      7
#define UART_C5_TDMAS_WIDTH                      1
#define UART_C5_TDMAS(x)                         (((uint8_t)(((uint8_t)(x))<<UART_C5_TDMAS_SHIFT))&UART_C5_TDMAS_MASK)
/* ED Bit Fields */
#define UART_ED_PARITYE_MASK                     0x40u
#define UART_ED_PARITYE_SHIFT                    6
#define UART_ED_PARITYE_WIDTH                    1
#define UART_ED_PARITYE(x)                       (((uint8_t)(((uint8_t)(x))<<UART_ED_PARITYE_SHIFT))&UART_ED_PARITYE_MASK)
#define UART_ED_NOISY_MASK                       0x80u
#define UART_ED_NOISY_SHIFT                      7
#define UART_ED_NOISY_WIDTH                      1
#define UART_ED_NOISY(x)                         (((uint8_t)(((uint8_t)(x))<<UART_ED_NOISY_SHIFT))&UART_ED_NOISY_MASK)
/* MODEM Bit Fields */
#define UART_MODEM_TXCTSE_MASK                   0x1u
#define UART_MODEM_TXCTSE_SHIFT                  0
#define UART_MODEM_TXCTSE_WIDTH                  1
#define UART_MODEM_TXCTSE(x)                     (((uint8_t)(((uint8_t)(x))<<UART_MODEM_TXCTSE_SHIFT))&UART_MODEM_TXCTSE_MASK)
#define UART_MODEM_TXRTSE_MASK                   0x2u
#define UART_MODEM_TXRTSE_SHIFT                  1
#define UART_MODEM_TXRTSE_WIDTH                  1
#define UART_MODEM_TXRTSE(x)                     (((uint8_t)(((uint8_t)(x))<<UART_MODEM_TXRTSE_SHIFT))&UART_MODEM_TXRTSE_MASK)
#define UART_MODEM_TXRTSPOL_MASK                 0x4u
#define UART_MODEM_TXRTSPOL_SHIFT                2
#define UART_MODEM_TXRTSPOL_WIDTH                1
#define UART_MODEM_TXRTSPOL(x)                   (((uint8_t)(((uint8_t)(x))<<UART_MODEM_TXRTSPOL_SHIFT))&UART_MODEM_TXRTSPOL_MASK)
#define UART_MODEM_RXRTSE_MASK                   0x8u
#define UART_MODEM_RXRTSE_SHIFT                  3
#define UART_MODEM_RXRTSE_WIDTH                  1
#define UART_MODEM_RXRTSE(x)                     (((uint8_t)(((uint8_t)(x))<<UART_MODEM_RXRTSE_SHIFT))&UART_MODEM_RXRTSE_MASK)
/* PFIFO Bit Fields */
#define UART_PFIFO_RXFIFOSIZE_MASK               0x7u
#define UART_PFIFO_RXFIFOSIZE_SHIFT              0
#define UART_PFIFO_RXFIFOSIZE_WIDTH              3
#define UART_PFIFO_RXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_RXFIFOSIZE_SHIFT))&UART_PFIFO_RXFIFOSIZE_MASK)
#define UART_PFIFO_RXFE_MASK                     0x8u
#define UART_PFIFO_RXFE_SHIFT                    3
#define UART_PFIFO_RXFE_WIDTH                    1
#define UART_PFIFO_RXFE(x)                       (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_RXFE_SHIFT))&UART_PFIFO_RXFE_MASK)
#define UART_PFIFO_TXFIFOSIZE_MASK               0x70u
#define UART_PFIFO_TXFIFOSIZE_SHIFT              4
#define UART_PFIFO_TXFIFOSIZE_WIDTH              3
#define UART_PFIFO_TXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_TXFIFOSIZE_SHIFT))&UART_PFIFO_TXFIFOSIZE_MASK)
#define UART_PFIFO_TXFE_MASK                     0x80u
#define UART_PFIFO_TXFE_SHIFT                    7
#define UART_PFIFO_TXFE_WIDTH                    1
#define UART_PFIFO_TXFE(x)                       (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_TXFE_SHIFT))&UART_PFIFO_TXFE_MASK)
/* CFIFO Bit Fields */
#define UART_CFIFO_RXUFE_MASK                    0x1u
#define UART_CFIFO_RXUFE_SHIFT                   0
#define UART_CFIFO_RXUFE_WIDTH                   1
#define UART_CFIFO_RXUFE(x)                      (((uint8_t)(((uint8_t)(x))<<UART_CFIFO_RXUFE_SHIFT))&UART_CFIFO_RXUFE_MASK)
#define UART_CFIFO_TXOFE_MASK                    0x2u
#define UART_CFIFO_TXOFE_SHIFT                   1
#define UART_CFIFO_TXOFE_WIDTH                   1
#define UART_CFIFO_TXOFE(x)                      (((uint8_t)(((uint8_t)(x))<<UART_CFIFO_TXOFE_SHIFT))&UART_CFIFO_TXOFE_MASK)
#define UART_CFIFO_RXOFE_MASK                    0x4u
#define UART_CFIFO_RXOFE_SHIFT                   2
#define UART_CFIFO_RXOFE_WIDTH                   1
#define UART_CFIFO_RXOFE(x)                      (((uint8_t)(((uint8_t)(x))<<UART_CFIFO_RXOFE_SHIFT))&UART_CFIFO_RXOFE_MASK)
#define UART_CFIFO_RXFLUSH_MASK                  0x40u
#define UART_CFIFO_RXFLUSH_SHIFT                 6
#define UART_CFIFO_RXFLUSH_WIDTH                 1
#define UART_CFIFO_RXFLUSH(x)                    (((uint8_t)(((uint8_t)(x))<<UART_CFIFO_RXFLUSH_SHIFT))&UART_CFIFO_RXFLUSH_MASK)
#define UART_CFIFO_TXFLUSH_MASK                  0x80u
#define UART_CFIFO_TXFLUSH_SHIFT                 7
#define UART_CFIFO_TXFLUSH_WIDTH                 1
#define UART_CFIFO_TXFLUSH(x)                    (((uint8_t)(((uint8_t)(x))<<UART_CFIFO_TXFLUSH_SHIFT))&UART_CFIFO_TXFLUSH_MASK)
/* SFIFO Bit Fields */
#define UART_SFIFO_RXUF_MASK                     0x1u
#define UART_SFIFO_RXUF_SHIFT                    0
#define UART_SFIFO_RXUF_WIDTH                    1
#define UART_SFIFO_RXUF(x)                       (((uint8_t)(((uint8_t)(x))<<UART_SFIFO_RXUF_SHIFT))&UART_SFIFO_RXUF_MASK)
#define UART_SFIFO_TXOF_MASK                     0x2u
#define UART_SFIFO_TXOF_SHIFT                    1
#define UART_SFIFO_TXOF_WIDTH                    1
#define UART_SFIFO_TXOF(x)                       (((uint8_t)(((uint8_t)(x))<<UART_SFIFO_TXOF_SHIFT))&UART_SFIFO_TXOF_MASK)
#define UART_SFIFO_RXOF_MASK                     0x4u
#define UART_SFIFO_RXOF_SHIFT                    2
#define UART_SFIFO_RXOF_WIDTH                    1
#define UART_SFIFO_RXOF(x)                       (((uint8_t)(((uint8_t)(x))<<UART_SFIFO_RXOF_SHIFT))&UART_SFIFO_RXOF_MASK)
#define UART_SFIFO_RXEMPT_MASK                   0x40u
#define UART_SFIFO_RXEMPT_SHIFT                  6
#define UART_SFIFO_RXEMPT_WIDTH                  1
#define UART_SFIFO_RXEMPT(x)                     (((uint8_t)(((uint8_t)(x))<<UART_SFIFO_RXEMPT_SHIFT))&UART_SFIFO_RXEMPT_MASK)
#define UART_SFIFO_TXEMPT_MASK                   0x80u
#define UART_SFIFO_TXEMPT_SHIFT                  7
#define UART_SFIFO_TXEMPT_WIDTH                  1
#define UART_SFIFO_TXEMPT(x)                     (((uint8_t)(((uint8_t)(x))<<UART_SFIFO_TXEMPT_SHIFT))&UART_SFIFO_TXEMPT_MASK)
/* TWFIFO Bit Fields */
#define UART_TWFIFO_TXWATER_MASK                 0xFFu
#define UART_TWFIFO_TXWATER_SHIFT                0
#define UART_TWFIFO_TXWATER_WIDTH                8
#define UART_TWFIFO_TXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TWFIFO_TXWATER_SHIFT))&UART_TWFIFO_TXWATER_MASK)
/* TCFIFO Bit Fields */
#define UART_TCFIFO_TXCOUNT_MASK                 0xFFu
#define UART_TCFIFO_TXCOUNT_SHIFT                0
#define UART_TCFIFO_TXCOUNT_WIDTH                8
#define UART_TCFIFO_TXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TCFIFO_TXCOUNT_SHIFT))&UART_TCFIFO_TXCOUNT_MASK)
/* RWFIFO Bit Fields */
#define UART_RWFIFO_RXWATER_MASK                 0xFFu
#define UART_RWFIFO_RXWATER_SHIFT                0
#define UART_RWFIFO_RXWATER_WIDTH                8
#define UART_RWFIFO_RXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RWFIFO_RXWATER_SHIFT))&UART_RWFIFO_RXWATER_MASK)
/* RCFIFO Bit Fields */
#define UART_RCFIFO_RXCOUNT_MASK                 0xFFu
#define UART_RCFIFO_RXCOUNT_SHIFT                0
#define UART_RCFIFO_RXCOUNT_WIDTH                8
#define UART_RCFIFO_RXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RCFIFO_RXCOUNT_SHIFT))&UART_RCFIFO_RXCOUNT_MASK)

/*!
 * @}
 */ /* end of group UART_Register_Masks */


/* UART - Peripheral instance base addresses */
/** Peripheral UART0 base address */
#define UART0_BASE                               (0x4006A000u)
/** Peripheral UART0 base pointer */
#define UART0                                    ((UART_Type *)UART0_BASE)
#define UART0_BASE_PTR                           (UART0)
/** Peripheral UART1 base address */
#define UART1_BASE                               (0x4006B000u)
/** Peripheral UART1 base pointer */
#define UART1                                    ((UART_Type *)UART1_BASE)
#define UART1_BASE_PTR                           (UART1)
/** Array initializer of UART peripheral base addresses */
#define UART_BASE_ADDRS                          { UART0_BASE, UART1_BASE }
/** Array initializer of UART peripheral base pointers */
#define UART_BASE_PTRS                           { UART0, UART1 }

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */


/* UART - Register instance definitions */
/* UART0 */
#define UART0_BDH                                UART_BDH_REG(UART0)
#define UART0_BDL                                UART_BDL_REG(UART0)
#define UART0_C1                                 UART_C1_REG(UART0)
#define UART0_C2                                 UART_C2_REG(UART0)
#define UART0_S1                                 UART_S1_REG(UART0)
#define UART0_S2                                 UART_S2_REG(UART0)
#define UART0_C3                                 UART_C3_REG(UART0)
#define UART0_D                                  UART_D_REG(UART0)
#define UART0_MA1                                UART_MA1_REG(UART0)
#define UART0_MA2                                UART_MA2_REG(UART0)
#define UART0_C4                                 UART_C4_REG(UART0)
#define UART0_C5                                 UART_C5_REG(UART0)
#define UART0_ED                                 UART_ED_REG(UART0)
#define UART0_MODEM                              UART_MODEM_REG(UART0)
#define UART0_PFIFO                              UART_PFIFO_REG(UART0)
#define UART0_CFIFO                              UART_CFIFO_REG(UART0)
#define UART0_SFIFO                              UART_SFIFO_REG(UART0)
#define UART0_TWFIFO                             UART_TWFIFO_REG(UART0)
#define UART0_TCFIFO                             UART_TCFIFO_REG(UART0)
#define UART0_RWFIFO                             UART_RWFIFO_REG(UART0)
#define UART0_RCFIFO                             UART_RCFIFO_REG(UART0)
/* UART1 */
#define UART1_BDH                                UART_BDH_REG(UART1)
#define UART1_BDL                                UART_BDL_REG(UART1)
#define UART1_C1                                 UART_C1_REG(UART1)
#define UART1_C2                                 UART_C2_REG(UART1)
#define UART1_S1                                 UART_S1_REG(UART1)
#define UART1_S2                                 UART_S2_REG(UART1)
#define UART1_C3                                 UART_C3_REG(UART1)
#define UART1_D                                  UART_D_REG(UART1)
#define UART1_MA1                                UART_MA1_REG(UART1)
#define UART1_MA2                                UART_MA2_REG(UART1)
#define UART1_C4                                 UART_C4_REG(UART1)
#define UART1_C5                                 UART_C5_REG(UART1)
#define UART1_ED                                 UART_ED_REG(UART1)
#define UART1_MODEM                              UART_MODEM_REG(UART1)
#define UART1_PFIFO                              UART_PFIFO_REG(UART1)
#define UART1_CFIFO                              UART_CFIFO_REG(UART1)
#define UART1_SFIFO                              UART_SFIFO_REG(UART1)
#define UART1_TWFIFO                             UART_TWFIFO_REG(UART1)
#define UART1_TCFIFO                             UART_TCFIFO_REG(UART1)
#define UART1_RWFIFO                             UART_RWFIFO_REG(UART1)
#define UART1_RCFIFO                             UART_RCFIFO_REG(UART1)

/*!
 * @}
 */ /* end of group UART_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group UART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- WDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral_Access_Layer WDOG Peripheral Access Layer
 * @{
 */

/** WDOG - Register Layout Typedef */
typedef struct {
  __IO uint16_t STCTRLH;                           /**< Watchdog Status and Control Register High, offset: 0x0 */
  __IO uint16_t STCTRLL;                           /**< Watchdog Status and Control Register Low, offset: 0x2 */
  __IO uint16_t TOVALH;                            /**< Watchdog Time-out Value Register High, offset: 0x4 */
  __IO uint16_t TOVALL;                            /**< Watchdog Time-out Value Register Low, offset: 0x6 */
  __IO uint16_t WINH;                              /**< Watchdog Window Register High, offset: 0x8 */
  __IO uint16_t WINL;                              /**< Watchdog Window Register Low, offset: 0xA */
  __IO uint16_t REFRESH;                           /**< Watchdog Refresh register, offset: 0xC */
  __IO uint16_t UNLOCK;                            /**< Watchdog Unlock register, offset: 0xE */
  __IO uint16_t TMROUTH;                           /**< Watchdog Timer Output Register High, offset: 0x10 */
  __IO uint16_t TMROUTL;                           /**< Watchdog Timer Output Register Low, offset: 0x12 */
  __IO uint16_t RSTCNT;                            /**< Watchdog Reset Count register, offset: 0x14 */
  __IO uint16_t PRESC;                             /**< Watchdog Prescaler register, offset: 0x16 */
} WDOG_Type, *WDOG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register accessors */
#define WDOG_STCTRLH_REG(base)                   ((base)->STCTRLH)
#define WDOG_STCTRLL_REG(base)                   ((base)->STCTRLL)
#define WDOG_TOVALH_REG(base)                    ((base)->TOVALH)
#define WDOG_TOVALL_REG(base)                    ((base)->TOVALL)
#define WDOG_WINH_REG(base)                      ((base)->WINH)
#define WDOG_WINL_REG(base)                      ((base)->WINL)
#define WDOG_REFRESH_REG(base)                   ((base)->REFRESH)
#define WDOG_UNLOCK_REG(base)                    ((base)->UNLOCK)
#define WDOG_TMROUTH_REG(base)                   ((base)->TMROUTH)
#define WDOG_TMROUTL_REG(base)                   ((base)->TMROUTL)
#define WDOG_RSTCNT_REG(base)                    ((base)->RSTCNT)
#define WDOG_PRESC_REG(base)                     ((base)->PRESC)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/* STCTRLH Bit Fields */
#define WDOG_STCTRLH_WDOGEN_MASK                 0x1u
#define WDOG_STCTRLH_WDOGEN_SHIFT                0
#define WDOG_STCTRLH_WDOGEN_WIDTH                1
#define WDOG_STCTRLH_WDOGEN(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_WDOGEN_SHIFT))&WDOG_STCTRLH_WDOGEN_MASK)
#define WDOG_STCTRLH_CLKSRC_MASK                 0x2u
#define WDOG_STCTRLH_CLKSRC_SHIFT                1
#define WDOG_STCTRLH_CLKSRC_WIDTH                1
#define WDOG_STCTRLH_CLKSRC(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_CLKSRC_SHIFT))&WDOG_STCTRLH_CLKSRC_MASK)
#define WDOG_STCTRLH_IRQRSTEN_MASK               0x4u
#define WDOG_STCTRLH_IRQRSTEN_SHIFT              2
#define WDOG_STCTRLH_IRQRSTEN_WIDTH              1
#define WDOG_STCTRLH_IRQRSTEN(x)                 (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_IRQRSTEN_SHIFT))&WDOG_STCTRLH_IRQRSTEN_MASK)
#define WDOG_STCTRLH_WINEN_MASK                  0x8u
#define WDOG_STCTRLH_WINEN_SHIFT                 3
#define WDOG_STCTRLH_WINEN_WIDTH                 1
#define WDOG_STCTRLH_WINEN(x)                    (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_WINEN_SHIFT))&WDOG_STCTRLH_WINEN_MASK)
#define WDOG_STCTRLH_ALLOWUPDATE_MASK            0x10u
#define WDOG_STCTRLH_ALLOWUPDATE_SHIFT           4
#define WDOG_STCTRLH_ALLOWUPDATE_WIDTH           1
#define WDOG_STCTRLH_ALLOWUPDATE(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_ALLOWUPDATE_SHIFT))&WDOG_STCTRLH_ALLOWUPDATE_MASK)
#define WDOG_STCTRLH_DBGEN_MASK                  0x20u
#define WDOG_STCTRLH_DBGEN_SHIFT                 5
#define WDOG_STCTRLH_DBGEN_WIDTH                 1
#define WDOG_STCTRLH_DBGEN(x)                    (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_DBGEN_SHIFT))&WDOG_STCTRLH_DBGEN_MASK)
#define WDOG_STCTRLH_STOPEN_MASK                 0x40u
#define WDOG_STCTRLH_STOPEN_SHIFT                6
#define WDOG_STCTRLH_STOPEN_WIDTH                1
#define WDOG_STCTRLH_STOPEN(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_STOPEN_SHIFT))&WDOG_STCTRLH_STOPEN_MASK)
#define WDOG_STCTRLH_WAITEN_MASK                 0x80u
#define WDOG_STCTRLH_WAITEN_SHIFT                7
#define WDOG_STCTRLH_WAITEN_WIDTH                1
#define WDOG_STCTRLH_WAITEN(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_WAITEN_SHIFT))&WDOG_STCTRLH_WAITEN_MASK)
#define WDOG_STCTRLH_TESTWDOG_MASK               0x400u
#define WDOG_STCTRLH_TESTWDOG_SHIFT              10
#define WDOG_STCTRLH_TESTWDOG_WIDTH              1
#define WDOG_STCTRLH_TESTWDOG(x)                 (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_TESTWDOG_SHIFT))&WDOG_STCTRLH_TESTWDOG_MASK)
#define WDOG_STCTRLH_TESTSEL_MASK                0x800u
#define WDOG_STCTRLH_TESTSEL_SHIFT               11
#define WDOG_STCTRLH_TESTSEL_WIDTH               1
#define WDOG_STCTRLH_TESTSEL(x)                  (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_TESTSEL_SHIFT))&WDOG_STCTRLH_TESTSEL_MASK)
#define WDOG_STCTRLH_BYTESEL_MASK                0x3000u
#define WDOG_STCTRLH_BYTESEL_SHIFT               12
#define WDOG_STCTRLH_BYTESEL_WIDTH               2
#define WDOG_STCTRLH_BYTESEL(x)                  (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_BYTESEL_SHIFT))&WDOG_STCTRLH_BYTESEL_MASK)
#define WDOG_STCTRLH_DISTESTWDOG_MASK            0x4000u
#define WDOG_STCTRLH_DISTESTWDOG_SHIFT           14
#define WDOG_STCTRLH_DISTESTWDOG_WIDTH           1
#define WDOG_STCTRLH_DISTESTWDOG(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLH_DISTESTWDOG_SHIFT))&WDOG_STCTRLH_DISTESTWDOG_MASK)
/* STCTRLL Bit Fields */
#define WDOG_STCTRLL_INTFLG_MASK                 0x8000u
#define WDOG_STCTRLL_INTFLG_SHIFT                15
#define WDOG_STCTRLL_INTFLG_WIDTH                1
#define WDOG_STCTRLL_INTFLG(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_STCTRLL_INTFLG_SHIFT))&WDOG_STCTRLL_INTFLG_MASK)
/* TOVALH Bit Fields */
#define WDOG_TOVALH_TOVALHIGH_MASK               0xFFFFu
#define WDOG_TOVALH_TOVALHIGH_SHIFT              0
#define WDOG_TOVALH_TOVALHIGH_WIDTH              16
#define WDOG_TOVALH_TOVALHIGH(x)                 (((uint16_t)(((uint16_t)(x))<<WDOG_TOVALH_TOVALHIGH_SHIFT))&WDOG_TOVALH_TOVALHIGH_MASK)
/* TOVALL Bit Fields */
#define WDOG_TOVALL_TOVALLOW_MASK                0xFFFFu
#define WDOG_TOVALL_TOVALLOW_SHIFT               0
#define WDOG_TOVALL_TOVALLOW_WIDTH               16
#define WDOG_TOVALL_TOVALLOW(x)                  (((uint16_t)(((uint16_t)(x))<<WDOG_TOVALL_TOVALLOW_SHIFT))&WDOG_TOVALL_TOVALLOW_MASK)
/* WINH Bit Fields */
#define WDOG_WINH_WINHIGH_MASK                   0xFFFFu
#define WDOG_WINH_WINHIGH_SHIFT                  0
#define WDOG_WINH_WINHIGH_WIDTH                  16
#define WDOG_WINH_WINHIGH(x)                     (((uint16_t)(((uint16_t)(x))<<WDOG_WINH_WINHIGH_SHIFT))&WDOG_WINH_WINHIGH_MASK)
/* WINL Bit Fields */
#define WDOG_WINL_WINLOW_MASK                    0xFFFFu
#define WDOG_WINL_WINLOW_SHIFT                   0
#define WDOG_WINL_WINLOW_WIDTH                   16
#define WDOG_WINL_WINLOW(x)                      (((uint16_t)(((uint16_t)(x))<<WDOG_WINL_WINLOW_SHIFT))&WDOG_WINL_WINLOW_MASK)
/* REFRESH Bit Fields */
#define WDOG_REFRESH_WDOGREFRESH_MASK            0xFFFFu
#define WDOG_REFRESH_WDOGREFRESH_SHIFT           0
#define WDOG_REFRESH_WDOGREFRESH_WIDTH           16
#define WDOG_REFRESH_WDOGREFRESH(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_REFRESH_WDOGREFRESH_SHIFT))&WDOG_REFRESH_WDOGREFRESH_MASK)
/* UNLOCK Bit Fields */
#define WDOG_UNLOCK_WDOGUNLOCK_MASK              0xFFFFu
#define WDOG_UNLOCK_WDOGUNLOCK_SHIFT             0
#define WDOG_UNLOCK_WDOGUNLOCK_WIDTH             16
#define WDOG_UNLOCK_WDOGUNLOCK(x)                (((uint16_t)(((uint16_t)(x))<<WDOG_UNLOCK_WDOGUNLOCK_SHIFT))&WDOG_UNLOCK_WDOGUNLOCK_MASK)
/* TMROUTH Bit Fields */
#define WDOG_TMROUTH_TIMEROUTHIGH_MASK           0xFFFFu
#define WDOG_TMROUTH_TIMEROUTHIGH_SHIFT          0
#define WDOG_TMROUTH_TIMEROUTHIGH_WIDTH          16
#define WDOG_TMROUTH_TIMEROUTHIGH(x)             (((uint16_t)(((uint16_t)(x))<<WDOG_TMROUTH_TIMEROUTHIGH_SHIFT))&WDOG_TMROUTH_TIMEROUTHIGH_MASK)
/* TMROUTL Bit Fields */
#define WDOG_TMROUTL_TIMEROUTLOW_MASK            0xFFFFu
#define WDOG_TMROUTL_TIMEROUTLOW_SHIFT           0
#define WDOG_TMROUTL_TIMEROUTLOW_WIDTH           16
#define WDOG_TMROUTL_TIMEROUTLOW(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_TMROUTL_TIMEROUTLOW_SHIFT))&WDOG_TMROUTL_TIMEROUTLOW_MASK)
/* RSTCNT Bit Fields */
#define WDOG_RSTCNT_RSTCNT_MASK                  0xFFFFu
#define WDOG_RSTCNT_RSTCNT_SHIFT                 0
#define WDOG_RSTCNT_RSTCNT_WIDTH                 16
#define WDOG_RSTCNT_RSTCNT(x)                    (((uint16_t)(((uint16_t)(x))<<WDOG_RSTCNT_RSTCNT_SHIFT))&WDOG_RSTCNT_RSTCNT_MASK)
/* PRESC Bit Fields */
#define WDOG_PRESC_PRESCVAL_MASK                 0x700u
#define WDOG_PRESC_PRESCVAL_SHIFT                8
#define WDOG_PRESC_PRESCVAL_WIDTH                3
#define WDOG_PRESC_PRESCVAL(x)                   (((uint16_t)(((uint16_t)(x))<<WDOG_PRESC_PRESCVAL_SHIFT))&WDOG_PRESC_PRESCVAL_MASK)

/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG base address */
#define WDOG_BASE                                (0x40052000u)
/** Peripheral WDOG base pointer */
#define WDOG                                     ((WDOG_Type *)WDOG_BASE)
#define WDOG_BASE_PTR                            (WDOG)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { WDOG_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG }

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register instance definitions */
/* WDOG */
#define WDOG_STCTRLH                             WDOG_STCTRLH_REG(WDOG)
#define WDOG_STCTRLL                             WDOG_STCTRLL_REG(WDOG)
#define WDOG_TOVALH                              WDOG_TOVALH_REG(WDOG)
#define WDOG_TOVALL                              WDOG_TOVALL_REG(WDOG)
#define WDOG_WINH                                WDOG_WINH_REG(WDOG)
#define WDOG_WINL                                WDOG_WINL_REG(WDOG)
#define WDOG_REFRESH                             WDOG_REFRESH_REG(WDOG)
#define WDOG_UNLOCK                              WDOG_UNLOCK_REG(WDOG)
#define WDOG_TMROUTH                             WDOG_TMROUTH_REG(WDOG)
#define WDOG_TMROUTL                             WDOG_TMROUTL_REG(WDOG)
#define WDOG_RSTCNT                              WDOG_RSTCNT_REG(WDOG)
#define WDOG_PRESC                               WDOG_PRESC_REG(WDOG)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group WDOG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- XBARA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Peripheral_Access_Layer XBARA Peripheral Access Layer
 * @{
 */

/** XBARA - Register Layout Typedef */
typedef struct {
  __IO uint16_t SEL0;                              /**< Crossbar A Select Register 0, offset: 0x0 */
  __IO uint16_t SEL1;                              /**< Crossbar A Select Register 1, offset: 0x2 */
  __IO uint16_t SEL2;                              /**< Crossbar A Select Register 2, offset: 0x4 */
  __IO uint16_t SEL3;                              /**< Crossbar A Select Register 3, offset: 0x6 */
  __IO uint16_t SEL4;                              /**< Crossbar A Select Register 4, offset: 0x8 */
  __IO uint16_t SEL5;                              /**< Crossbar A Select Register 5, offset: 0xA */
  __IO uint16_t SEL6;                              /**< Crossbar A Select Register 6, offset: 0xC */
  __IO uint16_t SEL7;                              /**< Crossbar A Select Register 7, offset: 0xE */
  __IO uint16_t SEL8;                              /**< Crossbar A Select Register 8, offset: 0x10 */
  __IO uint16_t SEL9;                              /**< Crossbar A Select Register 9, offset: 0x12 */
  __IO uint16_t SEL10;                             /**< Crossbar A Select Register 10, offset: 0x14 */
  __IO uint16_t SEL11;                             /**< Crossbar A Select Register 11, offset: 0x16 */
  __IO uint16_t SEL12;                             /**< Crossbar A Select Register 12, offset: 0x18 */
  __IO uint16_t SEL13;                             /**< Crossbar A Select Register 13, offset: 0x1A */
  __IO uint16_t SEL14;                             /**< Crossbar A Select Register 14, offset: 0x1C */
  __IO uint16_t SEL15;                             /**< Crossbar A Select Register 15, offset: 0x1E */
  __IO uint16_t SEL16;                             /**< Crossbar A Select Register 16, offset: 0x20 */
  __IO uint16_t SEL17;                             /**< Crossbar A Select Register 17, offset: 0x22 */
  __IO uint16_t SEL18;                             /**< Crossbar A Select Register 18, offset: 0x24 */
  __IO uint16_t SEL19;                             /**< Crossbar A Select Register 19, offset: 0x26 */
  __IO uint16_t SEL20;                             /**< Crossbar A Select Register 20, offset: 0x28 */
  __IO uint16_t SEL21;                             /**< Crossbar A Select Register 21, offset: 0x2A */
  __IO uint16_t SEL22;                             /**< Crossbar A Select Register 22, offset: 0x2C */
  __IO uint16_t SEL23;                             /**< Crossbar A Select Register 23, offset: 0x2E */
  __IO uint16_t SEL24;                             /**< Crossbar A Select Register 24, offset: 0x30 */
  __IO uint16_t SEL25;                             /**< Crossbar A Select Register 25, offset: 0x32 */
  __IO uint16_t SEL26;                             /**< Crossbar A Select Register 26, offset: 0x34 */
  __IO uint16_t SEL27;                             /**< Crossbar A Select Register 27, offset: 0x36 */
  __IO uint16_t SEL28;                             /**< Crossbar A Select Register 28, offset: 0x38 */
  __IO uint16_t SEL29;                             /**< Crossbar A Select Register 29, offset: 0x3A */
  __IO uint16_t CTRL0;                             /**< Crossbar A Control Register 0, offset: 0x3C */
  __IO uint16_t CTRL1;                             /**< Crossbar A Control Register 1, offset: 0x3E */
} XBARA_Type, *XBARA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- XBARA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Accessor_Macros XBARA - Register accessor macros
 * @{
 */


/* XBARA - Register accessors */
#define XBARA_SEL0_REG(base)                     ((base)->SEL0)
#define XBARA_SEL1_REG(base)                     ((base)->SEL1)
#define XBARA_SEL2_REG(base)                     ((base)->SEL2)
#define XBARA_SEL3_REG(base)                     ((base)->SEL3)
#define XBARA_SEL4_REG(base)                     ((base)->SEL4)
#define XBARA_SEL5_REG(base)                     ((base)->SEL5)
#define XBARA_SEL6_REG(base)                     ((base)->SEL6)
#define XBARA_SEL7_REG(base)                     ((base)->SEL7)
#define XBARA_SEL8_REG(base)                     ((base)->SEL8)
#define XBARA_SEL9_REG(base)                     ((base)->SEL9)
#define XBARA_SEL10_REG(base)                    ((base)->SEL10)
#define XBARA_SEL11_REG(base)                    ((base)->SEL11)
#define XBARA_SEL12_REG(base)                    ((base)->SEL12)
#define XBARA_SEL13_REG(base)                    ((base)->SEL13)
#define XBARA_SEL14_REG(base)                    ((base)->SEL14)
#define XBARA_SEL15_REG(base)                    ((base)->SEL15)
#define XBARA_SEL16_REG(base)                    ((base)->SEL16)
#define XBARA_SEL17_REG(base)                    ((base)->SEL17)
#define XBARA_SEL18_REG(base)                    ((base)->SEL18)
#define XBARA_SEL19_REG(base)                    ((base)->SEL19)
#define XBARA_SEL20_REG(base)                    ((base)->SEL20)
#define XBARA_SEL21_REG(base)                    ((base)->SEL21)
#define XBARA_SEL22_REG(base)                    ((base)->SEL22)
#define XBARA_SEL23_REG(base)                    ((base)->SEL23)
#define XBARA_SEL24_REG(base)                    ((base)->SEL24)
#define XBARA_SEL25_REG(base)                    ((base)->SEL25)
#define XBARA_SEL26_REG(base)                    ((base)->SEL26)
#define XBARA_SEL27_REG(base)                    ((base)->SEL27)
#define XBARA_SEL28_REG(base)                    ((base)->SEL28)
#define XBARA_SEL29_REG(base)                    ((base)->SEL29)
#define XBARA_CTRL0_REG(base)                    ((base)->CTRL0)
#define XBARA_CTRL1_REG(base)                    ((base)->CTRL1)

/*!
 * @}
 */ /* end of group XBARA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- XBARA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Masks XBARA Register Masks
 * @{
 */

/* SEL0 Bit Fields */
#define XBARA_SEL0_SEL0_MASK                     0x3Fu
#define XBARA_SEL0_SEL0_SHIFT                    0
#define XBARA_SEL0_SEL0_WIDTH                    6
#define XBARA_SEL0_SEL0(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL0_SEL0_SHIFT))&XBARA_SEL0_SEL0_MASK)
#define XBARA_SEL0_SEL1_MASK                     0x3F00u
#define XBARA_SEL0_SEL1_SHIFT                    8
#define XBARA_SEL0_SEL1_WIDTH                    6
#define XBARA_SEL0_SEL1(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL0_SEL1_SHIFT))&XBARA_SEL0_SEL1_MASK)
/* SEL1 Bit Fields */
#define XBARA_SEL1_SEL2_MASK                     0x3Fu
#define XBARA_SEL1_SEL2_SHIFT                    0
#define XBARA_SEL1_SEL2_WIDTH                    6
#define XBARA_SEL1_SEL2(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL1_SEL2_SHIFT))&XBARA_SEL1_SEL2_MASK)
#define XBARA_SEL1_SEL3_MASK                     0x3F00u
#define XBARA_SEL1_SEL3_SHIFT                    8
#define XBARA_SEL1_SEL3_WIDTH                    6
#define XBARA_SEL1_SEL3(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL1_SEL3_SHIFT))&XBARA_SEL1_SEL3_MASK)
/* SEL2 Bit Fields */
#define XBARA_SEL2_SEL4_MASK                     0x3Fu
#define XBARA_SEL2_SEL4_SHIFT                    0
#define XBARA_SEL2_SEL4_WIDTH                    6
#define XBARA_SEL2_SEL4(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL2_SEL4_SHIFT))&XBARA_SEL2_SEL4_MASK)
#define XBARA_SEL2_SEL5_MASK                     0x3F00u
#define XBARA_SEL2_SEL5_SHIFT                    8
#define XBARA_SEL2_SEL5_WIDTH                    6
#define XBARA_SEL2_SEL5(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL2_SEL5_SHIFT))&XBARA_SEL2_SEL5_MASK)
/* SEL3 Bit Fields */
#define XBARA_SEL3_SEL6_MASK                     0x3Fu
#define XBARA_SEL3_SEL6_SHIFT                    0
#define XBARA_SEL3_SEL6_WIDTH                    6
#define XBARA_SEL3_SEL6(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL3_SEL6_SHIFT))&XBARA_SEL3_SEL6_MASK)
#define XBARA_SEL3_SEL7_MASK                     0x3F00u
#define XBARA_SEL3_SEL7_SHIFT                    8
#define XBARA_SEL3_SEL7_WIDTH                    6
#define XBARA_SEL3_SEL7(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL3_SEL7_SHIFT))&XBARA_SEL3_SEL7_MASK)
/* SEL4 Bit Fields */
#define XBARA_SEL4_SEL8_MASK                     0x3Fu
#define XBARA_SEL4_SEL8_SHIFT                    0
#define XBARA_SEL4_SEL8_WIDTH                    6
#define XBARA_SEL4_SEL8(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL4_SEL8_SHIFT))&XBARA_SEL4_SEL8_MASK)
#define XBARA_SEL4_SEL9_MASK                     0x3F00u
#define XBARA_SEL4_SEL9_SHIFT                    8
#define XBARA_SEL4_SEL9_WIDTH                    6
#define XBARA_SEL4_SEL9(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL4_SEL9_SHIFT))&XBARA_SEL4_SEL9_MASK)
/* SEL5 Bit Fields */
#define XBARA_SEL5_SEL10_MASK                    0x3Fu
#define XBARA_SEL5_SEL10_SHIFT                   0
#define XBARA_SEL5_SEL10_WIDTH                   6
#define XBARA_SEL5_SEL10(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL5_SEL10_SHIFT))&XBARA_SEL5_SEL10_MASK)
#define XBARA_SEL5_SEL11_MASK                    0x3F00u
#define XBARA_SEL5_SEL11_SHIFT                   8
#define XBARA_SEL5_SEL11_WIDTH                   6
#define XBARA_SEL5_SEL11(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL5_SEL11_SHIFT))&XBARA_SEL5_SEL11_MASK)
/* SEL6 Bit Fields */
#define XBARA_SEL6_SEL12_MASK                    0x3Fu
#define XBARA_SEL6_SEL12_SHIFT                   0
#define XBARA_SEL6_SEL12_WIDTH                   6
#define XBARA_SEL6_SEL12(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL6_SEL12_SHIFT))&XBARA_SEL6_SEL12_MASK)
#define XBARA_SEL6_SEL13_MASK                    0x3F00u
#define XBARA_SEL6_SEL13_SHIFT                   8
#define XBARA_SEL6_SEL13_WIDTH                   6
#define XBARA_SEL6_SEL13(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL6_SEL13_SHIFT))&XBARA_SEL6_SEL13_MASK)
/* SEL7 Bit Fields */
#define XBARA_SEL7_SEL14_MASK                    0x3Fu
#define XBARA_SEL7_SEL14_SHIFT                   0
#define XBARA_SEL7_SEL14_WIDTH                   6
#define XBARA_SEL7_SEL14(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL7_SEL14_SHIFT))&XBARA_SEL7_SEL14_MASK)
#define XBARA_SEL7_SEL15_MASK                    0x3F00u
#define XBARA_SEL7_SEL15_SHIFT                   8
#define XBARA_SEL7_SEL15_WIDTH                   6
#define XBARA_SEL7_SEL15(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL7_SEL15_SHIFT))&XBARA_SEL7_SEL15_MASK)
/* SEL8 Bit Fields */
#define XBARA_SEL8_SEL16_MASK                    0x3Fu
#define XBARA_SEL8_SEL16_SHIFT                   0
#define XBARA_SEL8_SEL16_WIDTH                   6
#define XBARA_SEL8_SEL16(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL8_SEL16_SHIFT))&XBARA_SEL8_SEL16_MASK)
#define XBARA_SEL8_SEL17_MASK                    0x3F00u
#define XBARA_SEL8_SEL17_SHIFT                   8
#define XBARA_SEL8_SEL17_WIDTH                   6
#define XBARA_SEL8_SEL17(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL8_SEL17_SHIFT))&XBARA_SEL8_SEL17_MASK)
/* SEL9 Bit Fields */
#define XBARA_SEL9_SEL18_MASK                    0x3Fu
#define XBARA_SEL9_SEL18_SHIFT                   0
#define XBARA_SEL9_SEL18_WIDTH                   6
#define XBARA_SEL9_SEL18(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL9_SEL18_SHIFT))&XBARA_SEL9_SEL18_MASK)
#define XBARA_SEL9_SEL19_MASK                    0x3F00u
#define XBARA_SEL9_SEL19_SHIFT                   8
#define XBARA_SEL9_SEL19_WIDTH                   6
#define XBARA_SEL9_SEL19(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL9_SEL19_SHIFT))&XBARA_SEL9_SEL19_MASK)
/* SEL10 Bit Fields */
#define XBARA_SEL10_SEL20_MASK                   0x3Fu
#define XBARA_SEL10_SEL20_SHIFT                  0
#define XBARA_SEL10_SEL20_WIDTH                  6
#define XBARA_SEL10_SEL20(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL10_SEL20_SHIFT))&XBARA_SEL10_SEL20_MASK)
#define XBARA_SEL10_SEL21_MASK                   0x3F00u
#define XBARA_SEL10_SEL21_SHIFT                  8
#define XBARA_SEL10_SEL21_WIDTH                  6
#define XBARA_SEL10_SEL21(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL10_SEL21_SHIFT))&XBARA_SEL10_SEL21_MASK)
/* SEL11 Bit Fields */
#define XBARA_SEL11_SEL22_MASK                   0x3Fu
#define XBARA_SEL11_SEL22_SHIFT                  0
#define XBARA_SEL11_SEL22_WIDTH                  6
#define XBARA_SEL11_SEL22(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL11_SEL22_SHIFT))&XBARA_SEL11_SEL22_MASK)
#define XBARA_SEL11_SEL23_MASK                   0x3F00u
#define XBARA_SEL11_SEL23_SHIFT                  8
#define XBARA_SEL11_SEL23_WIDTH                  6
#define XBARA_SEL11_SEL23(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL11_SEL23_SHIFT))&XBARA_SEL11_SEL23_MASK)
/* SEL12 Bit Fields */
#define XBARA_SEL12_SEL24_MASK                   0x3Fu
#define XBARA_SEL12_SEL24_SHIFT                  0
#define XBARA_SEL12_SEL24_WIDTH                  6
#define XBARA_SEL12_SEL24(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL12_SEL24_SHIFT))&XBARA_SEL12_SEL24_MASK)
#define XBARA_SEL12_SEL25_MASK                   0x3F00u
#define XBARA_SEL12_SEL25_SHIFT                  8
#define XBARA_SEL12_SEL25_WIDTH                  6
#define XBARA_SEL12_SEL25(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL12_SEL25_SHIFT))&XBARA_SEL12_SEL25_MASK)
/* SEL13 Bit Fields */
#define XBARA_SEL13_SEL26_MASK                   0x3Fu
#define XBARA_SEL13_SEL26_SHIFT                  0
#define XBARA_SEL13_SEL26_WIDTH                  6
#define XBARA_SEL13_SEL26(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL13_SEL26_SHIFT))&XBARA_SEL13_SEL26_MASK)
#define XBARA_SEL13_SEL27_MASK                   0x3F00u
#define XBARA_SEL13_SEL27_SHIFT                  8
#define XBARA_SEL13_SEL27_WIDTH                  6
#define XBARA_SEL13_SEL27(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL13_SEL27_SHIFT))&XBARA_SEL13_SEL27_MASK)
/* SEL14 Bit Fields */
#define XBARA_SEL14_SEL28_MASK                   0x3Fu
#define XBARA_SEL14_SEL28_SHIFT                  0
#define XBARA_SEL14_SEL28_WIDTH                  6
#define XBARA_SEL14_SEL28(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL14_SEL28_SHIFT))&XBARA_SEL14_SEL28_MASK)
#define XBARA_SEL14_SEL29_MASK                   0x3F00u
#define XBARA_SEL14_SEL29_SHIFT                  8
#define XBARA_SEL14_SEL29_WIDTH                  6
#define XBARA_SEL14_SEL29(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL14_SEL29_SHIFT))&XBARA_SEL14_SEL29_MASK)
/* SEL15 Bit Fields */
#define XBARA_SEL15_SEL30_MASK                   0x3Fu
#define XBARA_SEL15_SEL30_SHIFT                  0
#define XBARA_SEL15_SEL30_WIDTH                  6
#define XBARA_SEL15_SEL30(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL15_SEL30_SHIFT))&XBARA_SEL15_SEL30_MASK)
#define XBARA_SEL15_SEL31_MASK                   0x3F00u
#define XBARA_SEL15_SEL31_SHIFT                  8
#define XBARA_SEL15_SEL31_WIDTH                  6
#define XBARA_SEL15_SEL31(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL15_SEL31_SHIFT))&XBARA_SEL15_SEL31_MASK)
/* SEL16 Bit Fields */
#define XBARA_SEL16_SEL32_MASK                   0x3Fu
#define XBARA_SEL16_SEL32_SHIFT                  0
#define XBARA_SEL16_SEL32_WIDTH                  6
#define XBARA_SEL16_SEL32(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL16_SEL32_SHIFT))&XBARA_SEL16_SEL32_MASK)
#define XBARA_SEL16_SEL33_MASK                   0x3F00u
#define XBARA_SEL16_SEL33_SHIFT                  8
#define XBARA_SEL16_SEL33_WIDTH                  6
#define XBARA_SEL16_SEL33(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL16_SEL33_SHIFT))&XBARA_SEL16_SEL33_MASK)
/* SEL17 Bit Fields */
#define XBARA_SEL17_SEL34_MASK                   0x3Fu
#define XBARA_SEL17_SEL34_SHIFT                  0
#define XBARA_SEL17_SEL34_WIDTH                  6
#define XBARA_SEL17_SEL34(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL17_SEL34_SHIFT))&XBARA_SEL17_SEL34_MASK)
#define XBARA_SEL17_SEL35_MASK                   0x3F00u
#define XBARA_SEL17_SEL35_SHIFT                  8
#define XBARA_SEL17_SEL35_WIDTH                  6
#define XBARA_SEL17_SEL35(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL17_SEL35_SHIFT))&XBARA_SEL17_SEL35_MASK)
/* SEL18 Bit Fields */
#define XBARA_SEL18_SEL36_MASK                   0x3Fu
#define XBARA_SEL18_SEL36_SHIFT                  0
#define XBARA_SEL18_SEL36_WIDTH                  6
#define XBARA_SEL18_SEL36(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL18_SEL36_SHIFT))&XBARA_SEL18_SEL36_MASK)
#define XBARA_SEL18_SEL37_MASK                   0x3F00u
#define XBARA_SEL18_SEL37_SHIFT                  8
#define XBARA_SEL18_SEL37_WIDTH                  6
#define XBARA_SEL18_SEL37(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL18_SEL37_SHIFT))&XBARA_SEL18_SEL37_MASK)
/* SEL19 Bit Fields */
#define XBARA_SEL19_SEL38_MASK                   0x3Fu
#define XBARA_SEL19_SEL38_SHIFT                  0
#define XBARA_SEL19_SEL38_WIDTH                  6
#define XBARA_SEL19_SEL38(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL19_SEL38_SHIFT))&XBARA_SEL19_SEL38_MASK)
#define XBARA_SEL19_SEL39_MASK                   0x3F00u
#define XBARA_SEL19_SEL39_SHIFT                  8
#define XBARA_SEL19_SEL39_WIDTH                  6
#define XBARA_SEL19_SEL39(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL19_SEL39_SHIFT))&XBARA_SEL19_SEL39_MASK)
/* SEL20 Bit Fields */
#define XBARA_SEL20_SEL40_MASK                   0x3Fu
#define XBARA_SEL20_SEL40_SHIFT                  0
#define XBARA_SEL20_SEL40_WIDTH                  6
#define XBARA_SEL20_SEL40(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL20_SEL40_SHIFT))&XBARA_SEL20_SEL40_MASK)
#define XBARA_SEL20_SEL41_MASK                   0x3F00u
#define XBARA_SEL20_SEL41_SHIFT                  8
#define XBARA_SEL20_SEL41_WIDTH                  6
#define XBARA_SEL20_SEL41(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL20_SEL41_SHIFT))&XBARA_SEL20_SEL41_MASK)
/* SEL21 Bit Fields */
#define XBARA_SEL21_SEL42_MASK                   0x3Fu
#define XBARA_SEL21_SEL42_SHIFT                  0
#define XBARA_SEL21_SEL42_WIDTH                  6
#define XBARA_SEL21_SEL42(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL21_SEL42_SHIFT))&XBARA_SEL21_SEL42_MASK)
#define XBARA_SEL21_SEL43_MASK                   0x3F00u
#define XBARA_SEL21_SEL43_SHIFT                  8
#define XBARA_SEL21_SEL43_WIDTH                  6
#define XBARA_SEL21_SEL43(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL21_SEL43_SHIFT))&XBARA_SEL21_SEL43_MASK)
/* SEL22 Bit Fields */
#define XBARA_SEL22_SEL44_MASK                   0x3Fu
#define XBARA_SEL22_SEL44_SHIFT                  0
#define XBARA_SEL22_SEL44_WIDTH                  6
#define XBARA_SEL22_SEL44(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL22_SEL44_SHIFT))&XBARA_SEL22_SEL44_MASK)
#define XBARA_SEL22_SEL45_MASK                   0x3F00u
#define XBARA_SEL22_SEL45_SHIFT                  8
#define XBARA_SEL22_SEL45_WIDTH                  6
#define XBARA_SEL22_SEL45(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL22_SEL45_SHIFT))&XBARA_SEL22_SEL45_MASK)
/* SEL23 Bit Fields */
#define XBARA_SEL23_SEL46_MASK                   0x3Fu
#define XBARA_SEL23_SEL46_SHIFT                  0
#define XBARA_SEL23_SEL46_WIDTH                  6
#define XBARA_SEL23_SEL46(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL23_SEL46_SHIFT))&XBARA_SEL23_SEL46_MASK)
#define XBARA_SEL23_SEL47_MASK                   0x3F00u
#define XBARA_SEL23_SEL47_SHIFT                  8
#define XBARA_SEL23_SEL47_WIDTH                  6
#define XBARA_SEL23_SEL47(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL23_SEL47_SHIFT))&XBARA_SEL23_SEL47_MASK)
/* SEL24 Bit Fields */
#define XBARA_SEL24_SEL48_MASK                   0x3Fu
#define XBARA_SEL24_SEL48_SHIFT                  0
#define XBARA_SEL24_SEL48_WIDTH                  6
#define XBARA_SEL24_SEL48(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL24_SEL48_SHIFT))&XBARA_SEL24_SEL48_MASK)
#define XBARA_SEL24_SEL49_MASK                   0x3F00u
#define XBARA_SEL24_SEL49_SHIFT                  8
#define XBARA_SEL24_SEL49_WIDTH                  6
#define XBARA_SEL24_SEL49(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL24_SEL49_SHIFT))&XBARA_SEL24_SEL49_MASK)
/* SEL25 Bit Fields */
#define XBARA_SEL25_SEL50_MASK                   0x3Fu
#define XBARA_SEL25_SEL50_SHIFT                  0
#define XBARA_SEL25_SEL50_WIDTH                  6
#define XBARA_SEL25_SEL50(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL25_SEL50_SHIFT))&XBARA_SEL25_SEL50_MASK)
#define XBARA_SEL25_SEL51_MASK                   0x3F00u
#define XBARA_SEL25_SEL51_SHIFT                  8
#define XBARA_SEL25_SEL51_WIDTH                  6
#define XBARA_SEL25_SEL51(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL25_SEL51_SHIFT))&XBARA_SEL25_SEL51_MASK)
/* SEL26 Bit Fields */
#define XBARA_SEL26_SEL52_MASK                   0x3Fu
#define XBARA_SEL26_SEL52_SHIFT                  0
#define XBARA_SEL26_SEL52_WIDTH                  6
#define XBARA_SEL26_SEL52(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL26_SEL52_SHIFT))&XBARA_SEL26_SEL52_MASK)
#define XBARA_SEL26_SEL53_MASK                   0x3F00u
#define XBARA_SEL26_SEL53_SHIFT                  8
#define XBARA_SEL26_SEL53_WIDTH                  6
#define XBARA_SEL26_SEL53(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL26_SEL53_SHIFT))&XBARA_SEL26_SEL53_MASK)
/* SEL27 Bit Fields */
#define XBARA_SEL27_SEL54_MASK                   0x3Fu
#define XBARA_SEL27_SEL54_SHIFT                  0
#define XBARA_SEL27_SEL54_WIDTH                  6
#define XBARA_SEL27_SEL54(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL27_SEL54_SHIFT))&XBARA_SEL27_SEL54_MASK)
#define XBARA_SEL27_SEL55_MASK                   0x3F00u
#define XBARA_SEL27_SEL55_SHIFT                  8
#define XBARA_SEL27_SEL55_WIDTH                  6
#define XBARA_SEL27_SEL55(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL27_SEL55_SHIFT))&XBARA_SEL27_SEL55_MASK)
/* SEL28 Bit Fields */
#define XBARA_SEL28_SEL56_MASK                   0x3Fu
#define XBARA_SEL28_SEL56_SHIFT                  0
#define XBARA_SEL28_SEL56_WIDTH                  6
#define XBARA_SEL28_SEL56(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL28_SEL56_SHIFT))&XBARA_SEL28_SEL56_MASK)
#define XBARA_SEL28_SEL57_MASK                   0x3F00u
#define XBARA_SEL28_SEL57_SHIFT                  8
#define XBARA_SEL28_SEL57_WIDTH                  6
#define XBARA_SEL28_SEL57(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL28_SEL57_SHIFT))&XBARA_SEL28_SEL57_MASK)
/* SEL29 Bit Fields */
#define XBARA_SEL29_SEL58_MASK                   0x3Fu
#define XBARA_SEL29_SEL58_SHIFT                  0
#define XBARA_SEL29_SEL58_WIDTH                  6
#define XBARA_SEL29_SEL58(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL29_SEL58_SHIFT))&XBARA_SEL29_SEL58_MASK)
/* CTRL0 Bit Fields */
#define XBARA_CTRL0_DEN0_MASK                    0x1u
#define XBARA_CTRL0_DEN0_SHIFT                   0
#define XBARA_CTRL0_DEN0_WIDTH                   1
#define XBARA_CTRL0_DEN0(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_DEN0_SHIFT))&XBARA_CTRL0_DEN0_MASK)
#define XBARA_CTRL0_IEN0_MASK                    0x2u
#define XBARA_CTRL0_IEN0_SHIFT                   1
#define XBARA_CTRL0_IEN0_WIDTH                   1
#define XBARA_CTRL0_IEN0(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_IEN0_SHIFT))&XBARA_CTRL0_IEN0_MASK)
#define XBARA_CTRL0_EDGE0_MASK                   0xCu
#define XBARA_CTRL0_EDGE0_SHIFT                  2
#define XBARA_CTRL0_EDGE0_WIDTH                  2
#define XBARA_CTRL0_EDGE0(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_EDGE0_SHIFT))&XBARA_CTRL0_EDGE0_MASK)
#define XBARA_CTRL0_STS0_MASK                    0x10u
#define XBARA_CTRL0_STS0_SHIFT                   4
#define XBARA_CTRL0_STS0_WIDTH                   1
#define XBARA_CTRL0_STS0(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_STS0_SHIFT))&XBARA_CTRL0_STS0_MASK)
#define XBARA_CTRL0_DEN1_MASK                    0x100u
#define XBARA_CTRL0_DEN1_SHIFT                   8
#define XBARA_CTRL0_DEN1_WIDTH                   1
#define XBARA_CTRL0_DEN1(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_DEN1_SHIFT))&XBARA_CTRL0_DEN1_MASK)
#define XBARA_CTRL0_IEN1_MASK                    0x200u
#define XBARA_CTRL0_IEN1_SHIFT                   9
#define XBARA_CTRL0_IEN1_WIDTH                   1
#define XBARA_CTRL0_IEN1(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_IEN1_SHIFT))&XBARA_CTRL0_IEN1_MASK)
#define XBARA_CTRL0_EDGE1_MASK                   0xC00u
#define XBARA_CTRL0_EDGE1_SHIFT                  10
#define XBARA_CTRL0_EDGE1_WIDTH                  2
#define XBARA_CTRL0_EDGE1(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_EDGE1_SHIFT))&XBARA_CTRL0_EDGE1_MASK)
#define XBARA_CTRL0_STS1_MASK                    0x1000u
#define XBARA_CTRL0_STS1_SHIFT                   12
#define XBARA_CTRL0_STS1_WIDTH                   1
#define XBARA_CTRL0_STS1(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_STS1_SHIFT))&XBARA_CTRL0_STS1_MASK)
/* CTRL1 Bit Fields */
#define XBARA_CTRL1_DEN2_MASK                    0x1u
#define XBARA_CTRL1_DEN2_SHIFT                   0
#define XBARA_CTRL1_DEN2_WIDTH                   1
#define XBARA_CTRL1_DEN2(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_DEN2_SHIFT))&XBARA_CTRL1_DEN2_MASK)
#define XBARA_CTRL1_IEN2_MASK                    0x2u
#define XBARA_CTRL1_IEN2_SHIFT                   1
#define XBARA_CTRL1_IEN2_WIDTH                   1
#define XBARA_CTRL1_IEN2(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_IEN2_SHIFT))&XBARA_CTRL1_IEN2_MASK)
#define XBARA_CTRL1_EDGE2_MASK                   0xCu
#define XBARA_CTRL1_EDGE2_SHIFT                  2
#define XBARA_CTRL1_EDGE2_WIDTH                  2
#define XBARA_CTRL1_EDGE2(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_EDGE2_SHIFT))&XBARA_CTRL1_EDGE2_MASK)
#define XBARA_CTRL1_STS2_MASK                    0x10u
#define XBARA_CTRL1_STS2_SHIFT                   4
#define XBARA_CTRL1_STS2_WIDTH                   1
#define XBARA_CTRL1_STS2(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_STS2_SHIFT))&XBARA_CTRL1_STS2_MASK)
#define XBARA_CTRL1_DEN3_MASK                    0x100u
#define XBARA_CTRL1_DEN3_SHIFT                   8
#define XBARA_CTRL1_DEN3_WIDTH                   1
#define XBARA_CTRL1_DEN3(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_DEN3_SHIFT))&XBARA_CTRL1_DEN3_MASK)
#define XBARA_CTRL1_IEN3_MASK                    0x200u
#define XBARA_CTRL1_IEN3_SHIFT                   9
#define XBARA_CTRL1_IEN3_WIDTH                   1
#define XBARA_CTRL1_IEN3(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_IEN3_SHIFT))&XBARA_CTRL1_IEN3_MASK)
#define XBARA_CTRL1_EDGE3_MASK                   0xC00u
#define XBARA_CTRL1_EDGE3_SHIFT                  10
#define XBARA_CTRL1_EDGE3_WIDTH                  2
#define XBARA_CTRL1_EDGE3(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_EDGE3_SHIFT))&XBARA_CTRL1_EDGE3_MASK)
#define XBARA_CTRL1_STS3_MASK                    0x1000u
#define XBARA_CTRL1_STS3_SHIFT                   12
#define XBARA_CTRL1_STS3_WIDTH                   1
#define XBARA_CTRL1_STS3(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_STS3_SHIFT))&XBARA_CTRL1_STS3_MASK)

/*!
 * @}
 */ /* end of group XBARA_Register_Masks */


/* XBARA - Peripheral instance base addresses */
/** Peripheral XBARA base address */
#define XBARA_BASE                               (0x40059000u)
/** Peripheral XBARA base pointer */
#define XBARA                                    ((XBARA_Type *)XBARA_BASE)
#define XBARA_BASE_PTR                           (XBARA)
/** Array initializer of XBARA peripheral base addresses */
#define XBARA_BASE_ADDRS                         { XBARA_BASE }
/** Array initializer of XBARA peripheral base pointers */
#define XBARA_BASE_PTRS                          { XBARA }

/* ----------------------------------------------------------------------------
   -- XBARA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Accessor_Macros XBARA - Register accessor macros
 * @{
 */


/* XBARA - Register instance definitions */
/* XBARA */
#define XBARA_SEL0                               XBARA_SEL0_REG(XBARA)
#define XBARA_SEL1                               XBARA_SEL1_REG(XBARA)
#define XBARA_SEL2                               XBARA_SEL2_REG(XBARA)
#define XBARA_SEL3                               XBARA_SEL3_REG(XBARA)
#define XBARA_SEL4                               XBARA_SEL4_REG(XBARA)
#define XBARA_SEL5                               XBARA_SEL5_REG(XBARA)
#define XBARA_SEL6                               XBARA_SEL6_REG(XBARA)
#define XBARA_SEL7                               XBARA_SEL7_REG(XBARA)
#define XBARA_SEL8                               XBARA_SEL8_REG(XBARA)
#define XBARA_SEL9                               XBARA_SEL9_REG(XBARA)
#define XBARA_SEL10                              XBARA_SEL10_REG(XBARA)
#define XBARA_SEL11                              XBARA_SEL11_REG(XBARA)
#define XBARA_SEL12                              XBARA_SEL12_REG(XBARA)
#define XBARA_SEL13                              XBARA_SEL13_REG(XBARA)
#define XBARA_SEL14                              XBARA_SEL14_REG(XBARA)
#define XBARA_SEL15                              XBARA_SEL15_REG(XBARA)
#define XBARA_SEL16                              XBARA_SEL16_REG(XBARA)
#define XBARA_SEL17                              XBARA_SEL17_REG(XBARA)
#define XBARA_SEL18                              XBARA_SEL18_REG(XBARA)
#define XBARA_SEL19                              XBARA_SEL19_REG(XBARA)
#define XBARA_SEL20                              XBARA_SEL20_REG(XBARA)
#define XBARA_SEL21                              XBARA_SEL21_REG(XBARA)
#define XBARA_SEL22                              XBARA_SEL22_REG(XBARA)
#define XBARA_SEL23                              XBARA_SEL23_REG(XBARA)
#define XBARA_SEL24                              XBARA_SEL24_REG(XBARA)
#define XBARA_SEL25                              XBARA_SEL25_REG(XBARA)
#define XBARA_SEL26                              XBARA_SEL26_REG(XBARA)
#define XBARA_SEL27                              XBARA_SEL27_REG(XBARA)
#define XBARA_SEL28                              XBARA_SEL28_REG(XBARA)
#define XBARA_SEL29                              XBARA_SEL29_REG(XBARA)
#define XBARA_CTRL0                              XBARA_CTRL0_REG(XBARA)
#define XBARA_CTRL1                              XBARA_CTRL1_REG(XBARA)

/*!
 * @}
 */ /* end of group XBARA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group XBARA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- XBARB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Peripheral_Access_Layer XBARB Peripheral Access Layer
 * @{
 */

/** XBARB - Register Layout Typedef */
typedef struct {
  __IO uint16_t SEL0;                              /**< Crossbar B Select Register 0, offset: 0x0 */
  __IO uint16_t SEL1;                              /**< Crossbar B Select Register 1, offset: 0x2 */
  __IO uint16_t SEL2;                              /**< Crossbar B Select Register 2, offset: 0x4 */
  __IO uint16_t SEL3;                              /**< Crossbar B Select Register 3, offset: 0x6 */
  __IO uint16_t SEL4;                              /**< Crossbar B Select Register 4, offset: 0x8 */
  __IO uint16_t SEL5;                              /**< Crossbar B Select Register 5, offset: 0xA */
  __IO uint16_t SEL6;                              /**< Crossbar B Select Register 6, offset: 0xC */
  __IO uint16_t SEL7;                              /**< Crossbar B Select Register 7, offset: 0xE */
} XBARB_Type, *XBARB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- XBARB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Accessor_Macros XBARB - Register accessor macros
 * @{
 */


/* XBARB - Register accessors */
#define XBARB_SEL0_REG(base)                     ((base)->SEL0)
#define XBARB_SEL1_REG(base)                     ((base)->SEL1)
#define XBARB_SEL2_REG(base)                     ((base)->SEL2)
#define XBARB_SEL3_REG(base)                     ((base)->SEL3)
#define XBARB_SEL4_REG(base)                     ((base)->SEL4)
#define XBARB_SEL5_REG(base)                     ((base)->SEL5)
#define XBARB_SEL6_REG(base)                     ((base)->SEL6)
#define XBARB_SEL7_REG(base)                     ((base)->SEL7)

/*!
 * @}
 */ /* end of group XBARB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- XBARB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Masks XBARB Register Masks
 * @{
 */

/* SEL0 Bit Fields */
#define XBARB_SEL0_SEL0_MASK                     0x1Fu
#define XBARB_SEL0_SEL0_SHIFT                    0
#define XBARB_SEL0_SEL0_WIDTH                    5
#define XBARB_SEL0_SEL0(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL0_SEL0_SHIFT))&XBARB_SEL0_SEL0_MASK)
#define XBARB_SEL0_SEL1_MASK                     0x1F00u
#define XBARB_SEL0_SEL1_SHIFT                    8
#define XBARB_SEL0_SEL1_WIDTH                    5
#define XBARB_SEL0_SEL1(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL0_SEL1_SHIFT))&XBARB_SEL0_SEL1_MASK)
/* SEL1 Bit Fields */
#define XBARB_SEL1_SEL2_MASK                     0x1Fu
#define XBARB_SEL1_SEL2_SHIFT                    0
#define XBARB_SEL1_SEL2_WIDTH                    5
#define XBARB_SEL1_SEL2(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL1_SEL2_SHIFT))&XBARB_SEL1_SEL2_MASK)
#define XBARB_SEL1_SEL3_MASK                     0x1F00u
#define XBARB_SEL1_SEL3_SHIFT                    8
#define XBARB_SEL1_SEL3_WIDTH                    5
#define XBARB_SEL1_SEL3(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL1_SEL3_SHIFT))&XBARB_SEL1_SEL3_MASK)
/* SEL2 Bit Fields */
#define XBARB_SEL2_SEL4_MASK                     0x1Fu
#define XBARB_SEL2_SEL4_SHIFT                    0
#define XBARB_SEL2_SEL4_WIDTH                    5
#define XBARB_SEL2_SEL4(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL2_SEL4_SHIFT))&XBARB_SEL2_SEL4_MASK)
#define XBARB_SEL2_SEL5_MASK                     0x1F00u
#define XBARB_SEL2_SEL5_SHIFT                    8
#define XBARB_SEL2_SEL5_WIDTH                    5
#define XBARB_SEL2_SEL5(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL2_SEL5_SHIFT))&XBARB_SEL2_SEL5_MASK)
/* SEL3 Bit Fields */
#define XBARB_SEL3_SEL6_MASK                     0x1Fu
#define XBARB_SEL3_SEL6_SHIFT                    0
#define XBARB_SEL3_SEL6_WIDTH                    5
#define XBARB_SEL3_SEL6(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL3_SEL6_SHIFT))&XBARB_SEL3_SEL6_MASK)
#define XBARB_SEL3_SEL7_MASK                     0x1F00u
#define XBARB_SEL3_SEL7_SHIFT                    8
#define XBARB_SEL3_SEL7_WIDTH                    5
#define XBARB_SEL3_SEL7(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL3_SEL7_SHIFT))&XBARB_SEL3_SEL7_MASK)
/* SEL4 Bit Fields */
#define XBARB_SEL4_SEL8_MASK                     0x1Fu
#define XBARB_SEL4_SEL8_SHIFT                    0
#define XBARB_SEL4_SEL8_WIDTH                    5
#define XBARB_SEL4_SEL8(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL4_SEL8_SHIFT))&XBARB_SEL4_SEL8_MASK)
#define XBARB_SEL4_SEL9_MASK                     0x1F00u
#define XBARB_SEL4_SEL9_SHIFT                    8
#define XBARB_SEL4_SEL9_WIDTH                    5
#define XBARB_SEL4_SEL9(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL4_SEL9_SHIFT))&XBARB_SEL4_SEL9_MASK)
/* SEL5 Bit Fields */
#define XBARB_SEL5_SEL10_MASK                    0x1Fu
#define XBARB_SEL5_SEL10_SHIFT                   0
#define XBARB_SEL5_SEL10_WIDTH                   5
#define XBARB_SEL5_SEL10(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL5_SEL10_SHIFT))&XBARB_SEL5_SEL10_MASK)
#define XBARB_SEL5_SEL11_MASK                    0x1F00u
#define XBARB_SEL5_SEL11_SHIFT                   8
#define XBARB_SEL5_SEL11_WIDTH                   5
#define XBARB_SEL5_SEL11(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL5_SEL11_SHIFT))&XBARB_SEL5_SEL11_MASK)
/* SEL6 Bit Fields */
#define XBARB_SEL6_SEL12_MASK                    0x1Fu
#define XBARB_SEL6_SEL12_SHIFT                   0
#define XBARB_SEL6_SEL12_WIDTH                   5
#define XBARB_SEL6_SEL12(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL6_SEL12_SHIFT))&XBARB_SEL6_SEL12_MASK)
#define XBARB_SEL6_SEL13_MASK                    0x1F00u
#define XBARB_SEL6_SEL13_SHIFT                   8
#define XBARB_SEL6_SEL13_WIDTH                   5
#define XBARB_SEL6_SEL13(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL6_SEL13_SHIFT))&XBARB_SEL6_SEL13_MASK)
/* SEL7 Bit Fields */
#define XBARB_SEL7_SEL14_MASK                    0x1Fu
#define XBARB_SEL7_SEL14_SHIFT                   0
#define XBARB_SEL7_SEL14_WIDTH                   5
#define XBARB_SEL7_SEL14(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL7_SEL14_SHIFT))&XBARB_SEL7_SEL14_MASK)
#define XBARB_SEL7_SEL15_MASK                    0x1F00u
#define XBARB_SEL7_SEL15_SHIFT                   8
#define XBARB_SEL7_SEL15_WIDTH                   5
#define XBARB_SEL7_SEL15(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL7_SEL15_SHIFT))&XBARB_SEL7_SEL15_MASK)

/*!
 * @}
 */ /* end of group XBARB_Register_Masks */


/* XBARB - Peripheral instance base addresses */
/** Peripheral XBARB base address */
#define XBARB_BASE                               (0x4005A000u)
/** Peripheral XBARB base pointer */
#define XBARB                                    ((XBARB_Type *)XBARB_BASE)
#define XBARB_BASE_PTR                           (XBARB)
/** Array initializer of XBARB peripheral base addresses */
#define XBARB_BASE_ADDRS                         { XBARB_BASE }
/** Array initializer of XBARB peripheral base pointers */
#define XBARB_BASE_PTRS                          { XBARB }

/* ----------------------------------------------------------------------------
   -- XBARB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Accessor_Macros XBARB - Register accessor macros
 * @{
 */


/* XBARB - Register instance definitions */
/* XBARB */
#define XBARB_SEL0                               XBARB_SEL0_REG(XBARB)
#define XBARB_SEL1                               XBARB_SEL1_REG(XBARB)
#define XBARB_SEL2                               XBARB_SEL2_REG(XBARB)
#define XBARB_SEL3                               XBARB_SEL3_REG(XBARB)
#define XBARB_SEL4                               XBARB_SEL4_REG(XBARB)
#define XBARB_SEL5                               XBARB_SEL5_REG(XBARB)
#define XBARB_SEL6                               XBARB_SEL6_REG(XBARB)
#define XBARB_SEL7                               XBARB_SEL7_REG(XBARB)

/*!
 * @}
 */ /* end of group XBARB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group XBARB_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- Backward Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Backward_Compatibility_Symbols Backward Compatibility
 * @{
 */

#define BR_LLWU_F1_WUF0                          BR_LLWU_PF1_WUF0
#define BR_LLWU_F1_WUF1                          BR_LLWU_PF1_WUF1
#define BR_LLWU_F1_WUF2                          BR_LLWU_PF1_WUF2
#define BR_LLWU_F1_WUF3                          BR_LLWU_PF1_WUF3
#define BR_LLWU_F1_WUF4                          BR_LLWU_PF1_WUF4
#define BR_LLWU_F1_WUF5                          BR_LLWU_PF1_WUF5
#define BR_LLWU_F1_WUF6                          BR_LLWU_PF1_WUF6
#define BR_LLWU_F1_WUF7                          BR_LLWU_PF1_WUF7
#define BW_LLWU_F1_WUF0                          BW_LLWU_PF1_WUF0
#define BW_LLWU_F1_WUF1                          BW_LLWU_PF1_WUF1
#define BW_LLWU_F1_WUF2                          BW_LLWU_PF1_WUF2
#define BW_LLWU_F1_WUF3                          BW_LLWU_PF1_WUF3
#define BW_LLWU_F1_WUF4                          BW_LLWU_PF1_WUF4
#define BW_LLWU_F1_WUF5                          BW_LLWU_PF1_WUF5
#define BW_LLWU_F1_WUF6                          BW_LLWU_PF1_WUF6
#define BW_LLWU_F1_WUF7                          BW_LLWU_PF1_WUF7
#define BR_LLWU_F2_WUF8                          BR_LLWU_PF2_WUF8
#define BR_LLWU_F2_WUF9                          BR_LLWU_PF2_WUF9
#define BR_LLWU_F2_WUF10                         BR_LLWU_PF2_WUF10
#define BR_LLWU_F2_WUF11                         BR_LLWU_PF2_WUF11
#define BR_LLWU_F2_WUF12                         BR_LLWU_PF2_WUF12
#define BR_LLWU_F2_WUF13                         BR_LLWU_PF2_WUF13
#define BR_LLWU_F2_WUF14                         BR_LLWU_PF2_WUF14
#define BR_LLWU_F2_WUF15                         BR_LLWU_PF2_WUF15
#define BW_LLWU_F2_WUF8                          BW_LLWU_PF2_WUF8
#define BW_LLWU_F2_WUF9                          BW_LLWU_PF2_WUF9
#define BW_LLWU_F2_WUF10                         BW_LLWU_PF2_WUF10
#define BW_LLWU_F2_WUF11                         BW_LLWU_PF2_WUF11
#define BW_LLWU_F2_WUF12                         BW_LLWU_PF2_WUF12
#define BW_LLWU_F2_WUF13                         BW_LLWU_PF2_WUF13
#define BW_LLWU_F2_WUF14                         BW_LLWU_PF2_WUF14
#define BW_LLWU_F2_WUF15                         BW_LLWU_PF2_WUF15
#define BR_LLWU_F3_WUF16                         BR_LLWU_PF3_WUF16
#define BR_LLWU_F3_WUF17                         BR_LLWU_PF3_WUF17
#define BR_LLWU_F3_WUF18                         BR_LLWU_PF3_WUF18
#define BR_LLWU_F3_WUF19                         BR_LLWU_PF3_WUF19
#define BR_LLWU_F3_WUF20                         BR_LLWU_PF3_WUF20
#define BR_LLWU_F3_WUF21                         BR_LLWU_PF3_WUF21
#define BR_LLWU_F3_WUF22                         BR_LLWU_PF3_WUF22
#define BR_LLWU_F3_WUF23                         BR_LLWU_PF3_WUF23
#define BW_LLWU_F3_WUF16                         BW_LLWU_PF3_WUF16
#define BW_LLWU_F3_WUF17                         BW_LLWU_PF3_WUF17
#define BW_LLWU_F3_WUF18                         BW_LLWU_PF3_WUF18
#define BW_LLWU_F3_WUF19                         BW_LLWU_PF3_WUF19
#define BW_LLWU_F3_WUF20                         BW_LLWU_PF3_WUF20
#define BW_LLWU_F3_WUF21                         BW_LLWU_PF3_WUF21
#define BW_LLWU_F3_WUF22                         BW_LLWU_PF3_WUF22
#define BW_LLWU_F3_WUF23                         BW_LLWU_PF3_WUF23
#define BR_LLWU_F3_MWUF0                         BR_LLWU_MF5_MWUF0
#define BR_LLWU_F3_MWUF1                         BR_LLWU_MF5_MWUF1
#define BR_LLWU_F3_MWUF2                         BR_LLWU_MF5_MWUF2
#define BR_LLWU_F3_MWUF3                         BR_LLWU_MF5_MWUF3
#define BR_LLWU_F3_MWUF4                         BR_LLWU_MF5_MWUF4
#define BR_LLWU_F3_MWUF5                         BR_LLWU_MF5_MWUF5
#define BR_LLWU_F3_MWUF6                         BR_LLWU_MF5_MWUF6
#define BR_LLWU_F3_MWUF7                         BR_LLWU_MF5_MWUF7
#define LPTimer_IRQHandler                       LPTMR0_IRQHandler
#define BP_SIM_SOPT7_ADC0TRGSEL                  BP_SIM_SOPT7_ADCATRGSEL
#define BM_SIM_SOPT7_ADC0TRGSEL                  BM_SIM_SOPT7_ADCATRGSEL
#define BS_SIM_SOPT7_ADC0TRGSEL                  BS_SIM_SOPT7_ADCATRGSEL
#define BR_SIM_SOPT7_ADC0TRGSEL(x)               BR_SIM_SOPT7_ADCATRGSEL(x)
#define BF_SIM_SOPT7_ADC0TRGSEL(v)               BF_SIM_SOPT7_ADCATRGSEL(v)
#define BW_SIM_SOPT7_ADC0TRGSEL(x, v)            BW_SIM_SOPT7_ADCATRGSEL(x, v)
#define BP_SIM_SOPT7_ADC0ALTTRGEN                BP_SIM_SOPT7_ADCAALTTRGEN
#define BM_SIM_SOPT7_ADC0ALTTRGEN                BM_SIM_SOPT7_ADCAALTTRGEN
#define BS_SIM_SOPT7_ADC0ALTTRGEN                BS_SIM_SOPT7_ADCAALTTRGEN
#define BR_SIM_SOPT7_ADC0ALTTRGEN(x)             BR_SIM_SOPT7_ADCAALTTRGEN(x)
#define BF_SIM_SOPT7_ADC0ALTTRGEN(v)             BF_SIM_SOPT7_ADCAALTTRGEN(v)
#define BW_SIM_SOPT7_ADC0ALTTRGEN(x, v)          BW_SIM_SOPT7_ADCAALTTRGEN(x, v)
#define BR_SIM_SOPT7_ADC1TRGSEL(x)               BR_SIM_SOPT7_ADCBTRGSEL(x)
#define BF_SIM_SOPT7_ADC1TRGSEL(v)               BF_SIM_SOPT7_ADCBTRGSEL(v)
#define BW_SIM_SOPT7_ADC1TRGSEL(x, v)            BW_SIM_SOPT7_ADCBTRGSEL(x, v)
#define BR_MCG_C5_PRDIV0(x)                      BR_MCG_C5_PRDIV(x)
#define BF_MCG_C5_PRDIV0(v)                      BF_MCG_C5_PRDIV(v)
#define BW_MCG_C5_PRDIV0(x, v)                   BW_MCG_C5_PRDIV(x, v)
#define BR_MCG_C5_PLLSTEN0(x)                    BR_MCG_C5_PLLSTEN(x)
#define BF_MCG_C5_PLLSTEN0(v)                    BF_MCG_C5_PLLSTEN(v)
#define BW_MCG_C5_PLLSTEN0(x, v)                 BW_MCG_C5_PLLSTEN(x, v)
#define BR_MCG_C5_PLLCLKEN0(x)                   BR_MCG_C5_PLLCLKEN(x)
#define BF_MCG_C5_PLLCLKEN0(v)                   BF_MCG_C5_PLLCLKEN(v)
#define BW_MCG_C5_PLLCLKEN0(x, v)                BW_MCG_C5_PLLCLKEN(x, v)
#define BR_MCG_C6_VDIV0(x)                       BR_MCG_C6_VDIV(x)
#define BF_MCG_C6_VDIV0(v)                       BF_MCG_C6_VDIV(v)
#define BW_MCG_C6_VDIV0(x, v)                    BW_MCG_C6_VDIV(x, v)
#define PTA_BASE                                 GPIOA_BASE
#define PTA                                      GPIOA
#define PTA_BASE_PTR                             (PTA)
#define PTB_BASE                                 GPIOB_BASE
#define PTB                                      GPIOB
#define PTB_BASE_PTR                             (PTB)
#define PTC_BASE                                 GPIOC_BASE
#define PTC                                      GPIOC
#define PTC_BASE_PTR                             (PTC)
#define PTD_BASE                                 GPIOD_BASE
#define PTD                                      GPIOD
#define PTD_BASE_PTR                             (PTD)
#define PTE_BASE                                 GPIOE_BASE
#define PTE                                      GPIOE
#define PTE_BASE_PTR                             (PTE)
#define DAC_IRQn                                 DAC0_IRQn
#define DAC_IRQHandler                           DAC0_IRQHandler
#define ENC_COMPARE_IRQn                         ENC0_COMPARE_IRQn
#define ENC_COMPARE_IRQHandler                   ENC0_COMPARE_IRQHandler
#define ENC_HOME_IRQn                            ENC0_HOME_IRQn
#define ENC_HOME_IRQHandler                      ENC0_HOME_IRQHandler
#define ENC_WDOG_SAB_IRQn                        ENC0_WDOG_SAB_IRQn
#define ENC_WDOG_SAB_IRQHandler                  ENC0_WDOG_SAB_IRQHandler
#define ENC_INDEX_IRQn                           ENC0_INDEX_IRQn
#define ENC_INDEX_IRQHandler                     ENC0_INDEX_IRQHandler
#define I2C_IRQn                                 I2C0_IRQn
#define I2C_IRQHandler                           I2C0_IRQHandler
#define LPTMR_IRQn                               LPTMR0_IRQn
#define LPTMR_IRQHandler                         LPTMR0_IRQHandler
#define SPI_IRQn                                 SPI0_IRQn
#define SPI_IRQHandler                           SPI0_IRQHandler
#define DAC_BASE                                 DAC0_BASE
#define DAC_BASE_PTR                             DAC0_BASE_PTR
#define DAC_DAT0L                                DAC0_DAT0L
#define DAC_DAT0H                                DAC0_DAT0H
#define DAC_DAT1L                                DAC0_DAT1L
#define DAC_DAT1H                                DAC0_DAT1H
#define DAC_DAT2L                                DAC0_DAT2L
#define DAC_DAT2H                                DAC0_DAT2H
#define DAC_DAT3L                                DAC0_DAT3L
#define DAC_DAT3H                                DAC0_DAT3H
#define DAC_DAT4L                                DAC0_DAT4L
#define DAC_DAT4H                                DAC0_DAT4H
#define DAC_DAT5L                                DAC0_DAT5L
#define DAC_DAT5H                                DAC0_DAT5H
#define DAC_DAT6L                                DAC0_DAT6L
#define DAC_DAT6H                                DAC0_DAT6H
#define DAC_DAT7L                                DAC0_DAT7L
#define DAC_DAT7H                                DAC0_DAT7H
#define DAC_DAT8L                                DAC0_DAT8L
#define DAC_DAT8H                                DAC0_DAT8H
#define DAC_DAT9L                                DAC0_DAT9L
#define DAC_DAT9H                                DAC0_DAT9H
#define DAC_DAT10L                               DAC0_DAT10L
#define DAC_DAT10H                               DAC0_DAT10H
#define DAC_DAT11L                               DAC0_DAT11L
#define DAC_DAT11H                               DAC0_DAT11H
#define DAC_DAT12L                               DAC0_DAT12L
#define DAC_DAT12H                               DAC0_DAT12H
#define DAC_DAT13L                               DAC0_DAT13L
#define DAC_DAT13H                               DAC0_DAT13H
#define DAC_DAT14L                               DAC0_DAT14L
#define DAC_DAT14H                               DAC0_DAT14H
#define DAC_DAT15L                               DAC0_DAT15L
#define DAC_DAT15H                               DAC0_DAT15H
#define DAC_SR                                   DAC0_SR
#define DAC_C0                                   DAC0_C0
#define DAC_C1                                   DAC0_C1
#define DAC_C2                                   DAC0_C2
#define DAC_DATL(index)                          DAC0_DATL(index)
#define DAC_DATH(index)                          DAC0_DATH(index)
#define ENC_BASE                                 ENC0_BASE
#define ENC                                      ENC0
#define ENC_BASE_PTR                             ENC0_BASE_PTR
#define ENC_CTRL                                 ENC0_CTRL
#define ENC_FILT                                 ENC0_FILT
#define ENC_WTR                                  ENC0_WTR
#define ENC_POSD                                 ENC0_POSD
#define ENC_POSDH                                ENC0_POSDH
#define ENC_REV                                  ENC0_REV
#define ENC_REVH                                 ENC0_REVH
#define ENC_UPOS                                 ENC0_UPOS
#define ENC_LPOS                                 ENC0_LPOS
#define ENC_UPOSH                                ENC0_UPOSH
#define ENC_LPOSH                                ENC0_LPOSH
#define ENC_UINIT                                ENC0_UINIT
#define ENC_LINIT                                ENC0_LINIT
#define ENC_IMR                                  ENC0_IMR
#define ENC_TST                                  ENC0_TST
#define ENC_CTRL2                                ENC0_CTRL2
#define ENC_UMOD                                 ENC0_UMOD
#define ENC_LMOD                                 ENC0_LMOD
#define ENC_UCOMP                                ENC0_UCOMP
#define ENC_LCOMP                                ENC0_LCOMP
#define I2C_BASE                                 I2C0_BASE
#define I2C                                      I2C0
#define I2C_BASE_PTR                             I2C0_BASE_PTR
#define I2C_A1                                   I2C0_A1
#define I2C_F                                    I2C0_F
#define I2C_C1                                   I2C0_C1
#define I2C_S                                    I2C0_S
#define I2C_D                                    I2C0_D
#define I2C_C2                                   I2C0_C2
#define I2C_FLT                                  I2C0_FLT
#define I2C_RA                                   I2C0_RA
#define I2C_SMB                                  I2C0_SMB
#define I2C_A2                                   I2C0_A2
#define I2C_SLTH                                 I2C0_SLTH
#define I2C_SLTL                                 I2C0_SLTL
#define LPTMR_BASE                               LPTMR0_BASE
#define LPTMR                                    LPTMR0
#define LPTMR_BASE_PTR                           LPTMR0_BASE_PTR
#define LPTMR_CSR                                LPTMR0_CSR
#define LPTMR_PSR                                LPTMR0_PSR
#define LPTMR_CMR                                LPTMR0_CMR
#define LPTMR_CNR                                LPTMR0_CNR
#define MCM_ISR_REG(base)                        MCM_ISCR_REG(base)
#define MCM_ISR_FIOC_MASK                        MCM_ISCR_FIOC_MASK
#define MCM_ISR_FIOC_SHIFT                       MCM_ISCR_FIOC_SHIFT
#define MCM_ISR_FIOC_WIDTH                       MCM_ISCR_FIOC_WIDTH
#define MCM_ISR_FIOC(x)                          MCM_ISCR_FIOC(x)
#define MCM_ISR_FDZC_MASK                        MCM_ISCR_FDZC_MASK
#define MCM_ISR_FDZC_SHIFT                       MCM_ISCR_FDZC_SHIFT
#define MCM_ISR_FDZC_WIDTH                       MCM_ISCR_FDZC_WIDTH
#define MCM_ISR_FDZC(x)                          MCM_ISCR_FDZC(x)
#define MCM_ISR_FOFC_MASK                        MCM_ISCR_FOFC_MASK
#define MCM_ISR_FOFC_SHIFT                       MCM_ISCR_FOFC_SHIFT
#define MCM_ISR_FOFC_WIDTH                       MCM_ISCR_FOFC_WIDTH
#define MCM_ISR_FOFC(x)                          MCM_ISCR_FOFC(x)
#define MCM_ISR_FUFC_MASK                        MCM_ISCR_FUFC_MASK
#define MCM_ISR_FUFC_SHIFT                       MCM_ISCR_FUFC_SHIFT
#define MCM_ISR_FUFC_WIDTH                       MCM_ISCR_FUFC_WIDTH
#define MCM_ISR_FUFC(x)                          MCM_ISCR_FUFC(x)
#define MCM_ISR_FIXC_MASK                        MCM_ISCR_FIXC_MASK
#define MCM_ISR_FIXC_SHIFT                       MCM_ISCR_FIXC_SHIFT
#define MCM_ISR_FIXC_WIDTH                       MCM_ISCR_FIXC_WIDTH
#define MCM_ISR_FIXC(x)                          MCM_ISCR_FIXC(x)
#define MCM_ISR_FIDC_MASK                        MCM_ISCR_FIDC_MASK
#define MCM_ISR_FIDC_SHIFT                       MCM_ISCR_FIDC_SHIFT
#define MCM_ISR_FIDC_WIDTH                       MCM_ISCR_FIDC_WIDTH
#define MCM_ISR_FIDC(x)                          MCM_ISCR_FIDC(x)
#define MCM_ISR_FIOCE_MASK                       MCM_ISCR_FIOCE_MASK
#define MCM_ISR_FIOCE_SHIFT                      MCM_ISCR_FIOCE_SHIFT
#define MCM_ISR_FIOCE_WIDTH                      MCM_ISCR_FIOCE_WIDTH
#define MCM_ISR_FIOCE(x)                         MCM_ISCR_FIOCE(x)
#define MCM_ISR_FDZCE_MASK                       MCM_ISCR_FDZCE_MASK
#define MCM_ISR_FDZCE_SHIFT                      MCM_ISCR_FDZCE_SHIFT
#define MCM_ISR_FDZCE_WIDTH                      MCM_ISCR_FDZCE_WIDTH
#define MCM_ISR_FDZCE(x)                         MCM_ISCR_FDZCE(x)
#define MCM_ISR_FOFCE_MASK                       MCM_ISCR_FOFCE_MASK
#define MCM_ISR_FOFCE_SHIFT                      MCM_ISCR_FOFCE_SHIFT
#define MCM_ISR_FOFCE_WIDTH                      MCM_ISCR_FOFCE_WIDTH
#define MCM_ISR_FOFCE(x)                         MCM_ISCR_FOFCE(x)
#define MCM_ISR_FUFCE_MASK                       MCM_ISCR_FUFCE_MASK
#define MCM_ISR_FUFCE_SHIFT                      MCM_ISCR_FUFCE_SHIFT
#define MCM_ISR_FUFCE_WIDTH                      MCM_ISCR_FUFCE_WIDTH
#define MCM_ISR_FUFCE(x)                         MCM_ISCR_FUFCE(x)
#define MCM_ISR_FIXCE_MASK                       MCM_ISCR_FIXCE_MASK
#define MCM_ISR_FIXCE_SHIFT                      MCM_ISCR_FIXCE_SHIFT
#define MCM_ISR_FIXCE_WIDTH                      MCM_ISCR_FIXCE_WIDTH
#define MCM_ISR_FIXCE(x)                         MCM_ISCR_FIXCE(x)
#define MCM_ISR_FIDCE_MASK                       MCM_ISCR_FIDCE_MASK
#define MCM_ISR_FIDCE_SHIFT                      MCM_ISCR_FIDCE_SHIFT
#define MCM_ISR_FIDCE_WIDTH                      MCM_ISCR_FIDCE_WIDTH
#define MCM_ISR_FIDCE(x)                         MCM_ISCR_FIDCE(x)
#define MCM_ISR                                  MCM_ISCR
#define SIM_MISCTRL                              SIM_MISCTRL0
#define SIM_MISCTRL_REG(base)                    SIM_MISCTRL0_REG(base)
#define SIM_MISCTRL_CMPWIN0SRC_MASK              SIM_MISCTRL0_CMPWIN0SRC_MASK
#define SIM_MISCTRL_CMPWIN0SRC_SHIFT             SIM_MISCTRL0_CMPWIN0SRC_SHIFT
#define SIM_MISCTRL_CMPWIN0SRC_WIDTH             SIM_MISCTRL0_CMPWIN0SRC_WIDTH
#define SIM_MISCTRL_CMPWIN0SRC(x)                SIM_MISCTRL0_CMPWIN0SRC(x)
#define SIM_MISCTRL_CMPWIN1SRC_MASK              SIM_MISCTRL0_CMPWIN1SRC_MASK
#define SIM_MISCTRL_CMPWIN1SRC_SHIFT             SIM_MISCTRL0_CMPWIN1SRC_SHIFT
#define SIM_MISCTRL_CMPWIN1SRC_WIDTH             SIM_MISCTRL0_CMPWIN1SRC_WIDTH
#define SIM_MISCTRL_CMPWIN1SRC(x)                SIM_MISCTRL0_CMPWIN1SRC(x)
#define SIM_MISCTRL_CMPWIN2SRC_MASK              SIM_MISCTRL0_CMPWIN2SRC_MASK
#define SIM_MISCTRL_CMPWIN2SRC_SHIFT             SIM_MISCTRL0_CMPWIN2SRC_SHIFT
#define SIM_MISCTRL_CMPWIN2SRC_WIDTH             SIM_MISCTRL0_CMPWIN2SRC_WIDTH
#define SIM_MISCTRL_CMPWIN2SRC(x)                SIM_MISCTRL0_CMPWIN2SRC(x)
#define SIM_MISCTRL_CMPWIN3SRC_MASK              SIM_MISCTRL0_CMPWIN3SRC_MASK
#define SIM_MISCTRL_CMPWIN3SRC_SHIFT             SIM_MISCTRL0_CMPWIN3SRC_SHIFT
#define SIM_MISCTRL_CMPWIN3SRC_WIDTH             SIM_MISCTRL0_CMPWIN3SRC_WIDTH
#define SIM_MISCTRL_CMPWIN3SRC(x)                SIM_MISCTRL0_CMPWIN3SRC(x)
#define SIM_MISCTRL_EWMINSRC_MASK                SIM_MISCTRL0_EWMINSRC_MASK
#define SIM_MISCTRL_EWMINSRC_SHIFT               SIM_MISCTRL0_EWMINSRC_SHIFT
#define SIM_MISCTRL_EWMINSRC_WIDTH               SIM_MISCTRL0_EWMINSRC_WIDTH
#define SIM_MISCTRL_EWMINSRC(x)                  SIM_MISCTRL0_EWMINSRC(x)
#define SIM_MISCTRL_DACTRIGSRC_MASK              SIM_MISCTRL0_DACTRIGSRC_MASK
#define SIM_MISCTRL_DACTRIGSRC_SHIFT             SIM_MISCTRL0_DACTRIGSRC_SHIFT
#define SIM_MISCTRL_DACTRIGSRC_WIDTH             SIM_MISCTRL0_DACTRIGSRC_WIDTH
#define SIM_MISCTRL_DACTRIGSRC(x)                SIM_MISCTRL0_DACTRIGSRC(x)
#define SIM_MISCTRL2                             SIM_MISCTRL1
#define SIM_MISCTRL2_REG(base)                   SIM_MISCTRL1_REG(base)
#define SIM_MISCTRL2_SYNCXBARAPITTRIG0_MASK      SIM_MISCTRL1_SYNCXBARAPITTRIG0_MASK
#define SIM_MISCTRL2_SYNCXBARAPITTRIG0_SHIFT     SIM_MISCTRL1_SYNCXBARAPITTRIG0_SHIFT
#define SIM_MISCTRL2_SYNCXBARAPITTRIG0_WIDTH     SIM_MISCTRL1_SYNCXBARAPITTRIG0_WIDTH
#define SIM_MISCTRL2_SYNCXBARAPITTRIG0(x)        SIM_MISCTRL1_SYNCXBARAPITTRIG0(x)
#define SIM_MISCTRL2_SYNCXBARAPITTRIG1_MASK      SIM_MISCTRL1_SYNCXBARAPITTRIG1_MASK
#define SIM_MISCTRL2_SYNCXBARAPITTRIG1_SHIFT     SIM_MISCTRL1_SYNCXBARAPITTRIG1_SHIFT
#define SIM_MISCTRL2_SYNCXBARAPITTRIG1_WIDTH     SIM_MISCTRL1_SYNCXBARAPITTRIG1_WIDTH
#define SIM_MISCTRL2_SYNCXBARAPITTRIG1(x)        SIM_MISCTRL1_SYNCXBARAPITTRIG1(x)
#define SIM_MISCTRL2_SYNCXBARAPITTRIG2_MASK      SIM_MISCTRL1_SYNCXBARAPITTRIG2_MASK
#define SIM_MISCTRL2_SYNCXBARAPITTRIG2_SHIFT     SIM_MISCTRL1_SYNCXBARAPITTRIG2_SHIFT
#define SIM_MISCTRL2_SYNCXBARAPITTRIG2_WIDTH     SIM_MISCTRL1_SYNCXBARAPITTRIG2_WIDTH
#define SIM_MISCTRL2_SYNCXBARAPITTRIG2(x)        SIM_MISCTRL1_SYNCXBARAPITTRIG2(x)
#define SIM_MISCTRL2_SYNCXBARAPITTRIG3_MASK      SIM_MISCTRL1_SYNCXBARAPITTRIG3_MASK
#define SIM_MISCTRL2_SYNCXBARAPITTRIG3_SHIFT     SIM_MISCTRL1_SYNCXBARAPITTRIG3_SHIFT
#define SIM_MISCTRL2_SYNCXBARAPITTRIG3_WIDTH     SIM_MISCTRL1_SYNCXBARAPITTRIG3_WIDTH
#define SIM_MISCTRL2_SYNCXBARAPITTRIG3(x)        SIM_MISCTRL1_SYNCXBARAPITTRIG3(x)
#define SIM_MISCTRL2_SYNCXBARBPITTRIG0_MASK      SIM_MISCTRL1_SYNCXBARBPITTRIG0_MASK
#define SIM_MISCTRL2_SYNCXBARBPITTRIG0_SHIFT     SIM_MISCTRL1_SYNCXBARBPITTRIG0_SHIFT
#define SIM_MISCTRL2_SYNCXBARBPITTRIG0_WIDTH     SIM_MISCTRL1_SYNCXBARBPITTRIG0_WIDTH
#define SIM_MISCTRL2_SYNCXBARBPITTRIG0(x)        SIM_MISCTRL1_SYNCXBARBPITTRIG0(x)
#define SIM_MISCTRL2_SYNCXBARBPITTRIG1_MASK      SIM_MISCTRL1_SYNCXBARBPITTRIG1_MASK
#define SIM_MISCTRL2_SYNCXBARBPITTRIG1_SHIFT     SIM_MISCTRL1_SYNCXBARBPITTRIG1_SHIFT
#define SIM_MISCTRL2_SYNCXBARBPITTRIG1_WIDTH     SIM_MISCTRL1_SYNCXBARBPITTRIG1_WIDTH
#define SIM_MISCTRL2_SYNCXBARBPITTRIG1(x)        SIM_MISCTRL1_SYNCXBARBPITTRIG1(x)
#define SIM_MISCTRL2_SYNCDACHWTRIG_MASK          SIM_MISCTRL1_SYNCDACHWTRIG_MASK
#define SIM_MISCTRL2_SYNCDACHWTRIG_SHIFT         SIM_MISCTRL1_SYNCDACHWTRIG_SHIFT
#define SIM_MISCTRL2_SYNCDACHWTRIG_WIDTH         SIM_MISCTRL1_SYNCDACHWTRIG_WIDTH
#define SIM_MISCTRL2_SYNCDACHWTRIG(x)            SIM_MISCTRL1_SYNCDACHWTRIG(x)
#define SIM_MISCTRL2_SYNCEWMIN_MASK              SIM_MISCTRL1_SYNCEWMIN_MASK
#define SIM_MISCTRL2_SYNCEWMIN_SHIFT             SIM_MISCTRL1_SYNCEWMIN_SHIFT
#define SIM_MISCTRL2_SYNCEWMIN_WIDTH             SIM_MISCTRL1_SYNCEWMIN_WIDTH
#define SIM_MISCTRL2_SYNCEWMIN(x)                SIM_MISCTRL1_SYNCEWMIN(x)
#define SIM_MISCTRL2_SYNCCMP0SAMPLEWIN_MASK      SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_MASK
#define SIM_MISCTRL2_SYNCCMP0SAMPLEWIN_SHIFT     SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_SHIFT
#define SIM_MISCTRL2_SYNCCMP0SAMPLEWIN_WIDTH     SIM_MISCTRL1_SYNCCMP0SAMPLEWIN_WIDTH
#define SIM_MISCTRL2_SYNCCMP0SAMPLEWIN(x)        SIM_MISCTRL1_SYNCCMP0SAMPLEWIN(x)
#define SIM_MISCTRL2_SYNCCMP1SAMPLEWIN_MASK      SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_MASK
#define SIM_MISCTRL2_SYNCCMP1SAMPLEWIN_SHIFT     SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_SHIFT
#define SIM_MISCTRL2_SYNCCMP1SAMPLEWIN_WIDTH     SIM_MISCTRL1_SYNCCMP1SAMPLEWIN_WIDTH
#define SIM_MISCTRL2_SYNCCMP1SAMPLEWIN(x)        SIM_MISCTRL1_SYNCCMP1SAMPLEWIN(x)
#define SIM_MISCTRL2_SYNCCMP2SAMPLEWIN_MASK      SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_MASK
#define SIM_MISCTRL2_SYNCCMP2SAMPLEWIN_SHIFT     SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_SHIFT
#define SIM_MISCTRL2_SYNCCMP2SAMPLEWIN_WIDTH     SIM_MISCTRL1_SYNCCMP2SAMPLEWIN_WIDTH
#define SIM_MISCTRL2_SYNCCMP2SAMPLEWIN(x)        SIM_MISCTRL1_SYNCCMP2SAMPLEWIN(x)
#define SIM_MISCTRL2_SYNCCMP3SAMPLEWIN_MASK      SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_MASK
#define SIM_MISCTRL2_SYNCCMP3SAMPLEWIN_SHIFT     SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_SHIFT
#define SIM_MISCTRL2_SYNCCMP3SAMPLEWIN_WIDTH     SIM_MISCTRL1_SYNCCMP3SAMPLEWIN_WIDTH
#define SIM_MISCTRL2_SYNCCMP3SAMPLEWIN(x)        SIM_MISCTRL1_SYNCCMP3SAMPLEWIN(x)
#define SPI_BASE                                 SPI0_BASE
#define SPI                                      SPI0
#define SPI_BASE_PTR                             SPI0_BASE_PTR
#define SPI_MCR                                  SPI0_MCR
#define SPI_TCR                                  SPI0_TCR
#define SPI_CTAR0                                SPI0_CTAR0
#define SPI_CTAR0_SLAVE                          SPI0_CTAR0_SLAVE
#define SPI_CTAR1                                SPI0_CTAR1
#define SPI_SR                                   SPI0_SR
#define SPI_RSER                                 SPI0_RSER
#define SPI_PUSHR                                SPI0_PUSHR
#define SPI_PUSHR_SLAVE                          SPI0_PUSHR_SLAVE
#define SPI_POPR                                 SPI0_POPR
#define SPI_TXFR0                                SPI0_TXFR0
#define SPI_TXFR1                                SPI0_TXFR1
#define SPI_TXFR2                                SPI0_TXFR2
#define SPI_TXFR3                                SPI0_TXFR3
#define SPI_RXFR0                                SPI0_RXFR0
#define SPI_RXFR1                                SPI0_RXFR1
#define SPI_RXFR2                                SPI0_RXFR2
#define SPI_RXFR3                                SPI0_RXFR3
#define SPI_CTAR(index2)                         SPI0_CTAR(index2)
#define SPI_CTAR_SLAVE(index2)                   SPI0_CTAR_SLAVE(index2)

/*!
 * @}
 */ /* end of group Backward_Compatibility_Symbols */


#else /* #if !defined(MKV46F16_H_) */
  /* There is already included the same memory map. Check if it is compatible (has the same major version) */
  #if (MCU_MEM_MAP_VERSION != 0x0100u)
    #if (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING))
      #warning There are included two not compatible versions of memory maps. Please check possible differences.
    #endif /* (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING)) */
  #endif /* (MCU_MEM_MAP_VERSION != 0x0100u) */
#endif  /* #if !defined(MKV46F16_H_) */

/* MKV46F16.h, eof. */
