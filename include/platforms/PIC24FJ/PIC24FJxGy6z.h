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
 * @file libohiboard/include/platforms/PIC24FJ/PIC24FJxGy6z.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief Register definitions for PIC24FJ Series.
 */

#ifndef __REGISTERS_PIC24FJ1024G_H
#define __REGISTERS_PIC24FJ1024G_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    volatile uint16_t WREG0;
    volatile uint16_t WREG1;
    volatile uint16_t WREG2;
    volatile uint16_t WREG3;
    volatile uint16_t WREG4;
    volatile uint16_t WREG5;
    volatile uint16_t WREG6;
    volatile uint16_t WREG7;
    volatile uint16_t WREG8;
    volatile uint16_t WREG9;
    volatile uint16_t WREG10;
    volatile uint16_t WREG11;
    volatile uint16_t WREG12;
    volatile uint16_t WREG13;
    volatile uint16_t WREG14;
    volatile uint16_t WREG15;
    volatile uint16_t SPLIM;
    volatile uint16_t PCL;
    volatile uint16_t PCH;
    volatile uint16_t DSRPAG;
    volatile uint16_t DSWPAG;
    volatile uint16_t RCOUNT;
    volatile uint16_t SR;
    volatile uint16_t CORCON;
    volatile uint16_t DISICNT;
    volatile uint16_t TBLPAG;
} CPU_TypeDef;

typedef struct
{
    volatile uint16_t INTCON1;
    volatile uint16_t INTCON2;
    volatile uint16_t INTCON3; //it is not defined on datasheet
    volatile uint16_t INTCON4;
    volatile uint16_t IFS[8];
    volatile uint16_t IEC[8];
    volatile uint16_t IPC[30];
    volatile uint16_t INTTREG;
} INTERRUPT_TypeDef;
    
typedef struct
{
    volatile uint16_t OSCCON;
    volatile uint16_t CLKDIV;
    volatile uint16_t _padding;
    volatile uint16_t OSCTUN;
    volatile uint16_t DCOTUN;
    volatile uint16_t DCOCON;
    volatile uint16_t OSCDIV;
    volatile uint16_t OSCFDIV;
} OSCILLATOR_TypeDef;

typedef struct
{
    volatile uint16_t RCON;
} RESET_TypeDef;

typedef struct
{
    volatile uint16_t HLVDCON;
} HLVD_TypeDef;

typedef struct
{
    volatile uint16_t REFOCONL;
    volatile uint16_t REFOCONH;
    volatile uint16_t REFOTRIML;
} REFO_TypeDef;

typedef struct
{
    volatile uint16_t PMD1;
    volatile uint16_t PMD2;
    volatile uint16_t PMD3;
    volatile uint16_t PMD4;
    volatile uint16_t PMD5;
    volatile uint16_t PMD6;
    volatile uint16_t PMD7;
    volatile uint16_t PMD8;
} PMD_Typedef;

typedef struct
{
    volatile uint16_t TMR1;
    volatile uint16_t PR1;
    volatile uint16_t T1CON;
} LPTIM_TypeDef;

typedef struct
{
    volatile uint16_t TMRx;
	volatile uint16_t TMRyHLD;
	volatile uint16_t TMRy;
    volatile uint16_t PRx;
	volatile uint16_t PRy;
    volatile uint16_t TxCON;
	volatile uint16_t TyCON;
} TMR_TypeDef;

typedef struct
{
    volatile uint16_t ANCFG;
} ANCFG_TypeDef;

typedef struct
{
    volatile uint16_t RTCCON1L;
    volatile uint16_t RTCCON1H;
    volatile uint16_t RTCCON2L;
    volatile uint16_t RTCCON2H;
    volatile uint16_t RTCCON3L;
    volatile uint16_t RTCSTATL;
    volatile uint16_t TIMEL;
    volatile uint16_t TIMEH;
    volatile uint16_t DATEL;
    volatile uint16_t DATEH;
    volatile uint16_t ALMTIMEL;
    volatile uint16_t ALMTIMEH;
    volatile uint16_t ALMDATEL;
    volatile uint16_t ALMDATEH;
    volatile uint16_t TSATIMEL;
    volatile uint16_t TSATIMEH;
    volatile uint16_t TSADATEL;
    volatile uint16_t TSADATEH;
} RTC_TypeDef;

typedef struct
{
    volatile uint16_t UMODE;
    volatile uint16_t USTA;
    volatile uint16_t UTXREG;
    volatile uint16_t URXREG;
    volatile uint16_t UBRG;
    volatile uint16_t UADMD;
} UART_TypeDef;

typedef struct
{
    volatile uint16_t SPICON1L;
	volatile uint16_t SPICON1H;
	volatile uint16_t SPICON2L;
    volatile uint16_t SPICON2H; //it is not defined on datasheet
    volatile uint16_t SPISTATL;
	volatile uint16_t SPISTATH;
    volatile uint16_t SPIBUFL;
	volatile uint16_t SPIBUFH;
	volatile uint16_t SPIBRGL;
    volatile uint16_t SPIBRGH; //it is not defined on datasheet
	volatile uint16_t SPIIMSK1;
    volatile uint16_t SPIIMSK2;
	volatile uint16_t SPIURDTL;
    volatile uint16_t SPIURDTH;
} SPI_TypeDef;

typedef struct
{
    volatile uint16_t I2CRCV;
	volatile uint16_t I2CTRN;
	volatile uint16_t I2CBRG;
    volatile uint16_t I2CCON1;
	volatile uint16_t I2CCON2;
    volatile uint16_t I2CSTAT;
	volatile uint16_t I2CADD;
	volatile uint16_t I2CMSK;
} I2C_TypeDef;

typedef struct
{
    volatile uint16_t PADCON;
	volatile uint16_t IOCSTAT;
} IO_TypeDef;

typedef struct
{
    volatile uint16_t TRIS; //Port direction
	volatile uint16_t PORT; //Port data
    volatile uint16_t LAT; //Port Latch
    volatile uint16_t ODC; //Open-Drain Control register
    volatile uint16_t ANS; //Configuring Analog Port Pins
    volatile uint16_t IOCP; //Enabling Interrupt-on-Change for RISING edge
    volatile uint16_t IOCN; //Enabling Interrupt-on-Change for FALLING edge
    volatile uint16_t IOCF; //Flag for Interrupt-on-Change
    volatile uint16_t IOCPU; //Enabling Pull Up
    volatile uint16_t IOCPD; //Enabling Pull Down
} GPIO_TypeDef;

typedef struct
{
    volatile uint16_t RPINR[30];
    volatile uint16_t _padding[4];
    volatile uint16_t RPOR[16];
} PPS_TypeDef;

typedef struct
{
    volatile uint16_t ADC1BUF[26];
    volatile uint16_t AD1CON1;
    volatile uint16_t AD1CON2;
    volatile uint16_t AD1CON3;
    volatile uint16_t AD1CHS;
    volatile uint16_t AD1CSSH;
    volatile uint16_t AD1CSSL;
    volatile uint16_t AD1CON4;
    volatile uint16_t AD1CON5;
    volatile uint16_t AD1CHITH;
    volatile uint16_t AD1CHITL;
    volatile uint16_t AD1CTMENH;
    volatile uint16_t AD1CTMENL;
    volatile uint16_t AD1RESDMA;
} ADC_TypeDef;

#define CPU_BASE            (0x0000ul)
#define INTERRUPT_BASE      (0x0080ul)
#define OSCILLATOR_BASE     (0x0100ul)
#define RESET_BASE          (0x0110ul)
#define HLVD_BASE           (0x0114ul)
#define REFO_BASE           (0x0168ul)
#define PMD_BASE            (0x0178ul)
#define LPTIM_BASE          (0x0190ul)
#define TMR23_BASE          (0x0196ul)
#define TMR45_BASE          (0x01A4ul)
#define RTC_BASE            (0x01CCul)
#define ANCFG_BASE          (0x02F4ul)
#define UART1_BASE          (0x0398ul)
#define UART2_BASE          (0x03AEul)
#define UART3_BASE          (0x03C4ul)
#define UART4_BASE          (0x03D0ul)
#define UART5_BASE          (0x03DCul)
#define UART6_BASE          (0x03E8ul)
#define SPI1_BASE           (0x03F4ul)
#define SPI2_BASE           (0x0410ul)
#define SPI3_BASE           (0x042Cul)
#define I2C1_BASE           (0x0494ul)
#define I2C2_BASE           (0x04A4ul)
#define I2C3_BASE           (0x04B4ul)
#define IO_BASE             (0x065Eul)
#define GPIOA_BASE          (0x0662ul)
#define GPIOB_BASE          (0x0676ul)
#define GPIOC_BASE          (0x068Aul)
#define GPIOD_BASE          (0x069Eul)
#define GPIOE_BASE          (0x06B2ul)
#define GPIOF_BASE          (0x06C6ul)
#define GPIOG_BASE          (0x06DAul)
#define ADC1_BASE           (0x0712ul)
#define PPS_BASE            (0x0790ul)

#define CPU                 ((CPU_TypeDef *) CPU_BASE)
#define INTERRUPT           ((INTERRUPT_TypeDef *) INTERRUPT_BASE)
#define OSCILLATOR          ((OSCILLATOR_TypeDef *) OSCILLATOR_BASE)
#define RESET               ((RESET_TypeDef *) RESET_BASE)
#define HLVD                ((HLVD_TypeDef *) HLVD_BASE)
#define REFO                ((REFO_TypeDef *)REFO_BASE)
#define PMD                 ((PMD_Typedef *) PMD_BASE)
#define LPTIM               ((LPTIM_TypeDef *) LPTIM_BASE)
#define TMR23               ((TMR_TypeDef *) TMR23_BASE)
#define TMR45               ((TMR_TypeDef *) TMR45_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define ANCFG1              ((ANCFG_TypeDef *) ANCFG_BASE)
#define UART1               ((UART_TypeDef *) UART1_BASE)
#define UART2               ((UART_TypeDef *) UART2_BASE)
#define UART3               ((UART_TypeDef *) UART3_BASE)
#define UART4               ((UART_TypeDef *) UART4_BASE)
#define UART5               ((UART_TypeDef *) UART5_BASE)
#define UART6               ((UART_TypeDef *) UART6_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define IO                  ((IO_TypeDef *) IO_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define PPS                 ((PPS_TypeDef *) PPS_BASE)

#ifdef __cplusplus
}
#endif

#endif // __REGISTERS_PIC24FJ1024G_H

