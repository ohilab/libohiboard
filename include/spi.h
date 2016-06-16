/******************************************************************************
 * Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *  Nicola Orlandini <n.orlandini90@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/include/spi.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Nicola Orlandini <n.orlandini90@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief SPI definitions and prototypes
 */

#ifdef LIBOHIBOARD_SPI

#ifndef __SPI_H
#define __SPI_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

#define HIGH 1
#define LOW  0

typedef enum {
    SPI_MASTER_MODE,
    SPI_SLAVE_MODE
} Spi_DeviceType;

typedef enum {
    SPI_TX_BUFFER,
    SPI_RX_BUFFER,
} Spi_BufferType;

#if defined (LIBOHIBOARD_K10D10)      || \
    defined (LIBOHIBOARD_K12D5)       || \
    defined (LIBOHIBOARD_K60DZ10)     || \
    defined (LIBOHIBOARD_OHIBOARD_R1) || \
    defined (LIBOHIBOARD_K64F12)      || \
    defined (LIBOHIBOARD_FRDMK64F)    || \
    defined (LIBOHIBOARD_KV46F)       || \
    defined (LIBOHIBOARD_TRWKV46F)

typedef enum {
    SPI_CONTINUOUS_SCK,                            /**< Set to Continuous SCK */
    SPI_NOT_CONTINUOUS_SCK                     /**< Set to not Continuous SCK */
} Spi_ContinousSck;

typedef enum {
    SPI_CS_NONE = 0x0,
    SPI_CS_0    = 0x1,
    SPI_CS_1    = 0x2,
    SPI_CS_2    = 0x4,
    SPI_CS_3    = 0x8,
    SPI_CS_4    = 0xF,
} Spi_ChiSelect;

typedef enum {
    SPI_CS_FORMAT_DISCONTINUOS = 0x0,
    SPI_CS_FORMAT_CONTINUOS    = 0x1,
} Spi_CSFormat;

#endif


typedef enum {
	SPI_SCK_INACTIVE_STATE_LOW,
	SPI_SCK_INACTIVE_STATE_HIGH,
} Spi_ClockPolarity;

typedef enum {
    SPI_SCK_LEADING_EDGE_DATA_CAPTURED,
    SPI_SCK_LEADING_EDGE_DATA_CHANGED,
} Spi_ClockPhase;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    SPI_PINS_PTA14,

    SPI_PINS_PTB10,

    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,

    SPI_PINS_PTE4,
    SPI_PINS_PTE16,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    SPI_PINS_PTA14,

    SPI_PINS_PTB10,

    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,

    SPI_PINS_PTE4,

#elif defined (LIBOHIBOARD_K10D10)

    SPI_PINS_PTA14,

    SPI_PINS_PTB9,
    SPI_PINS_PTB10,
    SPI_PINS_PTB20,
    SPI_PINS_PTB23,

    SPI_PINS_PTC0,
    SPI_PINS_PTC1,
    SPI_PINS_PTC2,
    SPI_PINS_PTC3,
    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,
    SPI_PINS_PTD5,
    SPI_PINS_PTD6,
    SPI_PINS_PTD11,
    SPI_PINS_PTD15,

    SPI_PINS_PTE0,
    SPI_PINS_PTE4,
    SPI_PINS_PTE5,
    SPI_PINS_PTE6,
    SPI_PINS_PTE16,

#elif defined (LIBOHIBOARD_K12D5)

    SPI_PINS_PTA14,

    SPI_PINS_PTB10,

    SPI_PINS_PTC0,
    SPI_PINS_PTC1,
    SPI_PINS_PTC2,
    SPI_PINS_PTC3,
    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,
    SPI_PINS_PTD5,
    SPI_PINS_PTD6,

    SPI_PINS_PTE0,
    SPI_PINS_PTE4,
    SPI_PINS_PTE5,
    SPI_PINS_PTE16,


#elif defined (LIBOHIBOARD_OHIBOARD_R1)

    SPI_PINS_PTA14,

    SPI_PINS_PTB9,
    SPI_PINS_PTB10,
    SPI_PINS_PTB20,
    SPI_PINS_PTB23,

    SPI_PINS_PTC0,
    SPI_PINS_PTC1,
    SPI_PINS_PTC2,
    SPI_PINS_PTC3,
    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,
    SPI_PINS_PTD5,
    SPI_PINS_PTD6,

    SPI_PINS_PTE0,
    SPI_PINS_PTE4,
    SPI_PINS_PTE5,
    SPI_PINS_PTE6,

#elif defined (LIBOHIBOARD_K60DZ10)

    SPI_PINS_PTA14,

    SPI_PINS_PTB9,
    SPI_PINS_PTB10,
    SPI_PINS_PTB20,
    SPI_PINS_PTB23,

    SPI_PINS_PTC0,
    SPI_PINS_PTC1,
    SPI_PINS_PTC2,
    SPI_PINS_PTC3,
    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4,
    SPI_PINS_PTD5,
    SPI_PINS_PTD6,
    SPI_PINS_PTD11,
    SPI_PINS_PTD15,

    SPI_PINS_PTE0,
    SPI_PINS_PTE4,
    SPI_PINS_PTE5,
    SPI_PINS_PTE6,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SPI_PINS_PTA14,

	SPI_PINS_PTB9,
	SPI_PINS_PTB10,
	SPI_PINS_PTB20,

	SPI_PINS_PTC0,
	SPI_PINS_PTC1,
	SPI_PINS_PTC2,
	SPI_PINS_PTC3,
	SPI_PINS_PTC4,

	SPI_PINS_PTD0,
	SPI_PINS_PTD4,
	SPI_PINS_PTD5,
	SPI_PINS_PTD6,
	SPI_PINS_PTD11,
	SPI_PINS_PTD15,

	SPI_PINS_PTE0,
	SPI_PINS_PTE4,
	SPI_PINS_PTE5,
	SPI_PINS_PTE6,

#elif defined (LIBOHIBOARD_KV46F)    || \
      defined (LIBOHIBOARD_TRWKV46F)

    SPI_PINS_PTA14,

    SPI_PINS_PTB23,

    SPI_PINS_PTC0,
    SPI_PINS_PTC1,
    SPI_PINS_PTC2,
    SPI_PINS_PTC3,
    SPI_PINS_PTC4,

    SPI_PINS_PTD0,
    SPI_PINS_PTD4_PCS0,
    SPI_PINS_PTD4_PCS1,
    SPI_PINS_PTD5_PCS,
    SPI_PINS_PTD6_PCS,

    SPI_PINS_PTE16,

#endif

	SPI_PINS_PCSNONE,
} Spi_PcsPins;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    SPI_PINS_PTA16O,
    SPI_PINS_PTA17O,

    SPI_PINS_PTB16O,
    SPI_PINS_PTB17O,

    SPI_PINS_PTC6O,
    SPI_PINS_PTC7O,

    SPI_PINS_PTD2O,
    SPI_PINS_PTD3O,
    SPI_PINS_PTD6O,
    SPI_PINS_PTD7O,

    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,
    SPI_PINS_PTE18O,
    SPI_PINS_PTE19O,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    SPI_PINS_PTA16O,
    SPI_PINS_PTA17O,

    SPI_PINS_PTB16O,
    SPI_PINS_PTB17O,

    SPI_PINS_PTC6O,
    SPI_PINS_PTC7O,

    SPI_PINS_PTD2O,
    SPI_PINS_PTD3O,
    SPI_PINS_PTD6O,
    SPI_PINS_PTD7O,

    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,

#elif defined (LIBOHIBOARD_K10D10)

    SPI_PINS_PTA16,

    SPI_PINS_PTB16,
    SPI_PINS_PTB22,

    SPI_PINS_PTC6,

    SPI_PINS_PTD2,
    SPI_PINS_PTD13,

    SPI_PINS_PTE18,
    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,

#elif defined (LIBOHIBOARD_K12D5)

    SPI_PINS_PTA16,

    SPI_PINS_PTB16,

    SPI_PINS_PTC6,

    SPI_PINS_PTD2,

    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,
    SPI_PINS_PTE18,

#elif defined (LIBOHIBOARD_OHIBOARD_R1)

    SPI_PINS_PTA16,

    SPI_PINS_PTB16,
    SPI_PINS_PTB22,

    SPI_PINS_PTC6,

    SPI_PINS_PTD2,

    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,

#elif defined (LIBOHIBOARD_K60DZ10)

    SPI_PINS_PTA16,

    SPI_PINS_PTB16,
    SPI_PINS_PTB22,

    SPI_PINS_PTC6,

    SPI_PINS_PTD2,
    SPI_PINS_PTD13,

    SPI_PINS_PTE1O,
    SPI_PINS_PTE3O,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

	SPI_PINS_PTA16,

	SPI_PINS_PTB16,
	SPI_PINS_PTB22,

	SPI_PINS_PTC6,

	SPI_PINS_PTD2,
	SPI_PINS_PTD13,

	SPI_PINS_PTE1,

#elif defined (LIBOHIBOARD_KV46F)    || \
      defined (LIBOHIBOARD_TRWKV46F)

	SPI_PINS_PTA16,
	SPI_PINS_PTC6,

	SPI_PINS_PTD2,
	SPI_PINS_PTD6_SOUT,

	SPI_PINS_PTE18,

#endif

	SPI_PINS_SOUTNONE,
} Spi_SoutPins;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    SPI_PINS_PTA16I,
    SPI_PINS_PTA17I,

    SPI_PINS_PTB16I,
    SPI_PINS_PTB17I,

    SPI_PINS_PTC6I,
    SPI_PINS_PTC7I,

    SPI_PINS_PTD2I,
    SPI_PINS_PTD3I,
    SPI_PINS_PTD6I,
    SPI_PINS_PTD7I,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,
    SPI_PINS_PTE18I,
    SPI_PINS_PTE19I,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    SPI_PINS_PTA16I,
    SPI_PINS_PTA17I,

    SPI_PINS_PTB16I,
    SPI_PINS_PTB17I,

    SPI_PINS_PTC6I,
    SPI_PINS_PTC7I,

    SPI_PINS_PTD2I,
    SPI_PINS_PTD3I,
    SPI_PINS_PTD6I,
    SPI_PINS_PTD7I,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,

#elif defined (LIBOHIBOARD_K10D10)

    SPI_PINS_PTA17,

    SPI_PINS_PTB17,
    SPI_PINS_PTB23I,

    SPI_PINS_PTC7,

    SPI_PINS_PTD3,
    SPI_PINS_PTD14,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,
    SPI_PINS_PTE19,

#elif defined (LIBOHIBOARD_K12D5)

    SPI_PINS_PTA17,

    SPI_PINS_PTB17,

    SPI_PINS_PTC7,

    SPI_PINS_PTD3,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,
    SPI_PINS_PTE19,

#elif defined (LIBOHIBOARD_OHIBOARD_R1)

    SPI_PINS_PTA17,

    SPI_PINS_PTB17,
    SPI_PINS_PTB23I,

    SPI_PINS_PTC7,

    SPI_PINS_PTD3,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,

#elif defined (LIBOHIBOARD_K60DZ10)

    SPI_PINS_PTA17,

    SPI_PINS_PTB17,
    SPI_PINS_PTB23I,

    SPI_PINS_PTC7,

    SPI_PINS_PTD3,
    SPI_PINS_PTD14,

    SPI_PINS_PTE1I,
    SPI_PINS_PTE3I,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SPI_PINS_PTA17,

	SPI_PINS_PTB17,
	SPI_PINS_PTB23,

	SPI_PINS_PTC7,

	SPI_PINS_PTD3,
	SPI_PINS_PTD7,
	SPI_PINS_PTD14,

	SPI_PINS_PTE3,

#elif defined (LIBOHIBOARD_KV46F)    || \
      defined (LIBOHIBOARD_TRWKV46F)

	SPI_PINS_PTA17,

	SPI_PINS_PTC7,

	SPI_PINS_PTD3,

	SPI_PINS_PTE19,

#endif
	SPI_PINS_SINNONE,
} Spi_SinPins;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,
    SPI_PINS_PTD5,

    SPI_PINS_PTE2,
    SPI_PINS_PTE17,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,
    SPI_PINS_PTD5,

    SPI_PINS_PTE2,

#elif defined (LIBOHIBOARD_K10D10)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,
    SPI_PINS_PTB21,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,
    SPI_PINS_PTD12,

    SPI_PINS_PTE2,
    SPI_PINS_PTE17,

#elif defined (LIBOHIBOARD_K12D5)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,

    SPI_PINS_PTE2,
    SPI_PINS_PTE17,

#elif defined (LIBOHIBOARD_OHIBOARD_R1)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,
    SPI_PINS_PTB21,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,

    SPI_PINS_PTE2,

#elif defined (LIBOHIBOARD_K60DZ10)

    SPI_PINS_PTA15,

    SPI_PINS_PTB11,
    SPI_PINS_PTB21,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,
    SPI_PINS_PTD12,

    SPI_PINS_PTE2,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

	SPI_PINS_PTA15,

	SPI_PINS_PTB11,
	SPI_PINS_PTB21,

	SPI_PINS_PTC5,

	SPI_PINS_PTD1,
	SPI_PINS_PTD12,

	SPI_PINS_PTE2,

#elif defined (LIBOHIBOARD_KV46F)    || \
      defined (LIBOHIBOARD_TRWKV46F)

    SPI_PINS_PTA15,

    SPI_PINS_PTC5,

    SPI_PINS_PTD1,
    SPI_PINS_PTD5_CLK,

    SPI_PINS_PTE17,

#endif

	SPI_PINS_SCKNONE,
} Spi_SckPins;

typedef struct _Spi_Config
{
    Spi_PcsPins           pcs0Pin;
    Spi_PcsPins           pcs1Pin;
    Spi_PcsPins           pcs2Pin;
    Spi_PcsPins           pcs3Pin;
    Spi_PcsPins           pcs4Pin;
    Spi_SoutPins          soutPin;
    Spi_SinPins           sinPin;
    Spi_SckPins           sckPin;

    Spi_DeviceType        devType;
    uint32_t              baudrate;
#if defined (LIBOHIBOARD_K10D10)      || \
    defined (LIBOHIBOARD_K12D5)       || \
    defined (LIBOHIBOARD_K60DZ10)     || \
    defined (LIBOHIBOARD_OHIBOARD_R1) || \
    defined (LIBOHIBOARD_K64F12)      || \
    defined (LIBOHIBOARD_FRDMK64F)    || \
    defined (LIBOHIBOARD_KV46F)       || \
    defined (LIBOHIBOARD_TRWKV46F)

    uint32_t              frameSize;

    Spi_ContinousSck      continuousSck;

#endif

#if defined (LIBOHIBOARD_KV46F)       || \
	defined (LIBOHIBOARD_TRWKV46F)

    void (*callback)(void);
    bool mtfEnable;
    Spi_CSFormat csFormat;
    bool rxFifoEn;
    bool txFifoEn;
    bool rooeEn;

    Spi_PcsPins pcs5Pin;
    union _csInactiveState
    {
        uint8_t sc0 :1;
        uint8_t sc1 :1;
        uint8_t sc2 :1;
        uint8_t sc3 :1;
        uint8_t sc4 :1;
        uint8_t sc5 :1;
    }csInactiveState;

    union _intEventEn
    {
        uint8_t TCF   :1;
        uint8_t TXRXS :1;
        uint8_t EOQF  :1;
        uint8_t TFUF  :1;
        uint8_t TFFF  :1;
        uint8_t RFOF  :1;
        uint8_t RFDF  :1;
    }intEventEn;

#endif

    Spi_ClockPolarity     sckPolarity;
    Spi_ClockPhase        sckPhase;

} Spi_Config;

typedef struct Spi_Device* Spi_DeviceHandle;


System_Errors Spi_init (Spi_DeviceHandle dev, Spi_Config *config);
System_Errors Spi_setBaudrate(Spi_DeviceHandle dev, uint32_t speed);

#if defined (LIBOHIBOARD_KV46F)  ||\
    defined (LIBOHIBOARD_TRWKV46F)

System_Errors Spi_read (Spi_DeviceHandle dev, uint16_t * data);
System_Errors Spi_write (Spi_DeviceHandle dev, uint16_t data, Spi_ChiSelect cs);

System_Errors Spi_txEnDisable (Spi_DeviceHandle dev, bool enable);
System_Errors Spi_flushBuffer (Spi_DeviceHandle dev, Spi_BufferType buffer);




#else

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data);
System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data);
#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;

#elif defined (LIBOHIBOARD_K10D10)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;
extern Spi_DeviceHandle OB_SPI2;

#elif defined (LIBOHIBOARD_K12D5)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
extern Spi_DeviceHandle SPI2;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
extern Spi_DeviceHandle SPI2;

#elif defined (LIBOHIBOARD_KV46F)      || \
      defined (LIBOHIBOARD_TRWKV46F)

extern Spi_DeviceHandle OB_SPI0;

void  SPI0_IRQHandler(void);

#endif

#endif /* __SPI_H */

#endif /* LIBOHIBOARD_SPI */
