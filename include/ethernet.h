/* Copyright (C) 2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Simone Giacomucci <simone.giacomucci@gmail.com>
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/ethernet.h
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Ethernet HA� definitions and prototypes.
 */

#ifdef LIBOHIBOARD_ETHERNET

#ifndef __ETHERNET_HAL_H
#define __ETHERNET_HAL_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

/* RX Acceleration Configuration flags */
#define ETHERNET_RXACCEL_CONFIG_ENABLE_PADDLE_REMOVAL              0x01 /**< Enable Padding Removal For Short IP Frames. */
#define ETHERNET_RXACCEL_CONFIG_WRONG_IPV4_FRAME_DISCARD           0x02 /**< Enable Discard Of Frames With Wrong IPv4 Header Checksum. */
#define ETHERNET_RXACCEL_CONFIG_WRONG_PROTO_FRAME_DISCARD          0x04 /**< Enable Discard Of Frames With Wrong Protocol Checksum. */
#define ETHERNET_RXACCEL_CONFIG_MAC_ERROR_FRAME_DISCARD            0x40 /**< Enable Discard Of Frames With MAC Layer Errors. */
#define ETHERNET_RXACCEL_CONFIG_FIFO_SHIFT_16                      0x80 /**< Instructs the MAC to write two additional bytes in front
                                                                             of each frame received into the RX FIFO. */
/* TX Acceleration Configuration flags */
#define ETHERNET_TXACCEL_FIFO_SHIFT_16                             0x01 /**< Indicates to TX FIFO that there are two more bytes in front of data frame */
#define ETHERNET_TXACCEL_CONFIG_IP_HEADER_CHECKSUM_ENABLED         0x08 /**< If set inserts automatically an IP header checksum */
#define ETHERNET_TXACCEL_CONFIG_PROTO_CHECKSUM_ENABLED             0x10 /**< If set inserts automatically a checksum of a known protocol */

/* MAC Configuration flags*/
#define ETHERNET_MACCONFIG_STOP_MODE_ENABLE                        0x00001U /* ENET Stop mode enable*/
#define ETHERNET_MACCONFIG_DEBUG_MODE_ENABLE                       0x00002U /* Enable MAC to enter hardware freeze when enter debug mode*/
#define ETHERNET_MACCONFIG_PAYLOAD_CHECK_ENABLE                    0x00004U /* ENET receive payload length check enable*/
#define ETHERNET_MACCONFIG_RX_FLOW_CONTROL_ENABLE                  0x00008U /* Enable ENET flow control*/
#define ETHERNET_MACCONFIG_RX_CRC_FWD_ENABLE                       0x00010U /* Received frame CRC is stripped from the frame*/
#define ETHERNET_MACCONFIG_RX_PAUSE_FWD_ENABLE                     0x00020U /* Pause frames are forwarded to the user application*/
#define ETHERNET_MACCONFIG_RX_PAD_MOVE_ENABLE                      0x00040U /* Padding is removed from received frames*/
#define ETHERNET_MACCONFIG_RX_BC_REJECT_ENABLE                     0x00080U /* Broadcast frame reject*/
#define ETHERNET_MACCONFIG_RX_PROMISCUOUS_ENABLE                   0x00100U /* Promiscuous mode enabled*/
#define ETHERNET_MACCONFIG_TX_CRC_FWD_ENABLE                       0x00200U /* Enable transmit frame with the CRC from application*/
#define ETHERNET_MACCONFIG_MAC_ADDR_INSERT                         0x00400U /* Enable MAC address insert*/
#define ETHERNET_MACCONFIG_TX_ACCEL_ENABLE                         0x00800U /* Transmit accelerator enable*/
#define ETHERNET_MACCONFIG_RX_ACCEL_ENABLE                         0x01000U /* Transmit accelerator enable*/
#define ETHERNET_MACCONFIG_STORE_AND_FWD_ENABLE                    0x02000U /* Switcher to enable store and forward*/
#define ETHERNET_MACCONFIG_MAC_MIB_ENABLE                          0x04000U /* Disable MIB module*/
#define ETHERNET_MACCONFIG_SMI_PREAMBLE_DISABLE                    0x08000U /* Enable SMI preamble*/
#define ETHERNET_MACCONFIG_MAC_ENHANCED_ENABLE                     0x10000U /* Enable enhanced MAC feature (IEEE 1588 feature/enhanced buff descriptor)*/

/* Receive Buffer Descriptor Control Status flags */
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_EMPTY                 0x8000U /* Empty bit.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_SOFTWARE_OWN1         0x4000U /* Receive software owner.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_WRAP                  0x2000U /* Update buffer descriptor.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_SOFTWARE_OWN2         0x1000U /* Receive software owner.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_LAST                  0x0800U /* Last Buffer Descriptor in the frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_MISS                  0x0100U /* Receive for promiscuous mode.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_BROADCAST             0x0080U /* Broadcast.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_MULTICAST             0x0040U /* Multicast.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_LENGTH_VIOLATION      0x0020U /* Receive length violation.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_NO_OCTECT             0x0010U /* Receive non-octet aligned frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_CRC                   0x0004U /* Receive CRC error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_OVERRUN               0x0002U /* Receive FIFO overrun.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_TRUNC                 0x0001U /* Frame is truncated.*/

/* Receive Buffer Descriptor Control Extend 0 flags */
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_RX_IPV4                 0x0001U /* Ipv4 frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_RX_IPV6                 0x0002U /* Ipv6 frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_RX_VLAN                 0x0004U /* VLAN.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_RX_PROTO_CHECKSUM_ERR   0x0010U /* Protocol checksum error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_RX_IP_HDR_CHECKSUM_ERR  0x0020U /* IP header checksum error.*/

/* Receive Buffer Descriptor Control Extend 1 flags */
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_INTERRUPT            0x0080U /* Buffer Descriptor interrupt.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_UNICAST              0x0100U /* Unicast frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_COLLISION            0x0200U /* Buffer Descriptor collision.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_PHY_ERROR            0x0400U /* PHY error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_MAC_ERROR            0x8000U /* Mac error.*/

/* Transmit Buffer Descriptor Control Status flags */
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_READY                 0x8000U /* Ready bit.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_SOFTWARE_OWN1         0x4000U /* Transmit software owner.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_WRAP                  0x2000U /* Wrap buffer descriptor.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_SOFTWARE_OWN2         0x1000U /* Transmit software owner.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_LAST                  0x0800U /* Last BD in the frame.*/
#define ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_TRANSMIT_CRC          0x0400U /* Receive for transmit CRC.*/

/* Transmit Buffer Descriptor Control Extend 0 flags */
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_ERROR                0x8000U /* Transmit error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_UNDERFLOW_ERROR      0x2000U /* Underflow error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_EXCESS_COLLISION_ERR 0x1000U /* Excess collision error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_FRAME_ERROR          0x0800U /* Frame error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_LATE_COLLISION_ERR   0x0400U /* Late collision error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_OVERFLOW_ERR         0x0200U /* Overflow error.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT0_TX_TIMESTAMP_ERROR      0x0100U /* Timestamp error.*/

/* Transmit Buffer Descriptor Control Extend 1 flags */
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_TX_INTERRUPT            0x4000U /* Transmit interrupt.*/
#define ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_TX_TIMESTAMP            0x2000U /* Transmit timestamp flag.*/

/** The MII configuration status.
 *  1 if the MII has been configured.
 *  0 if the MII has not been configured.
 */
#define ETHERNET_STATUS_MII_CONFIG_FLAG                            0x1

typedef struct Ethernet_Device* Ethernet_DeviceHandle;

typedef enum
{
#if defined (LIBOHIBOARD_K60DZ10) || \
    defined (LIBOHIBOARD_OHIBOARD_R1)

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    ETHERNET_PINS_PTA5,
    ETHERNET_PINS_PTA12,
    ETHERNET_PINS_PTA13,
    ETHERNET_PINS_PTA14,
    ETHERNET_PINS_PTA15,
    ETHERNET_PINS_PTA16,
    ETHERNET_PINS_PTA17,
    ETHERNET_PINS_PTA28,

    ETHERNET_PINS_PTB0,
    ETHERNET_PINS_PTB1,

    ETHERNET_PINS_PTC16,
    ETHERNET_PINS_PTC17,
    ETHERNET_PINS_PTC18,
    ETHERNET_PINS_PTC19,

#endif
} Ethernet_Pins;

/**
 *
 * See pag. 1262 of K64 Sub-Family Reference Manual for more info.
 */
typedef struct Ethernet_BufferDescriptor
{
    uint16_t length;                       /**< Buffer descriptor data length. */
    uint16_t control;                          /**< Buffer descriptor control. */
    uint8_t  *buffer;                                /**< Data buffer pointer. */
    uint16_t controlExtend0;          /**< Extend buffer descriptor control 0. */
    uint16_t controlExtend1;          /**< Extend buffer descriptor control 1. */
    uint16_t payloadChecksum;                  /**< Internal payload checksum. */
    uint8_t  headerLength;                                 /**< Header length. */
    uint8_t  protocolType;                                 /**< Protocol type. */
    uint16_t reserved0;
    uint16_t controlExtend2;           /**< Extend buffer descriptor control2. */
    uint32_t timestamp;                                         /**< Timestamp.*/
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
    uint16_t reserved4;
} Ethernet_BufferDescriptor;

/**
 *
 */
typedef struct Ethernet_BufferDescriptors
{
    /**
     * The start address of Ethernet transmit buffer descriptors.
     * This address must always be evenly divisible by 16.
     */
    volatile Ethernet_BufferDescriptor* tx;
    /**
     * The transmit data buffer start address.
     * This address must always be evenly divisible by 16.
     */
    uint8_t *txBuffer;
    uint32_t txBdNumber;         /**< The transmit buffer descriptor numbers. */
    uint32_t txBufferSizeAlign;        /**< The aligned transmit buffer size. */

    /**
     * The start address of ENET receive buffer descriptors.
     * This address must always be evenly divisible by 16.
     */
    volatile Ethernet_BufferDescriptor* rx;
    /**
     * The receive data buffer start address.
     * This address must always be evenly divisible by 16.
     */
    uint8_t *rxBuffer;
    uint32_t rxBdNumber;          /**< The receive buffer descriptor numbers. */
    uint32_t rxBufferSizeAlign; /**< The aligned receive transmit buffer size.*/
} Ethernet_BufferDescriptors;

/**
 * TODO: description...
 */
typedef enum
{
    ETHERNET_MACOPERATINGMODE_NORMAL,/**< Normal operating mode for ENET MAC. */
    ETHERNET_MACOPERATINGMODE_SLEEP,            /**< Sleep mode for ENET MAC. */
} Ethernet_MacOperatingMode;

/**
 * TODO: description...
 */
typedef enum
{
    ETHERNET_MODE_NORMAL,
    ETHERNET_MODE_1588,
} Ethernet_Mode;

/**
 * TODO: description...
 */
typedef enum
{
    ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT = 0,
    ETHERNET_SMI_WRITE_FRAME_COMPLIANT     = 1,
    ETHERNET_SMI_READ_FRAME_COMPLIANT      = 2,
    ETHERNET_SMI_READ_FRAME_NOT_COMPLIANT  = 3,
} Ethernet_Smi;

/**
 * TODO: description...
 */
typedef struct _Ethernet_MacConfig
{
    uint8_t macAddress[6]; /**< MAC Physical Address. First byte in the array
                                    is Least Significant Byte in MAC Address. */
    Ethernet_MacOperatingMode mode;                  /**< MAC Operating Mode. */
    uint32_t control;                  /**< MAC Configuration & Control Bits. */

    uint8_t rxAccelerConfig; /**< RX Acceleration Configuration Bits. To be
                  initialized only if MACCONFIG_RX_ACCEL_ENABLED flag is set. */
    uint8_t txAccelerConfig; /**< TX Acceleration Configuration Bits. To be
                  initialized only if MACCONFIG_TX_ACCEL_ENABLED flag is set. */

    uint16_t pauseDuration; /**< Pause duration used in PAUSE frame. To be
                        initialized only if MACCONFIG_RX_FLOW_CONTROL_ENABLE. */

    uint16_t rxMaxFrameLength;/**< RX Maximum Frame Length. Values from 0 to 16383.*/
    uint16_t rxTruncLength; /**< RX Truncate Length, must be greater than or
                        equal to maximum frame length. Values from 0 to 16383.*/

    uint16_t txInterPacketGap;                      /**< TX Inter-Packet Gap. */

} Ethernet_MacConfig;

/**
 * TODO: description...
 */
typedef struct _Ethernet_FifoConfig
{
    /**
     * Transmit FIFO write. This should be set when isStoreForwardEnabled
     * is false. This field indicates the number of bytes in step of 64 bytes
     * written to the transmit FiFO before transmission of a frame begins
     */
    uint8_t txFifoWrite;

    /** Transmit FIFO section empty threshold, default zero. */
    uint8_t txFifoEmpty;
    /**
     * Transmit FIFO section almost empty threshold,
     * The minimum value of 4 should be set.
     */
    uint8_t txFifoAlmostEmpty;
    /**
     * Transmit FIFO section almost full threshold,
     * The minimum value of 6 is required a recommended
     * value of at least 8 should be set
     */
    uint8_t txFifoAlmostFull;

    /** Receive FIFO section full threshold, default zero. */
    uint8_t rxFifoFull;
    /**
     * Receive FIFO section almost full threshold,
     * The minimum value of 4 should be set.
     */
    uint8_t rxFifoAlmostFull;
    /** Receive FIFO section empty threshold, default zero. */
    uint8_t rxFifoEmpty;
    /**
     *  Receive FIFO section almost empty threshold,
     *  The minimum value of 4 should be set.
     */
    uint8_t rxFifoAlmostEmpty;
} Ethernet_FifoConfig;

/**
 * TODO: description...
 */
typedef struct _Ethernet_RmiiConfig
{
    bool isRmiiMode;                              /**< Otherwise is MII mode. */
    bool isSpeed10T;                            /**< Otherwise speed is 100T. */
    bool isFullDuplex;                         /**< Otherwise is half duplex. */
    bool isRxOnTxDisabled;                 /**< Disable receive and transmit. */
    bool isLoopEnabled;                                   /**< MII loop mode. */
} Ethernet_RmiiConfig;

/**
 * TODO: description...
 */
typedef struct _Ethernet_Config
{
    Ethernet_Mode mode;

    Ethernet_BufferDescriptors* bd;        /**< Buffer Descriptors Structure. */

    Ethernet_MacConfig mac;                   /**< MAC configuration section. */

    Ethernet_FifoConfig fifo;                /**< FIFO configuration section. */

    Ethernet_RmiiConfig rmii;            /**< RMII/MII configuration section. */

    bool isPtpSlaveEnabled;                   /**< Master or slave PTP timer. */
    uint32_t ptpClockIncease;    /**< Timer increase value each clock period. */
    uint32_t ptpPeriod;       /**< Timer period for generate interrupt event. */

    uint32_t internalSpeed;               /**< Operating speed of PHY device. */

    void (*callbackRxIsr)(void);/**< Callback Function to handle RX Interrupt.*/
    void (*callbackTxIsr)(void);/**< Callback function to Handle TX Interrupt.*/
    void (*callbackTsIsr)(void);    /**< Callback function to handle Timestamp
                                                                    Interrupt.*/
} Ethernet_Config;

/**
 * TODO: description...
 *
 * @param[in] dev    The ethernet device
 * @param[in] config TODO
 */
System_Errors Ethernet_init (Ethernet_DeviceHandle dev, Ethernet_Config *config);

void Ethernet_updateStatus (Ethernet_DeviceHandle dev);

/**
 * TODO: description
 *
 * @param[in] dev         The ethernet device
 * @param[in] phyAddress  TODO
 * @param[in] phyRegister TODO
 * @param[in] smi         TODO
 * @param[in] data        TODO
 */
System_Errors Ethernet_smiOperation (Ethernet_DeviceHandle dev,
                                     uint32_t phyAddress,
                                     uint32_t phyRegister,
                                     Ethernet_Smi smi,
                                     uint32_t* data);

/** Common functions **/
uint8_t* Ethernet_getBufferDescriptorData (volatile Ethernet_BufferDescriptor* bd);
void Ethernet_clearRxBufferDescriptorAfterHandled (volatile Ethernet_BufferDescriptor* bd);
void Ethernet_clearTxBufferDescriptorAfterSend (volatile Ethernet_BufferDescriptor* bd);
void Ethernet_setTxBufferDescriptorBeforeSend (
        volatile Ethernet_BufferDescriptor* bd,
        uint16_t dataLength,
        bool isTxTsConfigured,
        bool isTxCrcEnable,
        bool isLast);

#endif /* __ETHERNET_HAL_H */

#endif /* LIBOHIBOARD_ETHERNET */
