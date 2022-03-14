/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * For libohiboard modifications:
 * \author    Marco Giammarini
 * \author    Niccolo' Paolinelli
 */
#include <stdlib.h>
//#include "utilities.h"
//#include "board-config.h"
//#include "delay.h"
#include "libohiboard.h"
#include "radio.h"
#include "sx1276-board.h"

#if !defined (RADIO_RESET)
    #error "You MUST define RADIO_RESET"
#endif

#if !defined (RADIO_ANT_SWITCH_RX)
    #error "You MUST define RADIO_ANT_SWITCH_RX"
#endif

#if !defined (RADIO_ANT_SWITCH_TX_BOOST)
    #error "You MUST define RADIO_ANT_SWITCH_TX_BOOST"
#endif

#if !defined (RADIO_ANT_SWITCH_TX_RFO)
    #error "You MUST define RADIO_ANT_SWITCH_TX_RFO"
#endif

#if !defined (RADIO_TCXO_POWER)
    #error "You MUST define RADIO_TCXO_POWER"
#endif

#if defined( USE_RADIO_DEBUG )

#if !defined (RADIO_DBG_PIN_TX)
    #error "You MUST define RADIO_DBG_PIN_TX"
#endif

#if !defined (RADIO_DBG_PIN_RX)
    #error "You MUST define RADIO_DBG_PIN_RX"
#endif

#endif

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect( int8_t power );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * TCXO power control pin
 */
Gpio_Pins TcxoPower;

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_Pins AntSwitchRx;
Gpio_Pins AntSwitchTxBoost;
Gpio_Pins AntSwitchTxRfo;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_pins DbgPinTx;
Gpio_pins DbgPinRx;
#endif

void SX1276IoInit( void )
{
    SX1276.Spi.Nss = RADIO_NSS;
    SX1276.DIO0 = RADIO_DIO_0;
    SX1276.DIO1 = RADIO_DIO_1;
    SX1276.DIO2 = RADIO_DIO_2;
    SX1276.DIO3 = RADIO_DIO_3;
    SX1276.DIO4 = RADIO_DIO_4;
    SX1276.DIO5 = RADIO_DIO_5;

    Gpio_config(SX1276.Spi.Nss, GPIO_PINS_OUTPUT);
    Gpio_set(SX1276.Spi.Nss);

    Gpio_config(SX1276.DIO0, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
    Gpio_config(SX1276.DIO1, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
    Gpio_config(SX1276.DIO2, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
    Gpio_config(SX1276.DIO3, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
    Gpio_config(SX1276.DIO4, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
    Gpio_config(SX1276.DIO5, GPIO_PINS_INPUT | GPIO_PINS_PULL | GPIO_PINS_ENABLE_PULLUP);
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    Gpio_configInterrupt(SX1276.DIO0, INTERRUPT_PRIORITY_0, irqHandlers[0] );
    Gpio_enableInterrupt(SX1276.DIO0, GPIO_EVENT_USE_INTERRUPT | GPIO_EVENT_ON_RISING );
    Gpio_configInterrupt(SX1276.DIO1, INTERRUPT_PRIORITY_0, irqHandlers[1] );
    Gpio_enableInterrupt(SX1276.DIO1, GPIO_EVENT_ON_RISING | GPIO_EVENT_ON_FALLING | GPIO_EVENT_USE_INTERRUPT);
    Gpio_configInterrupt(SX1276.DIO2, INTERRUPT_PRIORITY_0, irqHandlers[2] );
    Gpio_enableInterrupt(SX1276.DIO2, GPIO_EVENT_ON_RISING | GPIO_EVENT_USE_INTERRUPT);
    Gpio_configInterrupt(SX1276.DIO3, INTERRUPT_PRIORITY_0, irqHandlers[3] );
    Gpio_enableInterrupt(SX1276.DIO3, GPIO_EVENT_ON_RISING | GPIO_EVENT_USE_INTERRUPT);
    Gpio_configInterrupt(SX1276.DIO4, INTERRUPT_PRIORITY_0, irqHandlers[4] );
    Gpio_enableInterrupt(SX1276.DIO4, GPIO_EVENT_ON_RISING | GPIO_EVENT_USE_INTERRUPT);
//    Gpio_configInterrupt(SX1276.DIO5, INTERRUPT_PRIORITY_0, irqHandlers[5] );
//    Gpio_enableInterrupt(SX1276.DIO5, GPIO_EVENT_ON_RISING | GPIO_EVENT_USE_INTERRUPT);
}

void SX1276IoDeInit( void )
{
    SX1276.Spi.Nss = RADIO_NSS;
    SX1276.DIO0 = RADIO_DIO_0;
    SX1276.DIO1 = RADIO_DIO_1;
    SX1276.DIO2 = RADIO_DIO_2;
    SX1276.DIO3 = RADIO_DIO_3;
    SX1276.DIO4 = RADIO_DIO_4;
    SX1276.DIO5 = RADIO_DIO_5;

    Gpio_config(SX1276.Spi.Nss, GPIO_PINS_OUTPUT);
    Gpio_set(SX1276.Spi.Nss);

    Gpio_configAlternate(SX1276.DIO0, GPIO_ALTERNATE_ANALOG, 0);
    Gpio_configAlternate(SX1276.DIO1, GPIO_ALTERNATE_ANALOG, 0);
    Gpio_configAlternate(SX1276.DIO2, GPIO_ALTERNATE_ANALOG, 0);
    Gpio_configAlternate(SX1276.DIO3, GPIO_ALTERNATE_ANALOG, 0);
    Gpio_configAlternate(SX1276.DIO4, GPIO_ALTERNATE_ANALOG, 0);
    Gpio_configAlternate(SX1276.DIO5, GPIO_ALTERNATE_ANALOG, 0);
}

void SX1276IoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )

    DbgPinTx = RADIO_DBG_PIN_TX;
    DbgPinRx = RADIO_DBG_PIN_RX;

    Gpio_config( DbgPinTx, GPIO_PINS_OUTPUT );
    Gpio_config( DbgPinRx, GPIO_PINS_OUTPUT );
#endif
}

void SX1276IoTcxoInit( void )
{
    TcxoPower = RADIO_TCXO_POWER;
    Gpio_config( TcxoPower, GPIO_PINS_OUTPUT);
}

void SX1276SetBoardTcxo( uint8_t state )
{
    if( state == true )
    {
        if( Gpio_get( TcxoPower ) == 0 )
        { // TCXO OFF power it up.
            // Power ON the TCXO
            Gpio_set( TcxoPower);
            System_delay( BOARD_TCXO_WAKEUP_TIME );
        }
    }
    else
    {
        // Power OFF the TCXO
        Gpio_clear( TcxoPower);
    }
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset( void )
{
    // Enables the TCXO if available on the board design
    SX1276SetBoardTcxo( true );

    // Set RESET pin to 0
    SX1276.Reset = RADIO_RESET;

    Gpio_config( SX1276.Reset, GPIO_PINS_OUTPUT );
    Gpio_clear(SX1276.Reset);

    // Wait 1 ms
    System_delay( 1 );

    // Configure RESET as input
    Gpio_configAlternate(SX1276.Reset, GPIO_ALTERNATE_ANALOG, 0);

    // Wait 6 ms
    System_delay( 6 );
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( power );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

static uint8_t SX1276GetPaSelect( int8_t power )
{
    if( power > 14 )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    AntSwitchRx = RADIO_ANT_SWITCH_RX;
    AntSwitchTxBoost = RADIO_ANT_SWITCH_TX_BOOST;
    AntSwitchTxRfo = RADIO_ANT_SWITCH_TX_RFO;

    Gpio_config( AntSwitchRx, GPIO_PINS_OUTPUT );
    Gpio_config( AntSwitchTxBoost,  GPIO_PINS_OUTPUT );
    Gpio_config( AntSwitchTxRfo, GPIO_PINS_OUTPUT );
}

void SX1276AntSwDeInit( void )
{
    AntSwitchRx = RADIO_ANT_SWITCH_RX;
    AntSwitchTxBoost = RADIO_ANT_SWITCH_TX_BOOST;
    AntSwitchTxRfo = RADIO_ANT_SWITCH_TX_RFO;

    Gpio_configAlternate(AntSwitchRx, GPIO_ALTERNATE_ANALOG , GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN);
    Gpio_configAlternate(AntSwitchTxBoost, GPIO_ALTERNATE_ANALOG , GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN);
    Gpio_configAlternate(AntSwitchTxRfo, GPIO_ALTERNATE_ANALOG , GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN);
}

void SX1276SetAntSw( uint8_t opMode )
{
    uint8_t paConfig =  SX1276Read( REG_PACONFIG );
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
        {
            Gpio_set( AntSwitchTxBoost );
        }
        else
        {
            Gpio_set( AntSwitchTxRfo );
        }
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        Gpio_set( AntSwitchRx );
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX1276GetDio1PinState( void )
{
    return Gpio_get( SX1276.DIO1 );
}

#if defined( USE_RADIO_DEBUG )
void SX1276DbgPinTxWrite( uint8_t state )
{
    DbgPinTx = RADIO_DBG_PIN_TX;
    if (state == 0)
        Gpio_clear( DbgPinTx );
    else
        Gpio_set( DbgPinTx );
}

void SX1276DbgPinRxWrite( uint8_t state )
{
    DbgPinRx = RADIO_DBG_PIN_RX;
    if (state == 0)
        Gpio_clear( DbgPinRx );
    else
        Gpio_set( DbgPinRx );
}
#endif
