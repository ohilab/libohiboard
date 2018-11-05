#include "comm-utility.h"

#ifdef LIBOHIBOARD_IIC

System_Errors CommUtility_iicBusScanner (Iic_DeviceHandle dev,
                                         uint8_t* result,
                                         uint8_t  resultSize,
                                         uint8_t* countedDevice)
{
    uint8_t address         = 0;
    System_Errors ackResult = 0;
    uint8_t index           = 0;

    for (address = 1; address < 127; address++)
    {
        Iic_start(dev);

        Iic_writeByte(dev,((address << 1) & 0xFE));
        Iic_waitTransfer(dev);
        ackResult = Iic_getAck(dev);

        Iic_stop(dev);

        if (ackResult == ERRORS_IIC_TX_ACK_RECEIVED)
        {
            result[index] = address;
            index++;
        }

        if (index >= resultSize)
        {
            *countedDevice = index;
            return ERRORS_COMMUTILITY_MAX_DEVICE_ACHIEVE;
        }
    }
    *countedDevice = index;
    return ERRORS_NO_ERROR;
}


System_Errors CommUtility_iicBusCheckDevices (Iic_DeviceHandle dev,
                                              const uint8_t* devices,
                                              uint8_t  deviceNumber)
{
    System_Errors ackResult = 0;

    for (uint8_t i = 0; i < deviceNumber; i++)
    {
        Iic_start(dev);

        Iic_writeByte(dev,((devices[i] << 1) & 0xFE));
        Iic_waitTransfer(dev);
        ackResult = Iic_getAck(dev);

        Iic_stop(dev);

        if (ackResult != ERRORS_IIC_TX_ACK_RECEIVED)
        {
            return ERRORS_COMMUTILITY_DEVICE_NOT_FOUND;
        }
    }
    return ERRORS_NO_ERROR;
}


#endif
