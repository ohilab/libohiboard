#include "comm-utility.h"

#ifdef LIBOHIBOARD_IIC
uint8_t CommUtility_IicBusScanner (Iic_DeviceHandle dev, uint8_t* result)
{
    uint8_t address   = 0;
    uint8_t ackResult = 0;
    uint8_t index     = 0;

    for (address = 1; address < 127; address++)
    {
        Iic_start(dev);

        Iic_writeByte(dev,((address << 1) & 0xFE));
        Iic_waitTransfer(dev);
        ackResult = Iic_getAck(dev);

        if (ackResult == ERRORS_IIC_TX_ACK_RECEIVED)
        {
            result[index] = address;
            index++;
        }
    }
    return index;
}
#endif
