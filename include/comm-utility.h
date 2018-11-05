/**
 * @file libohiboard/include/utility.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful functions and definitions.
 */

#ifndef __COMM_UTILITY_H
#define __COMM_UTILITY_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

#ifdef LIBOHIBOARD_IIC

#include "i2c.h"

/**
 *
 */
System_Errors CommUtility_iicBusScanner (Iic_DeviceHandle dev,
                                         uint8_t* result,
                                         uint8_t  resultSize,
                                         uint8_t* countedDevice);

/**
 *
 */
System_Errors CommUtility_iicBusCheckDevices (Iic_DeviceHandle dev,
                                              const uint8_t* devices,
                                              uint8_t  deviceNumber);
#endif

#endif // __COMM_UTILITY_H
