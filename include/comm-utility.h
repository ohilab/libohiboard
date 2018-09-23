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
uint8_t CommUtility_iicBusScanner (Iic_DeviceHandle dev, uint8_t* result);
#endif

#endif // __COMM_UTILITY_H
