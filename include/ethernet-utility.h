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
 * @file libohiboard/include/ethernet-utility.h
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief network utility.
 */

#ifdef LIBOHIBOARD_ETHERNET
#ifndef __ETHERNET_UTILITY_H
#define __ETHERNET_UTILITY_H

#include "libohiboard.h"
#include "arch/sys_arch.h"
// lwip includes
#include "lwip/tcp.h"
#include "lwip/tcp_impl.h"
#include "lwip/mem.h"
#include "lwip/raw.h"
#include "lwip/icmp.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/timers.h"
#include "lwip/inet_chksum.h"
#include "lwip/init.h"
#include "netif/etharp.h"

//porting includes
#include "lwip_K64F12.h"

typedef struct _Network_Config
{
    ip_addr_t ip;
    ip_addr_t netMask;
    ip_addr_t gw;
    Ethernet_MacAddress macAdd;
    Pit_DeviceHandle pit;
    uint8_t channel;
}Network_Config;

void Ethernet_networkConfig(struct netif *netif, Network_Config *config);

#endif /* __ETHERNET_UTILITY_H */
#endif
