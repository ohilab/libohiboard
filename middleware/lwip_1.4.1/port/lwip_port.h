/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * Copyright (C) 2016217 A. C. Open Hardware Ideas Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * OHILab's Changes authors:
 * Simone Giacomucci
 * Marco Giammarini
 *
 */

#ifdef LIBOHIBOARD_ETHERNET_LWIP_1_4_1

#ifndef __LIBOHIBOARD_LWIP_PORT_H
#define __LIBOHIBOARD_LWIP_PORT_H

#include "libohiboard.h"

#include "lwip/err.h"
#include "lwip/netif.h"

typedef void (*LWIPPorting_PhyCallback) (Ethernet_DeviceHandle dev);

err_t LWIPPorting_init (struct netif *netif);

void LWIPPorting_setMacAddress (Ethernet_MacAddress address);
void LWIPPorting_setPhyCallback (LWIPPorting_PhyCallback callback);

#endif /* __LIBOHIBOARD_LWIP_PORT_H */

#endif /* LIBOHIBOARD_ETHERNET_LWIP_1_4_1 */
