/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * Copyright (C) 2016-2017 A. C. Open Hardware Ideas Lab
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
 * Simone Giacomucci <simone.giacomucci@gmail.com>
 *
 */

#ifdef LIBOHIBOARD_ETHERNET_LWIP_2_0_3

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "lwip_port.h"

/*lwip includes*/
#include "lwip/opt.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "netif/etharp.h"

/* Define those to better describe your network interface. */
#define IFNAME0 'o'
#define IFNAME1 'b'

static Ethernet_MacAddress LWIPPorting_macAddress;

static LWIPPorting_PhyCallback LWIPPorting_phyCallback;

/* Forward declarations. */
static void  LWIPPorting_input (EthernetInterface_DeviceHandle  dev,
                                EthernetInterface_BufferData *buffer);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init (struct netif *netif)
{
    netif->state = ENETIF0;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] = LWIPPorting_macAddress.addrByte.addr[0];
    netif->hwaddr[1] = LWIPPorting_macAddress.addrByte.addr[1];
    netif->hwaddr[2] = LWIPPorting_macAddress.addrByte.addr[2];
    netif->hwaddr[3] = LWIPPorting_macAddress.addrByte.addr[3];
    netif->hwaddr[4] = LWIPPorting_macAddress.addrByte.addr[4];
    netif->hwaddr[5] = LWIPPorting_macAddress.addrByte.addr[5];

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    /* Do whatever else is needed to initialize interface. */
    EthernetInterface_init(ENETIF0,LWIPPorting_input,LWIPPorting_macAddress);

    // Init physical layer
    if(LWIPPorting_phyCallback != 0)
    {
    	LWIPPorting_phyCallback(ENET0);
    }
    // Enable receiving interrupt
    EthernetInterface_enableRxInterrupt(ENETIF0);
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t
low_level_output (struct netif *netif, struct pbuf *p)
{
    EthernetInterface_DeviceHandle ethif = (EthernetInterface_DeviceHandle) netif->state;
    uint8_t *currBuffer = EthernetInterface_getCurrentBuffer(ethif,1);

    uint32_t bdBufferSize = EthernetInterface_getBufferDescriptorSize(ethif,1);
    uint8_t bdNumber = 0;
    uint8_t bdIndex;
    uint8_t dataTempLength;
    struct pbuf *q;

    bdNumber = p->tot_len / bdBufferSize;
    if ((p->tot_len % bdBufferSize) != 0)
        bdNumber++;

    for (bdIndex = 0; bdIndex < bdNumber; bdIndex++)
    {
        dataTempLength = 0;

        for (q = p; q != NULL; q = q->next)
        {
            /*
             * Send the data from the pbuf to the interface, one pbuf at a
             * time. The size of the data in each pbuf is kept in the ->len
             * variable.
             */
            if(bdBufferSize - dataTempLength > q->len)
            {
                memcpy(currBuffer+dataTempLength, (uint8_t *)q->payload, q->len);
                dataTempLength += q->len;
            }
            else
            {
                memcpy(currBuffer+dataTempLength, q->payload, bdBufferSize - dataTempLength);
                p->payload = (void *) ((uint32_t) q->payload + bdBufferSize - dataTempLength);
                p->len = q->len - dataTempLength + bdBufferSize;
                p->tot_len = q->tot_len - dataTempLength + bdBufferSize;
                p->next = q->next;
                break;
            }
        }
    }

    EthernetInterface_sendData(ethif, p->tot_len, bdNumber);
    return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input (struct netif *netif, EthernetInterface_BufferData* buffer)
{
    struct pbuf *p, *q, *head, *tempHead;
    uint16_t dataIndex;

    if (buffer->size)
        head = p = tempHead = pbuf_alloc(PBUF_RAW, buffer->size, PBUF_POOL);

    if (p == NULL)
    	return 0;

    do
    {
        /* We allocate a pbuf chain of pbufs from the pool. */
        dataIndex = 0;
        if (buffer->size != 0)
        {
            for (q = p ; q != NULL; q = q->next)
            {
                memcpy((uint8_t*) q->payload, buffer->data+dataIndex, q->len);
                dataIndex += q->len;
            }
        }

        buffer = buffer->next;

        if (buffer)
        {
            if (buffer->size != 0)
            {
                p = pbuf_alloc(PBUF_RAW, buffer->size, PBUF_POOL);
                pbuf_chain(tempHead,p);
                tempHead = p;

                if (p == NULL) return 0;
            }
        }
    } while (buffer != NULL);

    return head;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 */
static void
LWIPPorting_input (EthernetInterface_DeviceHandle dev, EthernetInterface_BufferData *buffer)
{
    struct netif *netif;
    struct eth_hdr *ethhdr;
    struct pbuf *p;


    for(netif = netif_list; netif != NULL; netif = netif->next)
    {
        if(netif->state == dev)
        {
            break;
        }
    }

    /* move received packet into a new pbuf */
    p = low_level_input(netif,buffer);
    /* no packet could be read, silently ignore this */
    if (p == NULL) return;
    /* points to packet payload, which starts with an Ethernet header */
    ethhdr = p->payload;

    switch (htons(ethhdr->type))
    {
    /* IP or ARP packet? */
    case ETHTYPE_IP:
    case ETHTYPE_ARP:
#if PPPOE_SUPPORT
    /* PPPoE packet? */
    case ETHTYPE_PPPOEDISC:
    case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
        /* full packet send to tcpip_thread to process */
        if (netif->input(p, netif)!=ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            pbuf_free(p);
            p = NULL;
        }
        break;

    default:
        pbuf_free(p);
        p = NULL;
        break;
    }
}

void LWIPPorting_setMacAddress (Ethernet_MacAddress address)
{
    LWIPPorting_macAddress.addrs = address.addrs;
}

void LWIPPorting_setPhyCallback (LWIPPorting_PhyCallback callback)
{
	LWIPPorting_phyCallback = callback;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
LWIPPorting_init (struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
//    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}
void LWIPPorting_statusCallback(struct netif *netif)
{
}

void LWIPPorting_linkCallback(struct netif *netif)
{
}
#endif

#endif /* LIBOHIBOARD_ETHERNET_LWIP_2_0_3 */
