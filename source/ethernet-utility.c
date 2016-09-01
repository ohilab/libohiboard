#ifdef LIBOHIBOARD_ETHERNET

#include "ethernet-utility.h"

void Ethernet_networkConfig(struct netif *netif, Network_Config *config)
{

    lwip_init();
    lwipTimerInit(config->pit, config->channel);

    LWIP_K64F12_setMacAddress(config->macAdd);

    // Initialize network interface
    netif_add(netif, &config->ip, &config->netMask, &config->gw, 0, LWIP_K64F12_init, ethernet_input);
    netif_set_default(netif);
    netif_set_up(netif);
}

#endif /* LIBOHIBOARD_ETHERNET */
