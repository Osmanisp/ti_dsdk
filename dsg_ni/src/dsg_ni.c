/*! \file dsg_ni.c
    \brief 
****************************************************************************/
#include "dsg_ni.h"
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include "dbridge_esafe.h"

MODULE_LICENSE("GPL");

/* This is the private data the DSG tunnel holds */
typedef struct dsgDevPrivate_st
{
    /* Pointer to the network device. */
    struct net_device*   ptr_device;

    /* Keep track of the statistics. */
    struct net_device_stats     stats;
}dsgDevPrivate_t;


/**************************************************************************/
/*      LOCAL FUNCTIONS DECLERATIONS:                                     */
/**************************************************************************/
static int dsgDev_open (struct net_device *dev);
static int dsgDev_release (struct net_device *dev);
static struct net_device_stats *dsgDev_get_net_stats(struct net_device *dev);
static int dsgDev_set_mac_addr(struct net_device *dev, void *addr);
static int dsgTunnel_init (struct net_device *dev);
static int dsgTunnel_tx (struct sk_buff *skb, struct net_device *dev);
static int dsgIP_init (struct net_device *dev);
static int dsgIP_tx (struct sk_buff *skb, struct net_device *dev);

static void logPacket(const struct sk_buff *skb);
/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/
static int dsgDev_open (struct net_device *dev)
{
    /* Start queue */
	netif_start_queue (dev);
	return 0;
}

static int dsgDev_release (struct net_device *dev)
{
    /* Stop queue */
	netif_stop_queue(dev);
	return 0;
}

static struct net_device_stats *dsgDev_get_net_stats(struct net_device *dev)
{
    //dsgDevPrivate_t *dsgDevPrivate = dev->priv;
    dsgDevPrivate_t *dsgDevPrivate = netdev_priv(dev);
    /* Return the statistics */
    return (struct net_device_stats *) &dsgDevPrivate->stats;
}

static int dsgDev_set_mac_addr(struct net_device *dev, void *addr)
{
    struct sockaddr *sa = addr;

    memcpy( dev->dev_addr, sa->sa_data, dev->addr_len );

    return 0;
}

static const struct net_device_ops dsgTunnel_netdev_ops = {
         .ndo_open               = dsgDev_open,
         .ndo_start_xmit         = dsgTunnel_tx,
         .ndo_stop               = dsgDev_release,
         .ndo_get_stats          = dsgDev_get_net_stats,
         .ndo_do_ioctl           = NULL,
         //.ndo_set_mac_address    = dsgDev_set_mac_addr,
};

static int dsgTunnel_init (struct net_device *dev)
{
    //unsigned char dsgTunnelAddr[ ETH_ALEN ] = { 0x08, 0x00, 0x28, 0x32, 0x06, 0x05 }; /* Dummy */

    /* Populate the device structure */
    /*dev->open = dsgDev_open;
    dev->stop = dsgDev_release;
    dev->hard_start_xmit = dsgTunnel_tx;
    dev->addr_len = ETH_ALEN;
    dev->do_ioctl = NULL;
    dev->get_stats = dsgDev_get_net_stats;*/
    //dev->set_mac_address = dsgDev_set_mac_addr;

    //memcpy( dev->dev_addr, dsgTunnelAddr, ETH_ALEN );
    dev->netdev_ops = &dsgTunnel_netdev_ops;
    dev->addr_len = ETH_ALEN;

    ether_setup(dev);
    printk ("dsgni: Network interface ESTB_DSG initialized successfully\n");
    return 0;
}

int dsgTunnel_rx (struct sk_buff *skb)
{
    kfree_skb (skb);
    return 0;
}


static int dsgTunnel_tx (struct sk_buff *skb, struct net_device *dev)
{
    kfree_skb (skb);
    return 0;
}

static const struct net_device_ops dsgIp_netdev_ops = {
         .ndo_open               = dsgDev_open,
         .ndo_start_xmit         = dsgIP_tx,
         .ndo_stop               = dsgDev_release,
         .ndo_get_stats          = dsgDev_get_net_stats,
         .ndo_do_ioctl           = NULL,
         //.ndo_set_mac_address    = dsgDev_set_mac_addr,
};

static int dsgIP_init (struct net_device *dev)
{
    //unsigned char dsgIPAddr[ ETH_ALEN ] = { 0x08, 0x00, 0x28, 0x32, 0x06, 0x04 }; /* Dummy */

    /* Populate the device structure */
    /*dev->open = dsgDev_open;
    dev->stop = dsgDev_release;
    dev->hard_start_xmit = dsgIP_tx;
    dev->addr_len = ETH_ALEN;
    dev->do_ioctl = NULL;
    dev->get_stats = dsgDev_get_net_stats;
    //dev->set_mac_address = dsgDev_set_mac_addr;*/

    //memcpy( dev->dev_addr, dsgIPAddr, ETH_ALEN );
    dev->netdev_ops = &dsgIp_netdev_ops;
    dev->addr_len = ETH_ALEN;

    ether_setup(dev);
    printk ("dsgni: Network interface ESTB_IP initialized successfully\n");
    return 0;
}


int dsgIP_rx (struct sk_buff *skb)
{
    kfree_skb (skb);
    return 0;
}

static int dsgIP_tx (struct sk_buff *skb, struct net_device *dev)
{
    kfree_skb (skb);
    return 0;
}

int __init dsgDev_init_module (void)
{
    int result;
    struct net_device *devDsgIp,*devDsgTn;
    dsgDevPrivate_t *dsgIpPrivate,*dsgTnPrivate;

    /* Allocate memory for ESTB_IP network interface */
 /*   if ( ( devDsgIp = alloc_netdev( sizeof(dsgDevPrivate_t), CONFIG_HTX_DSG_IP_INTERFACE, ether_setup ) ) == NULL )
    {
	printk ("dsgni: Error allocating memory for ESTB_IP device\n");
       return ENOMEM;
    }*/

    /* Allocate memory for ESTB_DSG network interface */
    /*if ( ( devDsgTn = alloc_netdev( sizeof(dsgDevPrivate_t), CONFIG_HTX_DSG_TUNNEL_INTERFACE, ether_setup ) ) == NULL )
    {
	printk ("dsgni: Error allocating memory for ESTB_DSG device\n");
       return ENOMEM;
    }*/

    /* Init function */
    //devDsgIp->init = dsgIP_init;
	  //devDsgTn->init = dsgTunnel_init;
    if ( ( devDsgIp = alloc_netdev( sizeof(dsgDevPrivate_t), CONFIG_HTX_DSG_IP_INTERFACE, dsgIP_init ) ) == NULL )
    {
    	printk ("dsgni: Error allocating memory for ESTB_IP device\n");
    	return ENOMEM;
    }
    /* Allocate memory for ESTB_DSG network interface */
        if ( ( devDsgTn = alloc_netdev( sizeof(dsgDevPrivate_t), CONFIG_HTX_DSG_TUNNEL_INTERFACE, dsgTunnel_init ) ) == NULL )
    {
    	printk ("dsgni: Error allocating memory for ESTB_DSG device\n");
    	return ENOMEM;
    }


    /* Save the network device in the private area */
    dsgIpPrivate = netdev_priv( devDsgIp);
    dsgIpPrivate->ptr_device = devDsgIp;
    dsgTnPrivate = netdev_priv( devDsgTn);
    dsgTnPrivate->ptr_device = devDsgTn;

    /* Register network interface*/
    if ((result = register_netdev (devDsgIp)))
    {
		printk ("dsgni: Error %d initializing logical ESTB_IP device\n",result);
		return result;
    }
    if ((result = register_netdev (devDsgTn)))
    {
		printk ("dsgni: Error %d initializing logical ESTB_DSG device\n",result);
		return result;
    }

	/* Register DSG network interface to DOCSIS bridge */
	DbridgeEsafe_AddNetDev( devDsgIp, DBR_ESAFE_ESTB_IP, dsgIP_rx);
	DbridgeEsafe_AddNetDev( devDsgTn, DBR_ESAFE_ESTB_DSG, dsgTunnel_rx);

	printk ("dsgni: Network interface registered successfully\n");

	return 0;
}

void __exit dsgDev_cleanup (void)
{
    struct net_device *devDsgIp,*devDsgTn;

    /* Get the pointer to the network device */
    devDsgIp = dev_get_by_name(&init_net, CONFIG_HTX_DSG_IP_INTERFACE );
    devDsgTn = dev_get_by_name(&init_net, CONFIG_HTX_DSG_TUNNEL_INTERFACE );

    if (devDsgIp)
    {
         /* Unregister network interface */
	     unregister_netdev (devDsgIp);
	     printk ("dsgni: Cleaning up ESTB_IP Network interface\n");
    }
    else
    {
	     printk ("dsgni: Failed to clean up ESTB_IP Network interface\n");
    }

    if (devDsgTn)
    {
         /* Unregister network interface */
	     unregister_netdev (devDsgTn);
	     printk ("dsgni: Cleaning up ESTB_DSG Network interface\n");
    }
    else
    {
	     printk ("dsgni: Failed to clean up ESTB_DSG Network interface\n");
    }
}

static void logPacket(const struct sk_buff *skb)
{
	if(skb->dev->hard_header_len == 14)
	{
           struct ethhdr*     ptr_ethhdr;
           ptr_ethhdr = eth_hdr(skb);
           printk("\nSRC MAC: %.2x-%.2x-%.2x-%.2x-%.2x-%.2x DEST MAC: %.2x-%.2x-%.2x-%.2x-%.2x-%.2x PROT %.4x\n",
		   (unsigned char)ptr_ethhdr->h_source[0],
		   (unsigned char)ptr_ethhdr->h_source[1],
		   (unsigned char)ptr_ethhdr->h_source[2],
		   (unsigned char)ptr_ethhdr->h_source[3],
		   (unsigned char)ptr_ethhdr->h_source[4],
		   (unsigned char)ptr_ethhdr->h_source[5],
		   (unsigned char)ptr_ethhdr->h_dest[0],
		   (unsigned char)ptr_ethhdr->h_dest[1],
		   (unsigned char)ptr_ethhdr->h_dest[2],
		   (unsigned char)ptr_ethhdr->h_dest[3],
		   (unsigned char)ptr_ethhdr->h_dest[4],
		   (unsigned char)ptr_ethhdr->h_dest[5],
		   (unsigned short)ptr_ethhdr->h_proto);

		   printk("\n");

	}
	else
	{
		printk("no ether packet\n");
	}
	return;
}


module_init (dsgDev_init_module);
module_exit (dsgDev_cleanup);

EXPORT_SYMBOL( dsgIP_rx );
EXPORT_SYMBOL( dsgTunnel_rx );
