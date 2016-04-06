/*
 *
 * vfe_iptv_legacy_mcast.c
 * Description:
 * VFE IPTV Legacy multicast sub-module implementation
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*! \file vfe_iptv_legacy_mcast.c
    \brief VFE IPTV Legacy multicast sub-module implementation
*/

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm-generic/errno-base.h>
#include <linux/uaccess.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include "vfe_iptv_brg.h"
#include "vfe_iptv_legacy_mcast.h"
#include "pal.h"


/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/
/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/
#ifdef IPTV_BRG_DEBUG
/* note: prints function name for you */
#  define IPTV_BRG_PRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#  define IPTV_BRG_PRINTK(fmt, args...)
#endif

/**/
#define MAX_NUM_IPTV_LEG_MCASTS		8

/*! \var typedef struct IpTvLegMcast_t 
 *  \brief Represents a data in an entry of the IPTV Legacy multicast list
 */
typedef struct IptvLegMcast_s 
{
	Uint8  macAddress[MAC_ADDR_LEN]; /* The Multicast Dest MAC address */
	Uint32 dsPortMap;                   /* Defines for each multicast from which DS-ports it is accepted */     

   /*  CMIM interfaces indexed (in Linux Kernel numeration) bitmap         */
   /*  a bit mask representing the Linux interfaces of the CM to which the   */
   /*  CM is to forward IPTV multicast traffic associated with the Mcast address     */
   Uint32 outputCmim;
   Uint32 inputFrames;                  /* Number of input/accepted frames per multicast */
} IptvLegMcast_t;

/*! \var typedef struct IpTvLegMcast_t 
 *  \brief Represents an entry of the IPTV Legacy multicast list
 */
typedef struct IptvLegMcastListEntry_s
{
    struct list_head list;
    IptvLegMcast_t   mcast;
}IptvLegMcastListEntry_t;

/*! \var typedef struct IpTvLegMcast_t 
 *  \brief IPTV Legacy multicast list local DB structure
 */
typedef struct IptvLegMcastDB_s
{
    struct list_head mcastListHead;
    Uint32           numMcasts;
    Uint32           totalInputFrames;
}IptvLegMcastDB_t;

static IptvLegMcastListEntry_t *IptvLegMcast_GetMcastEntryByMac(Uint8 *mac);
static void IptvLegMcast_PrintDb(void);

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/
static IptvLegMcastDB_t  localDb;/*IPTV Legacy Mcast local DB*/

/**************************************************************************/
/*! \fn int IptvLegMcast_Receive(struct sk_buff* skb, Uint32 dsPort,char * net_dev_name,network_Type_e dev_type)
 **************************************************************************
 *  \brief Main Legacy Mcast frame receive flow
 *  \param[in]   skb - The pkt sk buffer .
 *  \param[in]   dsPort - The DS channel ID that this frame came from.
 *  \param[in]   net_dev_name - network device name to receive IPTV packets. 
 *  \param[in]   dev_type - network device type - bridge or end-decive.
 *  \return 0 or -1 on error.
 */

int IptvLegMcast_Receive(struct sk_buff* skb, Uint32 dsPort,struct net_device * IPTV_net_device,network_Type_e dev_type)
{
   struct ethhdr  *ethHeader;
   Uint8  		  *destMac;
   IptvLegMcastListEntry_t *mcastEntry;

   /* if the packet is NOT Multicast */
   if ( !IPTVBR_IS_MCAST_BIT_ON(skb->ti_meta_info) )
   {
       IPTV_BRG_PRINTK(KERN_ERR"\nIPTV input frame is not multicast !!!\n");
       return -1; /* --> drop the frame */
   }

   /*Get Destination MAC address from ETH header*/
   ethHeader = eth_hdr(skb);
   destMac   = ethHeader->h_dest;

   IPTV_BRG_PRINTK(KERN_NOTICE"\nIPTV Legacy Mcast frame received. MAC = %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n", 
		              destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5]);

   /* Get Multicast info by Mcast Address from IPTV Legacy Mcast DB */
   mcastEntry = IptvLegMcast_GetMcastEntryByMac(destMac);

   if (!mcastEntry)
   {  /* drop the packet */
      return -1; /* --> drop the frame */
   }
#ifdef IPTV_BRG_DEBUG
   if (mcastEntry->mcast.outputCmim == 0 || mcastEntry->mcast.dsPortMap == 0)
   {  /* The flood map or ds_port_map is 0 - drop the packet */
      printk(KERN_ERR"\nIPTV Legacy Mcast frame receive from dev %s Failed - Invalid MAC %.2x:%.2x:%.2x:%.2x:%.2x:%.2x entry!!!\n", 
              skb->dev->name, destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5]);
      return -1; /* --> drop the frame */
   }
#endif

   if (!IPTVBR_IS_CHANNEL_ID_OK(dsPort, mcastEntry->mcast.dsPortMap))
   {
      return -1; /* --> no processing needed */
   }

   localDb.totalInputFrames++;
   mcastEntry->mcast.inputFrames++;

   /* At last forward this frame to output ports  */
   skb->ti_selective_fwd_dev_info = mcastEntry->mcast.outputCmim;
   skb->dev = IPTV_net_device;
   /* Check if the device is a Network Bridge or Network End-device */
   /* Case Network End-device */ 
   if (dev_type == NETWORK_LSD_DEVICE)
   {
       dev_queue_xmit(skb);
   }
   else/* Case Network Bridge */ 
   {
       /* Pull the Ethernet MAC Address before passing it to the stack. */
       skb_pull(skb,ETH_HLEN);
       netif_receive_skb(skb);
   }
   return 0; 
}

/**************************************************************************/
/*! \fn int IptvLegMcast_Init(void)
 **************************************************************************
 *  \brief Init IPTV legacy multicast module
 *  \return  0 or error code
 */
int IptvLegMcast_Init(void)
{
   INIT_LIST_HEAD(&localDb.mcastListHead);
   localDb.numMcasts = 0;
   localDb.totalInputFrames = 0;
   return 0;
}

/**************************************************************************/
/*! \fn int IptvLegMcast_Cleanup(void)
 **************************************************************************
 *  \brief Cleanup IPTV legacy multicast module
 *  \return  0 or error code
 */
int IptvLegMcast_Cleanup(void)
{
    struct list_head *pos;
    IptvLegMcastListEntry_t *curr=NULL;
    Uint32 lockKey;

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

    /* free multicast list */
    pos = localDb.mcastListHead.next;
    while(pos != &localDb.mcastListHead)
    {
        curr = list_entry(pos, IptvLegMcastListEntry_t, list);
        pos = pos->next;
        PAL_osMemFree(0, curr, sizeof(IptvLegMcastListEntry_t));
    }

    INIT_LIST_HEAD(&localDb.mcastListHead);
	localDb.numMcasts = 0;
	localDb.totalInputFrames = 0;

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

	return 0;
}

/**************************************************************************/
/*! \fn static int IptvLegAddMcast(Uint8 *destMac, Uint32 dsPortBitmap, Uint32 outputCmim)
 **************************************************************************
 *  \brief Add Multicast entry to IPTV legacy multicast list
 *  \param[in] Uint8 *destMac - destination MAC address
 *  \param[in] Uint32 dsPortBitmap - DS port Bitmap
 *  \param[in] Uint32 outputCmim - output linux interface mask
 *  \return  0 or error code
 */
int IptvLegMcast_AddMcast(Uint8 *destMac, Uint32 dsPortBitmap, Uint32 outputCmim)
{
    IptvLegMcastListEntry_t *new = NULL;;


    IPTV_BRG_PRINTK(KERN_NOTICE"%s: MAC=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x, DS port bitmap=0x%X, ouput CMIM=0x%X\n",
					__FUNCTION__, destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5],
					dsPortBitmap, outputCmim);

	if (localDb.numMcasts == MAX_NUM_IPTV_LEG_MCASTS) 
	{
		IPTV_BRG_PRINTK(KERN_WARNING"IPTV: Failed to add multicast - the multicast table is full\n");
		return -EINVAL;
	}
	new = IptvLegMcast_GetMcastEntryByMac(destMac);

    /*if multicast entry with such MAC already exists - return error*/
    if (new) 
	{
        IPTV_BRG_PRINTK(KERN_WARNING"IPTV: Failed to add multicast - - entry with such MAC already exists\n");
        return -EINVAL;
    }

    /*Create new list entry*/
	if (PAL_osMemAlloc(0, sizeof(IptvLegMcastListEntry_t), 0, (Ptr *)&new) != PAL_SOK) 
	{
		IPTV_BRG_PRINTK(KERN_ERR"%s: Failed to allocate dynamic memory\n",__FUNCTION__);
		return -EFAULT;
	}

	memcpy(new->mcast.macAddress, destMac, MAC_ADDR_LEN);
	new->mcast.dsPortMap = dsPortBitmap;
	new->mcast.outputCmim = outputCmim;
	new->mcast.inputFrames = 0;

    /* insert at the head of the list */
    list_add(&new->list, &localDb.mcastListHead);

    localDb.numMcasts ++;

	return 0;
}

/**************************************************************************/
/*! \fn int IptvLegMcast_ModifyMcast(Uint8 *destMac, Uint32 dsPortBitmap, Uint32 outputCmim)
 **************************************************************************
 *  \brief Modify multicast entry to IPTV legacy multicast list
 *  \param[in] Uint8 *destMac - destination MAC address
 *  \param[in] Uint32 dsPortBitmap - DS port Bitmap
 *  \param[in] Uint32 outputCmim - output linux interface mask
 *  \return  0 or error code
 */
int IptvLegMcast_ModifyMcast(Uint8 *destMac, Uint32 dsPortBitmap, Uint32 outputCmim)
{
    IptvLegMcastListEntry_t *entry = NULL;;

    IPTV_BRG_PRINTK(KERN_NOTICE"%s: MAC=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x, DS port bitmap=0x%X, ouput CMIM=0x%X\n",
					__FUNCTION__, destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5],
					dsPortBitmap, outputCmim);

	entry = IptvLegMcast_GetMcastEntryByMac(destMac);

    /*if multicast entry with such MAC already exists - return error*/
    if (!entry) 
	{
        IPTV_BRG_PRINTK(KERN_WARNING"IPTV: Failed to modify multicast - - entry with such MAC doesn't exist\n");
        return -EINVAL;
    }

	entry->mcast.dsPortMap = dsPortBitmap;
	entry->mcast.outputCmim = outputCmim;

    return 0;
}

/**************************************************************************/
/*! \fn int IptvLegMcast_DelMcast(Uint8 *destMac)
 **************************************************************************
 *  \brief Delete an entry from IPTV legacy Mcast tabel with the given MAC
 *  \param[in] Uint8 *destMac - Mcast MAC
 *  \return  0 or error code
 */
int IptvLegMcast_DelMcast(Uint8 *destMac)
{
    IptvLegMcastListEntry_t *entry;

    IPTV_BRG_PRINTK(KERN_NOTICE"%s: MAC=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
					__FUNCTION__, destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5]);

    if (!localDb.numMcasts) {
        IPTV_BRG_PRINTK(KERN_WARNING"Failed to delete multicast - the Mcast table is empty\n");
        return -EINVAL;
    }

	entry = IptvLegMcast_GetMcastEntryByMac(destMac);
    if (!entry)
    {
        IPTV_BRG_PRINTK(KERN_WARNING"Failed to delete multicast - entry not found\n");
        return -EINVAL;
    }

    /* delete current entry */
    list_del(&entry->list);
    localDb.numMcasts --;

    PAL_osMemFree(0, entry, sizeof(IptvLegMcastListEntry_t));

    return 0;
}

/**************************************************************************/
/*! \fn static int IptvLegMcast_Config(unsigned int cmd, IpTvLegMcastParams_t *params)
 **************************************************************************
 *  \brief Calls the Add/Del/Modify... Legacy Mcast functions according to the input command.
 *  \param[in] cmd - The command ID.
 *  \param[in] params -  Configuration parameters structure.
 *  \param[out] no output.
 *  \return 0 or error status.
 **************************************************************************/
static int IptvLegMcast_Config(unsigned int cmd, IpTvLegMcastParams_t *params)
{
	int retval = 0;

    switch (cmd) 
    {
    case IPTV_LEGMCST_ADD:
       retval = IptvLegMcast_AddMcast(params->macAddress, params->dsPortMap, params->linuxCmim );
       break;
    case IPTV_LEGMCST_DEL:
       retval = IptvLegMcast_DelMcast(params->macAddress);
       break;
    case IPTV_LEGMCST_MODIFY:
       retval = IptvLegMcast_ModifyMcast(params->macAddress, params->dsPortMap, params->linuxCmim );
       break;
    default:
       printk(KERN_ERR"\nIPTV Legacy Mcast unknown ioctl command %d\n",cmd);
       retval = -EINVAL;
    }

    return retval;
}

/**************************************************************************/
/*! \fn int IptvLegMcast_Actions(unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief Main function for the IPTV Legacy Mcast module configuration.
 *  \param[in]   cmd - The config command ID.
 *  \param[in]   arg - The config command argument (user data).
 *  \return 0 or error status.
 */
int IptvLegMcast_Actions(unsigned int cmd, unsigned long arg)
{
   IpTvLegMcastParams_t params;

   if (_IOC_NR(cmd) > IPTV_LEGMCST_MAXNR) 
      return -ENOTTY;

   /* Process the data */
   switch (cmd) 
   {
   case IPTV_LEGMCST_ADD:
   case IPTV_LEGMCST_DEL:
   case IPTV_LEGMCST_MODIFY:
      if (copy_from_user(&params, (void __user *)arg, sizeof(IpTvLegMcastParams_t)))
      {
         printk(KERN_ERR"\nIPTV Legacy Mcast failed to copy from user [IpTvLegMcastParams_t]\n");
         return -EFAULT;
      }
      return (IptvLegMcast_Config(cmd, &params));
   case IPTV_LEGMCST_PRINT_DB:
	  IptvLegMcast_PrintDb();
      break;
   case IPTV_LEGMCST_RESET_DB:
	  IptvLegMcast_Cleanup();
	  break;
   default:
      printk(KERN_ERR"\nIPTV Legacy Mcast unknown ioctl command %d\n",cmd);
      return -EINVAL;
   }

   return 0;
}

/**************************************************************************/
/*! \fn static IptvLegMcastListEntry_t *IptvLegMcast_GetMcastEntryByMac(Uint8 *mac)
 **************************************************************************
 *  \brief Get Multicast list entry by index
 *  \param[in]  Uint8 *mac - multicast MAC address
 *  \retrun     list entry data or NULL
 */
static IptvLegMcastListEntry_t *IptvLegMcast_GetMcastEntryByMac(Uint8 *mac)
{
    struct list_head        *pos;
    IptvLegMcastListEntry_t *curr=NULL;

    list_for_each(pos, &localDb.mcastListHead)
    {
        curr = list_entry(pos, IptvLegMcastListEntry_t, list);
        if (!memcmp(mac, curr->mcast.macAddress, MAC_ADDR_LEN)) 
            break;
    }

	/*If nothing has been found pos should point to list head*/
    if (pos != &localDb.mcastListHead) 
        return curr;

	return NULL;
}

/**************************************************************************/
/*! \fn static void *IptvLegMcast_PrintDb(void)
 **************************************************************************
 *  \brief Print IPTV Legacy Multicast DB
 */
static void IptvLegMcast_PrintDb(void)
{
    struct list_head        *pos;
    IptvLegMcastListEntry_t *curr=NULL;

	printk("IPTV Legacy Multicast DB:\n");
	printk("Current number of multicasts = %u\n", localDb.numMcasts);
	printk("Total number of input frames = %u\n", localDb.totalInputFrames);
    list_for_each(pos, &localDb.mcastListHead)
    {
        curr = list_entry(pos, IptvLegMcastListEntry_t, list);
		printk("MAC=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x, DS port bitmap=0x%X, ouput CMIM=0x%X, input frames=%u\n",
			   curr->mcast.macAddress[0], curr->mcast.macAddress[1], curr->mcast.macAddress[2],
			   curr->mcast.macAddress[3], curr->mcast.macAddress[4], curr->mcast.macAddress[5],
			   curr->mcast.dsPortMap, curr->mcast.outputCmim, curr->mcast.inputFrames);
    }
}

