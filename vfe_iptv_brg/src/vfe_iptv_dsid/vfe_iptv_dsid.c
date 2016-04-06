/*
 *
 * dbridge_vfe_iptv_dsid.c
 * Description:
 * IPTV bridge Multicast DSID Forwarding configuration and control implementation.
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

/*! \file vfe_iptv_dsid.c
    \brief IPTV bridge Multicast DSID Forwarding configuration and control implementation.
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
#include "vfe_iptv_dsid.h"


/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/
/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

/*! \var typedef struct IpTvBridgeDsidEntry_t 
 *  \brief Represents an entry in the IPTV DSID bridge DB. 
 */
typedef struct IpTvBridgeDsidEntry 
{
   Bool   isDsidValid;      /* This IPTV DSID is valid or Not */
   Uint32 dsidVal;          /* The IPTV DSID Value */
   Uint32 dsidDsPortMap;    /* Defines for each DSID from which DS-ports it is accepted */     

   /*  CMIM interfaces indexed (in Linux Kernel numeration) bitmap         */
   /*  a bit mask representing the Linux interfaces of the CM to which the   */
   /*  CM is to forward IPTV multicast traffic associated with the DSID     */
   Uint32 dsidLinuxCmim;
   Uint8  macAddress[MAC_ADDR_LEN]; /* The Multicast Dest MAC address corresponding to this DSID */
   Uint32 dsidInPkt;        /* DSID counter: Number of input/accepted packets per DSID */

} IpTvBridgeDsidEntry_t;

/*! \var typedef struct IpTvBridgeDsidDB_t 
 *  \brief The IPTV DSID bridge DB. 
 */
typedef struct IpTvBridgeDsidDB 
{
    Uint32 ipTvCurrDsidNum; /* current IPTV DSID indexes configured in this bridge. */

    Uint32 ipTvTotalInPkt;  /* DSID counter: Total number of input/accepted packets with valid DSID */

    IpTvBridgeDsidEntry_t ipTvDsidInfo[IPTVBR_MDF_MAX_DSIDS];   /* IPTV DSID Bridge sub DB */
    
} IpTvBridgeDsidDB_t;

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/
static int IpTvDsid_Add(IpTvBridgeDsidInfo_t    mdfInfoFromUser);
static int IpTvDsid_Delete(IpTvBridgeDsidInfo_t mdfInfoFromUser);
static int IpTvDsid_Modify(IpTvBridgeDsidInfo_t mdfInfoFromUser);
static int IpTvDsid_Action(unsigned int cmd, IpTvBridgeDsidInfo_t mdfInfoFromUser);
static inline int IpTvDsid_GetDsidInfo(Uint32 dsidIndex, Uint32* dsidLinuxCmim, Uint32* dsidDsPortMap);
static inline int IpTvDsid_UpdateDsidCounter(Uint32 dsidIndex);
static inline void IpTvDsid_PrintDb(void);
/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/
static IpTvBridgeDsidDB_t  ipTvBrgDsidDb; /* IPTV Bridge DSID DB */

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/

/**************************************************************************/
/*! \fn int iptv_dsid_init(void)
 **************************************************************************
 *  \brief Init IPTV DSID.
 *  \return  OK/NOK
 */
int iptv_dsid_init(void)
{
   memset(&ipTvBrgDsidDb,0,sizeof(IpTvBridgeDsidDB_t));
#ifdef IPTV_BRG_DSID_DEBUG
   printk(KERN_INFO"\nIPTV Bridge DSID DB init done\n");
#endif
   return 0;
}

/**************************************************************************/
/*! \fn int iptv_dsid_handler(struct sk_buff* skb, Uint32 ds_channel_id, char * net_dev_name, network_Type_e dev_type)
 **************************************************************************
 *  \brief Main function for the IPTV DSID bridge flow handling.
 *  \param[in]   skb - The pkt sk buffer .
 *  \param[in]   ds_channel_id - The DS channel ID that this pkt came from.
 *  \param[in]   net_dev_name - network device name to receive IPTV packets. 
 *  \param[in]   dev_type - network device type - bridge or end-decive.
 *  \return 0 or -1 on error.
 */

int iptv_dsid_handler(struct sk_buff* skb, Uint32 ds_channel_id, struct net_device *IPTV_net_device, network_Type_e dev_type)
{
   Uint32 dsid_index  = 0;
   Uint32 linux_cmim  = 0;
   Uint32 ds_port_map = 0;	

   /* if the packet is NOT Multicast */
   if ( !IPTVBR_IS_MCAST_BIT_ON(skb->ti_meta_info) )
   {
#ifdef IPTV_BRG_DSID_DEBUG
       printk(KERN_ERR"\nIPTV DSID pkt is not multicast !!!\n");
#endif
      return -1; /* --> no DSID processing needed */
   }

   /* Get DSID index from the packet - bits 24-31 */
   dsid_index = IPTVBR_GET_DSID_INX_FROM_CNID(skb->ti_meta_info);

   /* The value from the HW is a multiple of 4 */
   if (dsid_index != 0xFF)
   {
       dsid_index = dsid_index >> 2;
   }

   /* if DSID index value not valid - drop the packet */
   if (dsid_index >= IPTVBR_MDF_MAX_DSIDS)
   {
#ifdef IPTV_BRG_DSID_DEBUG
       printk(KERN_ERR"\nIPTV DSID packet from dev %s Failed - DSID index value %d not valid [ti_meta_info = %x]!!!\n", 
              skb->dev->name, dsid_index, skb->ti_meta_info);
#endif
       return -1; /* --> no DSID processing needed */
   }

   /* Get DSID info by DSID index from IPTV DSID DB */
   if (IpTvDsid_GetDsidInfo(dsid_index, &linux_cmim, &ds_port_map) == -1)
   {  /* if DSID index not found - drop the packet */
#ifdef IPTV_BRG_DSID_DEBUG
      printk(KERN_ERR"\nIPTV DSID packet from dev %s Failed - DSID index %d not found!!!\n",
             skb->dev->name, dsid_index);
#endif
      return -1; /* --> no DSID processing needed */
   }
#ifdef IPTV_BRG_DSID_DEBUG
   if (linux_cmim == 0 || ds_port_map == 0)
   {  /* The flood map or ds_port_map is 0 - drop the packet */
      printk(KERN_ERR"\nIPTV DSID packet from dev %s Failed - DSID index %u wrong parameters linux_cmim=%d ds_port_map=%d!!!\n", 
              skb->dev->name, dsid_index,linux_cmim,ds_port_map);
      return -1; /* --> no DSID processing needed */
   }
#endif
   if (!IPTVBR_IS_CHANNEL_ID_OK(ds_channel_id,ds_port_map))
   {
#ifdef IPTV_BRG_DSID_DEBUG
      printk(KERN_ERR"\nIPTV DSID packet from dev %s Failed - DSID index %u - This DS is not valid for this DSID ds_channel_id=%d ds_port_map=%d!!!\n", 
              skb->dev->name, dsid_index,ds_channel_id,ds_port_map);
#endif
      return -1; /* --> no DSID processing needed */
   }
   /* At last forward this IPTV multicast DSID  */
   IpTvDsid_UpdateDsidCounter(dsid_index);
   /* Flood the packet to all interfaces... */
   skb->ti_selective_fwd_dev_info = linux_cmim;
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
   return 0; /* DSID processing done and pkt is on is way to an happy client */
}

/**************************************************************************/
/*! \fn int iptv_dsid_actions(unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief Main function for the IPTV DSID bridge configuration.
 *  \param[in]   cmd - The config command ID.
 *  \param[in]   arg - The config command argument (user data).
 *  \return 0 or error status.
 */
int iptv_dsid_actions(unsigned int cmd, unsigned long arg)
{
   IpTvBridgeDsidInfo_t mdfInfoFromUser;
   IpTvDsidCounters_t ipTvCounters;
   Uint32 i;

   if (_IOC_NR(cmd) > IPTV_DSID_MAXNR) 
      return -ENOTTY;

   /* Process the data */
   switch (cmd) 
   {
   case IPTV_DSID_ADD:
   case IPTV_DSID_DEL:
   case IPTV_DSID_MODIFY:
      if (copy_from_user(&mdfInfoFromUser, (void __user *)arg, sizeof(IpTvBridgeDsidInfo_t)))
      {
         printk(KERN_ERR"\nIPTV Bridge failed to copy from user [mdfInfoFromUser]\n");
         return -EFAULT;
      }
      return (IpTvDsid_Action(cmd,mdfInfoFromUser));
   case IPTV_DSID_GET_COUNTER:
      memset(&ipTvCounters,0,sizeof(IpTvDsidCounters_t));
      ipTvCounters.totalInPkt = ipTvBrgDsidDb.ipTvTotalInPkt;
      for (i=0;i<IPTVBR_MDF_MAX_DSIDS;i++)
      {
         if (ipTvBrgDsidDb.ipTvDsidInfo[i].isDsidValid)
         {
            ipTvCounters.dsidInPkt[i].isDsidValid = True;
            ipTvCounters.dsidInPkt[i].dsidInPkt   = ipTvBrgDsidDb.ipTvDsidInfo[i].dsidInPkt;
            ipTvCounters.dsidInPkt[i].dsidVal     = ipTvBrgDsidDb.ipTvDsidInfo[i].dsidVal;
         }
      }
      if ( copy_to_user((void *)arg,(void *)(&ipTvCounters),sizeof(IpTvDsidCounters_t)) )
      {
         printk(KERN_ERR"\nIPTV Bridge: failed to copy to user\n");
         return -EFAULT;
      }
      break;
   case IPTV_DSID_RESET_DB:
      memset(&ipTvBrgDsidDb,0,sizeof(IpTvBridgeDsidDB_t));
#ifdef IPTV_BRG_DSID_DEBUG
      printk(KERN_INFO"\nIPTV Bridge DSID DB reset done\n");
#endif
      break;
   case IPTV_DSID_PRINT_DB:
      IpTvDsid_PrintDb();
      break;
   default:
      printk(KERN_ERR"\nIPTV Bridge unknown ioctl command %d\n",cmd);
      return -EINVAL;
   }

   return 0;
}


/**************************************************************************/
/*      LOCAL FUNCTIONS Implementation:                                   */
/**************************************************************************/


/**************************************************************************/
/*! \fn static inline void IpTvDsid_PrintDb(void)                                
 **************************************************************************
 *  \brief Print IPTV DSID DB.
 **************************************************************************/
static inline void IpTvDsid_PrintDb(void)
{
   Uint32 i;
   Uint8 * pAddress;

   printk("\n"); 
   printk("IPTV DSID Total number:     %u\n", ipTvBrgDsidDb.ipTvCurrDsidNum); 

   printk("\n IPTV DSID Information:\n");
   /* for each DSID in the DSID DB */
   for (i = 0; i < IPTVBR_MDF_MAX_DSIDS; i++)
   {
      if (ipTvBrgDsidDb.ipTvDsidInfo[i].isDsidValid == True)
      {
         printk("\nDSID = 0x%x [%d]\n",ipTvBrgDsidDb.ipTvDsidInfo[i].dsidVal,
                ipTvBrgDsidDb.ipTvDsidInfo[i].dsidVal);
         pAddress = ipTvBrgDsidDb.ipTvDsidInfo[i].macAddress;
		 printk("    Mcast Mac        = %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n", 
                pAddress[0], pAddress[1], pAddress[2], pAddress[3], pAddress[4], pAddress[5]);
         printk("    DS Port Map      = 0x%x\n",ipTvBrgDsidDb.ipTvDsidInfo[i].dsidDsPortMap);
         printk("    Linux IF Map     = 0x%x\n",ipTvBrgDsidDb.ipTvDsidInfo[i].dsidLinuxCmim);
         printk("    Total In packets = %u\n",ipTvBrgDsidDb.ipTvDsidInfo[i].dsidInPkt);
      }
   }
   printk("\n"); 
}

/**************************************************************************/
/*! \fn static inline int IpTvDsid_GetDsidInfo(Uint32 dsidIndex, Uint32* dsidLinuxCmim, Uint32* dsidDsPortMap)                                    
 **************************************************************************
 *  \brief Get the IPTV Multicast info according to the dsid index.
 *  \param[in] dsidIndex - dsid index value.
 *  \param[out] dsidLinuxCmim - Multicast Linux CMIM value to be returned.
 *  \param[out] dsidDsPortMap - Multicast DS port map value to be returned.
 *  \return 0 or -1 on error.
 **************************************************************************/
static inline int IpTvDsid_GetDsidInfo(Uint32 dsidIndex, Uint32* dsidLinuxCmim, Uint32* dsidDsPortMap)
{
    if (ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].isDsidValid == False)
    {
        printk(KERN_ERR"\nUnknown IPTV DSID index %u - Can't forward the packet!!! dsidLinuxCmim=%d dsidVal=%d dsidInPkt=%d\n", 
               dsidIndex,
               ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidLinuxCmim,
               ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidVal,
               ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidInPkt);
        return -1;
    }
    
    *dsidLinuxCmim = ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidLinuxCmim;
    *dsidDsPortMap = ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidDsPortMap;

    return 0;
}

/**************************************************************************/
/*! \fn static inline int IpTvDsid_UpdateDsidCounter(Uint32 dsidIndex)
 **************************************************************************
 *  \brief Update (inc) the IPTV Multicast dsid counters according to the dsid index.
 *  \param[in] dsidIndex - dsid index value.
 *  \return 0 or -1 on error.
 **************************************************************************/
static inline int IpTvDsid_UpdateDsidCounter(Uint32 dsidIndex)
{
   ipTvBrgDsidDb.ipTvDsidInfo[dsidIndex].dsidInPkt++;
   ipTvBrgDsidDb.ipTvTotalInPkt++;
   return 0;
}

/**************************************************************************/
/*! \fn static int IpTvDsid_Add(IpTvBridgeDsidInfo_t mdfInfoFromUser)                                 
 **************************************************************************
 *  \brief Add IPTV Multicast DSID to IPTV bridge DSID DB.
 *  \param[in] mdfInfoFromUser - All DSID info relevent for IPTV bridge.
 *  \param[out] no output.
 *  \return 0 or error status.
 **************************************************************************/
static int IpTvDsid_Add(IpTvBridgeDsidInfo_t mdfInfoFromUser)
{
    Uint8 * pAddress;

    /* Check if IPTV Bridge DSID DB is full */
    if (ipTvBrgDsidDb.ipTvCurrDsidNum == IPTVBR_MDF_MAX_DSIDS)
    {
        printk(KERN_ERR"\nFailed to add Multicast DSID index %d - IPTV Bridge DSID DB is full\n", mdfInfoFromUser.dsidPdsp1FwIndex);
        return -ECANCELED;
    }

    /* Check if DSID index exist */
    if (ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].isDsidValid == True)
    {
        printk(KERN_ERR"\nFailed to add Multicast DSID - DSID index %d alreay exist [total iptv dsid =%d]\n", mdfInfoFromUser.dsidPdsp1FwIndex,ipTvBrgDsidDb.ipTvCurrDsidNum);
        return -ECANCELED;
    }

    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidInPkt     = 0;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidDsPortMap = mdfInfoFromUser.dsidDsPortMap;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidLinuxCmim = mdfInfoFromUser.dsidLinuxCmim;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidVal       = mdfInfoFromUser.dsidVal;
    /* handle MAC address*/
    pAddress = ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].macAddress;
    memcpy((void *)pAddress, (void *)mdfInfoFromUser.macAddress, MAC_ADDR_LEN);  

    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].isDsidValid   = True;

    ipTvBrgDsidDb.ipTvCurrDsidNum++;
#ifdef IPTV_BRG_DSID_DEBUG
    printk(KERN_INFO"\nIPTV Bridge added new Multicast DSID succeed - DSID index %d DSID val %d\n", mdfInfoFromUser.dsidPdsp1FwIndex,mdfInfoFromUser.dsidVal);
#endif
    return 0;


}

/**************************************************************************/
/*! \fn static int IpTvDsid_Delete(IpTvBridgeDsidInfo_t mdfInfoFromUser)                              
 **************************************************************************
 *  \brief Delete IPTV Multicast DSID from IPTV bridge DSID DB.
 *  \param[in] mdfInfoFromUser - All DSID info relevent for IPTV bridge.
 *  \param[out] no output.
 *  \return 0 or error status.
 **************************************************************************/
static int IpTvDsid_Delete(IpTvBridgeDsidInfo_t mdfInfoFromUser)
{
    Uint8 * pAddress;
   
    /* Check if DSID index not exist */
    if (ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].isDsidValid == False)
    {
        printk(KERN_ERR"\nFailed to Delete Multicast DSID - DSID index %d NOT exist [total iptv dsid =%d]\n", mdfInfoFromUser.dsidPdsp1FwIndex,ipTvBrgDsidDb.ipTvCurrDsidNum);
        return -ECANCELED;
    }
    
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].isDsidValid   = False;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidInPkt     = 0;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidDsPortMap = 0;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidLinuxCmim = 0;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidVal       = 0;
    /* handle MAC Address */
    pAddress = ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].macAddress;
    memset((void *)pAddress, 0, MAC_ADDR_LEN);

    ipTvBrgDsidDb.ipTvCurrDsidNum--;
#ifdef IPTV_BRG_DSID_DEBUG
    printk(KERN_INFO"\nIPTV Bridge delete Multicast DSID succeed - DSID index %d \n", mdfInfoFromUser.dsidPdsp1FwIndex);
#endif
    return 0;
}


/**************************************************************************/
/*! \fn static int IpTvDsid_Modify(IpTvBridgeDsidInfo_t mdfInfoFromUser)                              
 **************************************************************************
 *  \brief Modify an IPTV Multicast DSID in IPTV bridge DSID DB.
 *  \param[in] mdfInfoFromUser - All DSID info relevent for IPTV bridge.
 *  \param[out] no output.
 *  \return 0 or error status.
 **************************************************************************/
static int IpTvDsid_Modify(IpTvBridgeDsidInfo_t mdfInfoFromUser)
{
    /* Check if DSID index not exist */
    if (ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].isDsidValid == False)
    {
        printk(KERN_ERR"\nFailed to Modify Multicast DSID - DSID index %d NOT exist\n", mdfInfoFromUser.dsidPdsp1FwIndex);
        return -ECANCELED;
    }

    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidDsPortMap = mdfInfoFromUser.dsidDsPortMap;
    ipTvBrgDsidDb.ipTvDsidInfo[mdfInfoFromUser.dsidPdsp1FwIndex].dsidLinuxCmim = mdfInfoFromUser.dsidLinuxCmim;

#ifdef IPTV_BRG_DSID_DEBUG
    printk(KERN_INFO"\nIPTV Bridge Modify Multicast DSID succeed- DSID index %d \n", mdfInfoFromUser.dsidPdsp1FwIndex);
#endif
    return 0;
}

/**************************************************************************/
/*! \fn static int IpTvDsid_Action(unsigned int cmd, IpTvBridgeDsidInfo_t mdfInfoFromUser)                           
 **************************************************************************
 *  \brief Calls the Add/Del/Modify... dsid functions according to the input command.
 *  \param[in] cmd - The command ID.
 *  \param[in] mdfInfoFromUser - All DSID info relevent for IPTV bridge.
 *  \param[out] no output.
 *  \return 0 or error status.
 **************************************************************************/
static int IpTvDsid_Action(unsigned int cmd, IpTvBridgeDsidInfo_t mdfInfoFromUser)
{
    switch (cmd) 
    {
    case IPTV_DSID_ADD:
       return (IpTvDsid_Add(mdfInfoFromUser));
       break;
    case IPTV_DSID_DEL:
       return (IpTvDsid_Delete(mdfInfoFromUser));
       break;
    case IPTV_DSID_MODIFY:
       return (IpTvDsid_Modify(mdfInfoFromUser));
       break;
    default:
       printk(KERN_ERR"\nIPTV Bridge DSID unknown ioctl command %d\n",cmd);
       return -EINVAL;
    }

    return 0;
}

