/*
 *
 * vfe_iptv_brg.c
 * Description:
 * IPTV bridge implementation
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

/*! \file vfe_iptv_brg.c
    \brief IPTV Bridge implementation
*/

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/
#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>		/* Needed for the macros */
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm-generic/errno-base.h>
#include <linux/uaccess.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include "pal.h"
#include "vfe_iptv_brg.h"
#include "vfe_iptv_dsid.h"
#include "vfe_iptv_legacy_mcast.h"
#include "dbridge_db.h"
#include "dbridge_netdev.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/
extern int Dbridge_register_IpTvRecCB(int (*IpTvBridge_Receive)(struct sk_buff *skb));
extern int Dbridge_unregister_IpTvRecCB(void);
extern DbridgeDevice_t* DbridgeDb_DevGetByInterface (struct net_device* ptr_interface);


/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/
#define IPTVBR_CDEV_MIN_MINOR       0
#define IPTVBR_CDEV_NUM_DEVICES     1

/* Network device for receiving IPTV packets */
struct net_device *IPTV_net_device = NULL;

typedef struct IpTvBridgeDataBase
{
    /* Global IPTV information */
    Uint32 ipTvMdfMode;     /* IPTV bridge MDF mode 1- MDF Enable 0- MDF Disabled  */
    Bool   ipTvBrgEnable;   /* IPTV bridge mode Enable/Disable */
    Uint32 ipTvDsPortMap;   /* The IPTV DS ports bit map */
    Char ipTvNetDevName[IFNAMSIZ];   /* IPTV network device name to receive IPTV packets */
    network_Type_e devType;         /* IPTV network device type (Bridge or End device) */

    /* IPTV Bridge Counters */
    Uint32 ipTvInPkt;      /* Total IPTV Input packets recived from IPTV DS channel to this Bridge. */
    Uint32 ipTvDropPkt;    /* Total IPTV Dropped packets in this Bridge. */

} IpTvBridgeDataBase_t;

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/
static int  create_iptv_brg_cdev(void);
static void delete_iptv_brg_cdev(void);
static long  iptv_bridge_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static inline void iptv_brg_PrintDb(void);

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/
static IpTvBridgeDataBase_t ipTvBridgeDb;

static struct cdev *iptv_bridge_cdev;
static dev_t iptv_bridge_dev_numbers;
static struct file_operations iptv_bridge_cdev_fops =
{
    .owner = THIS_MODULE,
    .unlocked_ioctl = iptv_bridge_ioctl,
};

/* To use semaphore locking, define VFE_PROTECT_SEMA, otherwise, interrupt locking is used. */
/* Use interrupt locking since scheduling is disabled in the packet RX path, 
    semaphores create kernel BUG reports and eventually oops. 
    Protection imported from docsis bridge
*/
//#define VFE_PROTECT_SEMA

#ifdef VFE_PROTECT_SEMA
/* IOCTL is locked during packet flow using this semaphore*/
static struct semaphore  ioctlSem; 

#define VFE_PROT_DCL(_sav) 

#define VFE_PROT_ON(_sav) down(&ioctlSem)

#define VFE_PROT_OFF(_sav) up(&ioctlSem)

#else
#define VFE_PROT_DCL DBRALT_PROT_DCL

#define VFE_PROT_ON DBRALT_PROT_ON

#define VFE_PROT_OFF DBRALT_PROT_OFF

#endif

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/

/**************************************************************************/
/*! \fn void IpTvBridge_receive (struct sk_buff *skb)                                     
 **************************************************************************
 *  \brief The function is the entry point by which packets are pushed into  
 *  \brief the IPTV Bridge. 
 *  \param[in] skb.
 *  \param[out] no output.
 *  \return -1 if this is not IPTV pkt or 0 if this is IPTV pkt (docsis will not process it).
 **************************************************************************/
int IpTvBridge_receive (struct sk_buff *skb)
{
    Uint32            ds_channel_id;
    DbridgeDevice_t   *ptr_dbridge_dev;
    VFE_PROT_DCL(lockKey);

    /* before handling the packet and accessing the DB - take the ioctl semaphore */
    VFE_PROT_ON(lockKey);
    
    if (ipTvBridgeDb.ipTvBrgEnable == IPTVBR_BRG_OFF)
    {
#ifdef IPTV_BRG_DEBUG
        printk("\nIPTV Bridge is Disabled\n");
#endif
        /* release the semaphore */
        VFE_PROT_OFF(lockKey);

        return -1;
    }
    /* Get the DOCSIS Device information. */
    ptr_dbridge_dev = DbridgeDb_DevGetByInterface (skb->dev);
    if (ptr_dbridge_dev == NULL)
    {
        printk("\nIPTV Bridge ptr_dbridge_dev is NULL\n");

        /* release the semaphore */
        VFE_PROT_OFF(lockKey);

        return -1;
    }
    
    switch(ptr_dbridge_dev->net_dev_type)
    {
    case DBR_CABLE_NET_DEV_TYPE: /* The IF IPTV works with */
        break; 
    default:
#ifdef IPTV_BRG_DEBUG
        printk("\nIPTV Bridge net_dev_type is not DBR_CABLE_NET_DEV_TYPE (%d)\n",ptr_dbridge_dev->net_dev_type);
#endif
        /* release the semaphore */
        VFE_PROT_OFF(lockKey);

        return -1; /* No IPTV processing if packet came from other IF */
    }
    
    /* Check if this pkt is from an IPTV DS port. */
    ds_channel_id = IPTVBR_GET_CHANNEL_ID_FROM_CNID(skb->ti_meta_info);
    if (!IPTVBR_IS_CHANNEL_ID_OK(ds_channel_id,ipTvBridgeDb.ipTvDsPortMap))
    {
#ifdef IPTV_BRG_DEBUG
        printk("\nIPTV Bridge Not an IPTV DS port (port=%d)\n",ds_channel_id);
#endif
        /* release the semaphore */
        VFE_PROT_OFF(lockKey);

        return -1; /* Not an IPTV DS port */
    }
    
    /* OK -> packet is from CABLE and from an IPTV DS port. */
    ipTvBridgeDb.ipTvInPkt++; /* Total IPTV IN pkts (including drop pkts) */

    if (ipTvBridgeDb.ipTvMdfMode == IPTVBR_MDF_ON)
    {
        if (iptv_dsid_handler(skb,ds_channel_id,IPTV_net_device, ipTvBridgeDb.devType))
        {
            if(IptvLegMcast_Receive(skb,ds_channel_id,IPTV_net_device, ipTvBridgeDb.devType))
            {
                kfree_skb (skb); /* Drop the packet -- The IPTV handler failed */
                ipTvBridgeDb.ipTvDropPkt++; /* Total IPTV drop pkt */
            }
        }
    }
    else
    {
        if(IptvLegMcast_Receive(skb,ds_channel_id,IPTV_net_device, ipTvBridgeDb.devType))
        {
            kfree_skb (skb); /* Drop the packet -- The IPTV handler failed */
            ipTvBridgeDb.ipTvDropPkt++; /* Total IPTV drop pkt */
        }
    }

    /* release the semaphore */
    VFE_PROT_OFF(lockKey);

    return 0; /* Docsis Brg will not process this packet */
}

/**************************************************************************/
/*! \fn static int __init IpTvBridge_Init(void)
 **************************************************************************
 *  \brief IPTV Bridge initialization.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return 0 or error code.
 **************************************************************************/
static int __init IpTvBridge_Init(void)
{
    int ret;
    IPTV_net_device = dev_get_by_name (&init_net,LBRIGE_CONNECTION_DEV_NAME);
    if (IPTV_net_device == NULL)
    {
        printk("Failed to locate device by name\n");
        return -1;
    }
    ret = create_iptv_brg_cdev();

    if (ret)
    {
        printk("\nFailed to create IPTV Bridge char device\n");
        return ret;
    }

    memset(&ipTvBridgeDb,0,sizeof(IpTvBridgeDataBase_t));

#ifdef VFE_PROTECT_SEMA
    /* Initialize ioctl semaphore */
    sema_init(&ioctlSem,1);
#endif

    /* Init both legacy and DSID mcast modules since we don't know the MDF mode at this stage */
    iptv_dsid_init();
    IptvLegMcast_Init();

    Dbridge_register_IpTvRecCB(IpTvBridge_receive);

    printk("\nIPTV Bridge char device init done\n");
        
    return 0;
}

/**************************************************************************/
/*! \fn static void __exit IpTvBridge_Exit(void)
 **************************************************************************
 *  \brief IPTV Bridge exit.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return 0 or error code.
 **************************************************************************/
static void __exit IpTvBridge_Exit(void)
{
    Dbridge_unregister_IpTvRecCB();

    delete_iptv_brg_cdev();

    printk(KERN_INFO"\nIPTV Bridge char device exit done\n");
}

/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/

/**************************************************************************/
/*! \fn static int create_iptv_brg_cdev(void)
 **************************************************************************
 *  \brief Create IPTV Bridge kernel module character device.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return 0 or error code.
 **************************************************************************/
static int create_iptv_brg_cdev(void)
{
    /* Allocate major and minor numbers for the device */
    if (alloc_chrdev_region(&iptv_bridge_dev_numbers, IPTVBR_CDEV_MIN_MINOR, 
                            IPTVBR_CDEV_NUM_DEVICES, IPTVBR_CDEV_NAME))
    {
        printk(KERN_ERR"\nFailed to allocate IPTV Bridge char device numbers\n");
        return -ENODEV;
    }

    /* Allocate the device */
    iptv_bridge_cdev = cdev_alloc();

    if (!iptv_bridge_cdev) 
    {
        printk(KERN_ERR"\nFailed to allocate IPTV Bridge char device\n");
        unregister_chrdev_region(iptv_bridge_dev_numbers, IPTVBR_CDEV_NUM_DEVICES);
        return -ENOMEM;
    }

    /* Init device structure */
    iptv_bridge_cdev->ops = &iptv_bridge_cdev_fops;
    iptv_bridge_cdev->owner = THIS_MODULE;

    /* Add the device to the kernel */
    if (cdev_add(iptv_bridge_cdev, iptv_bridge_dev_numbers, IPTVBR_CDEV_NUM_DEVICES))
    {
        printk(KERN_ERR"\nFailed to add IPTV Bridge char device\n");
        kfree(iptv_bridge_cdev);
        unregister_chrdev_region(iptv_bridge_dev_numbers, IPTVBR_CDEV_NUM_DEVICES);
        return -ENODEV;
    }

    return 0;
}

/**************************************************************************/
/*! \fn static void delete_iptv_brg_cdev(void)
 **************************************************************************
 *  \brief Delete IPTV Bridge kernel module character device.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return NA.
 **************************************************************************/
static void delete_iptv_brg_cdev(void)
{
    cdev_del(iptv_bridge_cdev);
    unregister_chrdev_region(iptv_bridge_dev_numbers, IPTVBR_CDEV_NUM_DEVICES);
}

/**************************************************************************/
/*! \fn static int iptv_brg_actions(unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief Main function for the IPTV bridge configuration.
 *  \param[in]   cmd - The config command ID.
 *  \param[in]   arg - The config command argument (user data).
 *  \return 0 or error status.
 */
static int iptv_brg_actions(unsigned int cmd, unsigned long arg)
{
   if (_IOC_NR(cmd) > IPTV_BRG_GLOBAL_MAXNR) 
      return -ENOTTY;

   /* Process the data */
   switch (cmd) 
   {
   /* Global IPTV Setting */
   case IPTV_SET_GLOBAL_BRIDGE_MODE:
      if (copy_from_user(&ipTvBridgeDb.ipTvBrgEnable, (void __user *)arg, sizeof(ipTvBridgeDb.ipTvBrgEnable)))
      {
         printk(KERN_ERR"\nIPTV Bridge failed to copy from user [ipTvBrgEnable]\n");
         return -EFAULT;
      }
      break;
   case IPTV_SET_GLOBAL_DS_PORTS:
      if (copy_from_user(&ipTvBridgeDb.ipTvDsPortMap, (void __user *)arg, sizeof(ipTvBridgeDb.ipTvDsPortMap)))
      {
         printk(KERN_ERR"\nIPTV Bridge failed to copy from user [ipTvDsPortMap]\n");
         return -EFAULT;
      }
      break;
   case IPTV_BRIDGE_GLOBAL_RESET:
      memset(&ipTvBridgeDb,0,sizeof(ipTvBridgeDb));
      printk(KERN_INFO"\nIPTV Bridge DB reset done\n");
      break;
   case IPTV_PRINT_GLOBAL_DB:
      iptv_brg_PrintDb();
      break;
   case IPTV_SET_MFD_MODE:
	  if (copy_from_user(&ipTvBridgeDb.ipTvMdfMode, (void __user *)arg, sizeof(ipTvBridgeDb.ipTvMdfMode)))
	  {
		 printk(KERN_ERR"\nIPTV Bridge failed to copy from user [ipTvMdfMode]\n");
		 return -EFAULT;
	  }
	  break;
   case IPTV_SET_NET_DEV_NAME:
       if (copy_from_user(&ipTvBridgeDb.ipTvNetDevName, (void __user *)arg, sizeof(ipTvBridgeDb.ipTvNetDevName)))
       {
		 printk(KERN_ERR"\nIPTV Bridge failed to copy from user [ipTvNetDevName]\n");
		 return -EFAULT;
       }
       IPTV_net_device = dev_get_by_name (&init_net,ipTvBridgeDb.ipTvNetDevName);
       if(IPTV_net_device==NULL)
       {
           printk(KERN_ERR"unknown device %s\n",ipTvBridgeDb.ipTvNetDevName);
           return -EFAULT; 
       }
       dev_put(IPTV_net_device);
       break;
   case IPTV_SET_NET_DEV_TYPE:
       if (copy_from_user(&ipTvBridgeDb.devType, (void __user *)arg, sizeof(ipTvBridgeDb.devType)))
       {
		 printk(KERN_ERR"\nIPTV Bridge failed to copy from user [ipTvNetDevName]\n");
		 return -EFAULT;
       }
       break;
   default:
      printk(KERN_ERR"\nIPTV Bridge unknown ioctl command %d\n",cmd);
      return -EINVAL;
   }
   return 0;
}

   
/**************************************************************************/
/*! \fn static int iptv_bridge_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief IPTV Bridge character device ioctl callback.
 *  \param[in] struct inode *inode - device inode.
 *  \param[in] struct file *filp - device file pointer.
 *  \param[in] unsigned int cmd - ioctl command id.
 *  \param[in/out] unsigned long arg - pointer to input/output buffer in user space.
 *  \return 0 or error code.
 **************************************************************************/
static long iptv_bridge_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    IpTvBrg_ModuleIds_e ipTvBrgModuleId;
    int err = 0;
    int rc;
    VFE_PROT_DCL(lockKey);
   
    /*
    * the direction is a bitmask, and VERIFY_WRITE catches R/W
    * transfers. `Type' is user-oriented, while
    * access_ok is kernel-oriented, so the concept of "read" and
    * "write" is reversed
    */
    if (_IOC_DIR(cmd) & _IOC_READ)
       err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
       err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (err)
    {
       printk(KERN_ERR"\nIPTV Bridge failed to use user data. IOCTL=%d\n",cmd);
       return -EFAULT;
    }

    /* take the ioctl semaphore */
    VFE_PROT_ON(lockKey);

    /* 
	 * Go over all IPTV Bridge modules and run the required ioctl callback
	 */
    ipTvBrgModuleId = (IpTvBrg_ModuleIds_e)_IOC_TYPE(cmd);
    switch (ipTvBrgModuleId)
    {
    case IPTVBRG_GLOBAL_MODULE_ID:
       rc = iptv_brg_actions(cmd,arg);
       break;
    case IPTVBRG_DSID_MODULE_ID:
	    rc = iptv_dsid_actions(cmd,arg);
		break;
	case IPTVBRG_LEGMCST_MODULE_ID:
        rc = IptvLegMcast_Actions(cmd,arg);
	    break;
    default:
       printk(KERN_ERR"\nIPTV Bridge unknown module id %d\n",ipTvBrgModuleId);
       rc = (-EINVAL);
    }

    /* release the semaphore */
    VFE_PROT_OFF(lockKey);

    return rc;
}

/**************************************************************************/
/*! \fn static inline void iptv_brg_PrintDb(void)                                
 **************************************************************************
 *  \brief Print IPTV DSID DB.
 **************************************************************************/
static inline void iptv_brg_PrintDb(void)
{
   printk("\n"); 
   printk("IPTV Bridge mode:           %s\n", (ipTvBridgeDb.ipTvBrgEnable==IPTVBR_BRG_ON ? "Enable":"Disable") ); 
   printk("IPTV MDF mode:              %s\n", (ipTvBridgeDb.ipTvMdfMode==IPTVBR_MDF_ON ? "Enable":"Disable") ); 
   printk("IPTV DS Port Map:           0x%x\n", ipTvBridgeDb.ipTvDsPortMap); 
   printk("IPTV Total In packets:      %u\n", ipTvBridgeDb.ipTvInPkt);
   printk("IPTV Total Drop packets:    %u\n", ipTvBridgeDb.ipTvDropPkt);
   printk("\n"); 
}

module_init(IpTvBridge_Init);
module_exit(IpTvBridge_Exit);
MODULE_AUTHOR("Texas Instruments, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPTV Bridge");


