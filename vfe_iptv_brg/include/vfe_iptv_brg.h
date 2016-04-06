/*
 *
 * vfe_iptv_brg.h
 * Description:
 * Declaration of functions and types/ioctl used in the IPTV Bridge.
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

#ifndef _VFE_IPTV_BRG_H_
#define _VFE_IPTV_BRG_H_

#include "_tistdtypes.h"

/**************************************************************************/
/*      INTERFACE  Defines and Structs                                    */
/**************************************************************************/
#define IPTVBR_CDEV_NAME      "vfe_iptv_bridge"
#define IPTVBR_DEV_NAME_API   "/dev/vfe_iptv_bridge0"


#define IPTVBR_BRG_ON       (1)
#define IPTVBR_BRG_OFF      (0)

#ifndef MAC_ADDR_LEN
#define MAC_ADDR_LEN        (6)
#endif

/*! \def typedef enum IpTvBrg_ModuleIds
 *  \brief Enumerates the internal IPTV Bridge sub-modules.
 */
typedef enum IpTvBrg_ModuleIds
{
   IPTVBRG_MODULE_START       = 0,
   IPTVBRG_GLOBAL_MODULE_ID,  /*1*/ 
   IPTVBRG_DSID_MODULE_ID,    /*2*/
   IPTVBRG_LEGMCST_MODULE_ID, /*3*/
   /* --- Add new IPTV Bridge Modules here --- */
   IPTVBRG_MODULE_EOF  /* Total number of modules */

} IpTvBrg_ModuleIds_e;

/*! \def typedef enum network_Type
 *  \brief Enumerates the network device types.
 */
typedef enum network_Type
{
    NETWORK_LSD_DEVICE = 0,
    NETWORK_BRIDGE_DEVICE 
}network_Type_e;

/*! \def IPTVBR_IS_MCAST_BIT_ON(x)
 *  \brief Extract from X (skb->ti_meta_info) the Mcast bit
 */
#define MCAST_FIELD_MASK                     (0x80)
#define IPTVBR_IS_MCAST_BIT_ON(x)            ((x)  & MCAST_FIELD_MASK)

/*! \def IPTVBR_GET_CHANNEL_ID_FROM_CNID(x)
 *  \brief Extract from X (skb->ti_meta_info) the DS channel id (from 0-N).
 */
#define CNID_DS_CHANNEL_ID_BIT_FIELD_OFFSET  (8)
#define IPTVBR_GET_CHANNEL_ID_FROM_CNID(x)   (((x) >> CNID_DS_CHANNEL_ID_BIT_FIELD_OFFSET) & 0xff)

/*! \def IPTVBR_IS_CHANNEL_ID_OK(x)
 *  \brief Checks if the bit number K (K=inPort) on B (B=dsidMap) is on.
 */
#define IPTVBR_IS_CHANNEL_ID_OK(inPort,dsidMap) ((1 << (inPort)) & (dsidMap))

/*****************************************/
/* Definitions                           */
/*****************************************/

#define IPTVBR_MDF_ON       (1)
#define IPTVBR_MDF_OFF      (0)

/* IOCTL commands:

   If you are adding new ioctl's to the kernel, you should use the _IO
   macros defined in <linux/ioctl.h> _IO macros are used to create ioctl numbers:

	_IO(type, nr)         - an ioctl with no parameter. 
   _IOW(type, nr, size)  - an ioctl with write parameters (copy_from_user), kernel would actually read data from user space 
   _IOR(type, nr, size)  - an ioctl with read parameters (copy_to_user), kernel would actually write data to user space 
   _IOWR(type, nr, size) - an ioctl with both write and read parameters 

   'Write' and 'read' are from the user's point of view, just like the
	system calls 'write' and 'read'.  For example, a SET_FOO ioctl would
	be _IOW, although the kernel would actually read data from user space;
	a GET_FOO ioctl would be _IOR, although the kernel would actually write
	data to user space.

	The first argument to _IO, _IOW, _IOR, or _IOWR is an identifying letter
	or number from the SoC_ModuleIds_e enum located in this file. 

	The second argument to _IO, _IOW, _IOR, or _IOWR is a sequence number
	to distinguish ioctls from each other.  

   The third argument to _IOW, _IOR, or _IOWR is the type of the data going
   into the kernel or coming out of the kernel (e.g.  'int' or 'struct foo').

   NOTE!  Do NOT use sizeof(arg) as the third argument as this results in 
   your ioctl thinking it passes an argument of type size_t.

*/

/* Global IPTV-Bridge IOCTLs */
#define	IPTV_SET_GLOBAL_BRIDGE_MODE   _IOW(IPTVBRG_GLOBAL_MODULE_ID, 1, Uint32)
#define	IPTV_SET_GLOBAL_DS_PORTS      _IOW(IPTVBRG_GLOBAL_MODULE_ID, 2, Uint32)
#define	IPTV_BRIDGE_GLOBAL_RESET      _IO(IPTVBRG_GLOBAL_MODULE_ID,  3)
#define	IPTV_PRINT_GLOBAL_DB          _IO(IPTVBRG_GLOBAL_MODULE_ID,  4)
#define	IPTV_SET_MFD_MODE             _IOW(IPTVBRG_GLOBAL_MODULE_ID, 5, Uint32)
#define IPTV_SET_NET_DEV_NAME         _IOW(IPTVBRG_GLOBAL_MODULE_ID, 6, Uint32)
#define IPTV_SET_NET_DEV_TYPE         _IOW(IPTVBRG_GLOBAL_MODULE_ID, 7, Uint32)

/* IPTV_BRG_GLOBAL_MAXNR: In case of adding new IPTV-Bridge Global IOCTL commands this number must be update */
#define IPTV_BRG_GLOBAL_MAXNR  7

#endif
