/*
 *
 * vfe_iptv_leg_mcast.h
 * Description:
 * Declaration of functions and types/ioctl used in the IPTV Legacy Multicast.
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

#ifndef _VFE_IPTV_LEGMCST_H_
#define _VFE_IPTV_LEGMCST_H_

#include "_tistdtypes.h"

/**************************************************************************/
/*      INTERFACE  Defines and Structs                                    */
/**************************************************************************/

/*****************************************/
/* Definitions                           */
/*****************************************/

/*! \var typedef struct IpTvBridgeMcastParams_t
    \brief Structure defines the Legacy Multicast configuration ioctl.
*/
typedef struct IpTvLegMcastParams_s 
{
	Uint32 dsPortMap;    /* Defines for each DSID from which DS-ports it is accepted */     
   /*  CMIM interfaces indexed (in Linux Kernel numeration) bitmap         */
   /*  a bit mask representing the Linux interfaces of the CM to which the   */
   /*  CM is to forward IPTV traffic associated with the multicast     */
   Uint32 linuxCmim;
   Uint8  macAddress[MAC_ADDR_LEN]; /* The Multicast Dest MAC address corresponding to this multicast */
} IpTvLegMcastParams_t;

/*! \var typedef struct IpTvDsidInfo_t
    \brief Structure defines the DSID counter (per DSID).
*/
typedef struct IpTvLegMcastCount_s
{
   Uint32 inputFrames;
   Uint8  macAddress[MAC_ADDR_LEN]; /* The Multicast Dest MAC address corresponding to this multicast */
} IpTvLegMcastCount_t;


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

/* IPTV Legacy Multicast IOCTLs */
#define	IPTV_LEGMCST_ADD          _IOW(IPTVBRG_LEGMCST_MODULE_ID, 1, IpTvLegMcastParams_t)
#define	IPTV_LEGMCST_DEL          _IOW(IPTVBRG_LEGMCST_MODULE_ID, 2, IpTvLegMcastParams_t)
#define	IPTV_LEGMCST_MODIFY       _IOW(IPTVBRG_LEGMCST_MODULE_ID, 3, IpTvLegMcastParams_t)
#define	IPTV_LEGMCST_PRINT_DB     _IO(IPTVBRG_LEGMCST_MODULE_ID,  4)
#define	IPTV_LEGMCST_RESET_DB     _IO(IPTVBRG_LEGMCST_MODULE_ID,  5)


/* IPTV_LEGMCST_MAXNR: In case of adding new Legacy Mcast IPTV-Bridge IOCTL commands this number must be updated */
#define IPTV_LEGMCST_MAXNR  5
#ifdef __KERNEL__
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

int IptvLegMcast_Receive(struct sk_buff* skb, Uint32 dsPort,struct net_device * IPTV_net_device,network_Type_e dev_type);

/**************************************************************************/
/*! \fn int IptvLegMcast_Init(void)
 **************************************************************************
 *  \brief Init IPTV legacy multicast module
 *  \return  0 or error code
 */
int IptvLegMcast_Init(void);

/**************************************************************************/
/*! \fn int IptvLegMcast_Cleanup(void)
 **************************************************************************
 *  \brief Cleanup IPTV legacy multicast module
 *  \return  0 or error code
 */
int IptvLegMcast_Cleanup(void);

/**************************************************************************/
/*! \fn int IptvLegMcast_Actions(unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief Main function for the IPTV Legacy Mcast module configuration.
 *  \param[in]   cmd - The config command ID.
 *  \param[in]   arg - The config command argument (user data).
 *  \return 0 or error status.
 */
int IptvLegMcast_Actions(unsigned int cmd, unsigned long arg);


#endif /*__KERNEL__*/
#endif /*_VFE_IPTV_LEGMCST_H_*/

