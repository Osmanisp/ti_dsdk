/*! \file dsg_ni.h
    \brief 
****************************************************************************/
#ifndef __DSG_NI_H__
#define __DSG_NI_H__

#include <autoconf.h>
#include <linux/skbuff.h>

#define CONFIG_HTX_DSG_TUNNEL_INTERFACE   "dsgtn0"
#define CONFIG_HTX_DSG_IP_INTERFACE   "dsgip0"

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/
int dsgTunnel_rx (struct sk_buff *skb);

#endif /* __DSG_NI_H__ */
