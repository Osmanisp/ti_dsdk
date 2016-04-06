/*
 *
 * dbridge_db.c
 * Description:
 * DOCSIS bridge database implementation
 *
 * Copyright (C) 2008 Texas Instruments Incorporated - http://www.ti.com/ 
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

#define _DBRIDGE_DB_C_

/*! \file dbridge_db.c
    \brief the docsis bridge database
*/

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/ti_hil.h>

#include "dbridge_common.h"
#include "dbridge_db.h"
#include "dbridge_hal_filter.h"
#include "dbridge_igmp.h"
#include "dbridge_l2vpn_ds_db.h"
#include "dbridge_mcast.h"
#include "dbridge_mdf_db.h"
#include "dbridge_netdev.h"
#include "dbridge_mac_ageing.h"
#include "pal.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/

#define MIN_NUM_CPE 1 

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/

/*  Structure which keeps track of all the information for the DOCSIS Bridge. */
DbridgeDataBase_t dbridge_db;
static DbridgeCounters_t  dbridge_counters; 
static unsigned  int devices_table_insert;
static unsigned  int alt_table_insert;
static unsigned  int data_base_init = 0;
static unsigned  int max_cpe = MIN_NUM_CPE;
static unsigned  long long lbridge_interface_mask = 0LL ;
static unsigned  long long cpe_interface_mask = 0LL;
//CISCO ADD BEGIN
static unsigned  int cnt_mta = 1;
static unsigned  int mta_dev = 1;
//CISCO ADD END

static void DbrigdeDb_AltDelEntry(DbridgeAltEntry_t *altEntry);

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/

/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/
#ifdef CONFIG_TI_DOCSIS_EGRESS_HOOK
static int configure_docsis_egress_hook_handler(int reg);
static int Dbridge_egress_hook(struct sk_buff* skb);
static struct net_device *usb0_netdev;
static struct net_device *eth0_netdev;

#endif /* CONFIG_TI_DOCSIS_EGRESS_HOOK */

/**************************************************************************/
/*! \fn int DbridgeDb_Init(void)                                     
 **************************************************************************
 *  \brief DOCSIS bridge data base initialization.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDb_Init(void)
{
    int i;

    #ifdef DSG
    unsigned char dsgMode = DbridgeDB_GetDsgMode();/*save the dsg mode setting from dbridge_conf by proc*/
    #endif
    memset ((void *)&dbridge_db, 0, sizeof(DbridgeDataBase_t));
    memset ((void *)&dbridge_counters, 0, sizeof(DbridgeCounters_t));

    devices_table_insert=0;
    alt_table_insert=0;
    data_base_init=1;

    /* Put all ALT entries on the FREE list */
    INIT_LIST_HEAD(&DBRALT_FREE_HEAD);
    INIT_LIST_HEAD(&DBRALT_OCCUPIED_HEAD);
    for (i = 0; i < DBR_MAX_ALT_SIZE; i++)
    {
        /* Init cpe num, only DBR_MAX_CPE_ENTRY, all the rest get 0 */
        if (i >= (DBR_MAX_ALT_SIZE - DBR_MAX_CPE_ENTRY))
        {
            /* cpe num [1..DBR_MAX_CPE_ENTRY] */
            dbridge_db.alt.altTable[i].cpe_number = i - (DBR_MAX_ALT_SIZE - DBR_MAX_CPE_ENTRY) + 1;
        }
        else
        {
            /* 0 (the memset above does the trick, but this is cleaner...) */
            dbridge_db.alt.altTable[i].cpe_number = 0;
        }
        /* Make the initial order: */
        /* - Group of cpe_num == 0 */
        /* - cpe_num == 1 */
        /* - cpe_num == 2 */
        /* - cpe_num == ... */
        /* - cpe_num == DBR_MAX_CPE_ENTRY */
        /* This is done by add_tail, which results in 1st element 1st */
        list_add_tail(&dbridge_db.alt.altTable[i].listAltEntry, &DBRALT_FREE_HEAD);
    }
#ifdef DSG
    DbridgeDB_SetDsgMode(dsgMode);
#endif

    return 0; /* OK*/
}

/**************************************************************************/
/*! \fn int DbridgeDb_ReInit(void)                                     
 **************************************************************************
 *  \brief DOCSIS bridge data base re-initialization.
 *  \param[in] no input.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDb_ReInit(void)
{
#ifdef DSG
    unsigned int index;
#endif
    DbridgeAltEntry_t *altEntry;
    DbridgeAltEntry_t *next = NULL;
    unsigned int cpe_type;
    DBRALT_PROT_DCL(lockKey);

    /* Protect alt_table_insert */
	DBRALT_PROT_ON(lockKey);

    /* remove all cpe from alt table */
    list_for_each_entry_safe(altEntry, next, &DBRALT_OCCUPIED_HEAD, listAltEntry)
    {
        cpe_type = altEntry->macLearningFrom;
#ifdef DSG
        if ((cpe_type ==CPE_STATIC)||(cpe_type ==CPE_DYNAMIC)||(cpe_type ==CPE_ESAFE))
#else
        if ((cpe_type == CPE_STATIC) || (cpe_type == CPE_DYNAMIC))
#endif
        {
            /* Remove HW filter for unicast MAC address */   
            HAL_MacDsUnicastDataFilterDelete(altEntry->macAddress);

            DbrigdeDb_AltDelEntry(altEntry);
        }
    }

	DBRALT_PROT_OFF(lockKey);

    dbridge_counters.StaticCpe = 0;
    dbridge_counters.DynamicCpe = 0;
    max_cpe = MIN_NUM_CPE;

    /* remove all HW filters */

    /* remove all multicast filters */
    DbridgeDb_DelAllMcastMac();
#ifdef DSG
    unsigned char macAddress[MAC_ADDRESS_LEN];
    if(DbridgeDB_IsDsgEnabled())
    {
        for(index = 0; index< DbridgeDB_GetDsgTunnelNums(); index++)
        {
            if(DbridgeDB_GetDsgTunnelMacAddress(index,(unsigned char*)&macAddress) != 0)
                continue;
            DbridgeDb_AddMcastMac(macAddress,DSG_FILTER);
        }
    }
#endif

    return 0; /* OK*/
}

/**************************************************************************/
/*! \fn DDbridgeAltEntry_t* DbridgeDb_AltGetMemberByMac(unsigned char* mac_address)
 **************************************************************************
 *  \brief The function retreives the dbridge ALT table member for the  
 *          corresponding mac address.
 *      ****   NOTE ****
 *          Protection must be supplied by the caller
 *          Protection may end when the output pointer is no longer in use
 *  \param[in] cpe mac address.
 *  \return Success-pointer to the Dbridge ALT table, NULL-If no entry is found.
 **************************************************************************/
DbridgeAltEntry_t* DbridgeDb_AltGetMemberByMac(unsigned char* mac_address)
{
    DbridgeAltEntry_t *altEntry = NULL;
    DbridgeAltEntry_t *altResult = NULL;

    DBRALT_FOREACH_ALT_ENTRY(altEntry)
    {
        if (memcmp(mac_address, altEntry->macAddress, MAC_ADDRESS_LEN) == 0)
        {
            altResult = altEntry;
            if (altResult && altResult->sourceInterface)
            {
                DBRIDGE_IFINDEX_CHK_PR_MAC(altResult->sourceInterface->ifindex, mac_address, "sourceInterface %p, sourceInterface->name %s", altResult->sourceInterface, altResult->sourceInterface->name);
            }
            break;
        }
    }

    return altResult;
}
#if 1
/**************************************************************************/
/*! \fn DDbridgeAltEntry_t*  DbridgeDb_AltGetMemberByIndex(int index);
 **************************************************************************
 *  \brief The function retreives the dbridge ALT table member for the  
 *  \brief corresponding mac address.
 *  \param[in] ALt table index.
 *  \return Success-pointer to the Dbridge ALT table, NULL-If no entry is found.
 **************************************************************************/
DbridgeAltEntry_t*  DbridgeDb_AltGetMemberByIndex(int index)
{
    DbridgeAltEntry_t *retval = NULL;
    DBRALT_PROT_DCL(lockKey);

    /* Protect alt_table_insert */
    DBRALT_PROT_ON(lockKey);

    if (index >= 0 && index < alt_table_insert)
    {
        retval = &dbridge_db.alt.altTable[index];
        if (retval && retval->sourceInterface)
        {
            DBRIDGE_IFINDEX_CHK_PR_MAC(retval->sourceInterface->ifindex, retval->macAddress, "sourceInterface %p, sourceInterface->name %s", retval->sourceInterface, retval->sourceInterface->name);
        }
    }
    else
    {
        retval = NULL;
    }

    DBRALT_PROT_OFF(lockKey);

    return retval;
}

/**************************************************************************/
/*! \fn int DbridgeDb_AltGetInfoByIndex(int index, unsigned char* mac_address, Maclearning_e *mac_learn_from, DbridgeDevice_t *dbridge_Devices)
 **************************************************************************
 *  \brief Get information from ALT table.
 *  \param[in] ALT table index.
 *  \param[out] mac address ,
 *  \param[out] mac address learn from - NOT_CPE, CPE_STATIC or CPE_DYNAMIC,
 *  \param[out] pointer to DbridgeDevice.
 *  \return 0 or error status.
 **************************************************************************/
int DbridgeDb_AltGetInfoByIndex(int index, unsigned char* mac_address, Maclearning_e *mac_learn_from, DbridgeDevice_t *dbridge_Devices)
{
    int retval;
    DBRALT_PROT_DCL(lockKey);

    /* Protect alt_table_insert */
    DBRALT_PROT_ON(lockKey);

    if (index >= 0 || index <alt_table_insert)
    {
        *mac_learn_from=dbridge_db.alt.altTable[index].macLearningFrom;
        dbridge_Devices=dbridge_db.alt.altTable[index].dbridgeDevice;
        memcpy ((void *)mac_address,(void *)&dbridge_db.alt.altTable[alt_table_insert].macAddress[0], MAC_ADDRESS_LEN);
        retval = 0;
    }
    else
    {
        *mac_learn_from=NOT_LEARN;
        dbridge_Devices=NULL;
        retval = -1;
    }

    DBRALT_PROT_OFF(lockKey);

    return retval;
}
#endif
/**************************************************************************/
/*! \fn int DbridgeDb_AltAddMac(unsigned char* mac_address, Maclearning_e mac_learn_from,struct net_device *ptr_netdev, struct net_device*  sourceInterface)
 **************************************************************************
 *  \brief this function Add MAC address to ALT table or update device information for
 *  \brif existing cpe MAC that add from config file .
 *  \param[in] cpe mac address.
 *  \param[in] mac address from - NOT_CPE, CPE_STATIC or CPE_DYNAMIC,
 *  \param[in] pointer to docsis device table.
 *  \param[in] pointer to first interface where the packet received from.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDb_AltAddMac(unsigned char* mac_address, Maclearning_e mac_learn_from, 
                        DbridgeDevice_t *dbridge_Devices,struct net_device*  sourceInterface)
{   
    DbridgeAltEntry_t* search_dbridge_alt; 
    DbridgeAltEntry_t* ptr_dbridge_alt; 
    DBRALT_PROT_DCL(lockKey);
    int retval;
    int st = 1;

	DBRALT_PROT_ON(lockKey);
    ptr_dbridge_alt = DbridgeDb_AltGetMemberByMac(mac_address);
    if (ptr_dbridge_alt!=NULL)
    {
        /* MAC address exists update the corresponding device */
        if (mac_learn_from == CPE_DYNAMIC && ptr_dbridge_alt->dbridgeDevice==NULL)
        {
            ptr_dbridge_alt->dbridgeDevice= dbridge_Devices;
            if (ptr_dbridge_alt->sourceInterface)
            {
                DBRIDGE_IFINDEX_CHK_PR_MAC(ptr_dbridge_alt->sourceInterface->ifindex, ptr_dbridge_alt->macAddress,
                                           "Old source interface %p, name %s",
                                           ptr_dbridge_alt->sourceInterface, ptr_dbridge_alt->sourceInterface->name);
            }
            ptr_dbridge_alt->sourceInterface = sourceInterface;
            if (ptr_dbridge_alt->sourceInterface)
            {
                DBRIDGE_IFINDEX_CHK_PR_MAC(ptr_dbridge_alt->sourceInterface->ifindex, ptr_dbridge_alt->macAddress,
                                           "New source interface %p, name %s",
                                           ptr_dbridge_alt->sourceInterface, ptr_dbridge_alt->sourceInterface->name);
            }

            printk(KERN_INFO "Update source from %.2x-%.2x-%.2x-%.2x-%.2x-%.2x %s\n",
                   (unsigned char)mac_address[0],
                   (unsigned char)mac_address[1],
                   (unsigned char)mac_address[2],
                   (unsigned char)mac_address[3],
                   (unsigned char)mac_address[4],
                   (unsigned char)mac_address[5],
                   sourceInterface->name);
        }

        DBRALT_PROT_OFF(lockKey);

        return 0; 
    }

    /* check if we have place for one more cpe */
    if ((mac_learn_from == CPE_DYNAMIC) || (mac_learn_from == CPE_STATIC))
    {
//CISCO ADD BEGIN
        int minus_mta = 0;
        ptr_dbridge_alt = NULL;
        if ((cnt_mta == 0)||(mta_dev == 0)) {    /* do not count MTA as CPE */
            DBRALT_FOREACH_ALT_ENTRY(ptr_dbridge_alt) {
                if(ptr_dbridge_alt->sourceInterface) {
                    if (strcmp(ptr_dbridge_alt->sourceInterface->name, "mta0") == 0)
                        minus_mta = 1;
                }
            }
        }
//CISCO ADD END

        if ((dbridge_counters.StaticCpe + dbridge_counters.DynamicCpe - minus_mta) >= max_cpe) //CISCO MODIFY
        {
            DBRALT_PROT_OFF(lockKey);

            /* no place for more CPEs */
            printk(KERN_INFO "No room for %s ( static %d, dynamic %d,  max %d) with MAC %.2x-%.2x-%.2x-%.2x-%.2x-%.2x\n",
                   (mac_learn_from == CPE_DYNAMIC) ? "CPE_DYNAMIC" : "CPE_STATIC",
                   dbridge_counters.StaticCpe, dbridge_counters.DynamicCpe, max_cpe,
                   mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
            return -1;
        }
    }
#ifdef DSG
        if (mac_learn_from==CPE_ESAFE)
        {
            if ((dbridge_counters.StaticCpe + dbridge_counters.DynamicCpe - minus_mta) >= max_cpe)
            {
                printk("No place for more eSafe Device\n");
                return -1;
            }
        }
#endif

    /* Create new, keep protection on from above */
    ptr_dbridge_alt = NULL;
    /* Get free element */
    list_for_each_entry(search_dbridge_alt, &DBRALT_FREE_HEAD, listAltEntry)
    {
        /* Check if this is a "good" entry. */
        if (/* For CPE entries, we need cpe_num > 0, < MAX */
            (((mac_learn_from == CPE_DYNAMIC) || (mac_learn_from == CPE_STATIC)) &&
             ((search_dbridge_alt->cpe_number <= 0) || (search_dbridge_alt->cpe_number > DBR_MAX_CPE_ENTRY))) ||
            /* For non-CPE entries, we need cpe_num == 0 */
            ((mac_learn_from == DBRIDGE_INIT) && (search_dbridge_alt->cpe_number != 0)) )
        {
            /* Not good, try next */
            continue;
        }
        else
        {
            /* Found */
            ptr_dbridge_alt = search_dbridge_alt;
            break;
        }
    }
    /* Did we find ? */
    if (ptr_dbridge_alt != NULL)
    {
        /* Found */
        if (mac_learn_from==CPE_DYNAMIC)
            dbridge_counters.DynamicCpe++;
        else if (mac_learn_from==CPE_STATIC)
            dbridge_counters.StaticCpe++;

        memcpy (ptr_dbridge_alt->macAddress, mac_address, MAC_ADDRESS_LEN);
        /* add ptr to docsis device */
        ptr_dbridge_alt->dbridgeDevice = dbridge_Devices;

        if (ptr_dbridge_alt->sourceInterface)
        {
            DBRIDGE_IFINDEX_CHK_PR_MAC(ptr_dbridge_alt->sourceInterface->ifindex, ptr_dbridge_alt->macAddress,
                                       "Old source interface %p, name %s",
                                       ptr_dbridge_alt->sourceInterface, ptr_dbridge_alt->sourceInterface->name);
        }
        ptr_dbridge_alt->sourceInterface = sourceInterface;
        if (ptr_dbridge_alt->sourceInterface)
        {
            DBRIDGE_IFINDEX_CHK_PR_MAC(ptr_dbridge_alt->sourceInterface->ifindex, ptr_dbridge_alt->macAddress,
                                       "New source interface %p, name %s",
                                       ptr_dbridge_alt->sourceInterface, ptr_dbridge_alt->sourceInterface->name);
        }

        ptr_dbridge_alt->macLearningFrom=mac_learn_from;

        if (DBRIDGE_INIT != mac_learn_from)
        {
            /* open HW filter for unicast MAC address */   
            st = HAL_MacDsUnicastDataFilterAdd(mac_address);
        }

        /* Add at end, so the INIT types alway (?) appear 1st */
        list_move_tail(&ptr_dbridge_alt->listAltEntry, &DBRALT_OCCUPIED_HEAD);

        retval = 0;
    }

    DBRALT_PROT_OFF(lockKey);

    /* Protection is OFF, do not use ptr_dbridge_alt */
    /* Can stil check if it was NULL, but do not use it... */
    if (ptr_dbridge_alt == NULL)
    {
        /* Not found */
        printk(KERN_ERR "Could not add a new ALT entry for MAC %.2x-%.2x-%.2x-%.2x-%.2x-%.2x\n",
               mac_address[0], mac_address[1], mac_address[2],mac_address[3], mac_address[4], mac_address[5]);
        retval = -1;
    }
    else
    {
        /* Found */
        printk(KERN_INFO "Add to ALT (with%s source filter) %.2x-%.2x-%.2x-%.2x-%.2x-%.2x %s\n",
               st == 0 ? "" : "out",
               mac_address[0], mac_address[1], mac_address[2],mac_address[3], mac_address[4], mac_address[5],
               sourceInterface == NULL ? "noName" : sourceInterface->name);
        retval = 0;
    }

    return retval;
}

/**************************************************************************/
/*! \fn int DbridgeDb_AltDelMac(unsigned char* mac_address, 
            Maclearning_e mac_learn_from, char *dest_dev_name, char *src_dev_name, 
            unsigned char cpe_number, int net_dev_type,, DbridgeDb_FieldMask_e fieldMask)
 **************************************************************************
 *  \brief DEL MAC address from ALT table.
 *          All ALT entries are compared against the input params selected by the mask.
 *          Depending on DBRALT_MASK_ALL/DBRALT_MASK_ANY, entries that match all/any input
 *          are deleted.
 *  \param[in] cpe mac address.
 *  \param[in] mac address from - DBRIDGE_INIT, CPE_STATIC or CPE_DYNAMIC,
 *  \param[in] dest_dev_name - name of destination device
 *  \param[in] src_dev_name - name of source device
 *  \param[in] cpe_number
 *  \param[in] net_dev_type - device type, e.g. DBR_ESAFE_NET_DEV_TYPE, DBR_CMCI_NET_DEV_TYPE
 *  \param[in] field_mask - Which fields are used for deciding which CPEs to del
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDb_AltDelMac(unsigned char* mac_address,
                        Maclearning_e mac_learn_from, 
                        char *dest_dev_name,
                        char *src_dev_name,
                        unsigned char cpe_number,
                        int net_dev_type,
                        DbridgeDb_FieldMask_e field_mask,
                        DbridgeMacList_t *deletedMacList)
{   
    DbridgeAltEntry_t* ptr_dbridge_alt; 
    DbridgeAltEntry_t *next = NULL;
    DBRALT_PROT_DCL(lockKey);
    DbridgeDb_FieldMask_e actualMatch;

#define DBRALT_DEL_TEST_MATCH(__field, __match) \
    do \
    { \
        if (field_mask & __field) \
        { \
            if (__match) \
            { \
                /* Found match */ \
                if (field_mask & DBRALT_MASK_ANY) \
                { \
                    /* 1 match is enough, del & continue to next entry */ \
                    printk(KERN_INFO "DEL ALT %.2x-%.2x-%.2x-%.2x-%.2x-%.2x, match ANY && %s\n", \
                            ptr_dbridge_alt->macAddress[0], ptr_dbridge_alt->macAddress[1], ptr_dbridge_alt->macAddress[2], \
                            ptr_dbridge_alt->macAddress[3], ptr_dbridge_alt->macAddress[4], ptr_dbridge_alt->macAddress[5], \
                            #__match); \
                    DbrigdeDb_AltDelEntry(ptr_dbridge_alt); \
                    continue; \
                } \
                else \
                { \
                    /* Record match */ \
                    actualMatch |= __field; \
                } \
            } \
            else if (field_mask & DBRALT_MASK_ALL) \
            { \
                /* Single mismatch is enough to continue to next entry */ \
                continue; \
            } \
        } \
    } while (0)

    if ( ((field_mask & DBRALT_MASK_ALL) == 0) && ((field_mask & DBRALT_MASK_ANY) == 0) )
    {
        printk(KERN_ERR "Mask (0x%08X) does not specify neither ANY nor ALL\n", field_mask);
        return -1;
    }

	DBRALT_PROT_ON(lockKey);
    list_for_each_entry_safe(ptr_dbridge_alt, next, &DBRALT_OCCUPIED_HEAD, listAltEntry)
    {
        if (field_mask & DBRALT_MASK_ALL)
        {
            actualMatch = DBRALT_MASK_ALL;
        }
        else
        {
            actualMatch = 0;
        }

        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_MAC, 
                              (mac_address != NULL) && 
                              (memcmp(mac_address, ptr_dbridge_alt->macAddress, MAC_ADDRESS_LEN) == 0) );
        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_LEARNFROM, mac_learn_from == ptr_dbridge_alt->macLearningFrom);
        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_DESTDEV, 
                              (dest_dev_name != NULL) && 
                              (ptr_dbridge_alt->dbridgeDevice != NULL) && 
                              (ptr_dbridge_alt->dbridgeDevice->ptr_interface != NULL) && 
                              (ptr_dbridge_alt->dbridgeDevice->ptr_interface->name != NULL) && 
                              strcmp(dest_dev_name, ptr_dbridge_alt->dbridgeDevice->ptr_interface->name) == 0);
        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_SRCDEV, 
                              (src_dev_name != NULL) && 
                              (ptr_dbridge_alt->sourceInterface != NULL) && 
                              (ptr_dbridge_alt->sourceInterface->name != NULL) && 
                              strcmp(src_dev_name, ptr_dbridge_alt->sourceInterface->name) == 0);
        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_CPENUM, cpe_number == ptr_dbridge_alt->cpe_number);
        DBRALT_DEL_TEST_MATCH(DBRALT_MASK_DEVTYPE, 
                              (ptr_dbridge_alt->dbridgeDevice != NULL) &&
                              (ptr_dbridge_alt->dbridgeDevice->net_dev_type == net_dev_type));

        if ((field_mask & DBRALT_MASK_ALL) && (actualMatch == field_mask))
        {
            /* ALL && all fields match ==> DEL */
            printk(KERN_INFO "DEL ALT %.2x-%.2x-%.2x-%.2x-%.2x-%.2x, match ALL", 
                    ptr_dbridge_alt->macAddress[0], ptr_dbridge_alt->macAddress[1], ptr_dbridge_alt->macAddress[2], 
                    ptr_dbridge_alt->macAddress[3], ptr_dbridge_alt->macAddress[4], ptr_dbridge_alt->macAddress[5]);
            if (field_mask & DBRALT_MASK_MAC) printk(" && DBRALT_MASK_MAC");
            if (field_mask & DBRALT_MASK_LEARNFROM) printk(" && DBRALT_MASK_LEARNFROM");
            if (field_mask & DBRALT_MASK_DESTDEV) printk(" && DBRALT_MASK_DESTDEV");
            if (field_mask & DBRALT_MASK_SRCDEV) printk(" && DBRALT_MASK_SRCDEV");
            if (field_mask & DBRALT_MASK_CPENUM) printk(" && DBRALT_MASK_CPENUM");
            if (field_mask & DBRALT_MASK_DEVTYPE) printk(" && DBRALT_MASK_DEVTYPE");
            printk("\n");

            if ((deletedMacList) && ((deletedMacList->index) < DBR_MAX_CPE_ENTRY))
            {                
                memcpy(&(deletedMacList->macList[deletedMacList->index]), ptr_dbridge_alt->macAddress, MAC_ADDRESS_LEN);
                deletedMacList->index++;
            }
            DbrigdeDb_AltDelEntry(ptr_dbridge_alt);
        }
    }
    DBRALT_PROT_OFF(lockKey);

    return 0;
}

/**************************************************************************/
/*! \fn int DbridgeDb_AltAddMacByDevName(unsigned char* mac_address, unsigned char* name)
 **************************************************************************
 *  \brief this function Add MAC address to ALT table for cm interface (wan0 or lan0)
 *  \param[in] mac address.
 *  \param[in] device name.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDb_AltAddMacByDevName(unsigned char* mac_address, unsigned char* name)
{
    int index;
    DbridgeDevice_t*   ptr_dbridge_dev;
    int name_len = strlen(name);

    for (index=0 ; index<devices_table_insert; index++)
    {
        ptr_dbridge_dev = &dbridge_db.DbridgeDevices[index];
        if ((strlen(ptr_dbridge_dev->ptr_interface->name)== name_len) &&
            (memcmp ((void *)name, (void *)&ptr_dbridge_dev->ptr_interface->name, name_len)==0))
        {
            /* find match between inpute name and dbridge device table */
            /* add the the mac information to ALT table */ 
            if (DbridgeDb_AltAddMac(mac_address,DBRIDGE_INIT, ptr_dbridge_dev,NULL)==0)
                return 0;
            else
            {
                return -1;
            }
        }
    }
    printk(KERN_ERR "Could not find %s with MAC %.2x-%.2x-%.2x-%.2x-%.2x-%.2x in DbrDB Devices\n",
           name, mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
    return -1;
}
#if 1
/***************************************************************************/
/*! \fn int DbridgeDb_AltGetNumMember(void)
 **************************************************************************
 *  \param[in] None.
 *  \param[out] None.
 *  \return Number of member in ALT table.
 **************************************************************************/
int DbridgeDb_AltGetNumMember(void)
{
    return alt_table_insert;
}

#endif
/***************************************************************************/
/*! \fn int DbridgeDB_SetMaxCpe(unsigned  int mac_cpe_config)                                     
 **************************************************************************
 *  \brief set maximum  supported CPEs.
 *  \param[in] Max allow CPEs .
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDB_SetMaxCpe(unsigned  int mac_cpe)
{
    max_cpe = mac_cpe;
    return 0;   /* OK*/
}

/***************************************************************************/
/*! \fn int DbridgeDB_GetMaxCpe()                                     
 **************************************************************************
 *  \brief Get maximum  supported CPEs.
 *  \param[in] None .
 *  \param[out] None.
 *  \return Max CPEs.
 **************************************************************************/
int DbridgeDB_GetMaxCpe(void)
{
    return max_cpe; 
}

//CISCO ADD BEGIN
/***************************************************************************/
/*! \fn int DbridgeDB_SetCntMta(unsigned int value)                                     
 **************************************************************************
 *  \brief Set CountMtaAsCpe.
 *  \param[in] 0 or 1.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDB_SetCntMta(unsigned int value)
{
    cnt_mta = value;
    return 0;   /* OK*/
}

/***************************************************************************/
/*! \fn int DbridgeDB_GetCntMta()                                     
 **************************************************************************
 *  \brief Get CountMtaAsCpe.
 *  \param[in] None .
 *  \param[out] None.
 *  \return 0 or 1.
 **************************************************************************/
int DbridgeDB_GetCntMta(void)
{
    return cnt_mta; 
}

/***************************************************************************/
/*! \fn int DbridgeDB_SetMtaDev(unsigned int value)
 **************************************************************************
 *  \brief Set MtaDevice.
 *  \param[in] 0 or 1.
 *  \param[out] no output.
 *  \return OK or error status.
 **************************************************************************/
int DbridgeDB_SetMtaDev(unsigned int value)
{
    mta_dev = value;
    return 0;   /* OK*/
}

/***************************************************************************/
/*! \fn int DbridgeDB_GetMtaDev()
 **************************************************************************
 *  \brief Get MtaDevice.
 *  \param[in] None .
 *  \param[out] None.
 *  \return 0 or 1.
 **************************************************************************/
int DbridgeDB_GetMtaDev(void)
{
    return mta_dev; 
}
//CISCO ADD END

/***************************************************************************/
/*! \fn int DbridgeDb_DevAdd(struct net_device* ptr_net_dev,int (*push)(struct sk_buff* skb, 
            unsigned int fc_flags))
 ***************************************************************************
 *  \brief Add member to device table.
 *  \param[in] pointer to net device.
 *  \param[in] The PUSH Function to send the packet to Interface.
 *  \param[in] docsis interface type  - CPE eSAFE CABLE WAN IP or LAN IP.
 *  \return 0=OK or error status.
 **************************************************************************/
int DbridgeDb_DevAdd(struct net_device* ptr_net_dev,int (*push)(struct sk_buff* skb, 
                                                                unsigned int fc_flags), int net_dev_type)
{
    if (devices_table_insert < DBR_MAX_INTERFACES)
    {
        dbridge_db.DbridgeDevices[devices_table_insert].push = push;
        dbridge_db.DbridgeDevices[devices_table_insert].ptr_interface = ptr_net_dev;
        dbridge_db.DbridgeDevices[devices_table_insert].net_dev_type = net_dev_type; 
        devices_table_insert++;
        return 0;
    }
    else
    {
        printk(KERN_WARNING "Device table is full\n");

        return -1;
    }

}

/*************************************************************************************/
/*! \fn DbridgeDevice_t* DbridgeDb_DevGetByInterface (struct net_device* ptr_interface)
 *************************************************************************************
 *  \brief The function retreives the dbridge device table member for the corresponding 
 *  \brief network interface.
 *  \param[in] pointer to net net device.
 *  \return Success-pointer to the Dbridge device table, NULL-If no entry is found.
 *************************************************************************************/
DbridgeDevice_t* DbridgeDb_DevGetByInterface (struct net_device* ptr_interface)
{
    int index;

    /* Search the DOCSIS Network device table. */
    for (index = 0; index <devices_table_insert; index++)
    {
        /* Compare the network interface for a match. */
        if (ptr_interface == dbridge_db.DbridgeDevices[index].ptr_interface)
            return &dbridge_db.DbridgeDevices[index];
    }

    /* No match found. */
    return NULL;
}

/*************************************************************************************/
/*! \fn DbridgeDevice_t* DbridgeDb_DevGetByIndex (struct net_device* ptr_interface)
 *************************************************************************************
 *  \brief The function retreives the dbridge device table member for the corresponding 
 *  \brief index.
 *  \param[in] index.
 *  \return Success-pointer to the Dbridge device table, NULL-If no entry is found.
 *************************************************************************************/
DbridgeDevice_t* DbridgeDb_DevGetByIndex (int index)
{
    if (index >= 0 && index < devices_table_insert)
    {
        return &dbridge_db.DbridgeDevices[index];
    }
    else
    {
        return NULL;
    }
}

/*************************************************************************************/
/*! \fn DDbridgeDevice_t* DbridgeDb_DevGetByType(int net_dev_type)
 *************************************************************************************
 *  \brief The function retreives the dbridge device table member for the corresponding 
 *  \brief type.
 *  \param[in] net dev type - CMCI, ESAFE CABLE WAN_IP or LAN_IP.
 *  \return Success-pointer to the Dbridge device table, NULL-If no entry is found.
 *************************************************************************************/
DbridgeDevice_t* DbridgeDb_DevGetByType(int net_dev_type)
{
    int index;

    /* Search the DOCSIS Network device table. */
    for (index = 0; index <devices_table_insert; index++)
    {
        /* Compare the network interface for a match. */
        if (net_dev_type == dbridge_db.DbridgeDevices[index].net_dev_type)
            return &dbridge_db.DbridgeDevices[index];
    }
    /* No match found. */
    return NULL;

}

#ifdef DSG
/*************************************************************************************/
/*! \fn DbridgeDevice_t* DbridgeDb_DevGetByName(const char* net_dev_name)
 *************************************************************************************
 *  \brief The function retreives the dbridge device table member for the corresponding
 *  \brief type.
 *  \param[in] net_dev_name - the name of the dev.
 *  \return Success-pointer to the Dbridge device table, NULL-If no entry is found.
 *************************************************************************************/
DbridgeDevice_t* DbridgeDb_DevGetByName(const char* net_dev_name)
{
    int index;
    DbridgeDevice_t*   ptr_dbridge_dev;
    int name_len = strlen(net_dev_name);

    /* Search the DOCSIS Network device table. */
    for (index = 0; index <devices_table_insert; index++)
    {
        /* Compare the network interface for a match. */
        ptr_dbridge_dev = &dbridge_db.DbridgeDevices[index];
        if((strlen(ptr_dbridge_dev->ptr_interface->name)== name_len) &&
            (memcmp ((void *)net_dev_name, (void *)&ptr_dbridge_dev->ptr_interface->name, name_len)==0))
            return &dbridge_db.DbridgeDevices[index];
    }
    /* No match found. */
    return NULL;
}
#endif

/*************************************************************************************/
/*! \fn int DbridgeDb_GetNumOfDevice (void)
 *************************************************************************************
 *  \brief The function retreives the number of device connect to docsis bridge
 *  \param[in] none.
 *  \return number of device connect to docsis bridge.
 *
 *************************************************************************************/
int DbridgeDb_GetNumOfDevice (void)
{
    return devices_table_insert;
}

/*************************************************************************************/
/*! \fn void DbridgeDb_SetMode (int mode)
 *************************************************************************************
 *  \brief set docsis bridge mode
 *  \param[in] docsis bridge mode - off, registered pre_registration or naco_off 
 *  \return none.
 *************************************************************************************/
void DbridgeDb_SetMode(int mode)
{
    unsigned int mdfMode;
    int erouter_mode;
    static int selective_packet_handler = 0;
#ifdef CONFIG_TI_DOCSIS_EGRESS_HOOK
    static int register_docsis_egress_hook = 0;
#endif

	extern int ti_register_selective_packet_handler (char *br_name, int (*packet_handler)(struct sk_buff *skb), int priority);
    extern int ti_unregister_selective_packet_handler (char *br_name, int (*packet_handler)(struct sk_buff *skb), int priority);

    erouter_mode = DbridgeDb_Erouter_GetMode();


    if ((dbridge_db.mode==DBR_REGISTERED_MODE)&&((mode == DBR_PRE_REGISTRATION_MODE) || (mode == DBR_NACO_OFF_MODE)))
    {
        DbridgeDb_ReInit();
        DbridgeMdfDb_ReInit();
        DbridgeL2vpnDsDb_Init();
        DbridgeMacAgeing_Init();
    }

    else if (mode == DBR_PRE_REGISTRATION_MODE)
    {
        DbridgeMdfDb_ReInit();
        DbridgeL2vpnDsDb_Init();
    }

    /* Set IGMP mode */
    /* need to move the ti_register_selective_packet_handler to deferent place */
    if (mode == DBR_REGISTERED_MODE)
    {
        if(EROUTER_DISABLE == erouter_mode)
        {
            ti_register_selective_packet_handler (LBRIGE_CONNECTION_DEV_NAME, ti_selective_forward, 1);
            selective_packet_handler = 1;
#ifdef CONFIG_TI_DOCSIS_EGRESS_HOOK
            configure_docsis_egress_hook_handler(1);
            register_docsis_egress_hook = 1;
#endif 
			cpe_interface_mask = lbridge_interface_mask;
        }

        if(EROUTER_ENABLE == erouter_mode)
        {
            struct net_device *erouter0_netdev;

            erouter0_netdev = dev_get_by_name (&init_net, EROUTER_NET_DEV_NAME);
            if(erouter0_netdev)
            {
                cpe_interface_mask = (1LL << ((erouter0_netdev->ifindex)-1));
            	dev_put(erouter0_netdev);
            }
            else 
            {
                printk(KERN_ERR "Could not get the eRouter0 device. cpe_interface_mask undefined.");
            }
        }
        /* if MDF is disabled - start the igmp module */
        DbridgeMdfDb_GetMdfMode(&mdfMode);
        if (mdfMode == DBR_MDF_DISABLE_MODE)
        {
            ti_igmp_start();
        }

        if (DBR_REGISTERED_MODE != dbridge_db.mode)
        {
            struct net_device * dev = dev_get_by_name (&init_net, CABLE_DEV_NAME);
            if(dev)
			{
				ti_hil_pp_event (TI_BRIDGE_PORT_FORWARD, (void *)dev);
                dev_put(dev);
			}
        }
    }
    else
    {
        ti_igmp_stop();
        if (DBR_REGISTERED_MODE == dbridge_db.mode)
        {
            struct net_device * dev = dev_get_by_name (&init_net, CABLE_DEV_NAME);
            if(dev)
			{
				ti_hil_pp_event (TI_BRIDGE_PORT_DISABLED, (void *)dev);
                dev_put(dev);
			}
        }
        if(selective_packet_handler)
        {
            ti_unregister_selective_packet_handler (LBRIGE_CONNECTION_DEV_NAME, ti_selective_forward, 1);
            selective_packet_handler = 0;
        }
#ifdef CONFIG_TI_DOCSIS_EGRESS_HOOK
		if(register_docsis_egress_hook)
        {
            configure_docsis_egress_hook_handler(0);
            register_docsis_egress_hook =0;
        }
#endif
    }

    dbridge_db.mode=mode;

}

/*************************************************************************************/
/*! \fn iint DbridgeDb_GetMode (void)
 *************************************************************************************
 *  \brief get docsis bridge mode
 *  \param[in] none
 *  \return docsis bridge mode - off, registered pre_registration or naco_off.
 *************************************************************************************/
int DbridgeDb_GetMode(void)
{
    return dbridge_db.mode;
}

/*************************************************************************************/
/*! \fn iDbridgeCounter_t* Dbridge_GetCounters(void)
 *************************************************************************************
 *  \brief get docsis bridge counters
 *  \param[in] none
 *  \return pointer to the Dbridge counter structure.
 *************************************************************************************/
DbridgeCounters_t* Dbridge_GetCounters(void)
{
    return &dbridge_counters;
}

/***************************************************************************/
/*! \fn int DbridgeDB_GetlbridgeInterfaceMask(void)                                     
 **************************************************************************
 *  \brief Get lbridge Interface Mask.
 *  \param[in] None .
 *  \param[out] None.
 *  \return lbridge Interface Masks.
 **************************************************************************/
unsigned long long DbridgeDB_GetlbridgeInterfaceMask(void)
{
    return lbridge_interface_mask;  
}



/***************************************************************************/
/*! \fn void DbridgeDB_SetlbridgeInterfaceMask(int ifindex)                                     
 **************************************************************************
 *  \brief set interfacelbridge Interface Mask.
 *  \param[in] interface index .
 *  \param[out] None.
 *  \return none.
 **************************************************************************/
void DbridgeDB_SetlbridgeInterfaceMask(int ifindex)
{
     lbridge_interface_mask |= (((typeof(lbridge_interface_mask))1) << (ifindex-1));
}

/***************************************************************************/
/*! \fn int DbridgeDB_GetCpeInterfaceMask(void)                                     
 **************************************************************************
 *  \brief Get CPE Interface Mask.
 *  \param[in] None .
 *  \param[out] None.
 *  \return CPE Interface Mask.
 **************************************************************************/
unsigned long long DbridgeDB_GetCpeInterfaceMask(void)
{
    return cpe_interface_mask;
}

/***************************************************************************/
/*! \fn void DbridgeDb_SetMode (int mode)
 ***************************************************************************
 *  \brief set docsis bridge erouter mode
 *  \param[in] docsis bridge erouter mode - enable/disable 
 *  \return none.
 ***************************************************************************/
void DbridgeDb_Erouter_SetMode(int mode)
{
    dbridge_db.eroter_mode = mode;
}

/***************************************************************************/
/*! \fn iint DbridgeDb_Erouter_GetMode (void)
 ***************************************************************************
 *  \brief get docsis bridge erouter mode
 *  \param[in] none
 *  \return docsis bridge erouter mode enable/disable.
 ***************************************************************************/
int DbridgeDb_Erouter_GetMode(void)
{
        return dbridge_db.eroter_mode;
}

/***************************************************************************/
/*! \fn void DbridgeDb_SetNlSocket(struct sock *socket)
 ***************************************************************************
 *  \brief set docsis bridge netlink socket
 *  \param[in] socket - netlink socket 
 *  \return none.
 ***************************************************************************/
void DbridgeDb_SetNlSocket(struct sock *socket)
{
    if (socket)
    {
        dbridge_db.nlsk = socket;
    }
}

/***************************************************************************/
/*! \fn struct sock* DbridgeDb_GetNlSocket(void);
 ***************************************************************************
 *  \brief get docsis bridge netlink socket
 *  \param[in] none
 *  \return docsis bridge netlink socket.
 ***************************************************************************/
struct sock* DbridgeDb_GetNlSocket(void)
{
    if (dbridge_db.nlsk)
    {
        return dbridge_db.nlsk;
    }

    return NULL;
}

/***************************************************************************/
/*! \fn void DbridgeDb_SetNlPid(int pid)
 ***************************************************************************
 *  \brief set docsis netlink pid
 *  \param[in] pid - netlink pid 
 *  \return none.
 ***************************************************************************/
void DbridgeDb_SetNlPid(int pid)
{
    dbridge_db.nl_pid = pid;
}

/***************************************************************************/
/*! \fn int DbridgeDb_GetNlPid (void)
 ***************************************************************************
 *  \brief get docsis bridge netlink pid
 *  \param[in] none
 *  \return docsis bridge netlink pid.
 ***************************************************************************/
int DbridgeDb_GetNlPid(void)
{
    return dbridge_db.nl_pid;
}

#ifdef CONFIG_TI_DOCSIS_EGRESS_HOOK

static int configure_docsis_egress_hook_handler(int reg)
{

	/* Install the Ingress Hook. */
	{
		extern int ti_register_docsis_egress_hook_handler (struct net_device* dev, int (*docsis_egress_hook)(struct sk_buff *skb));
        extern int ti_deregister_docsis_egress_hook_handler (struct net_device* dev);

        if(NULL == eth0_netdev)
        {
            eth0_netdev= dev_get_by_name (&init_net,ETH_DEV_NAME);
        }
		if(eth0_netdev)
		{
            if(reg)
            {
                if (ti_register_docsis_egress_hook_handler (eth0_netdev, Dbridge_egress_hook) == 0)
                    printk ("register ingress Hook on  %s \n",eth0_netdev->name);
            }
            else
            {
                if (ti_deregister_docsis_egress_hook_handler (eth0_netdev) == 0)
                    printk ("deregister ingress Hook on  %s \n",eth0_netdev->name);
                dev_put(eth0_netdev);
                eth0_netdev = NULL;
            }
		}
        else
        {
            printk ("warning: configure ingress hook fail no eth dev %s \n",usb0_netdev->name);
        }

        if(NULL == usb0_netdev)
        {
            usb0_netdev= dev_get_by_name (&init_net,USB_DEV_NAME);
        }
		if(usb0_netdev)
		{
            if(reg)
            {
                if (ti_register_docsis_egress_hook_handler (usb0_netdev, Dbridge_egress_hook) == 0)
                    printk ("register ingress Hook on  %s \n",usb0_netdev->name);
            }
            else
            {
                if (ti_deregister_docsis_egress_hook_handler (usb0_netdev) == 0)
                    printk ("deregister ingress Hook on  %s \n",usb0_netdev->name);
                dev_put(usb0_netdev);
                usb0_netdev = NULL;
            }
		}
        else
        {
            printk ("warning: configure ingress hook fail no usb dev %s \n",usb0_netdev->name);
        }

	}
    return 0;

}

/**************************************************************************/
/*! \fn int Dbridge_egress_hook(struct sk_buff *skb)
 **************************************************************************
 *  \brief   In
 *  \param[in] SKB.
 *  \return 
 **************************************************************************/
static int Dbridge_egress_hook(struct sk_buff *skb)
{
    struct ethhdr* ptr_ethhdr;

    if(skb)
    {
        ptr_ethhdr = eth_hdr(skb); 

        if(((eth0_netdev )&& (skb->skb_iif == eth0_netdev->ifindex)) || ((usb0_netdev) && (skb->skb_iif == usb0_netdev->ifindex)))
        {   
            dbridge_counters.DevCounters[skb->skb_iif - 1].In++;
            dbridge_counters.DevCounters[skb->dev->ifindex - 1].Out++;
            if ( !MAC_ISMULTICAST(ptr_ethhdr, h_dest) )
            {/*If this is unicast packet - increment unicast counters as well*/
                dbridge_counters.DevCounters[skb->skb_iif - 1].inUcast++;
                dbridge_counters.DevCounters[skb->dev->ifindex - 1].outUcast++;
            } else
            {  /*If this is broadcast packet - increment broadcast counter as well*/
                if ( MAC_ISBROADCAST(ptr_ethhdr, h_dest) )
                {
                    dbridge_counters.DevCounters[skb->skb_iif - 1].inBcast++;
                    dbridge_counters.DevCounters[skb->dev->ifindex - 1].outBcast++;
                } else
                {   /*If this is multicast packet - increment multicast counter as well*/
                    dbridge_counters.DevCounters[skb->skb_iif - 1].inMcast++;
                    dbridge_counters.DevCounters[skb->dev->ifindex - 1].outMcast++;
                }
            }
        }
    }
    return 0;
}

#endif /* CONFIG_TI_DOCSIS_EGRESS_HOOK */

/**************************************************************************/
/*! \fn void DbrigdeDb_AltDelEntry(DbridgeAltEntry_t *altEntry)
 **************************************************************************
 *  \brief Remove ALT entry from list
 *  \param[in] altEntry - entry to be removed
 *  \return void
 **************************************************************************/
static void DbrigdeDb_AltDelEntry(DbridgeAltEntry_t *altEntry)
{
    unsigned char cpe_num;

    /* Decrease CPE counters */
    if (altEntry->macLearningFrom==CPE_DYNAMIC)
        dbridge_counters.DynamicCpe--;
    else if (altEntry->macLearningFrom==CPE_STATIC)
        dbridge_counters.StaticCpe--;

    /* Remove from OCCUPIED list */
    list_del(&altEntry->listAltEntry);

    /* Preserve cpe num */
    cpe_num = altEntry->cpe_number;

    /* 0 */
    memset (altEntry, 0, sizeof(*altEntry));

    /* Restore cpe num */
    altEntry->cpe_number = cpe_num;

    /* Add to FREE list */
    /* add_tail, so reuse will be delayed till all other entries are used */
    list_add_tail(&altEntry->listAltEntry, &DBRALT_FREE_HEAD);
}




EXPORT_SYMBOL(DbridgeDb_DevGetByInterface);
#ifdef DSG
unsigned char DbridgeDB_GetDsgMode(void)
{
    return dbridge_db.dbridgeDsg.mode;
}
void DbridgeDB_SetDsgMode( unsigned char mode)
{
    dbridge_db.dbridgeDsg.mode = mode;
}
/************************************************************************
 *       DbridgeDB_IsDsgEnabled() and DbridgeDB_IsLoobEnabled()                 *
 ************************************************************************
 * DESCRIPTION: Gets DSG or OOB operational mode.                       *
 *              These modes are stored in docsis data base                         *
 *                                                                      *
 * INPUT:       None.                                                   *
 * OUTPUT:      None                                                    *
 * RETURN:      DSG or OOB operational (True,False)                     *
 *                                                                      *
 ************************************************************************/
int DbridgeDB_IsDsgEnabled(void)
{
    unsigned char mode = 0;

    mode = DbridgeDB_GetDsgMode();
    return (mode == DBR_DSGLOOB_DSG_ENABLE);
}

int DbridgeDB_IsLoobEnabled(void)
{
    unsigned char mode = 0;

    mode = DbridgeDB_GetDsgMode();
    return (mode == DBR_DSGLOOB_LOOB_ENABLE);
}

int DbridgeDB_IsDsgLoobEnabled(void)
{
    unsigned char mode = 0;

    mode = DbridgeDB_GetDsgMode();
    return (mode != DBR_DSGLOOB_DOCSIS_STANDARD);
}


int DbridgeDB_GetDsgTunnelNums(void)
{
    return dbridge_db.dbridgeDsg.tunlNum;
}


int DbridgeDB_AddDsgTunnel(char *addr)
{
    int index = dbridge_db.dbridgeDsg.tunlNum;

    if(index == DSG_FILTER_TAB_LEN)
    {
        printk(KERN_DEBUG "\nDsg tunnel table is full\n");
        return -1;
    }
    if(DbridgeDB_IsDsgMulticastAddress(addr) >= 0)
    {
        printk(KERN_DEBUG "\nInpute Mcast Mac exists in the table \n");
        return 0;
    }
    memcpy((char *)dbridge_db.dbridgeDsg.dbridegDsgTunnel[index].MacAddress, addr, MAC_ADDRESS_LEN);
    dbridge_db.dbridgeDsg.tunlNum++;
    return 0;
}

int DbridgeDB_DelDsgTunnel(char *addr)
{
    int index;

    if(DbridgeDB_IsDsgMulticastAddress(addr) == -1)
    {
        printk(KERN_DEBUG "\nInpute Mcast Mac does not exists in the table \n");
        return 0;
    }
    for(index=0;index<DbridgeDB_GetDsgTunnelNums();index++)
    {
        if(!memcmp(addr,dbridge_db.dbridgeDsg.dbridegDsgTunnel[index].MacAddress, MAC_ADDRESS_LEN))
            break;
    }
    dbridge_db.dbridgeDsg.tunlNum--;
    /* Entries collapse around deleted entry */
    if(index < dbridge_db.dbridgeDsg.tunlNum)
        memmove(&(dbridge_db.dbridgeDsg.dbridegDsgTunnel[index].MacAddress),
                 &(dbridge_db.dbridgeDsg.dbridegDsgTunnel[index+1].MacAddress),
                  (dbridge_db.dbridgeDsg.tunlNum - index)*MAC_ADDRESS_LEN);
    /* Clean previous last entry */
    memset(&(dbridge_db.dbridgeDsg.dbridegDsgTunnel[dbridge_db.dbridgeDsg.tunlNum].MacAddress),0,MAC_ADDRESS_LEN);
    return 0;
}

/***************************************************************************/
/*! \fn int DbridgeDB_IsDsgMulticastAddress(char *addr)
 **************************************************************************
 *  \brief Verify the mac address is a dsg tunnel address.
 *  \param[in] mac address .
 *  \param[out] None.
 *  \return the index if yes, return -1 is not.
 **************************************************************************/
int DbridgeDB_IsDsgMulticastAddress(char *addr)
{
    int i;
    for(i = 0; i<DSG_FILTER_TAB_LEN; i++)
    {
        if(memcmp((void *)dbridge_db.dbridgeDsg.dbridegDsgTunnel[i].MacAddress, (void *)addr, MAC_ADDRESS_LEN) == 0)
            return i;
    }
    return -1;
}

/**************************************************************************/
/*! \fn int DbridgeDB_GetDsgTunnelMacAddress(int index, unsigned char* mac_address)
 **************************************************************************
 *  \brief Get tunnel mac address by index.
 *  \param[in] tunnel table index.
 *  \param[out] mac address ,
 *  \return 0 or error status.
 **************************************************************************/
int DbridgeDB_GetDsgTunnelMacAddress(int index, unsigned char* mac_address)
{
    if(index >= DbridgeDB_GetDsgTunnelNums())
    {
        printk(KERN_DEBUG "\nInpute index does not exists in the table \n");
        return -1;
    }
    memcpy(mac_address, dbridge_db.dbridgeDsg.dbridegDsgTunnel[index].MacAddress, MAC_ADDRESS_LEN);
    return 0;
}


int DbridgeDB_SetDsgEstbIPAddress(unsigned long ip)
{
    dbridge_db.dbridgeDsg.dbridegEstbIP = ip;
    return 0;
}

int DbridgeDB_GetDsgEstbIPAddress(unsigned long *ip)
{
    *ip = dbridge_db.dbridgeDsg.dbridegEstbIP;
    return 0;
}

int DbridgeDB_IsDsgEstbIPAddress(unsigned long ip)
{
    if(dbridge_db.dbridgeDsg.dbridegEstbIP == 0)
    	return 0;
    if(dbridge_db.dbridgeDsg.dbridegEstbIP == ip)
        return 1;
    return 0;
}

#endif //DSG
