
/*
 * GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2011 Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify 
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but 
 *  WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *  General Public License for more details.
 *
 *
 *  You should have received a copy of the GNU General Public License 
 *  along with this program; if not, write to the Free Software 
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution 
 *  in the file called LICENSE.GPL.
 *
 *  Contact Information:
 *  Intel Corporation
 *  2200 Mission College Blvd.
 *  Santa Clara, CA  97052
 */

#include <linux/mman.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
//#include <linux/bootparam/bootparam.h>
#include <asm-arm/arch-avalanche/puma6/puma6_hardware.h>
#include <asm-arm/arch-avalanche/puma6/puma6.h>

#include "docsis_ip_boot_params.h"
#include "proc_bootparams.h"


DEFINE_MUTEX(bootparam_table_mutex);
//EXPORT_SYMBOL_GPL(bootparam_table_mutex);

#define BOOTRAM_PROC_NAME "bootparams"


/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/
 

static struct proc_dir_entry *procFile;


typedef struct _bp
{
    char      name[PROC_BP_MAX_NAME];
    char      size;
    unsigned int  addr;
}bp;

/* Print Docsis IP Boot Parameters */
static bp bp_array[] = {
    {PROC_BP_BOOT_PARAM_VERSION,        sizeof(int), ( AVALANCHE_SRAM_BASE + BOOT_PARAM_VER                )},
    {PROC_BP_ARM11_BOOT_STATUS,         sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_BOOT_STATUS             )},
    {PROC_BP_BOOT_MODE,                 sizeof(int), ( AVALANCHE_SRAM_BASE + BOOT_MODE                     )}, 
    {PROC_BP_BOARD_TYPE,                sizeof(int), ( AVALANCHE_SRAM_BASE + BOARD_TYPE                    )},
    {PROC_BP_NUMBER_OF_FLASHES,         sizeof(int), ( AVALANCHE_SRAM_BASE + NUMBER_OF_FLASHES             )},
    {PROC_BP_RAM_OFFSET,                sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_DDR_OFFSET              )},
    {PROC_BP_RAM_SIZE,                  sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_DDR_SIZE                )},
    {PROC_BP_ACTIVE_AID,                sizeof(int), ( AVALANCHE_SRAM_BASE + ACTIVE_AID                    )},
    {PROC_BP_AID_1_OFFSET,              sizeof(int), ( AVALANCHE_SRAM_BASE + AID_1_OFFSET                  )},
    {PROC_BP_AID_2_OFFSET,              sizeof(int), ( AVALANCHE_SRAM_BASE + AID_2_OFFSET                  )},
    {PROC_BP_UBOOT_OFFSET,              sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBOOT_OFFSET            )},
    {PROC_BP_UBOOT_SIZE,                sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBOOT_SIZE              )},
    {PROC_BP_UBOOT_ENV1_OFFSET,         sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_ENV1_OFFSET             )},
    {PROC_BP_UBOOT_ENV2_OFFSET,         sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_ENV2_OFFSET             )},
    {PROC_BP_UBOOT_ENV_SIZE,            sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_ENV_SIZE                )},
    {PROC_BP_ARM11_NVRAM_OFFSET,        sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_NVRAM_OFFSET            )},
    {PROC_BP_ARM11_NVRAM_SIZE,          sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_NVRAM_SIZE              )},
    {PROC_BP_ARM11_UBFI1_OFFSET,        sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBFI1_OFFSET            )},
    {PROC_BP_ARM11_UBFI1_SIZE,          sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBFI1_SIZE              )},
    {PROC_BP_ARM11_UBFI2_OFFSET,        sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBFI2_OFFSET            )},
    {PROC_BP_ARM11_UBFI2_SIZE,          sizeof(int), ( AVALANCHE_SRAM_BASE + ARM11_UBFI2_SIZE              )},
    {PROC_BP_ATOM_UBFI1_OFFSET,         sizeof(int), ( AVALANCHE_SRAM_BASE + ATOM_UBFI1_OFFSET             )},
    {PROC_BP_ATOM_UBFI1_SIZE,           sizeof(int), ( AVALANCHE_SRAM_BASE + ATOM_UBFI1_SIZE               )},
    {PROC_BP_ATOM_UBFI2_OFFSET,         sizeof(int), ( AVALANCHE_SRAM_BASE + ATOM_UBFI2_OFFSET             )},
    {PROC_BP_ATOM_UBFI2_SIZE,           sizeof(int), ( AVALANCHE_SRAM_BASE + ATOM_UBFI2_SIZE               )},
    {PROC_BP_ARM11_KERNEL_1_PARTITION,  sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_KERNEL_1_EMMC_PARTITION )},
    {PROC_BP_ARM11_KERNEL_2_PARTITION,  sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_KERNEL_2_EMMC_PARTITION )},
    {PROC_BP_ARM11_ROOT_FS_1_PARTITION, sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_ROOTFS_1_EMMC_PARTITION )},
    {PROC_BP_ARM11_ROOT_FS_2_PARTITION, sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_ROOTFS_2_EMMC_PARTITION )},
    {PROC_BP_ARM11_GW_FS_1_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_GW_FS_1_EMMC_PARTITION  )},
    {PROC_BP_ARM11_GW_FS_2_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_GW_FS_2_EMMC_PARTITION  )},
    {PROC_BP_ARM11_NVRAM_1_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_NVRAM_EMMC_PARTITION    )},
    {PROC_BP_ARM11_NVRAM_2_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ARM11_NVRAM_2_EMMC_PARTITION  )},
    {PROC_BP_ATOM_KERNEL_1_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ATOM_KERNEL_1_EMMC_PARTITION  )},
    {PROC_BP_ATOM_KERNEL_2_PARTITION,   sizeof(char),( AVALANCHE_SRAM_BASE + ATOM_KERNEL_2_EMMC_PARTITION  )},
    {PROC_BP_ATOM_ROOT_FS_1_PARTITION,  sizeof(char),( AVALANCHE_SRAM_BASE + ATOM_ROOTFS_1_EMMC_PARTITION  )},
    {PROC_BP_ATOM_ROOT_FS_2_PARTITION,  sizeof(char),( AVALANCHE_SRAM_BASE + ATOM_ROOTFS_2_EMMC_PARTITION  )},
	{PROC_BP_SILICON_STEPPING,			sizeof(int), ( AVALANCHE_SRAM_BASE + SILICON_STEPPING			   )},
	{PROC_BP_CEFDK_VERSION,				sizeof(int), ( AVALANCHE_SRAM_BASE + CEFDK_VERSION				   )},
	{PROC_BP_SIGNATURE1_OFFSET,			sizeof(int), ( AVALANCHE_SRAM_BASE + SIGNATURE1_OFFSET			   )},
    {PROC_BP_SIGNATURE2_OFFSET,			sizeof(int), ( AVALANCHE_SRAM_BASE + SIGNATURE2_OFFSET			   )},
    {PROC_BP_SIGNATURE_SIZE,			sizeof(int), ( AVALANCHE_SRAM_BASE + SIGNATURE_SIZE  			   )},
    {PROC_BP_SIGNATURE_NUMBER,			sizeof(int), ( AVALANCHE_SRAM_BASE + SIGNATURE_NUMBER  			   )},
	{PROC_BP_EMMC_FLASH_SIZE,			sizeof(int), ( AVALANCHE_SRAM_BASE + EMMC_FLASH_SIZE			   )}			
};

static int bootparams_read_proc (char *page, char **start, off_t off, int count,int *eof, void *data_unused)
{
    int i = 0;
	int len = 0;
    int l = 0;
    off_t   begin = 0;

	mutex_lock(&bootparam_table_mutex);

    
	l = snprintf(page,count-len, "Boot Parameters:\n");
    if (l>=count-len)
        goto done;
    len+=l;

	for (i=0;i<sizeof(bp_array)/sizeof(bp);i++) 
    {

        l=0;
        switch (bp_array[i].size)
        {
        case (sizeof(char)):
            l = snprintf(page +len,count-len, "%s 0x%.2X\n",bp_array[i].name,BOOT_PARAM_BYTE_READ(bp_array[i].addr));
            break;
        case (sizeof(int)):
            l = snprintf(page +len,count-len, "%s 0x%.8X\n",bp_array[i].name,BOOT_PARAM_DWORD_READ(bp_array[i].addr));
            break;
        }

        if (l>=count-len)
            goto done;
        len += l;

        if (len+begin < off)
        {
            begin += len;
            len = 0;
        }
    }

    *eof = 1; /* set EOF */

done:
	mutex_unlock(&bootparam_table_mutex);
    if (off >= len+begin)
    {
        return 0;
    }
    *start = page + (off-begin);
    return ((count < begin+len-off) ? count : begin+len-off);
}


static int __init init_bootparams(void)
{
    
    /* create the /proc file */

	procFile = create_proc_entry(BOOTRAM_PROC_NAME, 0644, NULL);
	
	if (procFile == NULL) {
		remove_proc_entry(BOOTRAM_PROC_NAME, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",BOOTRAM_PROC_NAME);
		return -ENOMEM;
	}

    procFile->read_proc = bootparams_read_proc;
    procFile->mode 	    = S_IFREG | S_IRUGO;

    printk(KERN_INFO "/proc/%s created\n", BOOTRAM_PROC_NAME);

	return 0;

}

static void __exit cleanup_bootparams(void)
{
        if (procFile)
            remove_proc_entry( BOOTRAM_PROC_NAME, NULL);
}

module_init(init_bootparams);
module_exit(cleanup_bootparams);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Core bootparams registration and access routines");

