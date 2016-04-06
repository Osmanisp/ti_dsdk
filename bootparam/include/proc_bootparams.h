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



/* DOCSIS Boot Parameters definition file */


#ifndef   _PROC_BOOTPARAMS_H_
#define   _PROC_BOOTPARAMS_H_

#define BOOT_PARAMS_FILENAME            "/proc/bootparams"

#define PROC_BP_MAX_NAME                 (50)
#define PROC_BP_MAX_LINE                 PROC_BP_MAX_NAME + 15

#define PROC_BP_BOOT_PARAM_VERSION        " Boot Params Version ......."
#define PROC_BP_ARM11_BOOT_STATUS         " ARM11 Boot Status ........."
#define PROC_BP_BOOT_MODE                 " Boot Mode ................."
#define PROC_BP_BOARD_TYPE                " Board Type ................"
#define PROC_BP_NUMBER_OF_FLASHES         " Numebr of flashes ........."
#define PROC_BP_RAM_OFFSET                " RAM Offset ................"
#define PROC_BP_RAM_SIZE                  " RAM Size .................."
#define PROC_BP_ACTIVE_AID                " Active AID  ..............."
#define PROC_BP_AID_1_OFFSET              " AID 1 Offset .............."
#define PROC_BP_AID_2_OFFSET              " AID 2 Offset .............."
#define PROC_BP_UBOOT_OFFSET              " Uboot Offset .............."
#define PROC_BP_UBOOT_SIZE                " Uboot Size ................"
#define PROC_BP_UBOOT_ENV1_OFFSET         " Uboot Env1 Offset ........."
#define PROC_BP_UBOOT_ENV2_OFFSET         " Uboot Env2 Offset ........."
#define PROC_BP_UBOOT_ENV_SIZE            " Uboot Env Size ............"
#define PROC_BP_ARM11_NVRAM_OFFSET        " ARM11 NVRAM Offset ........"
#define PROC_BP_ARM11_NVRAM_SIZE          " ARM11 NVRAM Size .........."
#define PROC_BP_ARM11_UBFI1_OFFSET        " ARM11 UBFI1 Offset ........"
#define PROC_BP_ARM11_UBFI1_SIZE          " ARM11 UBFI1 Size .........."
#define PROC_BP_ARM11_UBFI2_OFFSET        " ARM11 UBFI2 Offset ........"
#define PROC_BP_ARM11_UBFI2_SIZE          " ARM11 UBFI2 Size .........."
#define PROC_BP_ATOM_UBFI1_OFFSET         " ATOM UBFI1 Offset ........."
#define PROC_BP_ATOM_UBFI1_SIZE           " ATOM UBFI1 Size ..........."
#define PROC_BP_ATOM_UBFI2_OFFSET         " ATOM UBFI2 Offset ........."
#define PROC_BP_ATOM_UBFI2_SIZE           " ATOM UBFI2 Size ..........."
#define PROC_BP_ARM11_KERNEL_1_PARTITION  " ARM11 Kernel 1 partition .."
#define PROC_BP_ARM11_KERNEL_2_PARTITION  " ARM11 Kernel 2 partition..."
#define PROC_BP_ARM11_ROOT_FS_1_PARTITION " ARM11 Root FS 1 partition ."
#define PROC_BP_ARM11_ROOT_FS_2_PARTITION " ARM11 Root FS 2 partition ."
#define PROC_BP_ARM11_GW_FS_1_PARTITION   " ARM11 GW FS 1 partition ..."
#define PROC_BP_ARM11_GW_FS_2_PARTITION   " ARM11 GW FS 2 partition ..."
#define PROC_BP_ARM11_NVRAM_1_PARTITION   " ARM11 NVRAM 1 partition ..."
#define PROC_BP_ARM11_NVRAM_2_PARTITION   " ARM11 NVRAM 2 partition ..."
#define PROC_BP_ATOM_KERNEL_1_PARTITION   " ATOM Kernel 1 partition ..."
#define PROC_BP_ATOM_KERNEL_2_PARTITION   " ATOM Kernel 2 partition ..."
#define PROC_BP_ATOM_ROOT_FS_1_PARTITION  " ATOM Root FS 1 partition .."
#define PROC_BP_ATOM_ROOT_FS_2_PARTITION  " ATOM Root FS 2 partition .."
#define PROC_BP_SILICON_STEPPING		  " Silicon stepping..........."
#define PROC_BP_CEFDK_VERSION			  " CEFDK version.............."
#define PROC_BP_SIGNATURE1_OFFSET		  " Signature 1 offset........."
#define PROC_BP_SIGNATURE2_OFFSET		  " Signature 2 offset........."
#define PROC_BP_SIGNATURE_SIZE  		  " Signature size............."
#define PROC_BP_SIGNATURE_NUMBER  		  " Signature number..........."
#define PROC_BP_EMMC_FLASH_SIZE			  " EMMC flash size............"

#endif /*_PROC_BOOTPARAMS_H_*/



