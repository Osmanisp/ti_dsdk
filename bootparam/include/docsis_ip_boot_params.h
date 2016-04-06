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


#ifndef   _DOCSIS_IP_BOOT_PARAMS_H_
#define   _DOCSIS_IP_BOOT_PARAMS_H_




/* Boot Params top address id fix to 8KB */
#define BOOT_PARAMS_TOP_ADDR  	0x2000

/* Boot Paramas fields */
#define BOOT_PARAM_VER                  (BOOT_PARAMS_TOP_ADDR           - sizeof(unsigned int ))  /* 0x1FFC */
#define ARM11_BOOT_STATUS               (BOOT_PARAM_VER                 - sizeof(unsigned int ))  /* 0x1FF8 */
#define BOOT_MODE                       (ARM11_BOOT_STATUS              - sizeof(unsigned int ))  /* 0x1FF4 */
#define BOARD_TYPE                      (BOOT_MODE                      - sizeof(unsigned int ))  /* 0x1FF0 */
#define NUMBER_OF_FLASHES               (BOARD_TYPE                     - sizeof(unsigned int ))  /* 0x1FEC */
#define ARM11_DDR_OFFSET                (NUMBER_OF_FLASHES              - sizeof(unsigned int ))  /* 0x1FE8 */
#define ARM11_DDR_SIZE                  (ARM11_DDR_OFFSET               - sizeof(unsigned int ))  /* 0x1FE4 */
#define AID_1_OFFSET                    (ARM11_DDR_SIZE                 - sizeof(unsigned int ))  /* 0x1FE0 */
#define AID_2_OFFSET                    (AID_1_OFFSET                   - sizeof(unsigned int ))  /* 0x1FDC */
#define ARM11_UBOOT_OFFSET              (AID_2_OFFSET                   - sizeof(unsigned int ))  /* 0x1FD8 */
#define ARM11_UBOOT_SIZE                (ARM11_UBOOT_OFFSET             - sizeof(unsigned int ))  /* 0x1FD4 */
#define ARM11_ENV1_OFFSET               (ARM11_UBOOT_SIZE               - sizeof(unsigned int ))  /* 0x1FD0 */
#define ARM11_ENV2_OFFSET               (ARM11_ENV1_OFFSET              - sizeof(unsigned int ))  /* 0x1FCC */
#define ARM11_ENV_SIZE                  (ARM11_ENV2_OFFSET              - sizeof(unsigned int ))  /* 0x1FC8 */
#define ARM11_NVRAM_OFFSET              (ARM11_ENV_SIZE                 - sizeof(unsigned int ))  /* 0x1FC4 */
#define ARM11_NVRAM_SIZE                (ARM11_NVRAM_OFFSET             - sizeof(unsigned int ))  /* 0x1FC0 */
#define ARM11_UBFI1_OFFSET              (ARM11_NVRAM_SIZE               - sizeof(unsigned int ))  /* 0x1FBC */
#define ARM11_UBFI1_SIZE                (ARM11_UBFI1_OFFSET             - sizeof(unsigned int ))  /* 0x1FB8 */
#define ARM11_UBFI2_OFFSET              (ARM11_UBFI1_SIZE               - sizeof(unsigned int ))  /* 0x1FB4 */
#define ARM11_UBFI2_SIZE                (ARM11_UBFI2_OFFSET             - sizeof(unsigned int ))  /* 0x1FB0 */
#define ATOM_UBFI1_OFFSET               (ARM11_UBFI2_SIZE               - sizeof(unsigned int ))  /* 0x1FAC */
#define ATOM_UBFI1_SIZE                 (ATOM_UBFI1_OFFSET              - sizeof(unsigned int ))  /* 0x1FA8 */
#define ATOM_UBFI2_OFFSET               (ATOM_UBFI1_SIZE                - sizeof(unsigned int ))  /* 0x1FA4 */
#define ATOM_UBFI2_SIZE                 (ATOM_UBFI2_OFFSET              - sizeof(unsigned int ))  /* 0x1FA0 */
#define ARM11_KERNEL_1_EMMC_PARTITION   (ATOM_UBFI2_SIZE                - sizeof(unsigned char))  /* 0x1F9F */
#define ARM11_KERNEL_2_EMMC_PARTITION   (ARM11_KERNEL_1_EMMC_PARTITION  - sizeof(unsigned char))  /* 0x1F9E */
#define ARM11_ROOTFS_1_EMMC_PARTITION   (ARM11_KERNEL_2_EMMC_PARTITION  - sizeof(unsigned char))  /* 0x1F9D */
#define ARM11_ROOTFS_2_EMMC_PARTITION   (ARM11_ROOTFS_1_EMMC_PARTITION  - sizeof(unsigned char))  /* 0x1F9C */
#define ARM11_GW_FS_1_EMMC_PARTITION    (ARM11_ROOTFS_2_EMMC_PARTITION  - sizeof(unsigned char))  /* 0x1F9B */
#define ARM11_GW_FS_2_EMMC_PARTITION    (ARM11_GW_FS_1_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F9A */
#define ARM11_NVRAM_EMMC_PARTITION      (ARM11_GW_FS_2_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F99 */
#define ATOM_KERNEL_1_EMMC_PARTITION    (ARM11_NVRAM_EMMC_PARTITION     - sizeof(unsigned char))  /* 0x1F98 */
#define ATOM_KERNEL_2_EMMC_PARTITION    (ATOM_KERNEL_1_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F97 */
#define ATOM_ROOTFS_1_EMMC_PARTITION    (ATOM_KERNEL_2_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F96 */
#define ATOM_ROOTFS_2_EMMC_PARTITION    (ATOM_ROOTFS_1_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F95 */
#define ARM11_NVRAM_2_EMMC_PARTITION    (ATOM_ROOTFS_2_EMMC_PARTITION   - sizeof(unsigned char))  /* 0x1F94 */
#define ACTIVE_AID                      (ARM11_NVRAM_2_EMMC_PARTITION   - sizeof(unsigned int ))  /* 0x1F90 */
#define SILICON_STEPPING                (ACTIVE_AID                     - sizeof(unsigned int ))  /* 0x1F8C */
#define CEFDK_VERSION                   (SILICON_STEPPING               - sizeof(unsigned int ))  /* 0x1F88 */
#define SIGNATURE1_OFFSET               (CEFDK_VERSION                  - sizeof(unsigned int ))  /* 0x1F84 */
#define EMMC_FLASH_SIZE                 (SIGNATURE1_OFFSET              - sizeof(unsigned int ))  /* 0x1F80 */
#define SIGNATURE2_OFFSET               (EMMC_FLASH_SIZE                - sizeof(unsigned int ))  /* 0x1F7C */
#define SIGNATURE_SIZE                  (SIGNATURE2_OFFSET              - sizeof(unsigned int ))  /* 0x1F78 */
#define SIGNATURE_NUMBER                (SIGNATURE_SIZE                 - sizeof(unsigned int ))  /* 0x1F74 */
#define BOOT_PARAMS_BOTTOM_ADDR         (SIGNATURE_NUMBER               - sizeof(unsigned int ))  /* 0x1F70 */

/* Macro to read */
#define GET_BOOT_PARAM_REG(addr)       (*((volatile unsigned int *)(addr)))
#define SET_BOOT_PARAM_REG(addr,val)   (*((volatile unsigned int *)(addr)) = (val))

#define BOOT_PARAM_BYTE_READ(addr)     (*((volatile unsigned char *)(addr)))
#define BOOT_PARAM_SHORT_READ(addr)    (*((volatile unsigned short *)(addr)))
#define BOOT_PARAM_DWORD_READ(addr)    (*((volatile unsigned int *)(addr)))
#define BOOT_PARAM_LONG_READ(addr)     (*((volatile unsigned long *)(addr)))

/* Data definitions for Boot Mode */
#define BOOT_MODE_SPI   (0)
#define BOOT_MODE_eMMC  (1)
#ifdef CONFIG_BOOTRAM_ENABLE_SKIP_LOADING
#define BOOT_MODE_SKIP_LOADING   (2)
#endif

/* Data definitions for ARM Boot Status */
#define ARM_STAT_BOOT_RAM_START    0x00000001
#define ARM_STAT_BOOT_RAM_END      0x00000002
#define ARM_STAT_BOOT_UBOOT_START  0x00000004
#define ARM_STAT_BOOT_UBOOT_END    0x00000008
#define ARM_STAT_BOOT_KERNEL_START 0x00000010
 
/* Data definitions for board types */
#define PUMA6_UNKNOWN_BOARD_ID    (0x0) /* " ERROR "        */ 
#define PUMA6_HP_BOARD_ID         (0x1) /* "harborpark"     */ 
#define PUMA6_HP_MG_BOARD_ID      (0x2) /* "harborpark-mg"  */ 
#define PUMA6_FM_BOARD_ID         (0x3) /* "falconmine"     */ 
#define PUMA6_CI_BOARD_ID         (0x4) /* "catisland"      */ 
#define PUMA6_GS_BOARD_ID         (0x5) /* "golden-springs" */
#define PUMA6_CR_BOARD_ID         (0x6) /* "Cat River"      */

#define PUMA6_FM_BOARD_NAME       "falconmine"
#define PUMA6_CI_BOARD_NAME       "catisland"
#define PUMA6_HP_BOARD_NAME       "harborpark"
#define PUMA6_HP_MG_BOARD_NAME    "harborpark-mg"
#define PUMA6_GS_BOARD_NAME       "golden-springs"

#endif /*_DOCSIS_IP_BOOT_PARAMS_H_*/



