################################################################################
#
# Copyright(c) 2008 Texas Instruments, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 59
# Temple Place - Suite 330, Boston, MA  02111-1307, USA.
#
# The full GNU General Public License is included in this distribution in the
# file called LICENSE.
#
################################################################################
ifeq ($(TARGET_HOME),)
TARGET_HOME = /vobs/dsps_gtrgdk
endif

#
# Compilation options to add to the compiler (optional)
#
EXTRA_CFLAGS += -I$(TARGET_HOME)/ti/docsis/src/common/data_path/dsg_ni/include \
                    -I$(TARGET_HOME)/ti/docsis/src/common/data_path/docsis_bridge/dbridge_dlm/include \
                    -I$(TARGET_HOME)/ti/include \
                    -I$(KERNEL_DIR)/include/asm-arm/arch-avalanche/generic \
                    -I$(KERNEL_DIR)/arch/arm/mach-avalanche/include/mach

ifeq ($(DOCSIS_SOC), PUMA6)
EXTRA_CFLAGS += -I$(KERNEL_DIR)/include/asm-arm/arch-avalanche/puma6
else
EXTRA_CFLAGS += -I$(KERNEL_DIR)/include/asm-arm/arch-avalanche/puma5
endif
                    
ifneq ($(KERNELRELEASE),)

	TARGET := dsgni

	obj-m := $(TARGET).o

	# Add objects
	$(TARGET)-objs := dsg_ni.o
else

include $(TARGET_HOME)/TI_Build.make

unexport LDFLAGS
unexport CFLAGS

first_rule: all

all:
	@$(MAKE) -C $(KERNEL_DIR) M=`pwd` ARCH=arm modules CROSS_COMPILE=$(CROSS) 

install_rule:
	@$(MAKE) -C $(KERNEL_DIR) M=`pwd` ARCH=arm modules_install INSTALL_MOD_PATH=$(TI_FS_PATH) INSTALL_MOD_DIR=/drivers/net CROSS_COMPILE=$(CROSS) 

clean_rule:
	@$(MAKE) -C $(KERNEL_DIR) M=`pwd` ARCH=arm clean CROSS_COMPILE=$(CROSS) 

config_rule:

uninstall_rule:
	@find $(TI_FS_PATH)/lib/modules -iname dsgni.ko | xargs rm -rf
    
endif
