#!/bin/sh

IPTV_BRG_CDEV=vfe_iptv_bridge

IPTV_BRG_MAJOR=`grep ${IPTV_BRG_CDEV} /proc/devices | cut -f 1 -d ' '`
mknod /dev/${IPTV_BRG_CDEV}0 c $IPTV_BRG_MAJOR 0

