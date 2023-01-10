KMODULE_NAME = ssv6x5x

ifeq ($(KERNEL_OBJ_PATH),)
KERNEL_OBJ_PATH := /lib/modules/`uname -r`/build
endif

ifeq ($(SSV_DRV_PATH),)
SSV_DRV_PATH := drivers/net/wireless/ssv6x5x
endif

KBUILD_TOP := $(SSV_DRV_PATH)
ifeq ($(MAKELEVEL),0)
KBUILD_TOP := .
endif

ifeq ($(ARCH),)
ARCH := x86
endif

ifeq ($(INSTALL_PATH),)
INSTALL_PATH := kernel/drivers/net/wireless/ssv6x5x
endif

#include $(KBUILD_TOP)/config_common.mak
#include $(KBUILD_TOP)/config.mak

MOD_DEF_H = include/ssv_mod_conf.h

# Generate version strings
# GEN_VER := $(shell cd $(KBUILD_TOP); ./ver_info.pl include/ssv_version.h)
# Generate include/ssv_conf_parser.h
# GEN_CONF_PARSER := $(shell cd $(KBUILD_TOP); env ccflags="$(ccflags-y)" ./parser-conf.sh include/ssv_conf_parser.h)
# Generate $(KMODULE_NAME)-wifi.cfg
#BKP_CFG := $(shell cp $(KBUILD_TOP)/$(PLATFORMS)-wifi.cfg $(KBUILD_TOP)/image/$(KMODULE_NAME)-wifi.cfg)

EXTRA_CFLAGS := -I$(SSV_DRV_PATH) -I$(SSV_DRV_PATH)/include

#DEF_PARSER_H = $(SSV_DRV_PATH)/include/ssv_conf_parser.h
#$(shell env ccflags="$(ccflags-y)" $(SSV_DRV_PATH)/parser-conf.sh $(DEF_PARSER_H))
#-------------------------------
#extra config
ccflags-y += -D__CHECK_ENDIAN__
ccflags-y += -DDEBUG
ccflags-y += -DSSV_SUPPORT_SSV6020
ccflags-y += -DCONFIG_SSV6200_CLI_ENABLE
ccflags-y += -DCONFIG_SSV_BUILD_AS_ONE_KO
ccflags-y += -DCONFIG_SSV_TX_LOWTHRESHOLD
ccflags-y += -DCONFIG_SSV6200_HAS_RX_WORKQUEUE
ccflags-y += -DUSE_THREAD_RX
ccflags-y += -DUSE_THREAD_TX
ccflags-y += -DENABLE_AGGREGATE_IN_TIME
ccflags-y += -DENABLE_INCREMENTAL_AGGREGATION
ccflags-y += -DUSE_GENERIC_DECI_TBL
ccflags-y += -DSSV6200_ECO
ccflags-y += -DHWIF_SDIO_RX_IRQ
ccflags-y += -DCONFIG_STA_BCN_FILTER
ccflags-y += -DCONFIG_SSV_USE_SDIO_DAT1_AS_INT
ccflags-y += -DCONFIG_SDIO_FAVOR_RX
ccflags-y += -DHWIF_IGNORE_SAFE_RW_REG
ccflags-y += -DCONFIG_HW_SCAN
ccflags-y += -DREPORT_TX_STATUS_DIRECTLY
ccflags-y += -DCONFIG_SSV6XXX_DEBUGFS

ccflags-y += -DCONFIG_SDIO_FAVOR_RX
ccflags-y += -DCONFIG_MMC_DISALLOW_STACK
ccflags-y += -DSSV6X5X_PM_MOD
ccflags-y += -DCONFIG_BLE
ccflags-y += -DCONFIG_BLE_HCI_BUS=1
#------------------------------
# ssvdevice/
KERN_SRCS := ssvdevice/ssvdevice.c
KERN_SRCS += ssvdevice/ssv_cmd.c

# hci/
KERN_SRCS += hci/ssv_hci.c

# smac/
KERN_SRCS += smac/init.c
KERN_SRCS += smac/dev.c
KERN_SRCS += smac/bdev.c
KERN_SRCS += smac/regd.c
KERN_SRCS += smac/wow.c
KERN_SRCS += smac/hw_scan.c

KERN_SRCS += smac/lib.c
KERN_SRCS += smac/ap.c

ifeq ($(findstring -DCONFIG_SSV6XXX_DEBUGFS, $(ccflags-y)), -DCONFIG_SSV6XXX_DEBUGFS)
KERN_SRCS += smac/ssv6xxx_debugfs.c
endif
KERN_SRCS += smac/efuse.c
KERN_SRCS += smac/ssv_skb.c
ifeq ($(findstring -DCONFIG_SSV_CTL, $(ccflags-y)), -DCONFIG_SSV_CTL)
KERN_SRCS += smac/ssv_wifi_ctl.c
ifeq ($(findstring -DCONFIG_SSV_SMARTLINK, $(ccflags-y)), -DCONFIG_SSV_SMARTLINK)
KERN_SRCS += smac/kssvsmart.c
endif
endif

KERN_SRCS += smac/hal/hal.c
ifeq ($(findstring -DSSV_SUPPORT_SSV6006, $(ccflags-y)), -DSSV_SUPPORT_SSV6006)
KERN_SRCS += smac/hal/ssv6006c/ssv6006_common.c
KERN_SRCS += smac/hal/ssv6006c/ssv6006C_mac.c
KERN_SRCS += smac/hal/ssv6006c/ssv6006_phy.c
KERN_SRCS += smac/hal/ssv6006c/ssv6006_turismoC.c
endif
ifeq ($(findstring -DSSV_SUPPORT_SSV6020, $(ccflags-y)), -DSSV_SUPPORT_SSV6020)
KERN_SRCS += smac/hal/ssv6020/ssv6020_common.c
KERN_SRCS += smac/hal/ssv6020/ssv6020_mac.c
KERN_SRCS += smac/hal/ssv6020/ssv6020_phy.c
KERN_SRCS += smac/hal/ssv6020/ssv6020_turismoE.c
endif

# hwif/hal/
KERN_SRCS += hwif/hal/hwif_hal.c
ifeq ($(findstring -DSSV_SUPPORT_SSV6006, $(ccflags-y)), -DSSV_SUPPORT_SSV6006)
KERN_SRCS += hwif/hal/ssv6006c/ssv6006C_hwif.c
endif
ifeq ($(findstring -DSSV_SUPPORT_SSV6020, $(ccflags-y)), -DSSV_SUPPORT_SSV6020)
KERN_SRCS += hwif/hal/ssv6020/ssv6020_hwif.c
endif

# hwif/sdio/
KERN_SRCS += hwif/sdio/sdio.c


#------------------------------

KERN_SRCS += $(KMODULE_NAME)-generic-wlan.c

$(KMODULE_NAME)-y += $(KERN_SRCS_S:.S=.o)
$(KMODULE_NAME)-y += $(KERN_SRCS:.c=.o)

obj-$(CONFIG_SSV6X5X) += $(KMODULE_NAME).o

#export CONFIG_SSV6X5X=m

.PHONY: all ver modules clean dinfo

all: dinfo $(MOD_DEF_H) modules

dinfo:
	echo "KBUILD_TOP: $(KBUILD_TOP)"
	
modules:
	$(MAKE) -C $(KERNEL_OBJ_PATH) M=$(SSV_DRV_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

strip:
	$(CROSS_COMPILE)strip $(KMODULE_NAME).ko --strip-unneeded

install:
	$(MAKE) INSTALL_MOD_DIR=$(INSTALL_PATH) -C $(KERNEL_OBJ_PATH) \
            M=$(SSV_DRV_PATH) modules_install
	depmod -a

clean:
	$(MAKE) -C $(KERNEL_OBJ_PATH) M=$(SSV_DRV_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
	@rm -rf $(MOD_DEF_H)

$(MOD_DEF_H): config.mak config_common.mak
	#@echo "#ifndef __SSV_MOD_CONF_H__" > $@
	#@echo "#define __SSV_MOD_CONF_H__" >> $@
	#for flag in $(ccflags-y_no_dash); do \
	#   if [ "$$flag" =~ ^D.* ]; then \
	#       echo "#define $$flag" | sed -e s/D//g >> $@; \
	#   fi; \
	#done
	#echo "#endif // __SSV_MOD_CONF_H__" >> $@
	env ccflags="$(ccflags-y)" ./genconf.sh $@

