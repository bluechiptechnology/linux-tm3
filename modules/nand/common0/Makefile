mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
mkfile_dir := $(dir $(mkfile_path))

obj-$(CONFIG_SUNXI_NAND) += nand.o

libnand = libnand.a

$(obj)/$(libnand) : FORCE
ifneq ($(wildcard $(obj)/lib),)
	$(MAKE) -C $(obj)/.. build
endif

	cp $(mkfile_dir)/$(basename $(libnand)) $@

nand-objs += nand_interface.o \
	nfd/nand_ota_burn.o \
	nfd/nand_base.o \
	nfd/nand_blk.o \
	nfd/nand_dev.o \
	nfd/nand_class.o \
	nfd/nand_test.o \
	nfd/nand_osal_for_linux.o \
	nfd/nand_lib_cfg.o \
	$(libnand)
