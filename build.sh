#make tm3linux_defconfig
make -j8
cp arch/arm64/boot/Image /tftpboot/
cp ./arch/arm64/boot/dts/sunxi/tm3-hbpicm4-cm4io.dtb /tftpboot/tm3.dtb

cp arch/arm64/boot/Image ${TM3_LINUX_PACK_OUT_DIR}
cp arch/arm64/boot/dts/sunxi/tm3*.dtb ${TM3_LINUX_PACK_OUT_DIR}

