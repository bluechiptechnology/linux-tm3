make -j8 
cp arch/arm64/boot/Image /tftpboot/
#cp ./arch/arm64/boot/dts/sunxi/sun50iw6p1-soc.dtb /tftpboot/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /tftpboot/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /embedded/projects/h6/lichee/brandy/u-boot-2014.07/misc/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /tftpboot/OrangePiH6.dtb
#cp ./arch/arm64/boot/dts/sunxi/tm3-tmxdevboard.dtb /tftpboot/tm3.dtb
cp ./arch/arm64/boot/dts/sunxi/tm3-hb5.dtb /tftpboot/tm3.dtb


#cp ./arch/arm64/boot/dts/sunxi/tm3-hb5.dtb /embedded/projects/tm3/lichee/out/tm3/android/common/sunxi.dtb
#cp ./arch/arm64/boot/dts/sunxi/tm3-tmxdevboard.dtb ${LICHEE_OUT}/sunxi.dtb
#cp ./arch/arm64/boot/dts/sunxi/tm3-hb5.dtb ${LICHEE_OUT}/sunxi.dtb
cp arch/arm64/boot/Image ${LICHEE_OUT}/
cp arch/arm64/boot/Image /nfs/rootfs/opt/boot/

cd ${AWPACKTOOL}
set +e
pack -ctm3 -plinux -bhb5 -duart0
set -e
cd -

echo Copy firmware to nfs
cp ${AWPACKTOOL}/out/boot0_sdcard.fex /nfs/rootfs/opt/boot/boot0.bin
cp ${AWPACKTOOL}/out/boot_package.fex /nfs/rootfs/opt/boot/uboot.bin

