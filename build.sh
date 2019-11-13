make -j8 
cp arch/arm64/boot/Image /tftpboot/
#cp ./arch/arm64/boot/dts/sunxi/sun50iw6p1-soc.dtb /tftpboot/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /tftpboot/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /embedded/projects/h6/lichee/brandy/u-boot-2014.07/misc/PineH64.dtb
#cp ./arch/arm64/boot/dts/sunxi/PineH64.dtb /tftpboot/OrangePiH6.dtb
#cp ./arch/arm64/boot/dts/sunxi/tm3-tmxdevboard.dtb /tftpboot/tm3.dtb
cp ./arch/arm64/boot/dts/sunxi/tm3-hb5.dtb /tftpboot/tm3.dtb


cp ./arch/arm64/boot/dts/sunxi/tm3-hb5.dtb /embedded/projects/tm3/lichee/out/tm3/android/common/sunxi.dtb
#cp ./arch/arm64/boot/dts/sunxi/tm3-tmxdevboard.dtb /embedded/projects/tm3/lichee/out/tm3/android/common/sunxi.dtb
cp arch/arm64/boot/Image /nfs/rootfs/opt/boot/
