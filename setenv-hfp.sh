export PROJECT=tm3
export PRJROOT=/embedded/projects/${PROJECT}
export ARCH=arm64
export TARGET=aarch64-linux-gnu
export TOOLSDIR=/embedded/projects/tm3/lichee/brandy/toolchain/gcc-aarch64
export PATH=${TOOLSDIR}/bin:${PATH}
export CROSS_COMPILE=${TARGET}-
export LICHEE_KDIR=/embedded/projects/tm3/lichee/linux-4.9
export LICHEE_PLATFORM=linux
export LICHEE_MOD_DIR=/embedded/projects/tm3/lichee/linux-4.9/gpubinaries 

