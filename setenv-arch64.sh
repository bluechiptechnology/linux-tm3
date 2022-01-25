export PROJECT=tm3
export PRJROOT=/embedded/projects/${PROJECT}
export ARCH=arm64
export TARGET=aarch64-linux-gnu
export TOOLSDIR=/embedded/projects/tm3/lichee/out/gcc-linaro-5.3.1-2016.05/gcc-aarch64
export PATH=${TOOLSDIR}/bin:${PATH}
export CROSS_COMPILE=${TARGET}-
export LICHEE_KDIR=/embedded/projects/tm3/lichee/linux-4.9
export LICHEE_PLATFORM=linux
export LICHEE_MOD_DIR=/embedded/projects/tm3/lichee/linux-4.9/gpubinaries 
export LICHEE_OUT=/embedded/projects/tm3/lichee/out/tm3/linux/common
export LICHEE_CHIP=tm3
export LICHEE_BOARD=tmxdevboard
export LICHEE_ARCH=arm64
export LICHEE_TOOLS_DIR=/embedded/projects/tm3/lichee/tools
export LICHEE_OUT_DIR=/embedded/projects/tm3/lichee/out
export LICHEE_PLATFORM=linux
export LICHEE_PLAT_OUT="${LICHEE_OUT_DIR}/${LICHEE_CHIP}/${LICHEE_PLATFORM}/common"


