##
##	Configure for Eddy
##

export TOPMOST_DIR=`pwd`;

## Toolchain Path
##                                                      <<-- Config It !!!
export COMPILER_DIR=/opt/lemonix/cdt
export PATH=$TOPMOST_DIR/tool:$COMPILER_DIR/bin:$PATH
export CROSS_COMPILE=/opt/lemonix/cdt/bin/arm-linux-
export COMPILER_NAME=arm
export TARGET_BOARD=eddy

## FOR OS
##                                                      <<-- Config It !!!
export OS_VERSION=2.1.0.1
export OS_DIR=$TOPMOST_DIR/lemonix
export OS_INCLUDE_DIR=${OS_DIR}/include

## ETC
##																											<<-- 
export MKIMAGE_CMD=$TOPMOST_DIR/tool/mkimage
export EXT2_CMD=$TOPMOST_DIR/tool/genext2fs
export JFFS2_CMD=$TOPMOST_DIR/tool/mkfs.jffs2
export CRAMFS_CMD=$TOPMOST_DIR/tool/mkcramfs

