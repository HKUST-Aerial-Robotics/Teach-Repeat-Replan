#!/usr/bin/env bash

# Download the arm gcc cross compile toolchain
#wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2 -P ~/Downloads

# Extract to the user home folder
tar jxvf ~/Downloads/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2 -C ~

# Add the toolchain to path
export PATH=~/gcc-arm-none-eabi-5_4-2016q2/bin:$PATH

# Build code. Expected binary is named osdk_stm32_gcc.hex
make clean
make
