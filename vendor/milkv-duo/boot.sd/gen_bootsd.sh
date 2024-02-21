#!/usr/bin/env sh
lzma -c -9 -f -k ./kernel.bin > ./kernel.bin.lzma
#lzma -c -9 -f -k ./fs.img > ./fs.img.lzma
./mkimage -f ./multi.its -r ./boot.sd ./
