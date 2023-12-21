#!/usr/bin/env sh

. fip_prebuilt/blmacros.env && ./fiptool.py -v genfip \
 './fip.bin' \
 --MONITOR_RUNADDR="${MONITOR_RUNADDR}" \
 --BLCP_2ND_RUNADDR="${BLCP_2ND_RUNADDR}" \
 --CHIP_CONF='./fip_prebuilt/chip_conf.bin' \
 --NOR_INFO='FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF' \
 --NAND_INFO='00000000'\
 --BL2='./fip_prebuilt/bl2.bin' \
 --BLCP_IMG_RUNADDR=0x05200200 \
 --BLCP_PARAM_LOADADDR=0 \
 --BLCP='./fip_prebuilt/empty.bin' \
 --DDR_PARAM='./fip_prebuilt/ddr_param.bin' \
 --BLCP_2ND='./fip_prebuilt/cvirtos.bin' \
 --MONITOR='./fip_prebuilt/fw_dynamic.bin' \
 --LOADER_2ND='./fip_prebuilt/u-boot-raw.bin' \
 --compress='lzma'
