#!/usr/bin/env sh

rm -rf tmp
./genimage --config genimage-milkv-duo.cfg --rootpath ./root --inputpath ./ --outputpath ./
