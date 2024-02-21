#!/usr/bin/env sh

rm -rf tmp root
mkdir root
./genimage --config genimage-milkv-duo.cfg --rootpath ./root --inputpath ./ --outputpath ./
