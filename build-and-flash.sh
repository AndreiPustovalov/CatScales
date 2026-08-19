#!/bin/bash
set -e

COM_PORT=COM12
#COM_PORT=COM15

cargo build --release
arm-none-eabi-objcopy -O ihex target/thumbv7em-none-eabihf/release/CatScales target/CatScales.hex
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x0123 --application target/CatScales.hex target/CatScales.zip
adafruit-nrfutil --verbose dfu serial -pkg target/CatScales.zip -p $COM_PORT -b 115200 --singlebank