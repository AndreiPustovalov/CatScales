#!/bin/bash
set -e

candidates=("COM12" "COM15" "COM20")

COM_PORT=""
for port in "${candidates[@]}"; do
    if mode.com "$port" >/dev/null 2>&1; then
        COM_PORT="$port"
    fi
done

if [ -z "$COM_PORT" ]; then
  echo "No ports"
  exit 1
fi

cargo build --release
arm-none-eabi-objcopy -O ihex target/thumbv7em-none-eabihf/release/CatScales target/CatScales.hex
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x0123 --application target/CatScales.hex target/CatScales.zip
adafruit-nrfutil --verbose dfu serial -pkg target/CatScales.zip -p "$COM_PORT" -b 115200 --singlebank