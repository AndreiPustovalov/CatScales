import asyncio
import signal

from bleak import BleakScanner
from datetime import datetime
from struct import unpack
import json

# The official BTHome 16-bit UUID mapped to a standard 128-bit UUID
BTHOME_UUID = "0000fcd2-0000-1000-8000-00805f9b34fb"

def handler_factory(e: asyncio.Event):
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        e.set()
    return signal_handler

def detection_callback(device, advertisement_data):
    # Check if the device is broadcasting the BTHome service data
    if BTHOME_UUID in advertisement_data.service_data:
        raw_bytes = advertisement_data.service_data[BTHOME_UUID]
        hex_string = raw_bytes.hex(' ')  # Adds a space between bytes for readability
        (scaled_weight, bat, raw1, raw2) = unpack('<xxhxBxxlxxl', raw_bytes)
        scaled_weight /= 100
        data = {
            "timestamp": str(datetime.now()),
            "weight": scaled_weight,
            "bat": bat,
            "raw1": raw1,
            "raw2": raw2,
            "hex": hex_string,
            "address": device.address,
        }
        print(json.dumps(data))


async def main():
    e = asyncio.Event()
    signal.signal(signal.SIGINT, handler_factory(e))
    print("Scanning for BTHome devices... (Press Ctrl+C to stop)")
    scanner = BleakScanner(detection_callback=detection_callback)

    await scanner.start()
    await e.wait()
    await scanner.stop()


if __name__ == "__main__":
    asyncio.run(main())