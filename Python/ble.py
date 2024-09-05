import asyncio
from bleak import BleakScanner, BleakClient

address = '19DA62F0-C1A5-1216-B571-46B9465E68C3'
char_uuid = "0000ff01-0000-1000-8000-00805f9b34fb"

import time
READ_INTERVAL = 0 # How many ms to wait between read requests 

async def main(address,char_uuid):

    async with BleakClient(address) as client: # Connect to the device, disconnects when block exits
        print(f"Connected to {client.address}")
        print(f"Reading every {READ_INTERVAL}ms")

        # Loop
        while True:
            start = time.time()
            data = await client.read_gatt_char(char_uuid) # Takes about 100ms-200ms using 15-40ms connection interval
            print(f"This took {time.time()-start} seconds")


            for i in range(round(len(data)/6)):
                print(f"Distance: {data[i*6] | data[i*6+1]<<8} mm")

            await asyncio.sleep(READ_INTERVAL/1000)

asyncio.run(main(address,char_uuid))