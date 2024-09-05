import asyncio
from bleak import BleakClient
from config import Lidar, Cartesian, Imu, BYTES_IMU, BYTES_LIDAR

address = '19DA62F0-C1A5-1216-B571-46B9465E68C3'
char_uuid = "0000ff01-0000-1000-8000-00805f9b34fb"

# Create a queue that we will use to store our data
queue = asyncio.Queue() # Defaults to infinite length

# Establish ble connection
# Args: *args are the async functions to run concurrently with the ble loop
# These funcs run alongside the loop which constantly reads ble and adds data to queue
async def main(*args): 

    async with BleakClient(address) as client: # Connect to the device, disconnects when block exits
        print(f"Connected to {client.address}")

        # Run tasks
        await asyncio.gather(*args,_loop(client)) # This automatically schedules coroutines as tasks

# Infinite loop
# Constantly reads ble and adds data to the queue
async def _loop(client:BleakClient):
    while True:
        # Read ble
        data = await client.read_gatt_char(char_uuid) # Takes around 60ms using 15-30ms connection interval
        # Add data to queue
        queue.put_nowait(data)

# Function to call to read everything from the queue. Designed to be used in an external async function
# Blocks until at least one queue item has been read
# Args: none, but needs the queue which is defined globally in this file
# Returns: A dict with lidar and imu lists, which contain dataclasses
async def read() -> dict:

    result = { # To be returned
        'lidar':[],
        'imu':[]
    }

    # Keep reading queue objects from the queue until the queue is empty
    # One queue object represents one read from ble - a bunch of bytes
    while not queue.empty() or not len(result['lidar']): # Also make sure we get at least one queue read

        data = await queue.get() # Get data from queue, waits until it arrives if there's none
        queue.task_done() # Indicate queue item has been processed

        # Format data
        reads = len(data)/(BYTES_IMU+BYTES_LIDAR) # This should be a whole number
        if not reads.is_integer():
            print(f"Unexpected number of bytes: {reads}")
            reads = round(reads)-1 # Isn't a good fix, but shouldn't happen
        
        for _ in range(int(reads)): # For each lidar/imu pair
            lidar = Lidar()
            imu = Imu()
            lidar.populate(data[0:6])
            imu.populate(data[6:])
            result["lidar"].append(lidar) # Add to result
            result["imu"].append(imu)

    return result

if __name__ == "__main__":

    # Example usage: function for printing data every second
    # This whole thing becomes a task
    async def foo():
        while True:
            result = await read()
            await asyncio.sleep(1)
            print(f"Distance: {result['lidar'][-1].dist} mm")
            print(f"Acc x: {result['imu'][-1].acc.x}")
            print(f"Queue size: {queue.qsize()} items")

    asyncio.run(main(foo()),debug=True) # Creates the event loop, runs the ble loop and any functions we've included