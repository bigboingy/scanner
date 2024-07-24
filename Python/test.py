import bt

unprocessedBytes = bytearray() # Store bytes from incomplete packets
port = bt.getPortHandle(port="/dev/cu.HC-05")


bt.write(port,False,True,2)
while 1:
    data = bt.read(port, unprocessedBytes)["imu"]
    if data:
        print(f'found {len(data)}')
        bt.write(port,False,True,1)
