from bluetooth import *
import struct
import numpy as np

class connection():
    def __init__(self):
        serverMACAddress = '98:D3:31:F5:B0:BF'
        port = 1
        self.s = BluetoothSocket(RFCOMM)
        self.s.connect((serverMACAddress, port))

    def send(self, x, y):
        output = chr(x)+chr(y)
        self.s.send(output)

    def __del__(self):
        self.s.close()

# c = connection()
# while True:
#     text = input()
#     pos_list = text.split(",")
#     x, y = int(pos_list[0]), int(pos_list[1])
#     c.send(x, y)