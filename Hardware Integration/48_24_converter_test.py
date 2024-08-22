#Read the output of the 48->24V converter and display information (current, voltage, temp)

import smbus
import time

bus = smbus.SMBus(1)
address = 0x60
read_header = 0x51

bus.write_byte_data(address, read_header, 0x01) #start reading: 0 = WRITE, 1 = READ

time.sleep(0.7)
