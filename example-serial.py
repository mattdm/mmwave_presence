#!/usr/bin/python

# Important! Use Bluetooth and the HLK app to set the baud rate to
# 57600 before running this... or, change the below to 256000 to 
# match the default. I'm using 57600 because 256000 seems faster than
# the USB debug interface I have apparently supports reliably.


import serial
import time
import mmwave_presence as mmwave


port = serial.Serial("/dev/ttyUSB0", 57600, timeout=0.1)


mmwave = mmwave.MMWave(port)


mmwave.set_basic_config(8,8,presence_timeout=5)
mmwave.set_resolution(20)
mmwave.read()

print(mmwave)

mmwave.read_config()

while True:
    
    # update values
    mmwave.read()

    # printing like this is really just for debugging
    print(f"{mmwave}")

    time.sleep(0.1)