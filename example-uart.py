# This is the demo for CircuitPython. Connect the sensor to 
# UART pins (and power!) and watch the serial output (over USB)

# Important! Use Bluetooth and the HLK app to set the baud rate to
# 57600 before running this... or, change the below to 256000 to 
# match the default. I'm using 57600 because 256000 seems faster than
# the USB debug interface I have apparently supports reliably.


import board
import busio
import time
import mmwave_presence as mmwave


uart = busio.UART(board.TX, board.RX, baudrate=57600)

mmwave = mmwave.MMWave(uart)

mmwave.set_basic_config(8,8,presence_timeout=5)
mmwave.set_resolution(75)

while True:
    
    # update values
    mmwave.read()

    # printing like this is really just for debugging
    print(f"{mmwave}")

    time.sleep(0.1)