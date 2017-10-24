"""
Send spikes AER packets over the UART device to test and validate the SpiNNaker IO Board
"""

import serial
import time
import pause
import datetime
 
conn = serial.Serial("/dev/ttyUSB1", 12000000)
if not conn.isOpen():
    conn.open()

spike_number = 0
dt_init = datetime.datetime.now()
while spike_number < 1000:
    dt = datetime.datetime.now()
    conn.write("@67890000\n")
    spike_number += 1
    pause.until(dt + datetime.timedelta(milliseconds=2))

print(datetime.datetime.now() - dt_init)
