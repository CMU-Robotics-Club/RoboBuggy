import atexit
import curses
import rbsm_lib
import sys
import threading
import time
import struct
import signal
import serial



ser = serial.Serial('/dev/tty.usbmodem14121', 76800, timeout=0.5,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
time.sleep(1)
ser.flush()
values = [chr(67), chr(68), chr(69), chr(70), chr(71)]
#values = bytearray([67,68,69,70,71])
for i in xrange(10):
    ser.write(chr(67))
#    time.sleep(.1)
#for i in xrange(65,75):
#    ser.write(chr(i))
ser.flush()
time.sleep(2)
ser.close
