#! /usr/env/python

import serial
import sys
import os
import time


def main():
  # check args
  if(len(sys.argv) < 2):
    print "you didn't provide enough args, you idiot"
    print "%s /dev/tty.something [/log/folder/path]" % sys.argv[0]
    sys.exit()

  # get serial connection
  gps = serial.Serial(sys.argv[1], 9600, timeout=1)

  # make a new folder
  start_time = time.time()
  log_folder = str(start_time)
  os.mkdir(log_folder)

  while True:
    # collect gps data for 10 seconds
    with open("%s/gpslog.txt" % (log_folder), "w") as gpslog_file:
      gps_line = gps.readline()
      if("GPGGA" in gps_line):
        print gps_line
      gpslog_file.write(gps_line)





if __name__ == "__main__":
  main()
