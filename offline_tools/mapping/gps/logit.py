#! /usr/env/python

import serial
import pygame
import pygame.camera
import sys
import os
import time
import convertNmea

def main():
  # check args
  if(len(sys.argv) < 2):
    print "you didn't provide enough args, you idiot"
    print "%s /dev/tty.something [/log/folder/path]" % sys.argv[0]
    sys.exit()

  # get serial connection
  gps = serial.Serial(sys.argv[1], 9600, timeout=1)

  # init camera
  pygame.init()
  pygame.camera.init()
  camlist = pygame.camera.list_cameras()
  print 'avilable Camreas:' + str(camlist)+'\n'
  camToUse = "/dev/video1"
  cam =pygame.camera.Camera(camToUse,(640,480))
  print 'useing'+camToUse+'\n'
  cam.start()

  while True:
    print "press enter to make a new data point"
    raw_input()

    # make a new folder
    start_time = time.time()
    log_folder = str(start_time)
    os.mkdir(log_folder)

    # collect an image
    img = cam.get_image()
    pygame.image.save(img, "image.jpg")

    # collect gps data for 10 seconds
    gps.flushInput()
    with open("%s/gpslog.txt" % (log_folder), "w") as gpslog_file:
      while(time.time() < start_time + 10):
        gps_line = gps.readline()
        print gps_line
        gpslog_file.write(gps_line)





if __name__ == "__main__":
  main()
