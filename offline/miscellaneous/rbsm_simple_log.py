# rbsm_simple_log.py
# Author: Ian Hartwig (ihartwig)
#
# Quick python shell program to dump RBSM data to a text log file.
#

import rbsm_lib
import sys
import time


mid_to_str = {
  0: "ENC_TICKS_LAST",
  1: "ENC_TICKS_RESET",
  2: "ENC_TIMESTAMP",
  20: "MEGA_STEER_ANGLE",
  21: "MEGA_BRAKE_STATE",
  22: "MEGA_AUTON_STATE",
  23: "MEGA_BATTERY_LEVEL",
  252: "RESERVED",
  254: "ERROR",
  255: "DEVICE_ID"
}


def main():
  print("main")
  # process args
  if(len(sys.argv) < 2):
    print "You didn't provide enough arguments. Please run with:"
    print "%s /dev/tty.something" % (sys.argv[0])
    sys.exit()
  rbsm_dev = sys.argv[1]

  # setup incomming messages
  rbsm_endpoint = rbsm_lib.RBSerialMessage(rbsm_dev)

  # read new messages forever
  while(1):
    new_message = rbsm_endpoint.read()
    if(new_message["status"] == "locked"):
      # write out new data
      nice_mid = mid_to_str.get(new_message["id"], "%d"%(new_message["id"]))
      print("time: %d, mid: %s, data: %d" % (time.time, nice_mid, new_message["data"]))

    else:
      print("Stream unlocked!")

  return None


if __name__ == "__main__":
  main()
