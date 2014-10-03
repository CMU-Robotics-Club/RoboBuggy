# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

# if you have placed the alternate core in your sketchbook directory, then you can just mention the core name alone.
# ALTERNATE_CORE = attiny
# If not, you might have to include the full path.
#ALTERNATE_CORE_PATH = /home/sudar/Dropbox/code/arduino-sketches/hardware/attiny/

BOARD_TAG     = mega2560
ISP_PORT      = /dev/tty.asdf
USER_LIB_PATH = ../lib
ARDUINO_LIBS  = rbserialmessages

include ../Arduino.mk

# !!! Important. You have to use make ispload to upload when using ISP programmer
