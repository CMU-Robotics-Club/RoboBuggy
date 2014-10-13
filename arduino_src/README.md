Arduino code goes here!

## Sketches Overview

* **imu** - Our code and reference implementation for the razor imu
* **lib** - our custom libraries to share between sketches
* **Nano_encoder** - a usb-serial to encoder bridge using the robobuggy serial data protocol on an Arduino Nano
* **Protocol_prototype** - example of the robobuggy serial data protocol
* **RadioBuggyMega** - current steering/brake/rc implementation with robobuggy serial data protocol on the Arduino Mega
* **RadioBuggyMega2013** - legacy steering/brake/rc implementation for the Arduino Mega
* **unused** - who knows? maybe this is where we hide the driver!

## Compiling

We now have a Makefile for compiling arduino code outside of the arduino IDE. We need to do this mainly in order to share code between projects (like the serial protocol library).

Necessary:
* install the Arduino IDE (to get command line tools)
* may need to modify ARDUINO_DIR per your 
    * until we have a per-user config file, you may need to override this by providing it on the command line
* install make through your package manager of choice. you may need cygwin on windows

Compile:

```
$ cd directory/with/.ino
$ make
```

Upload (on a sane arduino board):

```
$ cd directory/with/.ino
$ make upload DEVICE_PATH=/dev/tty.usbserial######
```

The arduino nanos we have on the buggy right now do not have the proper auto-reset circuit, so they need to be programmed by ISP. This should able to be done with `make ispload`, but that command doesn't seem to work correctly in the makefile. I have been using the following command:

```
 /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude  -p atmega328p -C /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf -c avrispmkii -P usb -U flash:w:build-nano328/Nano_encoder.hex:i
```

## Useful Commands

Extract Hex Image From Flash Using AVR Programmer:

```
Arduino flash image can be extracted using this command:

avrdude -p m328p -Dn -c avrispmkII -U flash:r:outfile.bin:r

Replace "m328p" with the identifier for the MCU on your arduino
board. For the robobuggy, this is most likely "m2560" for atmega2560.

The list of possible MCU device ids can be displayed with:

avrdude -p ?

Replace "avrispmkII" with the identifier for you AVR programmer.
The list of possible programmer ids can be displayed with:

avrdude -c ?

If avrdude cannot find your device, you may need to specify the
serial port it is connected to. Use the -P option as follows:

avrdude -P <port>

Where <port> is a path to the serial port in /dev/

```

## Makefiles

We are using the Makefile from [https://github.com/sudar/Arduino-Makefile](https://github.com/sudar/Arduino-Makefile). Most of the implementation is in /arduino_src/Arduino.mk. Each sketch gets it's own mini makefile with sketch-specific information like board type.

These files belong to the make system:

* bin/
* [sketch]/Makefile
* Arduino.mk
* chipKIT.ml
* Common.mk

## Code standards

* We follow the code standards of the RoboBuggy project.
* We promote the [Google C++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html) where applicable.
* Please do not submit non-compilable code to the master branch.
* Please do make your own branch for any non-trivial change to avoid unexpected merge conflicts.

