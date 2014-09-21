Arduino code goes here!

## Sketches Overview

* **imu** - Our code and reference implementation for the razor imu
* **lib** - our custom libraries to share between sketches
* **Nano_encoder** - a usb-serial to encoder bridge using the robobuggy serial data protocol on an Arduino Nano
* **Protocol_prototype** - example of the robobuggy serial data protocol
* **RadioBuggyMega** - current (not yet!) steering/brake/rc implementation with robobuggy serial data protocol on the Arduino Mega
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

Upload:

```
$ cd directory/with/.ino
$ make upload ISP_PORT=/dev/tty.usbserial
```

```
 /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude  -p atmega328p -C /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf -c avrispmkii -P usb -U flash:w:build-nano328/Nano_encoder.hex:i
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

