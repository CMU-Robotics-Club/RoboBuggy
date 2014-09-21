Arduino code goes here!

## Sketches Overview

* **imu** - Our code and reference implementation for the razor imu
* **Nano_encoder** - a usb-serial to encoder bridge using the robobuggy serial data protocol on an Arduino Nano
* **Protocol_prototype** - example of the robobuggy serial data protocol
* **RadioBuggyMega** - current (not yet!) steering/brake/rc implementation with robobuggy serial data protocol on the Arduino Mega
* **RadioBuggyMega2013** - legacy steering/brake/rc implementation for the Arduino Mega
* **unused** - who knows? maybe this is where we hide the driver!

## Compiling

We don't have a clean way to make this code outside the Arduino IDE, so our solution right now is to open the ino file of whatever project you'd like to compile and run "Verify."

## Code standards

* We follow the code standards of the RoboBuggy project.
* We promote the [Google C++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html) where applicable.
* Please do not submit non-compilable code to the master branch.
* Please do make your own branch for any non-trivial change to avoid unexpected merge conflicts.

