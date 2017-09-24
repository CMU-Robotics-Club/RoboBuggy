#!/bin/bash
# This script initializes you to compile our Arduino code.
#
# Steps to your first build:
#  1. Download and install cygwin with svn and wget
#  1b. Download Arduino 1.0.6+. 
#      DO NOT INSTALL IT IN THE DEFAULT LOCATION
#      Install it into C:/Arduino
#  2. We will install apt-cyg, the cygwin package manager
#  3. We will install make through apt-cyg
#  4. We will confirm we installed python
#  5. We will install pip, the python package manager
#  6. We will install pyserial via pip. 

# 1. Do it on your own!

# 2. Install apt-cyg
svn --force export http://apt-cyg.googlecode.com/svn/trunk/ /bin/
chmod +x /bin/apt-cyg

# 3. Install make
apt-cyg install make

# 4. Install python
apt-cyg install python

# 5. Install pip
wget https://bootstrap.pypa.io/get-pip.py
/usr/bin/env python get-pip.py
# /usr/bin/env is necessary; otherwise, the windows python seems to take priority?
rm get-pip.py

# 6. Install pyserial



# Compiling should work!


# Current Pain Points/Excitement:
#  - Can't load programs correctly. Errors about serial port. have tried both cygwin and windows
#    syntax for serial ports 
