#!/usr/bin/env python

import rospy
import math
import json
import time
from robobuggy.msg import IMU

def magnetometer_callback(data):
    global x
    global y
    global z
    x = data.X_Mag
    y = data.Y_Mag
    z = data.Z_Mag

    rospy.loginfo("orientation: %f", math.degrees(math.atan2(y,x)))

    #plot on Tkinter window
    pass

def start_subscriber_spin():
    rospy.init_node("IMU_Plotter", anonymous=True)

    rospy.Subscriber("IMU", IMU, magnetometer_callback)

    #tkinter graphics window
    rospy.spin()
    pass

if __name__ == "__main__":
    start_subscriber_spin()