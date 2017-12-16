#!/usr/bin/env python

import rospy
from robobuggy.msg import GPS

def subscriber_callback(data):
    rospy.loginfo("got gps msg: %f, %f", data.Lat_deg, data.Lon_deg)
    pass

def start_subscriber_spin():

    rospy.init_node("GPS_Plotter", anonymous=True)
    rospy.Subscriber("GPS", GPS, subscriber_callback)

    rospy.spin()

    pass

if __name__ == "__main__":
    start_subscriber_spin()
