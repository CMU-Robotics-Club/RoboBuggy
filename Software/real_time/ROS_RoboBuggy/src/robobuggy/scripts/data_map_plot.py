#!/usr/bin/env python

import rospy
from gps_common.msg import GPSFix
from robobuggy.msg import GPS

viz_pub = None;

def subscriber_callback(data):
    global viz_pub;
    rospy.loginfo("got gps msg: %f, %f", data.Lat_deg, data.Long_deg)

    viz_msg = GPSFix(latitude=data.Lat_deg, longitude=data.Long_deg);
    viz_pub.publish(viz_msg);

    pass

def start_subscriber_spin():
    global viz_pub
    rospy.init_node("GPS_Plotter", anonymous=True)
    viz_pub = rospy.Publisher('GPS_VIZ', GPSFix, queue_size=10);
    rospy.Subscriber("GPS", GPS, subscriber_callback)

    rospy.spin()

    pass

if __name__ == "__main__":
    start_subscriber_spin()

