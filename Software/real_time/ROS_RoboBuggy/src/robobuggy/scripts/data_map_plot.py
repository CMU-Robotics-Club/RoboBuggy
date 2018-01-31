#!/usr/bin/env python

import rospy
import math
from gps_common.msg import GPSFix
from robobuggy.msg import GPS
from robobuggy.msg import Pose

viz_pub = None;

def subscriber_callback(data):
    global viz_pub;
    rospy.loginfo("got Pose msg: %f degrees lat, %f degrees long, %f rad", data.latitude_deg, data.longitude_deg, data.heading_rad)

    #data.heading_rad is in Radians from north clockwise, but GPSFix requires degrees from north
    float64 degrees_from_north = 180 * data.heading_rad / math.pi
    float64 steering_angle = 180 * data.steer_cmd / math.pi
   
    viz_msg_heading = GPSFix(latitude=data.latitude_deg, longitude=data.longitude_deg, track = degrees_from_north)
    viz_msg_steering = GPSFix(latitude=data.latitude_deg, longitude=data.longitude_deg, track = degrees_from_north + steering_angle)
 
    viz_pub.publish(viz_msg_heading)
    viz_pub.publish(viz_msg_steering)

    pass

def start_subscriber_spin():
    global viz_pub
    rospy.init_node("GPS_Plotter", anonymous=True)
    viz_pub = rospy.Publisher('GPS_VIZ', GPSFix, queue_size=10);
    rospy.Subscriber("Pose", Pose, subscriber_callback)
    rospy.Subscriber("Command", Command, subscriber_callback)

    rospy.spin()

    pass

if __name__ == "__main__":
    start_subscriber_spin()

