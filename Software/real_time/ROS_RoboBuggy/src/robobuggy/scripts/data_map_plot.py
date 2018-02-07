#!/usr/bin/env python

import rospy
import math
from gps_common.msg import GPSFix
from robobuggy.msg import GPS
from robobuggy.msg import Pose
from robobuggy.msg import Command

viz_pub = None;

def pose_callback(data):
    global viz_pub

    rospy.loginfo("got Pose msg: %f degrees lat, %f degrees long, %f rad", data.latitude_deg, data.longitude_deg, data.heading_rad)

    #data.heading_rad is in Radians from north clockwise, but GPSFix requires degrees from north
    degrees_from_north = (180 * data.heading_rad) / math.pi

    last_latitude = data.latitude_deg
    last_longitude = data.longitude_deg
    last_heading = degrees_from_north
    
    viz_msg_heading = GPSFix(latitude=data.latitude_deg, longitude=data.longitude_deg, track = degrees_from_north)
 
    viz_pub.publish(viz_msg_heading)

    pass

def command_callback(data):
    global viz_pub
    
    #steer_cmd should be degrees
    rospy.loginfo("got Steering Command msg: %f degrees", data.steer_cmd)

    viz_msg_steering = GPSFix(latitude=last_latitude, longitude=last_longitude, track = last_heading + data.steer_cmd)
 
    viz_pub.publish(viz_msg_steering)

    pass

def start_subscriber_spin():
    global viz_pub

    global last_latitude
    global last_longitude
    global last_heading
    
    rospy.init_node("GPS_Plotter", anonymous=True)
    viz_pub = rospy.Publisher('GPS_VIZ', GPSFix, queue_size=10);
    rospy.Subscriber("Pose", Pose, pose_callback)
    rospy.Subscriber("Command", Command, command_callback)

    rospy.spin()

    pass

if __name__ == "__main__":
    start_subscriber_spin()
