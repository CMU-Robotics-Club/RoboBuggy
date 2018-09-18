#!/usr/bin/env python

import rospy
import math
import json
import time
from gps_common.msg import GPSFix
from robobuggy.msg import Command
from robobuggy.msg import GPS
from robobuggy.msg import Pose
from robobuggy.msg import Command



def pose_callback(data):
    global viz_pose_pub
    global last_latitude
    global last_longitude
    global last_heading_deg

    #data.heading_rad is in Radians from north clockwise, but GPSFix requires degrees from north
    degrees_from_north = math.degrees(data.heading_rad)

    last_latitude = data.latitude_deg
    last_longitude = data.longitude_deg
    last_heading_deg = -degrees_from_north + 90

    rospy.loginfo("got Pose msg: %f degrees lat, %f degrees long, %f bearing", last_latitude, last_longitude, last_heading_deg)

    viz_msg_heading = GPSFix(latitude=last_latitude, longitude=last_longitude, track=last_heading_deg)
    viz_pose_pub.publish(viz_msg_heading)


def gps_callback(data):
    global viz_gps_pub
    rospy.loginfo("Got GPS message: %f lat, %f lon", data.Lat_deg, data.Long_deg)
    viz_msg_gps = GPSFix(latitude=data.Lat_deg, longitude=data.Long_deg)
    viz_gps_pub.publish(viz_msg_gps)

def command_callback(data):
    global viz_command_pub
    global last_latitude
    global last_longitude
    global last_heading_deg

    #steer_cmd should be degrees
    rospy.loginfo("got Steering Command msg: %f degrees", math.degrees(data.steer_cmd_rad))

    viz_msg_steering = GPSFix(latitude=last_latitude, longitude=last_longitude, track = last_heading_deg - math.degrees(data.steer_cmd_rad))

    viz_command_pub.publish(viz_msg_steering)

    pass

def ground_truth_callback(data):
    global viz_grndtruth_pub

    degrees_from_north = -math.degrees(data.heading_rad) + 90

    rospy.loginfo("Got Ground Truth message: (%f, %f, %f)", data.latitude_deg, data.longitude_deg, degrees_from_north)

    viz_msg_groundtruth = GPSFix(latitude=data.latitude_deg, longitude=data.longitude_deg, track=degrees_from_north)
    viz_grndtruth_pub.publish(viz_msg_groundtruth)
    pass


def waypoints_publisher():
    waypoints_pub = rospy.Publisher('WAYPOINTS_VIZ',GPSFix, queue_size=10)

    # read from waypoints file, parse JSON, publish
    waypoints = open("/home/bhai/RoboBuggy/Software/real_time/ROS_RoboBuggy/src/robobuggy/config/waypoints.txt", 'r')
    line = waypoints.readline()
    time.sleep(5)
    while line:
    	data = json.loads(line)
        viz_msg_waypoint = GPSFix(latitude=data['latitude'],longitude=data['longitude'])
        waypoints_pub.publish(viz_msg_waypoint)
        time.sleep(.01)
        line = waypoints.readline()
    waypoints.close()
    pass

def start_subscriber_spin():
    global viz_gps_pub
    global viz_pose_pub
    global viz_command_pub
    global viz_grndtruth_pub

    global last_latitude
    global last_longitude
    global last_heading_deg
    last_heading_deg = 0

    rospy.init_node("Data Visualizer", anonymous=True)

    viz_gps_pub = rospy.Publisher("GPS_VIZ", GPSFix, queue_size=10)
    viz_pose_pub = rospy.Publisher("POSE_VIZ", GPSFix, queue_size=10)
    viz_command_pub = rospy.Publisher('STEERING_COMMAND_VIZ', GPSFix, queue_size = 10)
    viz_grndtruth_pub = rospy.Publisher("SIM_GROUNDTRUTH_VIZ", GPSFix, queue_size=10)

    rospy.Subscriber("Pose", Pose, pose_callback)
    rospy.Subscriber("GPS", GPS, gps_callback)
    rospy.Subscriber("Command", Command, command_callback)
    rospy.Subscriber("SIM_Ground_Truth", Pose, ground_truth_callback)
    waypoints_publisher()

    rospy.spin()


if __name__ == "__main__":
    start_subscriber_spin()
