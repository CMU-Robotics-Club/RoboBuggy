#!/usr/bin/env/python
import rospy
from robobuggy.msg import GPS

def gps_callback(data):
    rospy.log("easting: %f",data.easting);
    rospy.log("northing: %f",data.northing);
    rospy.log("Latitude degrees: %f",data.Lat_deg);
    rospy.log("Longitude degrees: %f",data.Long_deg);

def gps_main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', GPS, gps_callback)
    rospy.spin()
    
if __name__ == '__main__':
    gps_main()
