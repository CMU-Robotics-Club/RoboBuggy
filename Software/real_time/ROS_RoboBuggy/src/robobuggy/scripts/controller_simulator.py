#!/usr/bin/env python

import rospy
import math
from gps_common.msg import GPSFix
from robobuggy.msg import Pose
from robobuggy.msg import Command
import numpy as np
import utm 

class ControllerTester:

    def __init__(self):
        #set up the initial position
        #Syntax see python package index
        #The syntax is utm.from_latlon(LATITUDE, LONGITUDE).
        #The return has the form (EASTING, NORTHING, ZONE NUMBER, ZONE LETTER).

        self.sim_period = 10.0
        self.dt = 1.0/self.sim_period
        self.steering = 0
        self.wheelbase = 1.13
        self.rate = rospy.Rate(10)
    #subject to change in the future
        self.utm_zone_number = 17
        self.utm_zone_letter = 'T'
        self.init_latitude = 40.441687
        self.init_longitude = -79.941594
        initial_pos_utm = utm.from_latlon(self.init_latitude, self.init_longitude)
        self.x = \
            [
                #UTM Easting
                [initial_pos_utm[0]], 
                #UTM Northing
                [initial_pos_utm[1]], 
                [3], 
                [250 * math.pi / 180], 
                [0]
            ]
        self.model = self.updateModel(self.x)

    def updateModel(self,x):
        matrix = \
        [
            [1, 0, self.dt * math.cos(x[3][0]), 0, 0],
            [0, 1, self.dt * math.sin(x[3][0]), 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, self.dt],
            [0, 0, math.tan(self.steering)/self.wheelbase, 0, 0]
        ]
        return matrix

    def command_callback(data):
        
        #steer_cmd should be radians
        rospy.loginfo("got Steering Command msg: %f degrees", data.steer_cmd)


        #data.steer_cmd is the information we get
        #this method will have to modify the model
        self.steering = data.steer_cmd
        self.model = updateModel(self.x)

        pass

    
def start_simulator_spin():

    rospy.init_node("Controller_Tester", anonymous=True)
    simulator_pub = rospy.Publisher('Pose', Pose, queue_size=10)
    c_tester = ControllerTester()
    command_listener = rospy.Subscriber("Command", Command, c_tester.command_callback)

    #initialize()


    while not rospy.is_shutdown():
        #apply the model
        #use np.dot(a, b) for matrix product of two 2d-lists a and b
        #print(model)
        c_tester.x = np.dot(c_tester.model, c_tester.x)

        #publish a new pose message using simulator_pub
        simulated_pose = Pose()

        #lat_long is a 2-tuple
        #for syntax of the utm package see python package index
        #The syntax is utm.to_latlon(EASTING, NORTHING, ZONE NUMBER, ZONE LETTER).
        #The return has the form (LATITUDE, LONGITUDE).
        
        lat_long = utm.to_latlon(c_tester.x[0][0], c_tester.x[1][0], c_tester.utm_zone_number, c_tester.utm_zone_letter)
        simulated_pose.latitude_deg = lat_long[0]
        simulated_pose.longitude_deg = lat_long[1]
        simulated_pose.heading_rad = c_tester.x[3][0]

        simulator_pub.publish(simulated_pose)
        c_tester.updateModel(c_tester.x)
        print(c_tester.x[2][0])
        #rospy.spin()

        c_tester.rate.sleep()

if __name__ == '__main__':
    try:
        start_simulator_spin()
    except rospy.ROSInterruptException:
        pass

