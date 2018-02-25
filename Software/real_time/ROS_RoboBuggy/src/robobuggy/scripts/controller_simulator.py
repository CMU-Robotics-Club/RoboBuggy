#!/usr/bin/env python

import rospy
import math
from gps_common.msg import GPSFix
from robobuggy.msg import Pose
import numpy as np
import utm 

global simulator_pub
global x 
global model 
global sim_period = 10
global dt = 1/sim_period
global steering = 0
global wheelbase = 1.13
global rate = rospy.Rate(10)
#subject to change in the future
global init_latitude = 40.4406
global init_longitude = -79.9959

def updateModel(x):
    matrix = 
    [
        [1, 0, dt* math.cos(x[3][0]), 0, 0],
        [0, 1, dt* math.sin(x[3][0]), 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, dt],
        [0, 0, math.tan(steering)/wheelbase, 0, 0]
    ]
    return matrix

#need to implement conversion with UTM library



def command_callback(data):
    
    #steer_cmd should be radians
    rospy.loginfo("got Steering Command msg: %f degrees", data.steer_cmd)


    #data.steer_cmd is the information we get
    #this method will have to modify the model
    steering = data.steer_cmd
    model = updateModel(x)

    pass

def initialize():
    #set up the initial position
    x = [
            [], 
            [northing], 
            [1], 
            [250 * math.pi / 180], 
            [0]
        ]
    model = np.identity(5)

    
def start_simulator_spin():

    rospy.init_node("GPS_Plotter", anonymous=True)
    simulator_pub = rospy.Publisher('simulated_Pose', Pose, queue_size=10)
    command_listener = rospy.Subscriber("Command", Command, command_callback)

    initialize()

    while not rospy.is_shutdown():
        #apply the model
        #use np.dot(a, b) for matrix product of two 2d-lists a and b
        x = np.dot(model, x)

        #publish a new pose message using simulator_pub
        simulated_pose = Pose()

        #This needs some conversion fixes
        simulated_pose.latitude_deg = northingToLatitude(x[1][0])
        simulated_pose.longitude_deg = eastingToLongitude(x[0[0]])

        simulator_pub.publish(simulated_pose)

        rospy.spinOnce()

        rate.sleep()

if __name__ == '__main__':
    try:
        start_simulator_spin()
    except rospy.ROSInterruptException:
        pass

