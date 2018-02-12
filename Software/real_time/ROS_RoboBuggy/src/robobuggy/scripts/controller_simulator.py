#!/usr/bin/env python

import rospy
import math
from gps_common.msg import GPSFix
from robobuggy.msg import GPS
from robobuggy.msg import Pose
import numpy as np

global simulator_pub
global x 
global model 
    

def command_callback(data):
    
    #steer_cmd should be degrees
    rospy.loginfo("got Steering Command msg: %f degrees", data.steer_cmd)

    #data.steer_cmd is the information we get
    #this method will have to modify the model
    pass

def initialize():
    #set up the initial position
    x = [[
    model = 

    
def start_simulator_spin():

    rospy.init_node("GPS_Plotter", anonymous=True)
    simulator_pub = rospy.Publisher('simulated_Pose', Pose, queue_size=10);
    rospy.Subscriber("Command", Command, command_callback)

    initialize()
    
    while (true):
        #apply the model
        #use np.dot(a, b) for matrix product of two 2d-lists a and b
        x = np.dot(model, x)
        
        #publish a new pose message using simulator_pub
        simulated_pose = 
        simulator_pub.publish(simulated_Pose)

   
    rospy.spin()

    pass

if __name__ == "__main__":
    start_simulator_spin()

