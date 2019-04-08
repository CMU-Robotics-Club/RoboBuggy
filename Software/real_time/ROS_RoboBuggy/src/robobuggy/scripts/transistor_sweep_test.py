#!/usr/bin/env python

import rospy
from robobuggy.msg import Command
import math

def sweep_steering():
    pub = rospy.Publisher('Command', Command, queue_size=1000)
    rospy.init_node('steering_sweep', anonymous=True)
    rate = rospy.Rate(4)

    steering_pos = -1500
    steering_change = 100
    command_msg = Command()
    while not rospy.is_shutdown():
        command_msg.steer_cmd_rad = math.radians(steering_pos/100);
        command_msg.brake_cmd = False
        pub.publish(command_msg)
        steering_pos += steering_change
        if (steering_pos == 1500):
            steering_change = -100
        if (steering_pos == -1500):
            steering_change = 100

        rate.sleep()

if __name__ == '__main__':
    try:
        sweep_steering()
    except rospy.ROSInterruptException:
        pass
