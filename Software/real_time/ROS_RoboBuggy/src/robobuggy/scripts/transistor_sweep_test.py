#!/usr/bin/env python

import rospy
from robobuggy.msg import Command

def sweep_steering():
    pub = rospy.Publisher('Command', Command, queue_size=1000)
    rospy.init_node('steering_sweep', anonymous=True)
    rate = rospy.Rate(20)

    steering_pos = -1500
    steering_change = 100
    command_msg = Command()
    while not rospy.is_shutdown():
        command_msg.steer_cmd = steering_pos
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
