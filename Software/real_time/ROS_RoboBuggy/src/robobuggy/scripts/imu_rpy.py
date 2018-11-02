#!/usr/bin/env python

import rospy
from robobuggy.msg import IMU
import math

def imu_callback(data):
    q0 = data.A_AngPos;
    q1 = data.B_AngPos;
    q2 = data.C_AngPos;
    q3 = data.D_AngPos;

    phi = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2)));
    theta = math.degrees(math.asin(2*(q0*q2 - q3*q1)));
    psi = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2+q3*q3)));

    rospy.loginfo("phi=%f\ttheta=%f\tpsi=%f", phi, theta, psi);
    pass

def main():
    rospy.init_node("IMU_Roll_Pitch_Yaw");
    rospy.Subscriber("IMU", IMU, imu_callback);
    rospy.spin();

if __name__ == "__main__":
    main();