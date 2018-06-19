#!/usr/bin/env python

# Full System Simulator
# Keeps track of a ground-truth position for the robot, but reports a noisy version to the rest of the system

# TODO Currently reports only GPS and encoder, include IMU as well
import rospy
import numpy as np
import utm
import math
from robobuggy.msg import GPS
from robobuggy.msg import Encoder
from robobuggy.msg import Command
from robobuggy.msg import Pose
import json
import pdb

class Simulator:

    def __init__(self, sigma_gps, sigma_odom, start_position):
        self.sigma_gps = sigma_gps
        self.sigma_odom = sigma_odom
        self.steering_angle = 0
        self.sim_update_hz = 10
        self.dt = 1.0 / self.sim_update_hz
        self.wheelbase = 1.13
        self.gps_update_rate_hz = 2
        self.odom_update_rate_hz = 5
        self.imu_update_rate_hz = 5
        self.ground_truth_update_rate_hz = 5
        self.utm_letter = 'T'
        self.utm_zone = 17
        self.velocity = 3

        x = [
            start_position[0],
            start_position[1],
            self.velocity,
            start_position[2],
            0
        ]
        self.x = np.array(x)
        self.x = np.reshape(x, [5,1])

    def apply_control_input(self, data):
        self.steering_angle = data.steer_cmd_rad

    def calculate_new_motion_model(self):
        A = [
            [1, 0, self.dt * np.cos(self.x[3]), 0, 0],
            [0, 1, self.dt * np.sin(self.x[3]), 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, self.dt],
            [0, 0, math.tan(self.steering_angle)/self.wheelbase, 0, 0]
        ]
        return np.matrix(A)

    def step(self):
        A = self.calculate_new_motion_model()
        self.x = A * self.x
        pass

    def generate_gps_message(self):
        (x, y) = (self.x[0], self.x[1])
        x = np.random.normal(x, self.sigma_gps)
        y = np.random.normal(y, self.sigma_gps)

        (lat, lon) = utm.to_latlon(x, y, self.utm_zone, self.utm_letter)

        noisy_msg = GPS()
        noisy_msg.Lat_deg = lat
        noisy_msg.Long_deg = lon
        noisy_msg.easting = x
        noisy_msg.northing = y
        
        return noisy_msg
    
    def generate_odometry_message(self):
        approx_distance = self.velocity * self.dt * self.sim_update_hz / self.odom_update_rate_hz
        approx_distance = np.random.normal(approx_distance, self.sigma_odom)

        noisy_msg = Encoder()
        noisy_msg.ticks = approx_distance * 1.0 / (0.61 / 7.0 * 0.3048 * 2)

        return noisy_msg
    
    def generate_ground_truth_message(self):
        (x, y) = (self.x[0], self.x[1])
        (lat, lon) = utm.to_latlon(x, y, self.utm_zone, self.utm_letter)

        msg = Pose()
        msg.latitude_deg = lat
        msg.longitude_deg = lon
        msg.heading_rad = self.x[3]
        
        return msg

    def generate_imu_message(self):
        # TODO
        pass

def main():

    # init
    NODE_NAME = "Full_System_Simulator"
    rospy.init_node(NODE_NAME)
    gps_pub = rospy.Publisher('GPS', GPS, queue_size=10)
    odom_pub = rospy.Publisher('Encoder', Encoder, queue_size=10)
    ground_truth_pub = rospy.Publisher("SIM_Ground_Truth", Pose, queue_size=10)
    # TODO
    # imu_pub = rospy.Publisher('IMU', IMU, queue_size=10)

    # 5m stddev
    sigma_gps = 2

    # 0.1m stddev
    sigma_odom = 0.1

    start_x = 0
    start_y = 0
    start_th = math.radians(250) # TODO calculate based on two waypoints
    # waypoint_file = rospy.get_param("/{}/waypoint_file".format(NODE_NAME))
    waypoint_file = "/mnt/c/Users/bhai/Documents/RoboBuggy/Software/real_time/ROS_RoboBuggy/src/robobuggy/config/waypoints.txt"
    with open(waypoint_file) as f:
        first_waypoint_str = f.readline()
        first_waypoint_json = json.loads(first_waypoint_str)
        first_waypoint = utm.from_latlon(first_waypoint_json['latitude'], first_waypoint_json['longitude'])
        start_x = first_waypoint[0]
        start_y = first_waypoint[1]

    s = Simulator(sigma_gps, sigma_odom, (start_x, start_y, start_th))
    command_sub = rospy.Subscriber('Command', Command, s.apply_control_input)

    # spin infinitely, while stepping each appropriate tick
    rate = rospy.Rate(s.sim_update_hz)
    loop_counter = 0
    while not rospy.is_shutdown():
        print loop_counter

        # Take a step
        s.step()

        # If needed, generate and publish messages
        if loop_counter % (s.sim_update_hz / s.gps_update_rate_hz) == 0:
            gps_msg = s.generate_gps_message()
            gps_pub.publish(gps_msg)

        if loop_counter % (s.sim_update_hz / s.odom_update_rate_hz) == 0:
            odom_msg = s.generate_odometry_message()
            odom_pub.publish(odom_msg)
        
        if loop_counter % (s.sim_update_hz / s.ground_truth_update_rate_hz) == 0:
            gt_msg = s.generate_ground_truth_message()
            ground_truth_pub.publish(gt_msg)

        # TODO
        # if loop_counter % s.imu_update_rate_hz:
        #     imu_msg = s.generate_imu_message()
        #     imu_pub.publish(imu_msg)

        loop_counter += 1
        rate.sleep()


        


if __name__ == "__main__":
    main()