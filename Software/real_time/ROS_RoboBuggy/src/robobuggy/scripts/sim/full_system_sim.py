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

class Simulator:

    def __init__(self, sigma_gps, sigma_odom, start_position):
        self.sigma_gps = sigma_gps
        self.sigma_odom = sigma_odom
        start_utm = utm.from_latlon(start_position[0], start_position[1])

        self.steering_angle = 0
        self.dt = 0.1
        self.sim_update_hz = 1/self.dt
        self.wheelbase = 1.13
        self.gps_update_rate_hz = 2
        self.odom_update_rate_hz = 5
        self.imu_update_rate_hz = 5
        self.utm_letter = 'T'
        self.utm_zone = 17
        self.velocity = 3

        x = [
            start_utm[0],
            start_utm[1],
            self.velocity,
            start_position[2],
            0
        ]
        self.x = np.array(x)
        self.x.reshape([5,1])

    def apply_control_input(self, data):
        self.steering_angle = data.steer_cmd_rad

    def calculate_new_motion_model(self):
        A = [
            [1, 0, self.dt * math.cos(self.x[3]), 0, 0],
            [0, 1, self.dt * math.sin(self.x[3]), 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, self.dt],
            [0, 0, math.tan(self.steering_angle)/self.wheelbase, 0, 0]
        ]
        return np.array(A)

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
        noisy_msg.latitude = lat
        noisy_msg.longitude = lon
        noisy_msg.easting = x
        noisy_msg.northing = y
        
        return noisy_msg
    
    def generate_odometry_message(self):
        approx_distance = self.velocity * self.dt * self.sim_update_hz / self.odom_update_rate_hz
        approx_distance = np.random.normal(approx_distance, self.sigma_odom)

        noisy_msg = Encoder()
        noisy_msg.distance = approx_distance

        return noisy_msg
    
    def generate_ground_truth_message(self):
        (x, y) = (self.x[0], self.x[1])
        (lat, lon) = utm.to_latlon(x, y, self.utm_zone, self.utm_letter)

        msg = GPS()
        msg.latitude = lat
        msg.longitude = lon
        msg.easting = x
        msg.northing = y
        
        return msg

    def generate_imu_message(self):
        # TODO
        pass

def main():

    # init
    rospy.init_node("Full_System_Simulator")
    gps_pub = rospy.Publisher('GPS', GPS, queue_size=10)
    odom_pub = rospy.Publisher('Encoder', Encoder, queue_size=10)

    # 5m stddev
    sigma_gps = 5

    # 0.1m stddev
    sigma_odom = 0.1

    # read in waypoints from waypoint file, figure out first one
    start_x = 0
    start_y = 0
    start_th = 0

    s = Simulator(sigma_gps, sigma_odom, (start_x, start_y, start_th))
    command_sub = rospy.Publisher('Command', Command, s.command_callback)

    pass

if __name__ == "__main__":
    main()