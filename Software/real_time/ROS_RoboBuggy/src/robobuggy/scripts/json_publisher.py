#!/usr/bin/env python

import rospy
import json
from robobuggy.msg import ENC, GPS, Diagnostics, Command, Feedback
import time
import requests

diag_json_dict = { "batteryLevel" : -1 }
enc_json_dict = { "ticks" : -1 }
gps_json_dict = { "latitude" : -1, "longitude" : -1 }
data_dict = { "batteryLevel" : -1, "autonState" : False, "diagnosticsError" : -1, "ticks" : -1, "latitude" : -1, "longitude" : -1, "brakeState" : False, "brakeCmdTeleop" : False, "brakeCmdAuton" : False }


def subscriber_callback_Diagnostics(data):
    global diag_json_dict, data_dict
    battery_level = data.battery_level
    diag_json_dict = {"batteryLevel" : battery_level}
    data_dict["autonState"] = data.auton_state
    data_dict["diagnosticsError"] = data.error
    data_dict["batteryLevel"] = battery_level
    pass
def subscriber_callback_ENC(data):
    global enc_json_dict, data_dict
    tick = data.ticks
    enc_json_dict = {"ticks" : tick}
    data_dict["ticks"] = tick
    pass

def subscriber_callback_GPS(data):
    global gps_json_dict, data_dict
    gps_json_dict = {"latitude" : data.Lat_deg, "longitude" : data.Long_deg}
    data_dict["latitude"] = data.Lat_deg
    data_dict["longitude"] = data.Long_deg
    pass

def subscriber_callback_Command(data):
    global data_dict
    data_dict["brakeCmdAuton"] = data.brake_cmd
    data_dict["steerCmd"] = data.steer_cmd
    pass

def subscriber_callback_Feedback(data):
    global data_dict
    data_dict["brakeState"] = data.brake_state
    data_dict["steerAngle"] = data.steer_angle
    pass

def start_subscriber_spin():
    rospy.init_node("JSON_Publisher", anonymous=True)
    rospy.Subscriber("Encoder", ENC, subscriber_callback_ENC)
    rospy.Subscriber("GPS", GPS, subscriber_callback_GPS)
    rospy.Subscriber("Diagnostics", Diagnostics, subscriber_callback_Diagnostics)
    rospy.Subscriber("Command", Command, subscriber_callback_Command)
    rospy.Subscriber("Feedback", Feedback, subscriber_callback_Feedback)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        request = requests.post('https://robobuggy-web-server.herokuapp.com/storeData', json = data_dict)
        rate.sleep()


    print("Success!")
    pass

if __name__ == "__main__":
    start_subscriber_spin()
