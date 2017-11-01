#!/usr/bin/env python

import rospy
import json
from robobuggy.msg import ENC, GPS, Diagnostics
import time
import requests
def subscriber_callback_Diagnostics(data):
    battery_level = data.battery_level
    json_dict = {"batteryLevel" : battery_level}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/diagnosticsData', json = json_dict)
    print("Published to Diagnostics")
    pass
def subscriber_callback_ENC(data):
    tick = data.ticks
    json_dict = {"ticks" : tick}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/encoderData', json = json_dict)
    print("Published to Encoder")
    pass

def subscriber_callback_GPS(data):
    json_dict = {"latitude" : data.Lat_deg, "longitude" : data.Long_deg}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/gpsData', json = json_dict)
    print("Published to GPS")
    pass

def start_subscriber_spin():
    rospy.init_node("JSON_Publisher", anonymous=True)
    rospy.Subscriber("Encoder", ENC, subscriber_callback_ENC)
    rospy.Subscriber("GPS", GPS, subscriber_callback_GPS)
    rospy.Subscriber("Diagnostics", Diagnostics, subscriber_callback_Diagnostics)
    print("Success!")
    rospy.spin()
    pass

if __name__ == "__main__":
    start_subscriber_spin()
