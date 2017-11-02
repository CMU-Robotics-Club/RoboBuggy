#!/usr/bin/env python

import rospy
import json
from robobuggy.msg import ENC, GPS
import time
import requests

def subscriber_callback_ENC(data):
    tick = data.ticks
    json_dict = {"ticks" : tick}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/encoderData', json = json_dict)
    print("Published to Encoder")
    pass

def subscriber_callback_GPS(data):
    json_dict = {"latitude" : data.Lat_deg, "longitude" : data.Lon_deg}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/gpsData', json = json_dict)
    print("Published to GPS")
    pass

def start_subscriber_spin():
    rospy.init_node("JSON_Publisher", anonymous=True)
    rospy.Subscriber("Encoder", ENC, subscriber_callback_ENC)
    rospy.Subscriber("GPS", GPS, subscriber_callback_GPS)
    print("Success!")
    rospy.spin()
    pass

if __name__ == "__main__":
    start_subscriber_spin()
