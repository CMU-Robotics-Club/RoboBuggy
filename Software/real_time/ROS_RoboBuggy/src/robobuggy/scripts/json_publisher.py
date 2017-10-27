#!/usr/bin/env python

import rospy
import json
from robobuggy.msg import ENC
import time
import requests

def subscriber_callback(data):
    tick = data.ticks
    json_dict = {"ticks" : tick}
    request = requests.post('https://robobuggy-web-server.herokuapp.com/encoderData', json = json_dict)
    pass

def start_subscriber_spin():
    rospy.init_node("JSON_Publisher", anonymous=True)
    rospy.Subscriber("Encoder", ENC, subscriber_callback)
    print("Success!")
    rospy.spin()
    pass

if __name__ == "__main__":
    start_subscriber_spin()
