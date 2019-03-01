#!/usr/bin/env python
import rospy
from robobuggy.msg import Command
from robobuggy.msg import Diagnostics
from robobuggy.msg import Feedback
from robobuggy.msg import Pose
from Tkinter import *
import math

# Get brake state, commanded steering, feedback steering, current orientation,
# battery level and visualize them on a user interfate

def commandUpdate(data):
    if abs(data.steer_cmd_rad) < 0.0001:
        steer_cmd_rad = "Straight"
    elif data.steer_cmd_rad>0:
        steer_cmd_rad = "%10.4f rad left" % data.steer_cmd_rad
    else:
        steer_cmd_rad = "%10.4f rad right" % (data.steer_cmd_rad * -1)
    canvas.itemconfigure(t1, text=steer_cmd_rad)

def diagnosticsUpdate(data):
    voltage = data.battery_level / 1000.00
    battery_level = "%10.3f V" % voltage
    canvas.itemconfigure(t2, text=battery_level)

def feedbackUpdate(data):
    if data.brake_state==True:
        canvas.itemconfigure(t3, text="Deployed")
    else:
        canvas.itemconfigure(t3, text="Suspended")

def poseUpdate(data):
    heading_rad = "%10.4f heading " % data.heading_rad
    if abs(data.heading_rad) < 0.0001:
        heading_rad += "E"
    elif abs(data.heading_rad - math.pi / 2) < 0.0001:
        heading_rad += "N"
    elif abs(data.heading_rad + math.pi / 2) < 0.0001:
        heading_rad += "S"
    elif abs(abs(data.heading_rad) - math.pi) < 0.0001:
        heading_rad += "W"
    elif 0 < data.heading_rad and data.heading_rad < math.pi / 2:
        heading_rad += "NE"
    elif math.pi / 2 < data.heading_rad and data.heading_rad < math.pi:
        heading_rad += "NW"
    elif -math.pi / 2 < data.heading_rad and data.heading_rad < 0:
        heading_rad += "SE"
    else:
        heading_rad += "SW"
    canvas.itemconfigure(t4, text=heading_rad)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("Command", Command, commandUpdate)
    rospy.Subscriber("Diagnostics", Diagnostics, diagnosticsUpdate)
    rospy.Subscriber("Feedback", Feedback, feedbackUpdate)
    rospy.Subscriber("Pose", Pose, poseUpdate)

if __name__ == '__main__':

    # Initialize GUI
    root = Tk()
    root.title("Robobuggy Status")
    root.resizable(width=True, height=True)
    canvas = Canvas(root, width=500, height=300)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    canvas.create_text(20, 45, text="Commanded Steering:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 115, text="Battery Level:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 185, text="Brake State:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 255, text="Current Orientation:", anchor="w", fill="black", font="Times 20 bold")
    t1 = canvas.create_text(480, 45, text="Fetching data...", anchor="e", fill="black", font="Times 16")
    t2 = canvas.create_text(480, 115, text="Fetching data...", anchor="e", fill="black", font="Times 16")
    t3 = canvas.create_text(480, 185, text="Fetching data...", anchor="e", fill="black", font="Times 16")
    t4 = canvas.create_text(480, 255, text="Fetching data...", anchor="e", fill="black", font="Times 16")

    listener()
    root.mainloop()
