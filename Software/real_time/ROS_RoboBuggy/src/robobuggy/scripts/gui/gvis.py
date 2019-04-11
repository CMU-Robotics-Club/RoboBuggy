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
    steer_cmd_deg = math.degrees(data.steer_cmd_rad)
    if abs(steer_cmd_deg) < 0.1:
        steer_cmd = "Straight"
    elif steer_cmd_deg > 0:
        steer_cmd = "%10.1f deg left" % steer_cmd_deg
    else:
        steer_cmd = "%10.1f deg right" % (steer_cmd_deg * -1)
    t0.set(steer_cmd + "   ")

def diagnosticsUpdate(data):
    voltage = data.battery_level / 1000.00
    battery_level = "%10.3f V" % voltage
    t1.set(battery_level + "   ")

def feedbackUpdate(data):
    if data.brake_state==True:
        t2.set("Deployed   ")
    else:
        t2.set("Suspended   ")

def poseUpdate(data):
    heading_deg = math.degrees(data.heading_rad)
    heading = "%10.1f deg heading " % heading_deg
    if abs(heading_deg) < 0.1:
        heading += "E"
    elif abs(heading_deg - 90) < 0.1:
        heading += "N"
    elif abs(heading_deg + 90) < 0.1:
        heading += "S"
    elif abs(abs(heading_deg) - 180) < 0.1:
        heading += "W"
    elif 0 < heading_deg and heading_deg < 90:
        heading += "NE"
    elif 90 < heading_deg and heading_deg < 180:
        heading += "NW"
    elif -90 < heading_deg and heading_deg < 0:
        heading += "SE"
    else:
        heading += "SW"
    t3.set(heading + "   ")

def rotate(points, angle, center):
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    new_points = []
    for x_old, y_old in points:
        x_old -= center
        y_old -= center
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append([x_new + center, y_new + center])
    return new_points

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("Command", Command, commandUpdate)
    rospy.Subscriber("Diagnostics", Diagnostics, diagnosticsUpdate)
    rospy.Subscriber("Feedback", Feedback, feedbackUpdate)
    rospy.Subscriber("Pose", Pose, poseUpdate)

if __name__ == '__main__':

    root = Tk()
    root.title("Robobuggy Status")
    root.resizable(width=True, height=True)

    l0 = Label(root, text = " Commanded Steering: ", font = ("Times", 20, "bold")).grid(row = 0, column = 0, sticky = W)
    l1 = Label(root, text = " Battery Level: ", font = ("Times", 20, "bold")).grid(row = 1, column = 0, sticky = W)
    l2 = Label(root, text = " Brake State: ", font = ("Times", 20, "bold")).grid(row = 2, column = 0, sticky = W)
    l3 = Label(root, text = " Current Orientation: ", font = ("Times", 20, "bold")).grid(row =3, column = 0, sticky = W)

    t0 = StringVar()
    t1 = StringVar()
    t2 = StringVar()
    t3 = StringVar()
    t0.set("Fetching data...   ")
    t1.set("Fetching data...   ")
    t2.set("Fetching data...   ")
    t3.set("Fetching data...   ")

    w0 = Label(root, textvariable = t0, font = ("Times", 16)).grid(row = 0, column = 1, sticky = E)
    w1 = Label(root, textvariable = t1, font = ("Times", 16)).grid(row = 1, column = 1, sticky = E)
    w2 = Label(root, textvariable = t2, font = ("Times", 16)).grid(row = 2, column = 1, sticky = E)
    w3 = Label(root, textvariable = t3, font = ("Times", 16)).grid(row = 3, column = 1, sticky = E)

    c = Canvas(root, bg = "white", width = 300, height = 300).grid(row = 0, rowspan = 4, column = 2)

    root.grid_columnconfigure(0, weight = 1)
    root.grid_columnconfigure(1, weight = 1)
    root.grid_columnconfigure(2, weight = 2)
    root.grid_rowconfigure(0, weight = 1)
    root.grid_rowconfigure(1, weight = 1)
    root.grid_rowconfigure(2, weight = 1)
    root.grid_rowconfigure(3, weight = 1)

    listener()

    root.mainloop()
