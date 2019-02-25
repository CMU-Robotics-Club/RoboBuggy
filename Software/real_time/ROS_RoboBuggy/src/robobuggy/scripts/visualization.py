#!/usr/bin/env python
import rospy
from robobuggy.msg import Command
from robobuggy.msg import Diagnostics
from robobuggy.msg import Feedback
from robobuggy.msg import Pose
from Tkinter import *

# Get brake state, commanded steering, feedback steering, current orientation,
# battery level and visualize them on a user interfate

def commandUpdate(data):
    steer_cmd_rad = "%10.4d" % data.steer_cmd_rad
    canvas.itemconfigure(t1, text=steer_cmd_rad)

def diagnosticsUpdate(data):
    battery_level = "%d" % data.battery_level
    canvas.itemconfigure(t2, text=battery_level)

def feedbackUpdate(data):
    if data.brake_state==True:
        canvas.itemconfigure(t3, text="Deployed")
    else:
        canvas.itemconfigure(t3, text="Suspended")

def poseUpdate(data):
    heading_rad = "%10.4f" % data.heading_rad
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
    root.resizable(width=False, height=False)
    canvas = Canvas(root, width=500, height=300)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    canvas.create_text(20, 45, text="Commanded Steering:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 115, text="Battery Level:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 185, text="Brake State:", anchor="w", fill="black", font="Times 20 bold")
    canvas.create_text(20, 255, text="Current Orientation:", anchor="w", fill="black", font="Times 20 bold")
    t1 = canvas.create_text(250, 45, text="Fetching data...", anchor="w", fill="black", font="Times 16")
    t2 = canvas.create_text(250, 115, text="Fetching data...", anchor="w", fill="black", font="Times 16")
    t3 = canvas.create_text(250, 185, text="Fetching data...", anchor="w", fill="black", font="Times 16")
    t4 = canvas.create_text(250, 255, text="Fetching data...", anchor="w", fill="black", font="Times 16")

    listener()
    root.mainloop()
