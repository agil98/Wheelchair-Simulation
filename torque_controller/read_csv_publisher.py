#!/usr/bin/env python

import rospy
import csv
import pandas as pd
import Tkinter as tk
import tkFileDialog as filedialog
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty


SAMPLING_RATE = 360
SCALING = 1

rospy.init_node('read_csv_publisher')
pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
finished = rospy.Publisher("finished_sim", Bool, queue_size = 1, latch = True)

# Reset simulation
# rospy.wait_for_service('/gazebo/reset_simulation')
# reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
# reset_simulation()

finished.publish(0)

root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()
# print(file_path)

data_df = pd.read_csv(file_path)
# print(data_df["Chair_LinVel"][0])

rate = rospy.Rate(SAMPLING_RATE)
for index, row in data_df.iterrows():
    vel_msg = Twist()
    vel_msg.linear.x = row["Chair_LinVel"]/SCALING
    vel_msg.linear.y= 0
    vel_msg.linear.z= 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z =  row["Chair_AngVel"]/SCALING
    # print(vel_msg)
    pub.publish(vel_msg)
    rate.sleep()

# Stop Wheelchair
pub.publish(Twist())
finished.publish(1)

