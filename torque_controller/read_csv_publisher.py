#!/usr/bin/env python

import rospy
import csv
import pandas as pd
import Tkinter as tk
import tkFileDialog as filedialog
from geometry_msgs.msg import Twist

SAMPLING_RATE = 10

rospy.init_node('read_csv_publisher')
pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()
# print(file_path)

data_df = pd.read_excel(file_path)
# print(data_df["Chair_LinVel"][0])


rate = rospy.Rate(SAMPLING_RATE)
for index, row in data_df.iterrows():
    vel_msg = Twist()
    vel_msg.linear.x = row["Chair_LinVel"]
    vel_msg.linear.y= 0
    vel_msg.linear.z= 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z =  row["Chair_AngVel"]
    # print(vel_msg)
    pub.publish(vel_msg)
    rate.sleep()
