#!/usr/bin/env python

import rospy
import csv
import pandas as pd
import Tkinter as tk
import tkFileDialog as filedialog
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty


SAMPLING_RATE = 240
SCALING = 1


rospy.init_node('csv_ang_vel_publisher')
pub_left = rospy.Publisher("WheelL_velocity_controller/command", Float64, queue_size = 10)
pub_right = rospy.Publisher("WheelR_velocity_controller/command", Float64, queue_size = 10)

finished = rospy.Publisher("finished_sim", Bool, queue_size = None)
started = rospy.Publisher("started_sim", Bool, queue_size = None)

# Reset simulation
# rospy.wait_for_service('/gazebo/reset_simulation')
# reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
# reset_simulation()

# Stop Wheelchair
pub_left.publish(0)
pub_right.publish(0)

# Choose file
root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()
# print(file_path)
rospy.set_param('command_csv', file_path)

data_df = pd.read_csv(file_path)

# SAMPLING_RATE = 1/(data_df["Time"][1] - data_df["Time"][0])

# Wait until all publishers have at least one subscriber
while pub_left.get_num_connections() == 0 or pub_right.get_num_connections() ==0 or started.get_num_connections() == 0:
    rospy.sleep(1)

# Start signal
started.publish(1)

rate = rospy.Rate(SAMPLING_RATE)
for index, row in data_df.iterrows():
    left = row["AngVel_L"]/SCALING
    right = row["AngVel_R"]/SCALING
    # print(vel_msg)
    pub_right.publish(right)
    pub_left.publish(left)
    rate.sleep()

# Stop Wheelchair
pub_left.publish(0)
pub_right.publish(0)

# End signal
finished.publish(1)

