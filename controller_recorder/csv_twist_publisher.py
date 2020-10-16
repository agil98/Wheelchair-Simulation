#!/usr/bin/env python

import rospy
import csv
import pandas as pd
import Tkinter as tk
import tkFileDialog as filedialog
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


SAMPLING_RATE = 240
SCALING = 1


rospy.init_node('csv_twist_publisher')
twist_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)

finished = rospy.Publisher("finished_sim", Bool, queue_size = None)
started = rospy.Publisher("started_sim", Bool, queue_size = None)

# Reset simulation
# rospy.wait_for_service('/gazebo/reset_simulation')
# reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
# reset_simulation()

# Stop Wheelchair
twist_pub.publish(Twist())

# Choose file
root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()
# print(file_path)
rospy.set_param('command_csv', file_path)

data_df = pd.read_csv(file_path)

# SAMPLING_RATE = 1/(data_df["Time"][1] - data_df["Time"][0])

# Wait until all publishers have at least one subscriber
while twist_pub.get_num_connections() == 0 or started.get_num_connections() == 0:
    rospy.sleep(1)

# Start signal
started.publish(1)

rate = rospy.Rate(SAMPLING_RATE)
for index, row in data_df.iterrows():
    twist_cmd = Twist()
    twist_cmd.linear.x = row["Chair_LinVel"]/SCALING
    twist_cmd.angular.z =row["Chair_AngVel"]/SCALING
    # print(vel_msg)
    twist_pub.publish(twist_cmd)
    rate.sleep()

# Stop Wheelchair
twist_pub.publish(Twist())

# End signal
finished.publish(1)

