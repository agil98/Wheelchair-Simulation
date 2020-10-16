#!/usr/bin/env python

import rospy
import rosbag
import csv
import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import Bool
from scipy.signal import butter, sosfilt


rospy.init_node('get_twist_subscriber')  

SAMPLING_RATE = 240
SCALING = 1
WHEEL_DIAMETER = 0.3045*2
LEFT_BAG_SIM = 'left.bag'
RIGHT_BAG_SIM = 'right.bag'

file_name = rospy.get_param('get_twist_subscriber/file_name')
csv_dir = rospy.get_param('get_twist_subscriber/csv_save_location')
plot_dir = rospy.get_param('get_twist_subscriber/plot_save_location')
if not os.path.exists(csv_dir):
    os.makedirs(csv_dir)
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

csv_file_path = os.path.join(csv_dir, file_name + ".csv")
plot_file_path = os.path.join(plot_dir, file_name + ".png")

bag_left = rosbag.Bag(LEFT_BAG_SIM, 'w')
bag_right = rosbag.Bag(RIGHT_BAG_SIM, 'w')

started = False

def ft_left_callback(msg):
    if started:
        bag_left.write('ft_sensor_left', msg)

def ft_right_callback(msg):
    if started:
        bag_right.write("ft_sensor_right", msg)

def start_callback(msg):
    global started
    started = True
    print("Started")

def print_callback(msg):
    print(msg)


# sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
sub_ft_left = rospy.Subscriber("ft_sensor_topic/left", WrenchStamped, ft_left_callback)
sub_ft_right = rospy.Subscriber("ft_sensor_topic/right", WrenchStamped, ft_right_callback)
sub_start_signal = rospy.Subscriber("started_sim", Bool, start_callback)

# Wait for start
print("Waiting for start...")

# start_time = rospy.Time.now().to_sec()

# Wait until finished
finished_msg = rospy.wait_for_message("finished_sim", Bool)
print("Finished")

sub_ft_left.unregister()
sub_ft_right.unregister()
bag_left.close()
bag_right.close()

# Read from rosbag

print("Reading from rosbag...")
bag_left = rosbag.Bag(LEFT_BAG_SIM)
bag_right = rosbag.Bag(RIGHT_BAG_SIM)

left_time_sim = []
left_torque_sim = []
right_time_sim = []
right_torque_sim = []


# bag.reindex()
for topic, msg, t in bag_right.read_messages(topics=['ft_sensor_right']):
    right_time_sim.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
    right_torque_sim.append(msg.wrench.torque.y)
for topic, msg, t in bag_left.read_messages(topics=['ft_sensor_left']):
    left_time_sim.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
    left_torque_sim.append(msg.wrench.torque.y)

# Get command csv
command_df = pd.read_csv(rospy.get_param('command_csv'))
command_time = np.transpose(command_df["Time"])
command_lin_vel = np.transpose(command_df["Chair_LinVel"])
command_ang_vel = np.transpose(command_df["Chair_AngVel"])

# Make left_time, right_time start from 0
left_time_sim = [x-left_time_sim[0] for x in left_time_sim]
right_time_sim = [x-right_time_sim[0] for x in right_time_sim]

columns = ["left_time_sim", "left_torque_sim", "right_time_sim", "right_torque_sim", "command_time", "command_lin_vel", "command_ang_vel"]
data = [left_time_sim, left_torque_sim, right_time_sim, right_torque_sim, 
    command_time, command_lin_vel, command_ang_vel]


# # Filter low-pass at 10Hz
# def butter_lowpass_filter(data, highcut, fs, order=5):
#     sos = butter(order, highcut, btype='low', fs = fs, output='sos')
#     y = sosfilt(sos, data)
#     return y

# left_torque_sim = butter_lowpass_filter(left_torque_sim, 10, SAMPLING_RATE, 5).tolist()
# right_torque_sim = butter_lowpass_filter(right_torque_sim, 10, SAMPLING_RATE, 5).tolist()

# Pad shorter lists
lmax = 0
for col in data:
    lmax = max(lmax, len(col))
for col in data:
    ll = len(col)
    if  ll < lmax:
        col += [float("NaN")] * (lmax - ll)

data_df = pd.DataFrame(np.transpose(data), columns=columns)
# print(data_df)
data_df.to_csv(csv_file_path)

matplotlib.rcParams["savefig.directory"] = plot_dir

torque_sampling = 20
# Plot data
fig, ax = plt.subplots(2, 2)
ax[0, 0].plot(command_time, command_lin_vel)
ax[0, 0].set_title("Cmd Linear Velocity")
ax[0, 0].set_ylabel("Linear Velocity (m/s)")

ax[0, 1].plot(command_time, command_ang_vel)
ax[0, 1].set_title("Cmd Angular Velocity")
ax[0, 1].set_ylabel("Angular Veclocity (rad/s)")

ax[1, 0].plot(left_time_sim[0::torque_sampling], left_torque_sim[0::torque_sampling])
ax[1, 0].set_title("Left Wheel")
ax[1, 0].set_ylabel("Torque (Nm)")

ax[1, 1].plot(right_time_sim[0::torque_sampling], right_torque_sim[0::torque_sampling])
ax[1, 1].set_title("Right Wheel")
ax[1, 1].set_ylabel("Torque (Nm)")


for row in ax:
    for subplt in row:
        subplt.set_xlabel("Time (s)")

fig.tight_layout()
plt.show()

# print(plot_file_path)
# plt.savefig(plot_file_path, dpi=300, bbox_inches='tight')

