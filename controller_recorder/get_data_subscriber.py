#!/usr/bin/env python

import rospy
import rosbag
import csv
import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from scipy.signal import butter, lfilter


rospy.init_node('get_data_subscriber')  

SAMPLING_RATE = 240
SCALING = 1
WHEEL_DIAMETER = 0.3045*2
LEFT_BAG_SIM = 'left.bag'
RIGHT_BAG_SIM = 'right.bag'

file_name = rospy.get_param('get_data_subscriber/file_name')
csv_dir = rospy.get_param('get_data_subscriber/csv_save_location')
plot_dir = rospy.get_param('get_data_subscriber/plot_save_location')
if not os.path.exists(csv_dir):
    os.makedirs(csv_dir)
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

csv_file_path = os.path.join(csv_dir, file_name + ".csv")
plot_file_path = os.path.join(plot_dir, file_name + ".png")

bag_left = rosbag.Bag(LEFT_BAG_SIM, 'w')
bag_right = rosbag.Bag(RIGHT_BAG_SIM, 'w')

started = False

def imu_left_callback(msg):
    if started:
        bag_left.write('imu_sensor_left', msg)

def imu_right_callback(msg):
    if started:
        bag_right.write("imu_sensor_right", msg)

def start_callback(msg):
    global started
    started = True
    print("Started")

def print_callback(msg):
    print(msg)


# sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
sub_imu_left = rospy.Subscriber("imu/WheelL", Imu, imu_left_callback)
sub_imu_right = rospy.Subscriber("imu/WheelR", Imu, imu_right_callback)
sub_start_signal = rospy.Subscriber("started_sim", Bool, start_callback)

# Wait for start
print("Waiting for start...")

# start_time = rospy.Time.now().to_sec()

# Wait until finished
finished_msg = rospy.wait_for_message("finished_sim", Bool)
print("Finished")

sub_imu_left.unregister()
sub_imu_right.unregister()
bag_left.close()
bag_right.close()

# Read from rosbag

print("Reading from rosbag...")
bag_left = rosbag.Bag(LEFT_BAG_SIM)
bag_right = rosbag.Bag(RIGHT_BAG_SIM)

left_time_sim = []
left_ang_vel_sim = []
right_time_sim = []
right_ang_vel_sim = []

left_time_command = [1]
left_ang_vel_command = []
right_time_command = []
right_ang_vel_command = []


# bag.reindex()
for topic, msg, t in bag_right.read_messages(topics=['imu_sensor_right']):
    right_time_sim.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
    right_ang_vel_sim.append(msg.angular_velocity.y)
for topic, msg, t in bag_left.read_messages(topics=['imu_sensor_left']):
    left_time_sim.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
    left_ang_vel_sim.append(msg.angular_velocity.y)

# Get command csv
command_df = pd.read_csv(rospy.get_param('command_csv'))
command_time = np.transpose(command_df["Time"])
command_left_ang_vel = np.transpose(command_df["AngVel_L"])
command_right_ang_vel = np.transpose(command_df["AngVel_R"])

# Make left_time, right_time start from 0
left_time_sim = [x-left_time_sim[0] for x in left_time_sim]
right_time_sim = [x-right_time_sim[0] for x in right_time_sim]

# # Filter
# def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
#     nyq = 0.5 * fs
#     low = lowcut / nyq
#     high = highcut / nyq
#     b, a = butter(order, [low, high], btype='band')
#     y = lfilter(b, a, data)
#     return y

# fs = 1.0/(left_time_sim[1] - left_time_sim[0])
# lowcut = 0.1
# highcut = 50
# if highcut > fs/2:
#     highcut  = fs/2
# left_ang_vel_sim = butter_bandpass_filter(left_ang_vel_sim, lowcut, highcut, fs).tolist()
# right_ang_vel_sim = butter_bandpass_filter(right_ang_vel_sim, lowcut, highcut, fs).tolist()

columns = ["left_time_sim", "left_ang_vel_sim", "right_time_sim", "right_ang_vel_sim", "command_time", "command_left_ang_vel", "command_right_ang_vel"]
data = [left_time_sim, left_ang_vel_sim, right_time_sim, right_ang_vel_sim, 
    command_time, command_left_ang_vel, command_right_ang_vel]

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
# Plot data
fig, ax = plt.subplots(2)
ax[0].plot(left_time_sim, left_ang_vel_sim, label = "Sim Data")
ax[0].plot(command_time, command_left_ang_vel, label = "Command Data")
ax[0].set_title("Left Wheel")
ax[1].plot(right_time_sim, right_ang_vel_sim, label = "Sim Data")
ax[1].plot(command_time, command_right_ang_vel, label = "Command Data")
ax[1].set_title("Right Wheel")

for subplt in ax:
    subplt.set_xlabel("Time (s)")
    subplt.set_ylabel("Angular Veclocity (rad/s)")
    subplt.legend()

fig.tight_layout()
plt.show()

# print(plot_file_path)
# plt.savefig(plot_file_path, dpi=300, bbox_inches='tight')

