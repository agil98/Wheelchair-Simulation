#!/usr/bin/env python

from scipy.signal import butter, sosfilt
import pandas as pd
import rosbag
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

MAX_ORDER = 4
CUTOFF_FREQ = 0.15

LEFT_BAG_SIM = 'left.bag'
RIGHT_BAG_SIM = 'right.bag'

bag_left = rosbag.Bag("/home/leo/.ros/left.bag")
bag_right = rosbag.Bag("/home/leo/.ros/right.bag")

left_time_sim = []
left_torque_sim = []
right_time_sim = []
right_torque_sim = []


# bag.reindex()
for topic, msg, t in bag_right.read_messages(topics=['ft_sensor_right']):
    right_time_sim.append(msg.header.stamp.secs +
                          msg.header.stamp.nsecs/1000000000.0)
    right_torque_sim.append(msg.wrench.torque.y)
for topic, msg, t in bag_left.read_messages(topics=['ft_sensor_left']):
    left_time_sim.append(msg.header.stamp.secs +
                         msg.header.stamp.nsecs/1000000000.0)
    left_torque_sim.append(msg.wrench.torque.y)

# Get command csv
command_df = pd.read_csv("~/Downloads/Turn180R_T1.csv")
command_time = np.transpose(command_df["Time"])
command_lin_vel = np.transpose(command_df["Chair_LinVel"])
command_ang_vel = np.transpose(command_df["Chair_AngVel"])
command_torque_L = np.transpose(command_df["Torque_L"])
command_torque_R = np.transpose(command_df["Torque_R"])

# Make left_time, right_time start from 0
left_time_sim = [x-left_time_sim[0] for x in left_time_sim]
right_time_sim = [x-right_time_sim[0] for x in right_time_sim]

# Invert left, right torques
left_torque_sim = [-x for x in left_torque_sim]
right_torque_sim = [-x for x in right_torque_sim]

columns = ["left_time_sim", "left_torque_sim", "right_time_sim",
           "right_torque_sim", "command_time", "command_lin_vel", "command_ang_vel"]
data = [left_time_sim, left_torque_sim, right_time_sim, right_torque_sim,
        command_time, command_lin_vel, command_ang_vel]


def butter_lowpass_filter(data, highcut, fs, order=5):
    sos = butter(order, highcut, btype='low', fs=fs, output='sos')
    y = sosfilt(sos, data)
    return y


fs = 240

# left_torque_sim_filtered = butter_lowpass_filter(
#     left_torque_sim, CUTOFF_FREQ, fs, ORDER).tolist()
# right_torque_sim_filtered = butter_lowpass_filter(
#     right_torque_sim, CUTOFF_FREQ, fs, ORDER).tolist()

# Pad shorter lists
# lmax = 0
# for col in data:
#     lmax = max(lmax, len(col))
# for col in data:
#     ll = len(col)
#     if ll < lmax:
#         col += [float("NaN")] * (lmax - ll)

# data_df = pd.DataFrame(np.transpose(data), columns=columns)
# print(data_df)
# data_df.to_csv(csv_file_path)

matplotlib.rcParams["savefig.directory"] = "~/catkin_ws/src/Wheelchair-Simulation/controller_recorder/Plots"

torque_sampling = 1

freq_range = np.linspace(1, 5, 5)

# Plot data
fig, ax = plt.subplots(len(freq_range) + 2, 2, sharex=True, figsize=(20, 100))
ax[0, 0].plot(command_time, command_lin_vel)
ax[0, 0].set_title("Cmd Linear Velocity", fontsize=8)
ax[0, 0].set_ylabel("Linear Velocity (m/s)")

ax[0, 1].plot(command_time, command_ang_vel)
ax[0, 1].set_title("Cmd Angular Velocity", fontsize=8)
ax[0, 1].set_ylabel("Angular Veclocity (rad/s)")

ax[1, 0].plot(left_time_sim[0::torque_sampling],
              left_torque_sim[0::torque_sampling])
ax[1, 0].set_title("Left Wheel (Raw)", fontsize=8)
ax[1, 0].set_ylabel("Torque (Nm)")

ax[1, 1].plot(right_time_sim[0::torque_sampling],
              right_torque_sim[0::torque_sampling])
ax[1, 1].set_title("Right Wheel (Raw)", fontsize=8)
ax[1, 1].set_ylabel("Torque (Nm)")


count = 2
for i in freq_range:
    left_torque_sim_filtered = butter_lowpass_filter(
        left_torque_sim, i, fs, MAX_ORDER).tolist()
    right_torque_sim_filtered = butter_lowpass_filter(
        right_torque_sim, i, fs, MAX_ORDER).tolist()

    # ax[count, 0].plot(left_time_sim[0::torque_sampling],
    #               left_torque_sim[0::torque_sampling], label="raw")
    ax[count, 0].plot(left_time_sim[0::torque_sampling],
                      left_torque_sim_filtered[0::torque_sampling], label="filtered")
    ax[count, 0].plot(command_time, command_torque_L,
                      "--", label="experimental")
    ax[count, 0].set_title("Left Wheel (Cutoff " + str(i) + ")", fontsize=8)
    ax[count, 0].set_ylabel("Torque (Nm)")
    # ax[i, 0].legend()

    # ax[count, 1].plot(right_time_sim[0::torque_sampling],
    #               right_torque_sim[0::torque_sampling], label="raw")
    ax[count, 1].plot(right_time_sim[0::torque_sampling],
                      right_torque_sim_filtered[0::torque_sampling], label="filtered")
    ax[count, 1].plot(command_time, command_torque_R,
                      "--", label="experimental")
    ax[count, 1].set_title("Right Wheel (Cutoff " + str(i) + ")", fontsize=8)
    ax[count, 1].set_ylabel("Torque (Nm)")
    # ax[i, 1].legend()
    count = count + 1

ax[MAX_ORDER + 2, 0].set_xlabel("Time (s)")
# ax[MAX_ORDER + 2, 0].legend()
ax[MAX_ORDER + 2, 1].set_xlabel("Time (s)")
# ax[MAX_ORDER + 2, 1].legend()

# for row in ax:
#     for subplt in row:
#         subplt.set_xlabel("Time (s)")

# fig.tight_layout()
plt.show()
