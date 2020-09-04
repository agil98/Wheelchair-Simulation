#!/usr/bin/env python

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks


# Import data
data_df = pd.read_csv("./StraightF_T1_WS2_Mahsa.csv")
data_df = data_df[["Time", "AngVel_L", "AngVel_R", "Torque_L", "Torque_R"]]
time = data_df["Time"]
left_vel = data_df["AngVel_L"]
right_vel = data_df["AngVel_R"]
left_torque = data_df["Torque_L"]
right_torque = data_df["Torque_R"]


# Parameters
wINDOW_SIZE = 50
TIME_STEP = time[1] - time[0]
CONST_ACC = 1.5
CONST_DEC = 1
# acc_time = 1
CRUISE_TIME = 1 # seconds
MAX_VEL = 5 # m/s
TORQUE_TO_VEL = 0.8 # used for scaling


# Given torque, find target velocity
def get_target_vel(torque):
    return torque*TORQUE_TO_VEL if torque*TORQUE_TO_VEL < MAX_VEL else MAX_VEL
def get_generated_vel(torques, vels, start_vel, cruise_count = 0):
    peaks, _ = find_peaks(torques)

    ref_torque = max(torques) if peaks.size == 0 else max(torques[peaks])
    target_vel = get_target_vel(ref_torque)

    length_count = 0
    generated = []
    generated.append(start_vel)

    # Acceleration
    while length_count < wINDOW_SIZE:
        if target_vel < 0:
            vel_to_append = start_vel - length_count*CONST_ACC*TIME_STEP
        else:
            vel_to_append = start_vel + length_count*CONST_ACC*TIME_STEP
    

        if vel_to_append > MAX_VEL:
            vel_to_append = MAX_VEL
            break
        # if vel_to_append > target_vel:
        #     vel_to_append = target_vel

        generated.append(vel_to_append)
        length_count = length_count + 1

    # Cruise
    while length_count < wINDOW_SIZE and cruise_count * TIME_STEP < CRUISE_TIME:
        generated.append(vel_to_append)
        length_count = length_count + 1
        cruise_count = cruise_count + 1

    if cruise_count * TIME_STEP > CRUISE_TIME:
        cruise_count = 0

    # Decelerate
    last_vel = generated[-1]
    dec_count = 0
    while length_count < wINDOW_SIZE:
        vel_to_append = last_vel - dec_count * CONST_DEC
        if vel_to_append < 0:
            vel_to_append = 0
        generated.append(vel_to_append)
        length_count = length_count + 1

    # Decelerate
    return generated, cruise_count



left_generated = [0]
right_generated = [0]
left_cruise_count = 0
right_cruise_count = 0

for i in range(0, len(data_df), wINDOW_SIZE): 
    window = data_df.iloc[i:i+wINDOW_SIZE, :]
    window.reset_index(drop=True, inplace=True)
    window.dropna()

    new_left, left_cruise_count = get_generated_vel(window["Torque_L"], window["AngVel_L"], left_generated[-1], left_cruise_count)
    new_right, right_cruise_count = get_generated_vel(window["Torque_R"], window["AngVel_R"], right_generated[-1], right_cruise_count)

    left_generated = left_generated + new_left
    right_generated = right_generated + new_right



print(len(time))
print(len(left_generated))

# Plot data
fig, ax = plt.subplots(2, 2, sharex='col')
ax[0, 0].plot(time, left_vel, label = "Measured")
ax[0, 0].plot(time, left_generated[0:len(time)], label = "Generated")
ax[0, 0].set_title("Left Ang Vel")

ax[0, 1].plot(time, left_torque)
ax[0, 1].set_title("Left Torque")

ax[1, 0].plot(time, right_vel, label = "Measured")
ax[1, 0].plot(time, right_generated[0:len(time)], label = "Generated")
ax[1, 0].set_title("Right Ang Vel")

ax[1, 1].plot(time, right_torque)
ax[1, 1].set_title("Right Torque")

for row in ax:
    for subplt in row:
        subplt.set_xlabel("Time (s)")
        subplt.set_ylabel("Angular Veclocity (rad/s)")
    row[0].legend()

fig.tight_layout()
plt.show()
