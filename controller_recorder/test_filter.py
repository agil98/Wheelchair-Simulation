#!/usr/bin/env python

from scipy.signal import butter, lfilter
import pandas as pd


data_df = pd.read_csv("csvs/test.csv")
left_time_sim = data_df["left_time_sim"]
left_ang_vel_sim = data_df["left_ang_vel_sim"]

# Filter
def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    y = lfilter(b, a, data)
    return y


fs = 1.0/(left_time_sim[1] - left_time_sim[0])
print(fs)
lowcut = 0.1
highcut = 5000
if highcut > fs/2:
    highcut  = fs/2

print(len(left_ang_vel_sim))
print(type(left_ang_vel_sim))
left_ang_vel_sim = butter_bandpass_filter(left_ang_vel_sim, lowcut, highcut, fs).tolist()
print(len(left_ang_vel_sim))
print(type(left_ang_vel_sim))
