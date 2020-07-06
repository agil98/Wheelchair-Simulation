#!/usr/bin/env python

import rospy
import csv
import pandas as pd
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


SAMPLING_RATE = 240
SCALING = 1
WHEEL_DIAMETER = 1
DURATION = 5

finished = False

rospy.init_node('get_data_subscriber')

cmd_vel_x = []
cmd_ang_vel_z = []
left_ang_vel = []
right_ang_vel = []
left_vel_x = []
right_vel_x = []
left_time = []
right_time = []

# def cmd_vel_callback(msg):
#     cmd_vel_x.append(msg.linear.x)
#     cmd_ang_vel_z.append(msg.angular.z)

def imu_left_callback(msg):
    left_time.append(msg.header.stamp.nsecs)
    left_ang_vel.append(msg.angular_velocity.y)
    left_vel_x.append(msg.angular_velocity.y * WHEEL_DIAMETER)

def imu_right_callback(msg):
    right_time.append(msg.header.stamp.nsecs)
    right_ang_vel.append(msg.angular_velocity.y)
    right_vel_x.append(msg.angular_velocity.y * WHEEL_DIAMETER)

def finished_sim_callback(msg):
    print(msg.data)
    global finished
    finished = msg.data

def print_callback(msg):
    print(msg)


# sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
sub_imu_left = rospy.Subscriber("imu_left_quat", Imu, imu_left_callback)
sub_imu_right = rospy.Subscriber("imu_right_quat", Imu, imu_right_callback)
check_finished = rospy.Subscriber("finished_sim", Bool, finished_sim_callback)

r = rospy.Rate(10)
start = rospy.Time.now().to_sec()
print(start)

finished = rospy.wait_for_message("finished_sim", Bool)
while not rospy.is_shutdown() and not finished and len(left_time) < 50000 and len(right_time) < 50000:
    print(finished)
    r.sleep()


data = {"left_time": left_time ,"left_ang_vel": left_ang_vel, "left_vel_x": left_vel_x,
    "right_time": right_time , "right_ang_vel": right_ang_vel, "right_vel_x": right_vel_x,}
data_df = pd.DataFrame(data)
print(data_df)

data_df.to_csv("test.csv")
