#!/usr/bin/env python

import rospy
import time
import os

from gazebo_msgs.srv import *
import rospkg
import csv

SAMPLING_RATE = 240

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('wheelchair_gazebo')
filename = os.path.join(pkg_path, 'scripts/StraightF_T1_WS64_Mahsa.csv')


def EffortControl():

	# Initialize node
	rospy.init_node('set_wheel_torque')

	# Advertize effort publisher
	effort_left_pub = rospy.Publisher('/WheelL_effort_controller/command', Float64, queue_size = 1)
	effort_right_pub = rospy.Publisher('/WheelR_effort_controller/command', Float64, queue_size = 1)

	# Setting publishing rate in Hz
	rate = rospy.Rate(SAMPLING_RATE)

	with open(filename, 'rb') as csvfile:
		csvfile.readline()
		data = csv.reader(csvfile, delimiter=',')
		
		for row in data:
			print(row[5], row[6])
			left_torque = float(row[5])
			right_torque = float(row[6])

			# Publish effort commands to wheelchair
			effort_left_pub.publish(left_torque)
			effort_right_pub.publish(right_torque)

			rate.sleep()

	left_torque = 0.0
	right_torque = 0.0

	while not rospy.is_shutdown():
		effort_left_pub.publish(left_torque)
		effort_right_pub.publish(right_torque)
		rate.sleep()


if __name__ == '__main__':
	try:
		EffortControl()
	except:
		"There was an error publishing effort messages"