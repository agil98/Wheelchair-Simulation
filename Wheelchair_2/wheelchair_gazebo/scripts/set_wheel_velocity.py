#!/usr/bin/env python
import rospy
import rospkg
import csv
import time
import os
from std_msgs.msg import Float64

SAMPLING_RATE = 240

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('wheelchair_gazebo')
filename = os.path.join(pkg_path, 'scripts/StraightF_T1_WS64_Mahsa.csv')

def VelocityControl():

	# Initialize the node and create two publishers for each wheel
	rospy.init_node('velocity_publisher')

	velocity_left_pub = rospy.Publisher('/joint_velocity_controller_left/command', Float64, queue_size=1)
	velocity_right_pub = rospy.Publisher('/joint_velocity_controller_right/command', Float64, queue_size=1)

	# Set publishing rate in Hz
	rate = rospy.Rate(SAMPLING_RATE)

	# Open the csv file with the data and split the values where the commas are located
	with open(filename, 'rb') as csvfile:
		csvfile.readline()
		date = csv.reader(csvfile, delimiter=',')

		for row in data:
			# Publish angular velocity to the wheels independently
			left_vel = float(row[1])
			right_vel = float(row[2])
			velocity_left_pub.publish(left_vel)
			velocity_right_pub.publish(right_vel)
			print(left_vel, right_vel)
			rate.sleep()
		left_vel = 0.0
		right_vel = 0.0

		while not rospy.is_shutdown():
			velocity_left_pub.publish(left_vel)
			velocity_right_pub.publish(right_vel)
			rate.sleep()

		# this and this to classify the data and explain how they were implemented
		# 

if __name__ == '__main__':
	try:
		VelocityControl()
	except:
		"There was an error publishing velocity messages"