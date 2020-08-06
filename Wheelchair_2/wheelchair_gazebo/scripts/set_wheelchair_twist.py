#!/usr/bin/env python

import rospkg
import os
import rospy
from geometry_msgs.msg import Twist
import csv
import time

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('wheelchair_gazebo')
filename = os.path.join(pkg_path, 'scripts/StraightF_T1_WS64_Mahsa.csv')

SAMPLING_RATE = 240


def TwistControl():
	# Initialize node
	rospy.init_node('twist_publisher')	

	# Advertise twist publisher
	twist_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)

	# Create twist message, add linear x velocity and angular z velocity
	twist_cmd = Twist()

	# Setting publishing rate in Hz
	rate = rospy.Rate(SAMPLING_RATE)

	with open(filename, 'rb') as csvfile:
		csvfile.readline()
		data = csv.reader(csvfile, delimiter=',')

		t_start = time.time()

		for row in data:
			lin_vel = float(row[3])
			ang_vel = float(row[4])

			# Assign linear and angular velocity components of the twist message
			twist_cmd.linear.x = lin_vel
			twist_cmd.angular.z = ang_vel

			# Publish twist command to wheelchair
			twist_pub.publish(twist_cmd)
			rate.sleep()

		twist_cmd.linear.x = 0.0
		twist_cmd.angular.z = 0.0

		while not rospy.is_shutdown() :
			twist_pub.publish(twist_cmd)
			rate.sleep()
		

		t_duration = (time.time()-t_start)
		print(t_duration)

if __name__ == '__main__':
	try:
		TwistControl()
	except:
		"There was an error publihing twist messages"