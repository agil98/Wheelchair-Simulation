#!/usr/bin/env python
import message_filters, rospy, rospkg
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from wheelchair.msg import ModifiedIMU
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

w = 0.53616 # wheel separation
prev = 0

def callback(frame, left, right):

	# Transform orientation from quaternion to euler
	orientation =  frame.orientation
	orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

	# Calculate linear velocity
	global prev
	linear_acc = frame.linear_acceleration

	dt = frame.header.stamp.nsecs - prev
	vel_x = (linear_acc.x / (1e9) * dt) 
	vel_y = (linear_acc.y / (1e9) * dt)
	vel_z = (linear_acc.z / (1e9) * dt)

	linear_vel = Vector3(vel_x, vel_y, vel_z)
	prev = frame.header.stamp.nsecs

	# Obtain header from the frame and change the id
	header = frame.header
	header.frame_id = 'Wheelchair'

	# Determine angular velocity of the wheelchair
	ang_vel_x = (-left.angular_velocity.x + right.angular_velocity.x) / w
	ang_vel_y = (-left.angular_velocity.y + right.angular_velocity.y) / w
	ang_vel_z = (-left.angular_velocity.z + right.angular_velocity.z) / w
	
	angular_vel = Vector3(ang_vel_x, ang_vel_y, ang_vel_z)

	# Publish the imu data of the entire wheelchair
	pub_msg = ModifiedIMU(header, Vector3(roll, pitch, yaw),  angular_vel, linear_vel, linear_acc)
	pub.publish(pub_msg)


rospy.init_node('calculate_imu_for_wheelchair')

# Create subscribers to the frame, left and right wheel imu sensors
sub_frame = message_filters.Subscriber ('imu_sensor_frame', Imu)
sub_left = message_filters.Subscriber ('imu_sensor_left', Imu)
sub_right = message_filters.Subscriber ('imu_sensor_right', Imu)

# Create publisher to send data to a new topic
pub = rospy.Publisher('/imu_entire_wheelchair', ModifiedIMU, queue_size=1000)

# Synchronizes incoming channels by the timestamps contained in their headers
# and outputs them in the form of a single callback that takes the same number of channels
# lastly, register the callback
ts = message_filters.TimeSynchronizer([sub_frame, sub_left, sub_right], 10)
ts.registerCallback(callback)
rospy.spin()