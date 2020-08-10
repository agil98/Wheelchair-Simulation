#!/usr/bin/env python
import rospy, time
from wheelchair.msg import ModifiedIMU
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

prev =  0

def callback(msg, args):

	# Obtain orientation from the imu sensor and convert it to euler
	orientation =  msg.orientation
	orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	

	# Calculate linear velocity
	linear_acc = msg.linear_acceleration
	global prev 
	
	dt = msg.header.stamp.nsecs - prev
	vel_x = (linear_acc.x / (1e9) * dt) 
	vel_y = (linear_acc.y / (1e9) * dt)
	vel_z = (linear_acc.z / (1e9) * dt)

	linear_vel = Vector3(vel_x, vel_y, vel_z)
	prev = msg.header.stamp.nsecs

	# Define message to be publishing by obtaining the header, angular velocity and linear acceleration from the sensor
	# and the euler angles and linear velocity
	pub_msg = ModifiedIMU(msg.header, Vector3(roll, pitch, yaw),  msg.angular_velocity, linear_vel, linear_acc)

	# Determine what publisher will be used to send the data
	if args == "frame":
		pub_frame.publish(pub_msg)
	elif args == "left":
		pub_left.publish(pub_msg)
	else:
		pub_right.publish(pub_msg)

# Initialize node that will subcribe to the imu sensors and publish modified data
rospy.init_node('imu_data_aquisition')

# Create publishers to publish the modified data
pub_frame = rospy.Publisher('/imu_modified_frame', ModifiedIMU, queue_size=1000)
pub_left = rospy.Publisher('/imu_modified_left', ModifiedIMU, queue_size=1000)
pub_right = rospy.Publisher('/imu_modified_right', ModifiedIMU, queue_size=1000)

# Create subscribers to the frame, left and right wheel imu sensors
sub_frame = rospy.Subscriber ('/imu_sensor_frame', Imu, callback, ("frame"))
sub_left = rospy.Subscriber ('/imu_sensor_left', Imu, callback, ("left"))
sub_right = rospy.Subscriber ('/imu_sensor_right', Imu, callback, ("right"))

# Prevent program from exiting and handle the callbacks from the subscribers 
rospy.spin()
