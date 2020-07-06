#!/usr/bin/env python
import rospy, time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from tf.transformations import euler_from_quaternion

frame_msg = TwistStamped()
left_msg = TwistStamped()
right_msg = TwistStamped()

def get_rotation_frame (msg):
	global roll_frame, pitch_frame, yaw_frame
	orientation_frame =  msg.orientation
	angular_velocity_frame = msg.angular_velocity
	linear_acceleration_frame = msg.linear_acceleration
	orientation_list_frame = [orientation_frame.x, orientation_frame.y, orientation_frame.z, orientation_frame.w]
	(roll_frame, pitch_frame, yaw_frame) = euler_from_quaternion (orientation_list_frame)
	
	frame_msg = TwistStamped(msg.header,Twist(Vector3(roll_frame, pitch_frame, yaw_frame), angular_velocity_frame))
	pub_frame.publish(frame_msg)

def get_rotation_left (msg):
	global roll_left, pitch_left, yaw_left
	orientation_left =  msg.orientation
	angular_velocity_left = msg.angular_velocity
	linear_acceleration_left = msg.linear_acceleration
	orientation_list_left = [orientation_left.x, orientation_left.y, orientation_left.z, orientation_left.w]
	(roll_left, pitch_left, yaw_left) = euler_from_quaternion (orientation_list_left)

	left_msg = TwistStamped( msg.header, Twist(Vector3(roll_left, pitch_left, yaw_left), angular_velocity_left))
	pub_left.publish(left_msg)

def get_rotation_right (msg):
	global roll_right, pitch_right, yaw_right
	orientation_right =  msg.orientation
	angular_velocity_right = msg.angular_velocity
	linear_acceleration_right = msg.linear_acceleration
	orientation_list_right = [orientation_right.x, orientation_right.y, orientation_right.z, orientation_right.w]
	(roll_right, pitch_right, yaw_right) = euler_from_quaternion (orientation_list_right)

	right_msg = TwistStamped(msg.header, Twist(Vector3(roll_right, pitch_right, yaw_right), angular_velocity_right))
	pub_right.publish(right_msg)

rospy.init_node('quaternion_to_euler')

# Create publishers
pub_frame = rospy.Publisher('/imu_frame', TwistStamped, queue_size=1000)
pub_left = rospy.Publisher('/imu_left', TwistStamped, queue_size=1000)
pub_right = rospy.Publisher('/imu_right', TwistStamped, queue_size=1000)

# Subscribe to the imu messages and pass the messages to get_rotation
sub_frame = rospy.Subscriber ('/imu_frame_quat', Imu, get_rotation_frame)
sub_left = rospy.Subscriber ('/imu_left_quat', Imu, get_rotation_left)
sub_right = rospy.Subscriber ('/imu_right_quat', Imu, get_rotation_right)
rospy.spin()
