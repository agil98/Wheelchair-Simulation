#!/usr/bin/env python
import rospy, time
from wheelchair_data.msg import WheelchairData
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from tf.transformations import euler_from_quaternion

frame_msg = WheelchairData()
left_msg = WheelchairData()
right_msg = WheelchairData()

prev =  0

def callback(msg, args):
	orientation =  msg.orientation
	orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	
	linear_acc = msg.linear_acceleration

	global prev 

	dt = msg.header.stamp.nsecs - prev
	vel_x = (linear_acc.x / (1e9) * dt) 
	vel_y = (linear_acc.y / (1e9) * dt)
	vel_z = (linear_acc.z / (1e9) * dt)

	prev = msg.header.stamp.nsecs

	pub_msg = WheelchairData(msg.header, Vector3(roll, pitch, yaw),  msg.angular_velocity, Vector3(vel_x, vel_y, vel_z), linear_acc)

	if args == "frame":
		pub_frame.publish(pub_msg)
	elif args == "left":
		pub_left.publish(pub_msg)
	else:
		pub_right.publish(pub_msg)

rospy.init_node('quaternion_to_euler')

# Create publishers
pub_frame = rospy.Publisher('/imu_data_frame', WheelchairData, queue_size=1000)
pub_left = rospy.Publisher('/imu_data_left', WheelchairData, queue_size=1000)
pub_right = rospy.Publisher('/imu_data_right', WheelchairData, queue_size=1000)

# Subscribe to the imu messages and pass the messages to get_rotation
sub_frame = rospy.Subscriber ('/imu_sensor_frame', Imu, callback, ("frame"))
sub_left = rospy.Subscriber ('/imu_sensor_left', Imu, callback, ("left"))
sub_right = rospy.Subscriber ('/imu_sensor_right', Imu, callback, ("right"))
rospy.spin()
