#!/usr/bin/env python
'''
Convert IMU data from RTSLAM dataset to ROS bag

Usage:
rtslam_imu_bag.py <path-to-MTI.log-file>
'''

import sys
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu

mtilog_path = sys.argv[1]

with open(mtilog_path) as f:
    lines = f.readlines()

mti_data = [map(float, line.split()) for line in lines if not line.startswith('#')]

imu_msg = Imu()
imu_msg.header.seq = 0

with rosbag.Bag("imu.bag", 'w') as bag:
    for date, acc_x, acc_y, acc_z, angvel_x, angvel_y, angvel_z, mag_x, mag_y, mag_z in mti_data:
		imu_msg.header.stamp = rospy.Time.from_sec(date)
		# imu_msg.header.frame_id = ???
		imu_msg.linear_acceleration.x = acc_x
		imu_msg.linear_acceleration.y = acc_y
		imu_msg.linear_acceleration.z = acc_z
		imu_msg.angular_velocity.x = angvel_x
		imu_msg.angular_velocity.y = angvel_y
		imu_msg.angular_velocity.z = angvel_z
		# Inform we doesn't have orientation estimates
		# see http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
		imu_msg.orientation_covariance[0] = -1
		bag.write("imu", imu_msg, imu_msg.header.stamp)
		imu_msg.header.seq += 1

