#!/usr/bin/env python
'''
Convert RTSLAM dataset to ROS bag

Usage:
./rtslam_dataset_to_ros_bag.py <path-to-MTI.log-file> <path-to-image-dir> <bagname>
'''

import sys
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu

mtilog_path = sys.argv[1]
imagedir_path = sys.argv[2]
bagname = sys.argv[3]

with open(mtilog_path) as f:
    lines = f.readlines()

mti_data = [map(float, line.split()) for line in lines if not line.startswith('#')]

with rosbag.Bag(bagname, 'w') as bag:
	imu_seq=1
    for date, acc_x, acc_y, acc_z, angvel_x, angvel_y, angvel_z, mag_x, mag_y, mag_z in mti_data:
		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.from_sec(date)
		imu_msg.header.seq = imu_seq
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
		imu_seq += 1

