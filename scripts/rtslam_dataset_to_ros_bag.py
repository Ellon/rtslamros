#!/usr/bin/env python
'''
Read IMU log file and image timestamps, and write a bag that publishes 
IMU and image messages in order.

Usage: 
./rtslam_dataset_to_ros_bag.py <path-to-rtslam-dataset> <bagname>
'''

import sys
import math
import rosbag
from sensor_msgs.msg import Image, Imu


data_path = sys.argv[1]
bagname = sys.argv[2]

seq = 1;
with rosbag.Bag(bagname, 'w') as outbag:
	with open(data_path + 'MTI.log', 'r') as f:
		for line in f:
			imu_data = line[5:-1].split(',')
			imu_msg = Imu()
			imu_msg.header.seq = seq
#			imu_msg.header.frame_id = ???
			imu_msg.header.stamp.nsecs = int(math.modf(float(imu_data[0]))[0]*1e9)
			imu_msg.header.stamp.secs = int(math.modf(float(imu_data[0]))[1])
			imu_msg.linear_acceleration.x = float(imu_data[1])
			imu_msg.linear_acceleration.y = float(imu_data[2])
			imu_msg.linear_acceleration.z = float(imu_data[3])
			imu_msg.angular_velocity.x = float(imu_data[4])
			imu_msg.angular_velocity.y = float(imu_data[5])
			imu_msg.angular_velocity.z = float(imu_data[6])
			seq = seq + 1
			outbag.write("imu", imu_msg, imu_msg.header.stamp)

	
