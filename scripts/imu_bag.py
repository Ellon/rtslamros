#!/usr/bin/env python
"""
Convert IMU (MTI) log to ROS bag

usage: imu_bag.py [MTI.log]
"""
import sys
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu

filepath = 'MTI.log' if len(sys.argv) < 2 else sys.argv[1]

with open(filepath) as f:
    lines = f.readlines()

data = [map(float, line.split()) for line in lines if not line.startswith('#')]

imu = Imu()
imu.header.frame_id = 'imu'
imu.header.seq = 0

with rosbag.Bag(filepath+'.bag', 'w') as bag:
    for date, imu.linear_acceleration.x, imu.linear_acceleration.y, \
            imu.linear_acceleration.z, imu.angular_velocity.x, \
            imu.angular_velocity.y, imu.angular_velocity.z, \
            mag_x, mag_y, mag_z in data:
        imu.header.stamp = rospy.Time.from_sec(date)
        # Inform we doesn't have orientation estimates
        # see http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        imu.orientation_covariance[0] = -1
        bag.write("/imu", imu, imu.header.stamp)
        imu.header.seq += 1
