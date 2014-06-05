#!/usr/bin/env python

import rospy
import tf
import copy

from sensor_msgs.msg import *

f = open('imu_topic.log', 'w')

def imu_callback(data):
    global f

    f.write(str(data.header.stamp.to_sec()) + "\t" + str(data.linear_acceleration.x) + "\t" + str(data.linear_acceleration.y) + "\t"+ str(data.linear_acceleration.z) + "\n")

def logger():
    rospy.init_node('log_imu_topic', anonymous=False)

    rospy.Subscriber("uav0/imu", Imu, imu_callback)
    
    while not rospy.is_shutdown ():
        rospy.sleep(0.1)
        

if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException: pass

f.close()
