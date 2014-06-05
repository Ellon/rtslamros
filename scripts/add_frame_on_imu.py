#!/usr/bin/env python

import rospy
import tf
import copy

from sensor_msgs.msg import *

def imu_callback(data):
    global pubImu
    data.header.frame = "uav0/body"
    pubImu.publish(data)

if __name__ == '__main__':
    rospy.init_node('add_frame_on_imu', anonymous=False)
    ns = rospy.get_namespace ()
    print "NAMESPACE: ", ns

    rospy.Subscriber("imu", Imu, imu_callback)
    pubImu = rospy.Publisher("imu_frame", Imu)
    
    while not rospy.is_shutdown ():
        rospy.sleep(0.1)
