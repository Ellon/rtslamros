#! /usr/bin/env python
"""
usage:   image_bag.py 1|2|mono [begin]
example: image_bag.py 1 0
example: image_bag.py mono 42
"""
import sys
import cv2
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo

def image_time(filepath):
    img = cv2.imread(filepath + '.png', 0) # Return a grayscale image.
    with open(filepath + '.time') as f:
        time = float(f.read())
    return img, time

def get_intrinsic(filepath='setup.cfg', camera=''):
    with open(filepath) as f:
        lines = f.readlines()
    parser = lambda label: map(float, [line for line in lines \
        if line.startswith(label)][0].strip().split('(')[1].split(')')[0].split(','))
    i = parser('CAMERA%s_INTRINSIC:'%camera)
    d = parser('CAMERA%s_DISTORTION:'%camera)
    # TODO
    return [[0]*3]*3


image = Image()
image.encoding = "mono8"
image.header.frame_id = 'image' if sys.argv[1] == 'mono' \
                        else 'image_%i'%int(sys.argv[1])
image.header.seq = 0 if len(sys.argv) < 3 else int(sys.argv[2])
camera = '' if sys.argv[1] == 'mono' else sys.argv[1]
intrinsic = get_intrinsic('setup.cfg', camera)
# http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
# http://wiki.ros.org/image_pipeline/CameraInfo
camera_info = CameraInfo()
camera_info.distortion_model = 'plumb_bob'
# fill this 3 parameters to get correcty image with stereo camera
Tx = 0
Ty = 0
R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
camera_info.D = [0]
camera_info.K = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
camera_info.R = R
camera_info.P = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]

with rosbag.Bag('%s.bag'%image.header.frame_id, 'w') as bag:
    while 1:
        try:
            img, time = image_time(image.header.frame_id + \
                                   '_%07i'%image.header.seq)
        except IOError as error:
            print("stop at %i: %s"%(image.header.seq, str(error)))
            break
        image.data = img.tostring()
        image.height, image.width = img.shape
        image.step = image.width # grayscale image
        image.header.stamp = rospy.Time.from_sec(time)
        camera_info.header = image.header
        camera_info.height = image.height
        camera_info.width = image.width
        bag.write('/%s/image'%image.header.frame_id, image, image.header.stamp)
        bag.write('/%s/camera_info'%image.header.frame_id, camera_info, \
                  image.header.stamp)
        image.header.seq += 1
