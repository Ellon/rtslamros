#! /usr/bin/env python
"""
usage:   image_bag.py 1|2|mono [begin]
example: image_bag.py 1
example: image_bag.py mono 42
"""
import sys
import cv2
import yaml
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo

def image_time(filepath):
    img = cv2.imread(filepath + '.png', 0) # Return a grayscale image.
    with open(filepath + '.time') as f:
        time = float(f.read())
    return img, time

image = Image()
image.encoding = "mono8"
image.header.frame_id = 'image' if sys.argv[1] == 'mono' \
                        else 'image_%i'%int(sys.argv[1])
image.header.seq = 0 if len(sys.argv) < 3 else int(sys.argv[2])

with open('camera_info.yml') as f:
    data = yaml.load(f.read())

del data['roi'], data['header']
camera_info = CameraInfo(**data)

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
