#! /usr/bin/env python
"""
Convert rtslam setup.cfg to ROS CameraInfo message in YAML

usage: camera_info_yaml.py [setup.cfg] [camera_info.yml] [camera_id]
"""
import sys
from sensor_msgs.msg import CameraInfo

file_cfg = 'setup.cfg' if len(sys.argv) < 2 else sys.argv[1]
file_yml = 'camera_info.yml' if len(sys.argv) < 3 else sys.argv[2]
camera = '' if len(sys.argv) < 4 else sys.argv[3]

with open(file_cfg) as f:
    lines = f.readlines()

parser = lambda label: map(float, [line for line in lines \
    if line.startswith(label)][0].strip().split('(')[1].split(')')[0].split(','))

i = parser('CAMERA%s_INTRINSIC:'%camera)
d = parser('CAMERA%s_DISTORTION:'%camera)
# TODO
intrinsic = [[0]*3]*3
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

with open(file_yml, 'w') as f:
    f.write(str(camera_info))
