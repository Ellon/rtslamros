#! /usr/bin/env python
"""
Convert Image data from RTSLAM dataset to ROS bag

usage:   rtslam_image_to_bag.py <path-to-rtslam-dataset> 1|2|mono [begin]
example: rtslam_image_to_bag.py dataset/mana/ 1 0
example: rtslam_image_to_bag.py dataset/mana/ mono 42

"""
import os
import sys
import cv2
import rospy
import rosbag
import numpy
from math import sin, cos
from sensor_msgs.msg import Image, CameraInfo
from tf.msg import tfMessage
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from geometry_msgs.msg import TransformStamped

def restart_line():
    sys.stdout.write('\r')
    sys.stdout.flush()

def image_time(filepath):
    img = cv2.imread(filepath + '.png', 0) # Return a grayscale image.
    with open(filepath + '.time') as f:
        time = float(f.read())
    return img, time

def get_intrinsic_distortion(filepath='setup.cfg', camera=''):
    with open(filepath) as f:
      lines = f.readlines()
    parser = lambda label: map(float, [line for line in lines \
        if line.startswith(label)][0].strip().split('(')[1].split(')')[0].split(','))
    i = parser('CAMERA%s_INTRINSIC:'%camera)
    d = parser('CAMERA%s_DISTORTION:'%camera)

    # From rtslam/main.hpp:334
    # CAMERA_INTRINSIC parameter order: u0,v0,alphaU,alphaV
    intrinsic = numpy.zeros([3,3])
    intrinsic[0][0] = i[2]
    intrinsic[0][2] = i[0]
    intrinsic[1][1] = i[3]
    intrinsic[1][2] = i[1]
    intrinsic[2][2] = 1

    # From rtslam/main.hpp:335
    # CAMERA_DISTORTION parameter order:  r1, r2, r3
    distortion = [d[0], d[1], 0, 0, d[2]]

    return intrinsic, distortion

def matrix(yaw, pitch, roll, x, y, z):
    # from pom-genom/libeuler/pomEuler.c:287 (pomWriteSensorPos)
    # euler.{yaw,pitch,roll,x,y,z}
    ca, sa = cos(yaw),   sin(yaw)
    cb, sb = cos(pitch), sin(pitch)
    cg, sg = cos(roll),  sin(roll)
    return [[ ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg, x],
            [ sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg, y],
            [ -sb, cb*sg, cb*cg, z],
            [ 0.0, 0.0, 0.0, 1.0]]

def tfm(matrix, parent, child, stamp):
    rotation = quaternion_from_matrix(matrix)
    translation = translation_from_matrix(matrix)
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp
    t.child_frame_id = child
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return tfMessage([t])

def get_camera_pose_as_matrix(filepath='setup.cfg', camera=''):
    with open(filepath) as f:
      lines = f.readlines()
    parser = lambda label: map(float, [line for line in lines \
        if line.startswith(label)][0].strip().split('(')[1].split(')')[0].split(','))
    p = parser('CAMERA%s_POSE_INERTIAL:'%camera)
    x, y, z = p[0:3]
    roll, pitch, yaw = numpy.deg2rad(p[3:6])

    return matrix(yaw, pitch, roll, x, y, z)


# Get program arguments
dataset_dir = sys.argv[1]
camera_id = sys.argv[2]
begin = 0 if len(sys.argv) < 4 else int(sys.argv[3])

# Set names used along the script
name = 'image' if camera_id == 'mono' \
               else 'image_%i'%int(camera_id)
camera_name = 'camera' if camera_id == 'mono' \
               else 'camera%i'%int(camera_id)
template = name+'_%07i'

base_frame = "base"

# Common attributes for image msg
image = Image()
image.encoding = "mono8"
image.header.frame_id = camera_name
image.header.seq = begin

# Common attributes for camera info msg
camera = '' if camera_id == 'mono' else camera_id
intrinsic, distortion = get_intrinsic_distortion(os.path.join(dataset_dir,'setup.cfg'), camera)
# http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
# http://wiki.ros.org/image_pipeline/CameraInfo
camera_info = CameraInfo()
camera_info.header.seq = image.header.seq
camera_info.header.frame_id = image.header.frame_id
camera_info.distortion_model = 'plumb_bob'
Tx = 0
Ty = 0
R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
camera_info.D = distortion
camera_info.K = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
camera_info.R = R
camera_info.P = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]

# Fixed base-to-camera transform
base_to_camera = get_camera_pose_as_matrix(os.path.join(dataset_dir,'setup.cfg'), camera)


with rosbag.Bag('%s.bag'%camera_name, 'w') as bag:
    while 1:
        # Print info
        sys.stdout.write("processing " + template%image.header.seq + "... ")
        sys.stdout.flush()
        try:
            img, time = image_time( os.path.join(dataset_dir,template%image.header.seq) )
        except IOError as error:
            print("stop at %i: %s"%(image.header.seq, str(error)))
            break

        # Fill image msg
        image.data = img.tostring()
        image.height, image.width = img.shape
        image.step = image.width # grayscale image
        image.header.stamp = rospy.Time.from_sec(time)

        # Fill camera info msg
        camera_info.header = image.header
        camera_info.header.stamp = image.header.stamp
        camera_info.header.seq = image.header.seq
        camera_info.height = image.height
        camera_info.width = image.width

       # Write on bag
        bag.write('/%s/image'%camera_name, image, image.header.stamp)
        bag.write('/%s/camera_info'%camera_name, camera_info, camera_info.header.stamp)
        bag.write('/tf', tfm(base_to_camera, base_frame, camera_name, image.header.stamp), image.header.stamp)

        # Advance seq
        image.header.seq += 1

        # Restart line for pretty printing
        restart_line()

