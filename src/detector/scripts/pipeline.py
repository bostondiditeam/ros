#!/usr/bin/env python

import numpy as np
import sys
import cv2
import os
import time
import math

#os.system('roslaunch velodyne_pointcloud 32e_points.launch &')
sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))
#import net.utility.draw  as nud

import rospy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker,MarkerArray
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import xmlrpclib
import py2utils


rpc=xmlrpclib.ServerProxy('http://localhost:8080/')

# Instantiate CvBridge
bridge = CvBridge()

front = np.zeros((1, 1), dtype=np.float32)
pub = None

# https://github.com/eric-wieser/ros_numpy #############################################################################################

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

def msg_to_arr(msg):

    dtype_list = fields_to_dtype(msg.fields, msg.point_step)
    arr = np.fromstring(msg.data, dtype_list)

    # remove the dummy fields that were added
    arr = arr[[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if msg.height == 1:
        return np.reshape(arr, (msg.width,))
    else:
        return np.reshape(arr, (msg.height, msg.width))

def gpsfix_front_callback(msg):
    print("Receive front gps fix message")

def gpsfix_rear_callback(msg):
    print("Receive rear gps fix message")

def radar_points_callback(msg):
    print("Receive radar_points message")

def image_callback(msg):
    print("Receive image_callback message seq=%d, timestamp=%19d" % (msg.header.seq, msg.header.stamp.to_nsec()))
    if 0:
        camera_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        print("camera_image is {}".format(camera_image.shape))
        cv2.imshow("image", camera_image)
        cv2.waitKey(1)

def velodyne_points(msg):
    print("Receive velodyne_points message seq=%d, timestamp=%19d" % (msg.header.seq, msg.header.stamp.to_nsec()))

def sync_callback(msg1, msg2):
    # msg1: /image_raw   # msg2: /velodyne_points: velodyne_points
    timestamp1 = msg1.header.stamp.to_nsec()
    timestamp2 = msg2.header.stamp.to_nsec()
    print('=====image_callback: msg : seq=%d, timestamp=%19d' % (msg1.header.seq, timestamp1))
    print('=====velodyne_callback: msg : seq=%d, timestamp=%19d' % (msg2.header.seq, timestamp2))

    time_check_0 = time.time()
    arr = msg_to_arr(msg2)
    lidar = np.array([[item[0], item[1], item[2], item[3]/255.] for item in arr])
    time_check_1 = time.time()

    camera_image = bridge.imgmsg_to_cv2(msg1, "bgr8")
    time_check_2 = time.time()
    rgb = py2utils.crop_image(camera_image)
    time_check_3 = time.time()

    top = py2utils.g_lidar_to_top(lidar)
    time_check_4 = time.time()

    np.save(os.path.join(sys.path[0], "../MV3D/data/", "top.npy"), top)
    np.save(os.path.join(sys.path[0], "../MV3D/data/", "rgb.npy"), rgb)
    time_check_5 = time.time()
    boxes3d = rpc.predict()
    time_check_6 = time.time()
    if len(boxes3d) > 0:
        translation, size, rotation = py2utils.boxes3d_decompose(np.array(boxes3d))
        # publish (boxes3d) to tracker_node
        markerArray = MarkerArray()
        for i in range(len(boxes3d)):
            m = Marker()
            m.type = Marker.CUBE
            m.header.frame_id = "velodyne"
            m.header.stamp = msg2.header.stamp
            m.scale.x, m.scale.y, m.scale.z = size[i][0],size[i][1],size[i][2]
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                translation[i][0], translation[i][1], translation[i][2]
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = \
                rotation[i][0], rotation[i][1], rotation[i][2], 0.
            m.color.a, m.color.r, m.color.g, m.color.b = \
                1.0, 0.0, 1.0, 0.0
            markerArray.markers.append(m)
        pub.publish(markerArray)
    time_check_7 = time.time()
    print("use {:.4f} seconds for read lidar to numpy".format(time_check_1 - time_check_0))
    print("use {:.4f} seconds for read image to numpy".format(time_check_2 - time_check_1))
    print("use {:.4f} seconds for crop_image".format(time_check_3 - time_check_2))
    print("use {:.4f} seconds for lidar_to_top".format(time_check_4 - time_check_3))
    print("use {:.4f} seconds for save top.npy and rgb.npy".format(time_check_5 - time_check_4))
    print("use {:.4f} seconds for rpc predict: boxes len={}".format(time_check_6 - time_check_5, len(boxes3d)))
    print("use {:.4f} seconds for publish MarkerArray".format(time_check_7 - time_check_6))
    print("use {:.4f} seconds for total".format(time_check_7 - time_check_0))

    if 1:   # if show the images for debug
        top_image = py2utils.draw_top_image(top)
        rgb_image = py2utils.draw_box3d_on_camera(rgb, np.array(boxes3d))
        rgb_image = cv2.resize(rgb_image,(rgb_image.shape[1]//2, rgb_image.shape[0]//2))
        top_image_height = rgb_image.shape[0]
        top_image_width = top_image.shape[1] * rgb_image.shape[0] // top_image.shape[0]
        top_image = py2utils.draw_box3d_on_top(top_image, boxes3d)
        top_image = cv2.resize(top_image,(top_image_width, top_image_height))
        show_image = np.concatenate((top_image, rgb_image), axis=1)
        cv2.imshow("top", show_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('detect_node')
    pub = rospy.Publisher("bbox", MarkerArray, queue_size=1)

    # rospy.Subscriber('/image_raw', Image, image_callback)
    # rospy.Subscriber('/velodyne_points', PointCloud2, velodyne_points)
    # rospy.Subscriber('/objects/capture_vehicle/front/gps/fix', NavSatFix, gpsfix_front_callback)
    # rospy.Subscriber('/objects/capture_vehicle/rear/gps/fix', NavSatFix, gpsfix_rear_callback)
    # rospy.Subscriber('/radar/points', PointCloud2, radar_points_callback)

    image_raw_sub = message_filters.Subscriber('/image_raw', Image)
    velodyne_points_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([image_raw_sub, velodyne_points_sub], 3, 0.03)
    ts.registerCallback(sync_callback)

    print("detecter node initialzed")

    # Spin until ctrl + c
    rospy.spin()