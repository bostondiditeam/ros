#!/usr/bin/env python

import rospy
#from beginner_tutorials.msg import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2, PointField
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import numpy as np
import sys
import cv2

# Instantiate CvBridge
bridge = CvBridge()
camera_image = None

#---------------------------------------------------------------------------------------------------------

# PointCloud2 to array
# 		https://gist.github.com/dlaz/11435820
#       https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/src/point_cloud2.py
#       http://answers.ros.org/question/202787/using-pointcloud2-data-getting-xy-points-in-python/
#       https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/point_cloud2.py

def point_cloud_2_top(points,
                      res=0.1,
                      zres=0.3,
                      side_range=(-10., 10.),  # left-most to right-most
                      fwd_range=(-10., 10.),  # back-most to forward-most
                      height_range=(-2., 2.),  # bottom-most to upper-most
                      ):
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]
    reflectance = points[:,3]

    # INITIALIZE EMPTY ARRAY - of the dimensions we want
    x_max = int((side_range[1] - side_range[0]) / res)
    y_max = int((fwd_range[1] - fwd_range[0]) / res)
    z_max = int((height_range[1] - height_range[0]) / zres)
    # z_max =
    top = np.zeros([y_max+1, x_max+1, z_max+1], dtype=np.float32)

    # FILTER - To return only indices of points within desired cube
    # Three filters for: Front-to-back, side-to-side, and height ranges
    # Note left side is positive y axis in LIDAR coordinates
    f_filt = np.logical_and(
        (x_points > fwd_range[0]), (x_points < fwd_range[1]))
    s_filt = np.logical_and(
        (y_points > -side_range[1]), (y_points < -side_range[0]))
    filter = np.logical_and(f_filt, s_filt)

    for i, height in enumerate(np.arange(height_range[0], height_range[1], zres)):

        z_filt = np.logical_and((z_points >= height),
                                (z_points < height + zres))
        zfilter = np.logical_and(filter, z_filt)
        indices = np.argwhere(zfilter).flatten()

        # KEEPERS
        xi_points = x_points[indices]
        yi_points = y_points[indices]
        zi_points = z_points[indices]
        ref_i = reflectance[indices]

        # CONVERT TO PIXEL POSITION VALUES - Based on resolution
        #print("[{},{},{},{}] {}".format(xi_points, yi_points, zi_points, ref_i, res))
        x_img = (-yi_points / res).astype(np.int32)  # x axis is -y in LIDAR
        y_img = (-xi_points / res).astype(np.int32)  # y axis is -x in LIDAR

        # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
        # floor & ceil used to prevent anything being rounded to below 0 after
        # shift
        x_img -= int(np.floor(side_range[0] / res))
        y_img += int(np.floor(fwd_range[1] / res))

        # CLIP HEIGHT VALUES - to between min and max heights
        pixel_values = zi_points - height_range[0]
        # pixel_values = zi_points

        # FILL PIXEL VALUES IN IMAGE ARRAY
        top[y_img, x_img, i] = pixel_values

        # max_intensity = np.max(prs[idx])
        top[y_img, x_img, z_max] = ref_i
    return top

def draw_top_image(top):
    top_binary = np.zeros_like(top)
    top_binary[top > 0] = 128
    return np.dstack((top_binary, top_binary, top_binary)).astype(np.uint8)

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
    # msg1: /image_raw   /velodyne_points: velodyne_points
    timestamp1 = msg1.header.stamp.to_nsec()
    print('image_callback: msg : seq=%d, timestamp=%19d' % (msg1.header.seq, timestamp1))
    timestamp2 = msg2.header.stamp.to_nsec()
    print('velodyne_callback: msg : seq=%d, timestamp=%19d' % (msg2.header.seq, timestamp2))

    arr = msg_to_arr(msg2)
    lidar = np.array([[item[0], item[1], item[2], item[3]] for item in arr])
    side_range = (-45., 45.)
    fwd_range = (-45., 45.)
    height_range = (-2., 1.)
    res = 0.1

    camera_image = bridge.imgmsg_to_cv2(msg1, "bgr8")
    print("camera_image is {}".format(camera_image.shape))

    top_view = point_cloud_2_top(lidar, res=res, zres=0.3, side_range=side_range, fwd_range=fwd_range,
                                 height_range=height_range)
    top_image = draw_top_image(top_view[:, :, -1])

    if 1:           # if show the images
        cemara_show_image = cv2.resize(camera_image,(camera_image.shape[1]//2, camera_image.shape[0]//2))
        top_show_image_width = camera_image.shape[0]//2
        top_show_image = cv2.resize(top_image,(top_show_image_width, top_show_image_width))
        show_image = np.concatenate((top_show_image, cemara_show_image), axis=1)
        cv2.imshow("top", show_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    print(sys.version)

    rospy.init_node('test_node')

    rospy.Subscriber('/image_raw', Image, image_callback)
    rospy.Subscriber('/velodyne_points', PointCloud2, velodyne_points)
    rospy.Subscriber('/objects/capture_vehicle/front/gps/fix', NavSatFix, gpsfix_front_callback)
    rospy.Subscriber('/objects/capture_vehicle/rear/gps/fix', NavSatFix, gpsfix_rear_callback)
    rospy.Subscriber('/radar/points', PointCloud2, radar_points_callback)


    image_raw_sub = message_filters.Subscriber('/image_raw', Image)
    velodyne_points_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([image_raw_sub, velodyne_points_sub], 10, 0.1)
    ts.registerCallback(sync_callback)

    #
    # gpgfix_topic = "/obs1/gps/fix"
    # rospy.Subscriber(gpgfix_topic, NavSatFix, gpsfix_callback)
    # # radar_points
    # radar_points_topic = "/radar/points"
    # rospy.Subscriber(radar_points_topic, PointCloud2, radar_points_callback)

    # Spin until ctrl + c
    rospy.spin()
