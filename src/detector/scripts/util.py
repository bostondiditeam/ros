import numpy as np
import os
import sys
import math
sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))
from config import cfg
from config import TOP_X_MAX,TOP_X_MIN,TOP_Y_MAX,TOP_Z_MIN,TOP_Z_MAX, \
    TOP_Y_MIN,TOP_X_DIVISION,TOP_Y_DIVISION,TOP_Z_DIVISION

def boxes3d_decompose(boxes3d):

    # translation
    if cfg.DATA_SETS_TYPE == 'didi2' or cfg.DATA_SETS_TYPE == 'didi' or cfg.DATA_SETS_TYPE == 'test':
        T_x = np.sum(boxes3d[:, 0:8, 0], 1) / 8.0
        T_y = np.sum(boxes3d[:, 0:8, 1], 1) / 8.0
        T_z = np.sum(boxes3d[:, 0:8, 2], 1) / 8.0
    elif cfg.DATA_SETS_TYPE == 'kitti':
        T_x = np.sum(boxes3d[:, 0:4, 0], 1) / 4.0
        T_y = np.sum(boxes3d[:, 0:4, 1], 1) / 4.0
        T_z = np.sum(boxes3d[:, 0:4, 2], 1) / 4.0

    Points0 = boxes3d[:, 0, 0:2]
    Points1 = boxes3d[:, 1, 0:2]
    Points2 = boxes3d[:, 2, 0:2]

    dis1=np.sum((Points0-Points1)**2,1)**0.5
    dis2=np.sum((Points1-Points2)**2,1)**0.5

    dis1_is_max=dis1>dis2

    #length width heigth
    L=np.maximum(dis1,dis2)
    W=np.minimum(dis1,dis2)
    H=np.sum((boxes3d[:,0,0:3]-boxes3d[:,4,0:3])**2,1)**0.5

    # rotation
    yaw=lambda p1,p2,dis: math.atan2(p2[1]-p1[1],p2[0]-p1[0])
    R_x = np.zeros(len(boxes3d))
    R_y = np.zeros(len(boxes3d))
    R_z = [yaw(Points0[i],Points1[i],dis1[i]) if is_max else  yaw(Points1[i],Points2[i],dis2[i])
           for is_max,i in zip(dis1_is_max,range(len(dis1_is_max)))]
    R_z=np.array(R_z)

    translation = np.c_[T_x,T_y,T_z]
    size = np.c_[H,W,L]
    rotation= np.c_[R_x,R_y,R_z]
    return translation,size,rotation

def crop_image(image):
    image_crop=image.copy()
    left=cfg.IMAGE_CROP_LEFT  #pixel
    right=cfg.IMAGE_CROP_RIGHT
    top=cfg.IMAGE_CROP_TOP
    bottom=cfg.IMAGE_CROP_BOTTOM
    bottom_index= -bottom if bottom!= 0 else None
    right_index = -right if right != 0 else None
    image_crop=image_crop[top:bottom_index,left:right_index,:]
    return image_crop


def filter_center_car(lidar):
    idx = np.where(np.logical_or(np.abs(lidar[:, 0]) > 4.7/2, np.abs(lidar[:, 1]) > 2.1/2))
    lidar = lidar[idx]
    return lidar

def g_lidar_to_top(lidar):
    if cfg.USE_CLIDAR_TO_TOP:
        top = clidar_to_top(lidar)
    else:
        top = lidar_to_top(lidar)
    return top

# ## lidar to top ##
# def lidar_to_top(lidar):
#
#     idx = np.where (lidar[:,0]>TOP_X_MIN)
#     lidar = lidar[idx]
#     idx = np.where (lidar[:,0]<TOP_X_MAX)
#     lidar = lidar[idx]
#
#     idx = np.where (lidar[:,1]>TOP_Y_MIN)
#     lidar = lidar[idx]
#     idx = np.where (lidar[:,1]<TOP_Y_MAX)
#     lidar = lidar[idx]
#
#     idx = np.where (lidar[:,2]>TOP_Z_MIN)
#     lidar = lidar[idx]
#     idx = np.where (lidar[:,2]<TOP_Z_MAX)
#     lidar = lidar[idx]
#
#     if (cfg.DATA_SETS_TYPE == 'didi' or cfg.DATA_SETS_TYPE == 'test' or cfg.DATA_SETS_TYPE=='didi2'):
#         lidar=filter_center_car(lidar)
#
#     pxs=lidar[:,0]
#     pys=lidar[:,1]
#     pzs=lidar[:,2]
#     prs=lidar[:,3]
#     qxs=((pxs-TOP_X_MIN)//TOP_X_DIVISION).astype(np.int32)
#     qys=((pys-TOP_Y_MIN)//TOP_Y_DIVISION).astype(np.int32)
#     #qzs=((pzs-TOP_Z_MIN)//TOP_Z_DIVISION).astype(np.int32)
#     qzs=(pzs-TOP_Z_MIN)/TOP_Z_DIVISION
#     quantized = np.dstack((qxs,qys,qzs,prs)).squeeze()
#
#     X0, Xn = 0, int((TOP_X_MAX-TOP_X_MIN)//TOP_X_DIVISION)+1
#     Y0, Yn = 0, int((TOP_Y_MAX-TOP_Y_MIN)//TOP_Y_DIVISION)+1
#     Z0, Zn = 0, int((TOP_Z_MAX-TOP_Z_MIN)/TOP_Z_DIVISION)
#     height  = Xn - X0
#     width   = Yn - Y0
#     channel = Zn - Z0  + 2
#     # print('height,width,channel=%d,%d,%d'%(height,width,channel))
#     top = np.zeros(shape=(height,width,channel), dtype=np.float32)
#
#
#     # histogram = Bin(channel, 0, Zn, "z", Bin(height, 0, Yn, "y", Bin(width, 0, Xn, "x", Maximize("intensity"))))
#     # histogram.fill.numpy({"x": qxs, "y": qys, "z": qzs, "intensity": prs})
#
#     if 1:  #new method
#         for x in range(Xn):
#             ix  = np.where(quantized[:,0]==x)
#             quantized_x = quantized[ix]
#             if len(quantized_x) == 0 : continue
#             yy = -x
#
#             for y in range(Yn):
#                 iy  = np.where(quantized_x[:,1]==y)
#                 quantized_xy = quantized_x[iy]
#                 count = len(quantized_xy)
#                 if  count==0 : continue
#                 xx = -y
#
#                 top[yy,xx,Zn+1] = min(1, np.log(count+1)/math.log(32))
#                 max_height_point = np.argmax(quantized_xy[:,2])
#                 top[yy,xx,Zn]=quantized_xy[max_height_point, 3]
#
#                 for z in range(Zn):
#                     iz = np.where ((quantized_xy[:,2]>=z) & (quantized_xy[:,2]<=z+1))
#                     quantized_xyz = quantized_xy[iz]
#                     if len(quantized_xyz) == 0 : continue
#                     zz = z
#
#                     #height per slice
#                     max_height = max(0,np.max(quantized_xyz[:,2])-z)
#                     top[yy,xx,zz]=max_height
#
#     # if 0: #unprocess
#     #     top_image = np.zeros((height,width,3),dtype=np.float32)
#     #
#     #     num = len(lidar)
#     #     for n in range(num):
#     #         x,y = qxs[n],qys[n]
#     #         if x>=0 and x <width and y>0 and y<height:
#     #             top_image[y,x,:] += 1
#     #
#     #     max_value=np.max(np.log(top_image+0.001))
#     #     top_image = top_image/max_value *255
#     #     top_image=top_image.astype(dtype=np.uint8)
#     return top

# PointCloud2 to array
# 		https://gist.github.com/dlaz/11435820
#       https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/src/point_cloud2.py
#       http://answers.ros.org/question/202787/using-pointcloud2-data-getting-xy-points-in-python/
#       https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/point_cloud2.py

def lidar_to_top(points):
    xres = TOP_X_DIVISION
    yres = TOP_Y_DIVISION
    zres = TOP_Z_DIVISION
    side_range = (TOP_X_MIN, TOP_X_MAX)         # left-most to right-most
    fwd_range = (TOP_Y_MIN, TOP_Y_MAX)          # back-most to forward-most
    height_range = (TOP_Z_MIN, TOP_Z_MAX)       # bottom-most to upper-most

    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]
    reflectance = points[:,3]

    # INITIALIZE EMPTY ARRAY - of the dimensions we want
    x_max = int((side_range[1] - side_range[0]) / xres)
    y_max = int((fwd_range[1] - fwd_range[0]) / yres)
    z_max = int((height_range[1] - height_range[0]) / zres)
    # z_max =
    top = np.zeros([x_max, y_max, z_max+2], dtype=np.float32)

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
        x_img = (-yi_points / xres).astype(np.int32)  # x axis is -y in LIDAR
        y_img = (-xi_points / yres).astype(np.int32)  # y axis is -x in LIDAR

        # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
        # floor & ceil used to prevent anything being rounded to below 0 after
        # shift
        x_img -= int(np.floor(side_range[0] / xres))
        y_img += int(np.floor(fwd_range[1] / yres))

        # CLIP HEIGHT VALUES - to between min and max heights
        pixel_values = zi_points - height_range[0]
        # pixel_values = zi_points

        # FILL PIXEL VALUES IN IMAGE ARRAY
        top[x_img, y_img, i] = pixel_values

        # max_intensity = np.max(prs[idx])
        top[x_img, y_img, z_max] = ref_i
    return top
