import os
import numpy as np
import cv2
import sys
import time

import xmlrpclib
from util import *

sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))

rpc=xmlrpclib.ServerProxy('http://localhost:8080/')

dir = os.path.join(sys.path[0], "/ext2/round2_data/output/suburu_driving_past_it/suburu07")
rgb_path = os.path.join(dir, "image_02/data", "1492888603962510690.png")
rgb = crop_image(cv2.imread(rgb_path))
print(rgb.shape)
lidar_path = os.path.join(dir, "velodyne_points/data", "1492888603962510690.bin")
lidar =np.fromfile(lidar_path, np.float32)
lidar = lidar.reshape((-1, 4))
top = g_lidar_to_top(lidar)

if 0: # test top is ok?
    top_image = draw_top_image(top)
    cv2.imshow("top", top_image)
    cv2.waitKey(10000)

start = time.time()
np.save(os.path.join(sys.path[0], "../MV3D/data/", "top.npy"), top)
np.save(os.path.join(sys.path[0], "../MV3D/data/", "rgb.npy"), rgb)
end = time.time()
print("save npy use time={} seconds".format(end-start))

start = time.time()
boxes3d = rpc.predict()
end = time.time()
print("predict boxes len={} use time={} seconds".format(len(boxes3d),end-start))
if len(boxes3d) > 0:
    translation, size, rotation = boxes3d_decompose(np.array(boxes3d))
    print(boxes3d)
    print("test translation: {}".format(translation))
    print("test size: {}".format(size))
    print("test rotation: {}".format(rotation))
