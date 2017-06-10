import os
import numpy as np
import cv2
import sys

sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))

import model as mod
from data import draw_top_image,draw_box3d_on_top
from net.utility.draw import imsave ,draw_box3d_on_camera, draw_box3d_on_camera
from net.processing.boxes3d import boxes3d_decompose

print("init the network")
LIDAR_TOP_SHAPE = (450, 100, 9) # to be decided (400, 400, 8)
LIDAR_FRONT_SHAPE = (1, 1)  # to be decided.
RGB_SHAPE = (596, 1368, 3)  # to be decided (1096, 1368, 3)

m3 = mod.MV3D()
m3.predict_init(LIDAR_TOP_SHAPE, LIDAR_FRONT_SHAPE, RGB_SHAPE)

dir = os.path.join(sys.path[0], "../MV3D/data/preprocessed/didi")

rgb_path = os.path.join(dir, "rgb", "1/6_f", "00000.png")
rgb = cv2.imread(rgb_path)
top_path = os.path.join(dir, "top", "1/6_f", "00000.npy")
top = np.load(top_path)
front = np.zeros((1, 1), dtype=np.float32)

np_reshape = lambda np_array: np_array.reshape(1, *(np_array.shape))
top_view = np_reshape(top)
front_view = np_reshape(front)
rgb_image = np_reshape(rgb)

print(top_view.shape)
print(front_view.shape)
print(rgb_image.shape)

boxes3d, probs = m3.predict(top_view, front_view, rgb_image)
print("predict boxes len=%d" % len(boxes3d))