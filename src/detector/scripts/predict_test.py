import os
import numpy as np
import cv2
import sys
import time

import xmlrpclib
rpc=xmlrpclib.ServerProxy('http://localhost:8080/')

dir = os.path.join(sys.path[0], "../MV3D/data/preprocessed/didi")

rgb_path = os.path.join(dir, "rgb", "1/6_f", "00000.png")
rgb = cv2.imread(rgb_path)
top_path = os.path.join(dir, "top", "1/6_f", "00000.npy")
top = np.load(top_path)
front = np.zeros((1, 1), dtype=np.float32)

np_reshape = lambda np_array: np_array.reshape(1, *(np_array.shape))
top_view = np_reshape(top)
front_view = np_reshape(front)
rgb_view = np_reshape(rgb)

start = time.time()
np.save(os.path.join(sys.path[0], "../MV3D/data/", "top.npy"), top_view)
np.save(os.path.join(sys.path[0], "../MV3D/data/", "rgb.npy"), rgb_view)
end = time.time()
print("save npy use time={} seconds".format(end-start))

start = time.time()
boxes3d  = rpc.predict()
end = time.time()
print("predict boxes len={} use time={} seconds".format(len(boxes3d),end-start))