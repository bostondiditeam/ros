from twisted.web import xmlrpc, server
import numpy as np
import argparse
import os
os.environ['DISPLAY']=':0'
import numpy as np
import cv2
import sys
import time

sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))
from config import cfg
import mv3d
import data
from net.processing.boxes3d import boxes3d_decompose
from data import preprocess


parser = argparse.ArgumentParser(description='predict_rpc')
parser.add_argument('-n', '--tag', type=str, nargs='?', default='unknown_tag',
                    help='set weights tag, which is your training tag')
args = parser.parse_args()
print('\n\n{}\n\n'.format(args))
tag = args.tag
if tag == 'unknown_tag':
    tag = input('Enter log tag : ')
    print('\nSet log tag :"%s" ok !!\n' % tag)

print("init the network")
LIDAR_TOP_SHAPE = (500, 300, 15) # to be decided (400, 400, 8)
LIDAR_FRONT_SHAPE = (0,)  # to be decided.
RGB_SHAPE = (596, 1368, 3)  # to be decided (1096, 1368, 3)


predict = mv3d.Predictor(LIDAR_TOP_SHAPE, LIDAR_FRONT_SHAPE, RGB_SHAPE, log_tag=tag)
front = np.zeros((0, ), dtype=np.float32)

def testCase():
    print("testing predict:")
    dir = os.path.join(cfg.RAW_DATA_SETS_DIR, "suburu_driving_past_it/suburu07")
    rgb_path = os.path.join(dir, "image_02/data", "1492888603962510690.png")
    rgb = preprocess.rgb(cv2.imread(rgb_path))
    print(rgb.shape)
    lidar_path = os.path.join(dir, "velodyne_points/data", "1492888603962510690.bin")
    lidar =np.fromfile(lidar_path, np.float32)
    lidar = lidar.reshape((-1, 4))
    top = preprocess.lidar_to_top(lidar)
    if 0: # test top is ok?
        top_image = data.draw_top_image(top)
        cv2.imshow("top", top_image)
        cv2.waitKey(10000)

    start = time.time()
    boxes3d, probs = predict([top], [front], [rgb])
    translation, size, rotation = boxes3d_decompose(boxes3d)
    timestamp = time.time()-start
    print("predict boxes len=%d use %f seconds" % (len(boxes3d),timestamp))
    print("test translation: {}".format(translation))
    print("test size: {}".format(size))
    print("test rotation: {}".format(rotation))

class MioServer(xmlrpc.XMLRPC):
    def xmlrpc_predict(self):
        top = np.load(os.path.join(sys.path[0], "../MV3D/data/", "top.npy"))
        rgb = np.load(os.path.join(sys.path[0], "../MV3D/data/", "rgb.npy"))
        print("receice rpc pridect request: top-{} rgb-{}".format(top.shape, rgb.shape))
        start = time.time()
        boxes3d, probs = predict([top], [front], [rgb])
        print("predict boxes len: {} use time {}".format(boxes3d.shape[0], time.time()-start))
        return boxes3d.tolist()

if __name__ == '__main__':
    from twisted.internet import reactor

    testCase()

    print("begin receive request")
    r = MioServer()
    reactor.listenTCP(8080, server.Site(r))
    reactor.run()