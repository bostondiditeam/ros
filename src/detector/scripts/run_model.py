import os
import argparse
import subprocess


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='tracking')
    parser.add_argument('-m', '--model', type=str, nargs='?', default=None,
                        help='set log tag')
    args = parser.parse_args()

    if args.model ==None:
        print('Please input model name')

    command = 'cp ../MV3D/src/config_%s.py ../MV3D/src/config.py -f' % args.model
    ret = subprocess.call(command,shell=True)
    print(command)
    if ret != 0:
        print('run: "{}" fail'.format(command))
        exit(-1)

    command = 'python predict_rpc.py'
    ret = subprocess.call(command,shell=True)
    if ret != 0:
        print('run: "{}" fail'.format(command))
        exit(-1)

