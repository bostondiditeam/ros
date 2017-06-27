# Detector ROS Node

### 1.first
 
 run rpc predict serivice, which is use python 3.5
 
```sh
# must in python3.5 env
python predict_rpc.py <<tag>>
```

the tag is the tag for taining weightsi, which tag is your training tag

### 2. run ros node

run ros detecter node , which is use python 3.7

```buildoutcfg
# must in python2.7 env
rosrun detector pipeline.py
```

### 3. Play ROS bag
