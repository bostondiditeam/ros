#ifndef _UTILS_HPP
#define _UTILS_HPP


#include <iostream>
#include<iomanip>

#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> 
#include <string.h>
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <iomanip>  
#include <vector>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <fstream>    
#include <string>
#include <vector>

// rpc c++ version
#include "Python.h"
#include <numpy/arrayobject.h>
#include <iostream>
#include<iomanip>

#include "visualization_msgs/MarkerArray.h"



using namespace std;

#define TOP_SHAPE_0 500
#define TOP_SHAPE_1 300
#define TOP_SHAPE_2 15
#define RGB_SHAPE_0 596
#define RGB_SHAPE_1 1368
#define RGB_SHAPE_2 3

typedef struct{
    float data[TOP_SHAPE_0][TOP_SHAPE_1][TOP_SHAPE_2];
    void add_data() {
        for(int i = 0; i < TOP_SHAPE_0; i ++) {
            for(int j = 0; j < TOP_SHAPE_1; j ++) {
                for(int k = 0; k < TOP_SHAPE_1; k ++) 
                    data[i][j][k] = 10000*i + 100*j + k;
            }
        }
    }
}TOP;
typedef struct{
    float data[RGB_SHAPE_0][RGB_SHAPE_1][RGB_SHAPE_2];
    void add_data() {
        for(int i = 0; i < TOP_SHAPE_0; i ++) {
            for(int j = 0; j < TOP_SHAPE_1; j ++) {
                for(int k = 0; k < TOP_SHAPE_1; k ++) 
                    data[i][j][k] = 10000*i + 100*j + k;
            }
        }
    }
}RGB;

typedef struct PointT {
    float x;
    float y;
    float z;
    float intensity;
} PointT;

// def clidar_to_top(lidar):
//     # Calculate map size and pack parameters for top view and front view map (DON'T CHANGE THIS !)
//     Xn = int((TOP_X_MAX - TOP_X_MIN) / TOP_X_DIVISION)
//     Yn = int((TOP_Y_MAX - TOP_Y_MIN) / TOP_Y_DIVISION)
//     Zn = int((TOP_Z_MAX - TOP_Z_MIN) / TOP_Z_DIVISION)
// 
//     top_flip = np.ones((Xn, Yn, Zn + 2), dtype=np.float32)  # DON'T CHANGE THIS !
// 
//     num = lidar.shape[0]  # DON'T CHANGE THIS !
// 
//     # call the C function to create top view maps
//     # The np array indata will be edited by createTopViewMaps to populate it with the 8 top view maps
//     SharedLib.createTopMaps(ctypes.c_void_p(lidar.ctypes.data),
//                             ctypes.c_int(num),
//                             ctypes.c_void_p(top_flip.ctypes.data),
//                             ctypes.c_float(TOP_X_MIN), ctypes.c_float(TOP_X_MAX),
//                             ctypes.c_float(TOP_Y_MIN), ctypes.c_float(TOP_Y_MAX),
//                             ctypes.c_float(TOP_Z_MIN), ctypes.c_float(TOP_Z_MAX),
//                             ctypes.c_float(TOP_X_DIVISION), ctypes.c_float(TOP_Y_DIVISION),
//                             ctypes.c_float(TOP_Z_DIVISION),
//                             ctypes.c_int(Xn), ctypes.c_int(Yn), ctypes.c_int(Zn)
//                             )
//     top = np.flipud(np.fliplr(top_flip))
//     return top

void createTopMaps(const void * raw_data, int num, const void * top_data, 
                   float x_MIN, float x_MAX, float y_MIN, 
                   float y_MAX, float z_MIN, float z_MAX, 
                   float x_DIVISION, float y_DIVISION, float z_DIVISION, 
                   int X_SIZE, int Y_SIZE, int Z_SIZE);



void init_numpy();
void printPyArray(PyObject *object);
/**
 * to be singleton
 * to do
 */
class RPCPredictor{
public:
    RPCPredictor();
    ~RPCPredictor();
    
    void predict(void * top_data, void * rgb_data);
    visualization_msgs::MarkerArray getmarkers();
    
private:
    PyObject *pFunc = NULL;
    PyObject *pValue = NULL;
    PyObject *pArgs=NULL;
    
    npy_intp  topDims[3];
    npy_intp  rgbDims[3];
//     bool new_marker;
    //PyObject *pList=NULL;
};

#endif