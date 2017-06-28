/*
* g++ -o test  test.cpp -I/usr/include/python3.4 -L/usr/lib/python3.4/config -lpython3.4m
*/
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "Python.h"
#include <numpy/arrayobject.h>
#include <iostream>
using namespace std;

// const int TOP_SHAPE_0 = 500;
// const int TOP_SHAPE_1 = 300;
// const int TOP_SHAPE_2 = 15;
// const int RGB_SHAPE_0 = 596;
// const int RGB_SHAPE_1 = 1368;
// const int RGB_SHAPE_2 = 3;

const int TOP_SHAPE_0 = 3;
const int TOP_SHAPE_1 = 3;
const int TOP_SHAPE_2 = 3;
const int RGB_SHAPE_0 = 3;
const int RGB_SHAPE_1 = 3;
const int RGB_SHAPE_2 = 3;

int init_numpy(){
    import_array();
}

float*** create3DArray(int nx, int ny, int nz) {
    float*** p = new float**[nx];
    for(int i = 0; i < nx; i ++) {
        p[i] = new float*[ny];
        for(int j = 0; j < ny; j ++) {
            p[i][j] = new float[nz];
            for(int k = 0; k < nz; k ++) {
                p[i][j][k] = (100.0*i + 10.0*j + k);
                //cout << p[i][j][k] << endl;
            }
        }
    }
    return p;
}

void free3DArray(float*** p, int nx, int ny) {
    for(int i = 0; i < nx; i ++) {
        for(int j = 0; j < ny; j ++)
            delete[] p[i][j];
        delete[] p[i];
    }
    delete[] p;
}

int main(int argc, char* argv[])
{
    Py_Initialize();

	if (!Py_IsInitialized())
	{
		return -1;
	}
	init_numpy();

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./')");

	PyObject *pModule = NULL;
	pModule = PyImport_ImportModule("predict");
	if (!pModule)
	{
		printf("not found .py file\n");
		return 0;
	}

	PyObject *pFunc = NULL;
	PyObject *pValue = NULL;
	PyObject *pArgs=NULL;
	//PyObject *pList=NULL;

	cout<< " call : "<<endl;
	pFunc = PyObject_GetAttrString(pModule, "predict_test");
	pArgs=PyTuple_New(2);

	float*** topArrays = create3DArray(TOP_SHAPE_0, TOP_SHAPE_1, TOP_SHAPE_2);
	float*** rgbArrays = create3DArray(RGB_SHAPE_0, RGB_SHAPE_1, RGB_SHAPE_2);

	cout << topArrays[1][1][1] << endl;

    npy_intp  topDims[3] = {TOP_SHAPE_0, TOP_SHAPE_1, TOP_SHAPE_2};
    npy_intp  rgbDims[3] = {RGB_SHAPE_0, RGB_SHAPE_1, RGB_SHAPE_2};

	cout << "init ok" << endl;

	PyObject *pa  = PyArray_SimpleNewFromData(3, topDims, NPY_FLOAT, topArrays);
	PyObject *pb  = PyArray_SimpleNewFromData(3, rgbDims, NPY_FLOAT, rgbArrays);

	PyTuple_SetItem(pArgs, 0, pa);
	PyTuple_SetItem(pArgs, 1, pb);

	cout << topArrays[1][1][1] << endl;

	pValue = PyObject_CallObject(pFunc, pArgs);
	cout<< " \n ====================\n result : "<<endl;

	cout << topArrays[1][1][1] << endl;

	free3DArray(topArrays, TOP_SHAPE_0, TOP_SHAPE_1);
	free3DArray(rgbArrays, RGB_SHAPE_0, RGB_SHAPE_1);

	Py_Finalize();

	return 0;
}