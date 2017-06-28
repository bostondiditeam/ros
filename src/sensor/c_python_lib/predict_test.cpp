/*
* g++ -o test  test.cpp -I/usr/include/python3.4 -L/usr/lib/python3.4/config -lpython3.4m
*/
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "Python.h"
#include <numpy/arrayobject.h>
#include <iostream>
using namespace std;

int init_numpy(){
    import_array();
}

double*** create3DArray(int nx, int ny, int nz) {
    double*** p = new double**[nx];
    for(int i = 0; i < nx; i ++) {
        p[i] = new double*[ny];
        for(int j = 0; j < ny; j ++) {
            p[i][j] = new double[nz];
            for(int k = 0; k < nz; k ++) {
                p[i][j][k] = (100.0*i + 10.0*j + k);
                cout << p[i][j][k] << endl;
            }
        }
    }
    return p;
}

void free3DArray(double*** p, int nx, int ny) {
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
	pFunc = PyObject_GetAttrString(pModule, "predict");
	pArgs=PyTuple_New(2);

	double*** topArrays = create3DArray(3, 3, 3);
	double*** rgbArrays = create3DArray(3, 3, 3);

    npy_intp  topDims[4] = {3, 3, 3};
    npy_intp  rgbDims[4] = {3, 3, 3};

	cout << "init ok" << endl;

	PyObject *pa  = PyArray_SimpleNewFromData(3, topDims, NPY_DOUBLE, topArrays);
	PyObject *pb  = PyArray_SimpleNewFromData(3, rgbDims, NPY_DOUBLE, rgbArrays);

	PyTuple_SetItem(pArgs, 0, pa);
	PyTuple_SetItem(pArgs, 1, pb);

	free3DArray(topArrays, 3, 3);
	free3DArray(rgbArrays, 3, 3);

	pValue = PyObject_CallObject(pFunc, pArgs);
	cout<< " \n ====================\n result : "<<endl;

	Py_Finalize();

	return 0;
}
