/*
* g++ -o test  test.cpp -I/usr/include/python3.4 -L/usr/lib/python3.4/config -lpython3.4m
*/
#include "Python.h"
#include <numpy/arrayobject.h>
#include <iostream>
#include<iomanip>
using namespace std;

#define TOP_SHAPE_0 500
#define TOP_SHAPE_1 300
#define TOP_SHAPE_2 15
#define RGB_SHAPE_0 596
#define RGB_SHAPE_1 1368
#define RGB_SHAPE_2 3

struct TOP {
	float data[TOP_SHAPE_0][TOP_SHAPE_1][TOP_SHAPE_2];
	void add_data() {
		for(int i = 0; i < TOP_SHAPE_0; i ++) {
			for(int j = 0; j < TOP_SHAPE_1; j ++) {
				for(int k = 0; k < TOP_SHAPE_1; k ++) 
					data[i][j][k] = 10000*i + 100*j + k;
			}
		}
	}
};
struct RGB {
	float data[RGB_SHAPE_0][RGB_SHAPE_1][RGB_SHAPE_2];
	void add_data() {
		for(int i = 0; i < TOP_SHAPE_0; i ++) {
			for(int j = 0; j < TOP_SHAPE_1; j ++) {
				for(int k = 0; k < TOP_SHAPE_1; k ++) 
					data[i][j][k] = 10000*i + 100*j + k;
			}
		}
	}
};

int init_numpy(){
    import_array();
}

void printPyArray(PyObject *object){
	cout<<" ======== C++ printPyArray : " << endl;
	if (object == NULL){
		cout << "printPyArray error: object is NULL ."<< endl;
		return;		
	}
	
	int index_i = 0, index_m = 0, index_n = 0;
	
    if(PyArray_Check(object)){
		PyArrayObject *pAarray = (PyArrayObject *)object;  
		// ��ǰά��
        int nd = pAarray -> nd;   

		// print shape 
		for(int i = 0; i < nd; i ++){	
			int count = pAarray -> dimensions[i];
			if(i == 0){
				cout << "(";
				cout<< count ;
				if(nd == 1){
					cout<<  " )" << endl;
				}else {
					cout << ", ";
				}
			}else if (i == nd -1){
				cout<< count;
				cout << ")" << endl;
			}else {
				cout<< count << ", " ;
			}
		}
			
		// ��3άnumpy����Ϊ�����			
		int x = pAarray -> dimensions[0];
		int	y = pAarray->dimensions[1];
		int	z = pAarray->dimensions[2];		
		/**		
		*	int nd��Numpy Array�����ά�ȡ�
		*	int *dimensions ��Numpy Array ����ÿһά�����ݵĸ�����
		*	int *strides��Numpy Array ����ÿһά�ȵĲ�����
		*	char *data�� Numpy Array ��ָ�����ݵ�ͷָ�롣		
		*/
		cout << endl << endl <<"[";
		for(int j = 0; j < x; j ++ ){
			cout <<"[";
			for (int k = 0; k < y; k++){
				cout << "[";
				for(int h = 0; h < z; h ++){
					cout<< setprecision(8) <<*(double *)(pAarray->data + j * pAarray->strides[0] + 
					k * pAarray->strides[1] + h * pAarray->strides[2] )<<" ";
					if( h != z - 1){
						cout <<", ";
					}
				}
			
				cout << "]";	
				if(k != y -1 ){
					cout << endl;
				}
			}			
			cout << "]" ;
			if(j != x -1 ){
				cout << endl << endl;
			}			
		}
		cout << "] "<< endl<< endl;		
    }else{
        cout<<"Not a PyArrayObject"<<endl;
    }
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
	pFunc = PyObject_GetAttrString(pModule, "predict_func");
	pArgs=PyTuple_New(2);

	TOP *top = new TOP;
	//top->add_data();
	RGB *rgb = new RGB;
	//rgb->add_data();

    npy_intp  topDims[3] = {TOP_SHAPE_0, TOP_SHAPE_1, TOP_SHAPE_2};
    npy_intp  rgbDims[3] = {RGB_SHAPE_0, RGB_SHAPE_1, RGB_SHAPE_2};

	cout << "init ok" << endl;

	PyObject *pa  = PyArray_SimpleNewFromData(3, topDims, NPY_FLOAT, (void*)top->data);
	PyObject *pb  = PyArray_SimpleNewFromData(3, rgbDims, NPY_FLOAT, (void*)rgb->data);

	PyTuple_SetItem(pArgs, 0, pa);
	PyTuple_SetItem(pArgs, 1, pb);

	pValue = PyObject_CallObject(pFunc, pArgs);
	cout<< " \n ====================\n result : "<<endl;

	printPyArray(pValue);

	Py_Finalize();

	delete top;
	delete rgb;

	return 0;
}