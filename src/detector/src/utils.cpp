#include "detector/utils.hpp"


void createTopMaps(const void * raw_data, int num, const void * top_data, float x_MIN, float x_MAX, float y_MIN, float y_MAX, float z_MIN, float z_MAX, float x_DIVISION, float y_DIVISION, float z_DIVISION, int X_SIZE, int Y_SIZE, int Z_SIZE)
{       
    float * raw_cube = (float *) raw_data;
    float * top_cube = (float *) top_data;
//      std::cout<<X_SIZE<<","<<Y_SIZE<<","<<Z_SIZE<<std::endl;

    //int32_t num = 123586;
    float *px = raw_cube+0;
    float *py = raw_cube+1;
    float *pz = raw_cube+2;
    float *pr = raw_cube+3;
    
    //3D grid box index
    int X = 0;  
    int Y = 0;
    int Z = 0;

    //height features X_SIZE * Y_SIZE * Z_SIZE
    vector<vector<vector<float> > > height_maps;  // quantized 

    //max height X_SIZE * Y_SIZE * 1  (used to record current highest cell for X,Y while parsing data)
    vector<vector<float> > max_height_map;

    //density feature X_SIZE * Y_SIZE * 1
    vector<vector<float> > density_map;

    //intensity feature X_SIZE * Y_SIZE * 1
    vector<vector<float > > intensity_map;

    height_maps.resize(X_SIZE);
    max_height_map.resize(X_SIZE);
    density_map.resize(X_SIZE);
    intensity_map.resize(X_SIZE);

    for (int i=0; i<X_SIZE; i++)
    {
        height_maps[i].resize(Y_SIZE);
        max_height_map[i].resize(Y_SIZE);
        density_map[i].resize(Y_SIZE);
        intensity_map[i].resize(Y_SIZE);

        for (int j=0; j<Y_SIZE; j++)
        {
            height_maps[i][j].resize(Z_SIZE);
            
            //initialization for height_maps
            for (int k=0; k<Z_SIZE; k++)
                height_maps[i][j][k] = 0;   //value stored inside always >= 0 (relative to z_MIN, unit : m)
        
            //initialization for density_map, max_height_map, intensity_map
            density_map[i][j] = 0;  //value stored inside always >= 0, usually < 1 ( log(count#+1)/log(64), no unit )
            max_height_map[i][j] = 0;   //value stored inside always >= 0  (relative to z_MIN, unit : m)
            intensity_map[i][j] = 0;    //value stored inside always >= 0 && <=255 (range=0~255, no unit)
        }
    }

    //allocate point cloud for temporally data visualization (only used for validation)
    vector<std::vector<PointT> > height_cloud_vec;

    // Initialize the vector of vectors with a dummy. The dummy is later deleted.
    for(int i = 0; i < Z_SIZE; i++){
        std::vector<PointT> dummy;
        PointT dummy_member;
        dummy.push_back(dummy_member);
        height_cloud_vec.push_back(dummy);
    }

    vector<PointT> intensity_cloud;
    vector<PointT> density_cloud;

    for (int32_t i=0; i<num; i++) {
        PointT point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.intensity = (*pr);// * 255;   //TODO : check if original Kitti data normalized between 0 and 1 ?
        
        X = (int)((point.x-x_MIN)/x_DIVISION);  // qxs in Python version  
        Y = (int)((point.y-y_MIN)/y_DIVISION);  // qys in Python version  
        Z = (int)((point.z-z_MIN)/z_DIVISION);  // qzs in Python version
        
        //For every point in each cloud, only select points inside a predefined 3D grid box
        if (X >= 1 && Y >= 1 && Z >= 0 && X < X_SIZE && Y < Y_SIZE && Z < Z_SIZE)  // 1 used for X,Y offset compensation
        {
            // For every point in predefined 3D grid box.....
            // height map
            if ((point.z - z_MIN) > height_maps[X][Y][Z])
            {   
                height_maps[X][Y][Z] = point.z - z_MIN;
                
                //Save to point cloud for visualization -----               
                PointT grid_point;
                grid_point.x = X-1;   //-1 used for offset compensation between C and Python version
                grid_point.y = Y-1;   //-1 used for offset compensation between C and Python version
                grid_point.z = 0;
                grid_point.intensity = point.z - z_MIN;
                
                // height_cloud_vec[Z].push_back(grid_point);
                height_cloud_vec.at(Z).push_back(grid_point);
            }
        
            // density map
            density_map[X][Y]++;    // update count#, need to be normalized afterwards
            PointT grid_point;
            grid_point.x = X-1;   //-1 used for offset compensation between C and Python version
            grid_point.y = Y-1;   //-1 used for offset compensation between C and Python version    
            grid_point.z = 0;
            grid_point.intensity = density_map[X][Y];
            density_cloud.push_back(grid_point);

            // intensity map
            if ((point.z - z_MIN) > max_height_map[X][Y])
            {
                max_height_map[X][Y] = point.z - z_MIN;
                intensity_map[X][Y] = point.intensity;
                //Save to point cloud for visualization -----
                PointT grid_point;
                grid_point.x = X-1;   //-1 used for offset compensation between C and Python version
                grid_point.y = Y-1;   //-1 used for offset compensation between C and Python version
                grid_point.z = 0;
                grid_point.intensity = point.intensity;
                intensity_cloud.push_back(grid_point);
            }
        }
        px+=4; py+=4; pz+=4; pr+=4;
    }       



    // erase the dummy vector added at the start
    for(int i = 0; i < Z_SIZE; i++){
        height_cloud_vec.at(i).erase(height_cloud_vec.at(i).begin());
    }

    // normalization
    for (unsigned int i=0; i < density_cloud.size(); i++){
//          float val = log10(density_cloud.at(i).intensity+1)/log10(64);
        float val = log(density_cloud.at(i).intensity+1)/log(32);   //In DiDi, we use velodyne 32 beam

        if(val < 1) 
            density_cloud.at(i).intensity = val;
        else
            density_cloud.at(i).intensity = 1;
    }

    int top_cube_size = X_SIZE * Y_SIZE * (Z_SIZE + 2);
    // float top_cube[top_cube_size];   // dimensions 400x400x8

    // initialize the top cube to be all zeros
    for(int i = 0; i < top_cube_size; i++){
        top_cube[i] = 0;
    }
    
    // Treat the height maps first
    vector<PointT> *cloud_demo;

    for (int k = 0; k<Z_SIZE; k++){
        // *cloud_demo = height_cloud_vec[k];
        cloud_demo = &height_cloud_vec.at(k);
        int cloud_size = cloud_demo->size();
        int plane_idx = k;  // because this is the kth plane (starting from zero)

        for(int i = 0; i < cloud_size; i++){                
            int x_coord = (int)cloud_demo->at(i).x;
            int y_coord = (int)cloud_demo->at(i).y;

            // array_idx equals 400*8*y + 8*x + plane_idx where plane_idx goes from 0 to 5 (for the 6 height maps)
            // The formula for array_idx below was empirically calculated to fit the exact order in which the elements
            // of a 3D array are stored in memory. Elements are read starting with the top left corner of the bottom plane
            // the 3D array, then moving up in the z direction, then across (x) and then below (y)
            int array_idx = Y_SIZE * (Z_SIZE + 2) * x_coord + (Z_SIZE + 2) * y_coord + plane_idx;

            float value = cloud_demo->at(i).intensity;

            //top_cube[array_idx] = value;
            top_cube[array_idx] = value/z_DIVISION - k; //normalized relative to the current grid (within 0~1)

        }
    }
    
    // Treat the intensity map
    cloud_demo = &intensity_cloud;
    int cloud_size = cloud_demo->size();
    int plane_idx = Z_SIZE; // because this is the 6th plane (starting from zero)

    for(int i = 0; i < cloud_size; i++){
        int x_coord = (int)cloud_demo->at(i).x;
        int y_coord = (int)cloud_demo->at(i).y;

        // array_idx equals 400*8*x + 8*y + 6 where 6 is the 6th (starting at 0) 400x400 plane
        int array_idx = Y_SIZE * (Z_SIZE + 2) * x_coord + (Z_SIZE + 2) * y_coord + plane_idx;
        float value = cloud_demo->at(i).intensity;

        top_cube[array_idx] = value;
    }

    // Treat the density map    (last layer)
    cloud_demo = &density_cloud;
    cloud_size = cloud_demo->size();
    plane_idx = Z_SIZE + 1; // because this is the 7th plane (starting from zero)

    for(int i = 0; i < cloud_size; i++){
        int x_coord = (int)cloud_demo->at(i).x;
        int y_coord = (int)cloud_demo->at(i).y;

        // array_idx equals 400*8*x + 8*y + 7 where 7 is the 7th (starting at 0) 400x400 plane
        int array_idx = Y_SIZE * (Z_SIZE + 2) * x_coord + (Z_SIZE + 2) * y_coord + plane_idx;
        float value = cloud_demo->at(i).intensity;

        top_cube[array_idx] = value;
    }
    
    // cout << "C++ part exited. LiDAR data pre-processing completed" << endl;
}

void init_numpy(){
    //https://stackoverflow.com/questions/32899621/numpy-capi-error-with-import-array-when-compiling-multiple-modules
    if(PyArray_API == NULL)
    {
        import_array(); 
    }
//     import_array();
//     return 0;
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
                        
        int x = pAarray -> dimensions[0];
        int y = pAarray->dimensions[1];
        int z = pAarray->dimensions[2];     

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

RPCPredictor::RPCPredictor()
{
    std::cout << "11111111111"  <<std::endl;
    Py_Initialize();
    std::cout << "22222222222"  <<std::endl;
    
    if (!Py_IsInitialized())
    {
        return ;
    }
    std::cout << "333333333333333333"  <<std::endl;
    init_numpy();
    std::cout << "4444444444444"  <<std::endl;
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/fangli/didi/development1/ros/src/detector/src/')");
    PyRun_SimpleString("sys.path.append('./')");
    std::cout << "555555555555555555"  <<std::endl;
    PyObject *pModule = NULL;
    pModule = PyImport_ImportModule("predict");
    if (!pModule)
    {
        printf("not found .py file\n");
        return ;
    }

    pFunc = NULL;
    pValue = NULL;
    pArgs=NULL;
    //PyObject *pList=NULL;

    cout<< " call : "<<endl;
    pFunc = PyObject_GetAttrString(pModule, "predict_func");
    pArgs=PyTuple_New(2);

    topDims[0] = TOP_SHAPE_0;
    topDims[1] = TOP_SHAPE_1;
    topDims[2] = TOP_SHAPE_2;
    
    rgbDims[0] = RGB_SHAPE_0;
    rgbDims[1] = RGB_SHAPE_1;
    rgbDims[2] = RGB_SHAPE_2;
    
//     topDims = {TOP_SHAPE_0, TOP_SHAPE_1, TOP_SHAPE_2};
//     rgbDims = {RGB_SHAPE_0, RGB_SHAPE_1, RGB_SHAPE_2};

    cout << "init ok" << endl;

}


void RPCPredictor::predict(void * top_data, void * rgb_data)
{
    std::cout << " come into predict" << std::endl;
    PyObject *pa  = PyArray_SimpleNewFromData(3, topDims, NPY_FLOAT, (void*)top_data);
    PyObject *pb  = PyArray_SimpleNewFromData(3, rgbDims, NPY_FLOAT, (void*)rgb_data);

    PyTuple_SetItem(pArgs, 0, pa);
    PyTuple_SetItem(pArgs, 1, pb);

    pValue = PyObject_CallObject(pFunc, pArgs);
    cout<< " \n ====================\n result : "<<endl;

    printPyArray(pValue);
}

RPCPredictor::~RPCPredictor()
{
    Py_Finalize();

}
// def boxes3d_decompose(boxes3d):
// 
//     # translation
//     if cfg.DATA_SETS_TYPE == 'didi2' or cfg.DATA_SETS_TYPE == 'didi' or cfg.DATA_SETS_TYPE == 'test':
//         T_x = np.sum(boxes3d[:, 0:8, 0], 1) / 8.0
//         T_y = np.sum(boxes3d[:, 0:8, 1], 1) / 8.0
//         T_z = np.sum(boxes3d[:, 0:8, 2], 1) / 8.0
//     elif cfg.DATA_SETS_TYPE == 'kitti':
//         T_x = np.sum(boxes3d[:, 0:4, 0], 1) / 4.0
//         T_y = np.sum(boxes3d[:, 0:4, 1], 1) / 4.0
//         T_z = np.sum(boxes3d[:, 0:4, 2], 1) / 4.0
// 
//     Points0 = boxes3d[:, 0, 0:2]
//     Points1 = boxes3d[:, 1, 0:2]
//     Points2 = boxes3d[:, 2, 0:2]
// 
//     dis1=np.sum((Points0-Points1)**2,1)**0.5
//     dis2=np.sum((Points1-Points2)**2,1)**0.5
// 
//     dis1_is_max=dis1>dis2
// 
//     #length width heigth
//     L=np.maximum(dis1,dis2)
//     W=np.minimum(dis1,dis2)
//     H=np.sum((boxes3d[:,0,0:3]-boxes3d[:,4,0:3])**2,1)**0.5
// 
//     # rotation
//     yaw=lambda p1,p2,dis: math.atan2(p2[1]-p1[1],p2[0]-p1[0])
//     R_x = np.zeros(len(boxes3d))
//     R_y = np.zeros(len(boxes3d))
//     R_z = [yaw(Points0[i],Points1[i],dis1[i]) if is_max else  yaw(Points1[i],Points2[i],dis2[i])
//            for is_max,i in zip(dis1_is_max,range(len(dis1_is_max)))]
//     R_z=np.array(R_z)
// 
//     translation = np.c_[T_x,T_y,T_z]
//     size = np.c_[H,W,L]
//     rotation= np.c_[R_x,R_y,R_z]
//     return translation,size,rotation
/**
 * need liufeng's help
 * 
 */
visualization_msgs::MarkerArray RPCPredictor::getmarkers()
{
    visualization_msgs::MarkerArray marker;
    cout<<" ======== C++ printPyArray : " << endl;
    PyObject * object = pValue;
    if (object == NULL){
        cout << "printPyArray error: object is NULL ."<< endl;
        return marker;     
    }
    
    int index_i = 0, index_m = 0, index_n = 0;
    
    
    if(PyArray_Check(object)){
        PyArrayObject *pAarray = (PyArrayObject *)object;  
        int nd = pAarray->nd;   

        marker.markers.resize(nd);
        // print shape 
        // to do , fill the marker
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
                        
        int x = pAarray->dimensions[0];
        int y = pAarray->dimensions[1];
        int z = pAarray->dimensions[2];     

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
    
    return marker;
}


