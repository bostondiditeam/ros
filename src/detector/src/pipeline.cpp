#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/time.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include "detector/utils.hpp"

ros::Publisher cloud_pub;
ros::Publisher image_pub;
ros::Publisher marker_pub;

RPCPredictor g_rpc_predictor;

/**
 * to do
 * how 
 * 
 */
bool point_cloud_2_top(const sensor_msgs::PointCloud2ConstPtr& point_ptr,
//                        cv::Mat & output_top_image,
                       float * top_image,
                       int top_image_lenght,
                       double res = 0.1, 
                       double zres = 0.3, 
                       double left_most = -10,
                       double right_most = 10,
                       double back_most = -10,
                       double front_most = 10,
                       double bottom_most = -2,
                       double upper_most = 2
){
    
    
    
    // top image x, y, z
    // left right -> x
    // front back -> y
    int x_max = (right_most - left_most) / res + 1;
    int y_max = (front_most - back_most) / res + 1;
    int z_max = (upper_most - bottom_most) / res + 1;
    
    // only work with gcc
    int total_len = (x_max ) * (y_max  ) * (z_max );
    
    if( total_len != top_image_lenght){
        std::cout << "total_len != top_image_lenght" <<std::endl;
        return false;
    }
    
    for(int i=0; i<total_len; i++){
        top_image[i] = 0;
    }
    
    
/*    
    float top_image2[x_max + 1][y_max + 1][z_max + 1];
    
    for(int i=0; i<= x_max; i++){
        for(int j=0; j<=y_max; j++){
            for(int k = 0; k<=z_max; k++){
                
            }
        }
    }*/

    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*point_ptr, cloud);
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(&cloud);
    
    std::cout  << "before pass cloud.point.size = " << cloud.points.size() <<std::endl;
    
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (source_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (bottom_most, upper_most);
    pass.filter (*source_cloud);
    
    pass.setFilterFieldName("y");;
    pass.setFilterLimits(left_most, right_most);
    pass.filter (*source_cloud);
    
    pass.setFilterFieldName("x");
    pass.setFilterLimits(back_most, front_most);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*source_cloud);
    
    int after_pass_size = cloud.points.size();
    
    int x = 0;
    int y = 0;
    int z = 0;
    int intensify = 0;
    pcl::PointXYZI *point = NULL;
    
    // here is differ from 
    for(int i=0; i<after_pass_size; i++){
        point = &cloud.points[i];
        
        int x_image = (point->y - left_most) / res;
        int y_image = (point->x - back_most) / res;
        int z_image = (point->z - bottom_most) / zres;
    }
    
    std::cout  << "after pass cloud.point.size = " << cloud.points.size() <<std::endl;

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

//     viewer.addCoordinateSystem (1.0, "original_cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
}
void draw_top_image();
void msg_to_arr();

void imagecallback(const sensor_msgs::ImageConstPtr & msg_ptr){
    std::cout <<  "come into imagecallback " << std::endl;
    if(msg_ptr->data.empty()){
        std::cout << "msg_ptr->data is null" << std::endl;
        return ;
    }
    
    cv::imshow("src", cv_bridge::toCvShare(msg_ptr, "mono8")->image );
    cv::waitKey(1);
}

void pointcloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    std::cout <<  "come into pointcloud_callback " << std::endl;
  // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    output = *input;
  // Publish the data.
    cloud_pub.publish (output);
}

void sync_callback(const sensor_msgs::ImageConstPtr& image_ptr, 
                   const sensor_msgs::PointCloud2ConstPtr& pointcloud2_ptr){

    ros::Time func_start = ros::Time::now();
  // Solve all of perception here...
    int timestamp1 = image_ptr->header.stamp.toNSec();
    printf("image_callback: msg : seq=%d, timestamp=%19d\n", image_ptr->header.seq, timestamp1);
    
    int timestamp2 = pointcloud2_ptr->header.stamp.toNSec();
    printf("velodyne_callback: msg : seq=%d, timestamp=%19d\n", pointcloud2_ptr->header.seq, timestamp2);
    
//     arr = msg_to_arr(msg2)
//     lidar = np.array([[item[0], item[1], item[2], item[3]] for item in arr])
    
    cv::Mat camera_image = cv_bridge::toCvCopy(image_ptr, "bgr8")->image;
    printf("camera_image width = %d, height = %d\n", camera_image.cols, camera_image.rows);

    
    double res = 0.1;
    double zres = 0.3; 
    double left_most = -10;
    double right_most = 10;
    double back_most = -10;
    double front_most = 10;
    double bottom_most = -2;
    double upper_most = 2;
    int x_max = (right_most - left_most) / res ;
    int y_max = (front_most - back_most) / res ;
    int z_max = (upper_most - bottom_most) / zres ;
//     x_max = 500;
//     y_max = 300;
//     z_max = 15;
    int total_len = (x_max ) * (y_max )  * (z_max + 2);
    float * top_image = new float[total_len];
    
    
    // convert pcl_msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*pointcloud2_ptr, cloud);
    float float_cloud_data[cloud.points.size()][4];
    for(int i=0; i<cloud.points.size(); i++){
        pcl::PointXYZI * pt = &(cloud.points[i]);
        float_cloud_data[i][0] = pt->x;
        float_cloud_data[i][1] = pt->y;
        float_cloud_data[i][2] = pt->z;
        float_cloud_data[i][3] = pt->intensity;
    }
    // have to be the same with the config.py
    createTopMaps((void*)float_cloud_data, 
                  cloud.points.size(),
                  (void*)top_image, 
                  -10,
                  10,
                  -10, 
                  10,
                  -2, 
                  2,
                  0.1,
                  0.1,
                  0.3,
                  x_max,
                  y_max,
                  z_max
                  
    );
//     memset(top_image)
    
//     point_cloud_2_top(pointcloud2_ptr, top_image, total_len);
    
    cv::Mat cv_rgb_image = cv_bridge::toCvCopy(image_ptr, "rgb8")->image;
    //　not include right bottom point
    cv::Rect roi(cv::Point(0, 400), cv::Point(cv_rgb_image.cols, cv_rgb_image.rows - 100));

    
    std::cout << "before crop, cv_rgb_image.size = " << cv_rgb_image.size() << std::endl;
    cv_rgb_image = cv_rgb_image(roi);
    std::cout << "after crop, cv_rgb_image.size = " << cv_rgb_image.size() << std::endl;

    float * rgb_image = new float[cv_rgb_image.rows * cv_rgb_image.cols * 3];
    int rgb_counter = 0;
    for(int i=0; i<cv_rgb_image.rows; i++){
        for(int j=0; j<cv_rgb_image.cols; j++){
            rgb_image[rgb_counter++] = cv_rgb_image.at<cv::Vec3b>(i, j)[0];
            rgb_image[rgb_counter++] = cv_rgb_image.at<cv::Vec3b>(i, j)[1];
            rgb_image[rgb_counter++] = cv_rgb_image.at<cv::Vec3b>(i, j)[2];
        }
    }
    
    
   
//    g_rpc_predictor.predict(top_image, rgb_image);
   visualization_msgs::MarkerArray markers = g_rpc_predictor.getmarkers();
   if(markers.markers.size() > 0){
        marker_pub.publish(markers);
   }
    
    delete [] rgb_image;
    delete [] top_image;
    
    ros::Duration dt = ros::Time::now() - func_start;
    std::cout << "sync dt = " << dt.toSec() << std::endl << std::endl;
//     cloud_pub.publish(*pointcloud2_ptr);
//     image_pub.publish(*image_ptr);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "detect_cpp_node");
    ros::NodeHandle nh;
    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("bbox", 1);
    
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_pointcloud2", 1);
    image_pub = nh.advertise<sensor_msgs::Image>("test_image", 1);
    
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/velodyne_points", 1);

 
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync_approxi(MySyncPolicy(10), image_sub, pointcloud_sub);
    sync_approxi.registerCallback(boost::bind(&sync_callback, _1, _2));
  
    printf("detecter node initialzed\n");
    
    ros::spin();
    
    return 0;
        
}




