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


ros::Publisher cloud_pub;
ros::Publisher image_pub;
ros::Publisher marker_pub;

/**
 * to do
 * how 
 * 
 */
void point_cloud_2_top(const sensor_msgs::PointCloud2ConstPtr& point_ptr,
                       cv::Mat & output_top_image,
                       double res = 0.1, 
                       double zres = 0.3, 
                       double left_most = -10,
                       double right_most = 10,
                       double back_most = -10,
                       double front_most = 10,
                       double bottom_most = -2,
                       double upper_most = 2
){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*point_ptr, cloud);
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(&cloud);
    
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

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    viewer.addCoordinateSystem (1.0, "original_cloud", 0);
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
    
    cv::Mat top;
    point_cloud_2_top(pointcloud2_ptr, top);
    
    
    cloud_pub.publish(*pointcloud2_ptr);
    image_pub.publish(*image_ptr);
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




