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

ros::Publisher cloud_pub;
ros::Publisher image_pub;

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

void filter_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pointcloud2_ptr){
  // Solve all of perception here...
    std::cout  << "come into filter_callback" << std::endl;
    static ros::Time last_time = ros::Time::now();
    static ros::Time current_time = ros::Time::now();
    
    current_time = ros::Time::now();
    ros::Duration dt = current_time - last_time;
    
    std::cout << "filter_callback rate = " << 1.0 / (dt.toSec()) << " hz" << std::endl;
    last_time = current_time;
    cloud_pub.publish(*pointcloud2_ptr);
    image_pub.publish(*image);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "image_pointcloud_sync");
    ros::NodeHandle nh;
    
//     image_transport::ImageTransport it(nh);
//     image_transport::Subscriber image_sub1 = it.subscribe("/image_raw", 1, imagecallback);
// //     ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/image_raw",1, imagecallback);
//     // Create a ROS subscriber for the input point cloud
//     ros::Subscriber pointcloud_sub1 = nh.subscribe ("/velodyne_points", 1, pointcloud_callback);
//     
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_pointcloud2", 1);
    image_pub = nh.advertise<sensor_msgs::Image>("test_image", 1);
    
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/velodyne_points", 1);
//     message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, pointcloud_sub, 10);
//     sync.registerCallback(boost::bind(&filter_callback, _1, _2));
 
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync_approxi(MySyncPolicy(10), image_sub, pointcloud_sub);
    sync_approxi.registerCallback(boost::bind(&filter_callback, _1, _2));
  
    ros::spin();
    
    return 0;
        
}


