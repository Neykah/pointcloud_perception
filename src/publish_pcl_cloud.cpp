#include <iostream>
#include "ros/ros.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/publisher.h"
#include "pcl_conversions/pcl_conversions.h"
#include <string>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void print_usage() {
    ROS_INFO("Publish a saved pcd cloud as a pointcloud2 message.");
    ROS_INFO("Usage: rosrun perception publish_pcl_cloud pcl_pointcloud.");
}

int main(int argc, char** argv) {
    // Initialize ROS, check the call and provide help if needed.
    ros::init(argc, argv, "mock_pcd");
    ros::NodeHandle nh;
    if(argc < 2)
        print_usage();
    
    // Create a new PCL PointCloud, load the PCD file
    PointCloudC::Ptr cloud(new PointCloudC);
    if (pcl::io::loadPCDFile<PointC>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %ld \n", argv[1]);
        return(-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << argv[1] << std::endl;
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    msg_out.header.frame_id = "camera_depth_optical_frame";
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("mock_cloud", 1);

    while(ros::ok()) {
        msg_out.header.stamp = ros::Time::now();
        pub.publish(msg_out);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}