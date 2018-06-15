#include "perception/crop.h"
#include "perception/downsampler.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_demo");
    ros::NodeHandle nh;
    ros::Publisher table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    perception::Segmenter segmenter(table_pub);
    ros::Subscriber sub = nh.subscribe("cloud_in",1, &perception::Segmenter::Callback, &segmenter);
    ros::spin();
    return 0;
}