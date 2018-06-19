#include "perception/crop.h"
#include "perception/downsampler.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_demo");
    ros::NodeHandle nh;

    // Crop the background
    ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber crop_sub = nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

    // Segment the planes
    ros::Publisher table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    ros::Publisher objects_pub = nh.advertise<sensor_msgs::PointCloud2>("objects_cloud", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualisation_marker", 1, true);
    perception::Segmenter segmenter(table_pub, objects_pub, marker_pub);
    ros::Subscriber sub = nh.subscribe("cropped_cloud",1, &perception::Segmenter::Callback, &segmenter);
    ros::spin();
    return 0;
}