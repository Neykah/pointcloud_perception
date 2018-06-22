#include "perception/crop.h"
#include "perception/downsampler.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

// Object recognition
#include <vector>
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_demo");

    if (argc < 2)
    {
        ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
        ros::spinOnce();
    }
    std::string data_dir(argv[1]);
    ros::NodeHandle nh;

    // Crop the background
    ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber crop_sub = nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

    // Create the object recognizer
    std::vector<perception_msgs::ObjectFeatures> dataset;
    perception::LoadData(data_dir, &dataset);
    perception::ObjectRecognizer recognizer(dataset);

    // Segment the planes
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualisation_marker", 1, true);
    perception::Segmenter segmenter(marker_pub, recognizer);
    ros::Subscriber sub = nh.subscribe("cropped_cloud",1, &perception::Segmenter::Callback, &segmenter);
    ros::spin();
    return 0;
}