// Ce programme est en fait inutile pour rtabmap car on peut enregistrer la cloud base_link générée par RTABMap automatiquement en 
// utilisant l'app externe.
// Il sert toufefois a enregistrer un pointcloud sous la forme d'un bag directement depuis le rostopic points.


#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"

void print_usage() {
    std::cout << "Saves a point cloud on kinect2/qhd/points to "
              << "NAME.bag in the current directory."
              << std::endl;
    std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_cloud_main");
    if (argc < 2) {
        print_usage();
        return 1;
    }

    std::string name(argv[1]);
    std::cout << "Hello, " << name << std::endl;

    sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "kinect2/qhd/points");
    
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base_footprint", cloud->header.frame_id, ros::Time(0), ros::Duration(5.0)); // map or base_link
    tf::StampedTransform transform;
    try {
        tf_listener.lookupTransform("base_footprint", cloud->header.frame_id, ros::Time(0), transform);
    } catch (tf::LookupException& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    } catch (tf::ExtrapolationException& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud("base_footprint", transform, *cloud, cloud_out);

    std::string filename(name + ".bag");
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);
    bag.write("kinect2/qhd/points", ros::Time::now(), cloud_out);
    bag.close();


    return 0;
}