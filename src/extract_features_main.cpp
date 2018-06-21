#include <iostream>
#include <string>
#include <vector>

#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

#include "perception/feature_extraction.h"
#include "perception/object.h"
#include "perception/segmentation.h"
#include "perception/typedefs.h"
#include "perception_msgs/ObjectFeatures.h"

void Crop(PointCloudC::Ptr cloud_in, PointCloudC::Ptr cloud_out)
{
    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param("crop_min_x", min_x, -1.0);
    ros::param::param("crop_min_y", min_y, -1.0);
    ros::param::param("crop_min_z", min_z, 0.0);
    ros::param::param("crop_max_x", max_x, 1.05);
    ros::param::param("crop_max_y", max_y, 1.0);
    ros::param::param("crop_max_z", max_z, 1.2);
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);

    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud_in);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cloud_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extract_features");
    ros::Time::init();
    if (argc < 3)
    {
        std::cout << "Extract features from a bag file with a point cloud with a "
                     "single object on a table. The features are saved to LABEL.bag"
                  << std::endl;
        std::cout << "Usage: rosrun perception extract_features FILE.bag LABEL"
                  << std::endl;
        return 0;
    }
    std::string path(argv[1]);
    std::string label(argv[2]);

    rosbag::Bag bag;
    bag.open(path, rosbag::BagMode::Read);
    std::vector<std::string> topics;
    std::string cloud_topic("head_camera/depth_registered/points"); // Name of the pointcloud topic published by the bag file
    topics.push_back(cloud_topic); 
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    if (view.size() == 0)
    {
        std::cerr << "No message on topic " << cloud_topic
                  << std::endl;
    }
    
    PointCloudC::Ptr pcl_cloud(new PointCloudC());
    for (rosbag::View::const_iterator it=view.begin() ; it != view.end() ; it++)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud = 
            it->instantiate<sensor_msgs::PointCloud2>();
        if (cloud == NULL)
        {
            std::cerr << "Unable to instantiate point cloud." << std::endl;
            return 1;
        }
        pcl::fromROSMsg(*cloud, *pcl_cloud);
        break;
    }

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    Crop(pcl_cloud, cropped_cloud);

    std::vector<perception::Object> objects;
    perception::SegmentTabletopScene(cropped_cloud, &objects);
    if (objects.size() != 1)
    {
        std::cerr << "Expected to see exactly one object, found " << objects.size()
                  << std::endl;
        return 1;
    }

    const perception::Object& object = objects[0];
    perception_msgs::ObjectFeatures features;
    features.object_name = label;
    perception::ExtractSizeFeatures(object, &features);

    rosbag::Bag bag_out;
    bag_out.open(label + "_label.bag", rosbag::bagmode::Write);
    bag_out.write("object_features", ros::Time::now(), features);
    bag_out.close();

    return 0;
}