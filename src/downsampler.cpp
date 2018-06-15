#include "perception/downsampler.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace perception;

Downsampler::Downsampler() {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    ROS_INFO("The downsampler got %ld points.", cloud->size());

    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
    ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());
}