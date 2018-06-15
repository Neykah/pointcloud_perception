#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// RANSAC Segmentation
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

// PointCloud reification
#include "pcl/filters/extract_indices.h"

#include <iostream>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception 
{
    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) 
    {
        std::cout << "Input cloud has " << cloud->size() << " points";
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        seg.setOptimizeCoefficients(true);
        // Seach for a plane perpendicular to some axis (specified below)
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // Set the distance to the plane for a point to be an inlier
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);

        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance
        Eigen::Vector3f axis;
        axis << 0, 0, 1;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(10.0));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        pcl::ModelCoefficients coeff;
        seg.segment(indices_internal, coeff);

        *indices = indices_internal;

        if (indices->indices.size() == 0)
        {
            ROS_ERROR("Unable to find surface.");
            return;
        }
    }

    Segmenter::Segmenter(const ros::Publisher& surface_points_pub)
        : surface_points_pub_(surface_points_pub) {}
    
    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg)
    {
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);

        // Extract the indices corresponding to the table
        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        SegmentSurface(cloud, table_inliers);

        // Reify the pointcloud
        PointCloudC::Ptr table_cloud(new PointCloudC());
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(table_inliers);
        extract.filter(*table_cloud);

        // Conversion to a PointCloud2 message and publication
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*table_cloud, output_cloud);
        surface_points_pub_.publish(output_cloud);
    }
} //namespace perception