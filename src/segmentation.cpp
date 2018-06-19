#include "perception/segmentation.h"

#include "pcl/common/common.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

// RANSAC Segmentation
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

// PointCloud reification
#include "pcl/filters/extract_indices.h"

#include <iostream>

// Parameter: distance threshold between a point and a plane to be considered inlier.
#define INLIER_DIST_THRESHOLD 0.005
#define TOLERANCE_DEGREE 10.0

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception 
{
    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::PointIndices::Ptr indices_above) 
    {
        double dist_thresh, tol_deg;
        ros::param::param("/perception/seg_dist_threshold", dist_thresh, INLIER_DIST_THRESHOLD);
        ros::param::param("/perception/seg_tolerance_degree", tol_deg, TOLERANCE_DEGREE);
        std::cout << "Input cloud has " << cloud->size() << " points" << std::endl;
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        seg.setOptimizeCoefficients(true);
        // Seach for a plane perpendicular to some axis (specified below)
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // Set the distance to the plane for a point to be an inlier
        seg.setDistanceThreshold(dist_thresh);
        seg.setInputCloud(cloud);

        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance
        Eigen::Vector3f axis;
        axis << 0, 0, 1;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(tol_deg));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        pcl::ModelCoefficients coeff;
        seg.segment(indices_internal, coeff);

        // Recover the set of points under the table
        double distance_above_plane;
        ros::param::param("distance_above_plane", distance_above_plane, 0.005);
        for (size_t i=0 ; i < cloud->size() ; i++)
        {
            const PointC& pt_i = cloud->points[i];
            double value = coeff.values[0] * pt_i.x + coeff.values[1] * pt_i.y + 
                           coeff.values[2] * pt_i.z + coeff.values[3];

            if (value >= distance_above_plane)
            {
                indices_above->indices.push_back(i);
            }
        }


        *indices = indices_internal;

        if (indices->indices.size() == 0)
        {
            ROS_ERROR("Unable to find surface.");
            return;
        }
    }

    void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               pcl::PointIndices::Ptr surface_indices,
                               std::vector<pcl::PointIndices>* objects_indices)
    {

    }
        

    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                    geometry_msgs::Pose* pose,
                                    geometry_msgs::Vector3* dimensions)
        {
            PointC min_pcl;
            PointC max_pcl;
            pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

            pose->position.x = 0.5 * (min_pcl.x + max_pcl.x);
            pose->position.y = 0.5 * (min_pcl.y + max_pcl.y);
            pose->position.z = 0.5 * (min_pcl.z + max_pcl.z);
            pose->orientation.x = 0.0;
            pose->orientation.y = 0.0;
            pose->orientation.z = 0.0;
            pose->orientation.w = 1.0;

            dimensions->x = max_pcl.x - min_pcl.x;
            dimensions->y = max_pcl.y - min_pcl.y;
            dimensions->z = max_pcl.z - min_pcl.z;
        }

    Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& objects_points_pub, 
                         const ros::Publisher& marker_pub)
        : surface_points_pub_(surface_points_pub), objects_points_pub_(objects_points_pub), marker_pub_(marker_pub) 
    {}
    
    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg)
    {
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);

        // Extract the indices corresponding to the table
        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        pcl::PointIndices::Ptr table_above(new pcl::PointIndices());
        SegmentSurface(cloud, table_inliers, table_above);

        // Reify the pointclouds
        PointCloudC::Ptr table_cloud(new PointCloudC());
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(table_inliers);
        extract.filter(*table_cloud);

        PointCloudC::Ptr objects_cloud(new PointCloudC());
        pcl::ExtractIndices<PointC> extract_above;
        extract_above.setInputCloud(cloud);
        extract_above.setIndices(table_above);
        extract_above.filter(*objects_cloud);

        // Conversion to a PointCloud2 message and publication
        sensor_msgs::PointCloud2 output_table_cloud;
        pcl::toROSMsg(*table_cloud, output_table_cloud);
        surface_points_pub_.publish(output_table_cloud);

        sensor_msgs::PointCloud2 output_objects_cloud;
        pcl::toROSMsg(*objects_cloud, output_objects_cloud);
        objects_points_pub_.publish(output_objects_cloud);

        // Creation of a bounding box and publication
        visualization_msgs::Marker table_marker;
        table_marker.ns = "table";
        table_marker.header.frame_id = "base_link";
        table_marker.type = visualization_msgs::Marker::CUBE;
        GetAxisAlignedBoundingBox(table_cloud, &table_marker.pose, &table_marker.scale);
        table_marker.color.r = 1;
        table_marker.color.a = 0.8;
        marker_pub_.publish(table_marker);
    }
} //namespace perception