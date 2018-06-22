#include "perception/segmentation.h"
#include "perception/object.h"

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

// Euclidean Cluster Extraction
#include "pcl/segmentation/extract_clusters.h"

// PointCloud reification
#include "pcl/filters/extract_indices.h"

// Object recognition
#include <math.h>
#include <sstream>
#include "perception/object_recognizer.h"

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
        ros::param::param("seg_dist_threshold", dist_thresh, INLIER_DIST_THRESHOLD);
        ros::param::param("seg_tolerance_degree", tol_deg, TOLERANCE_DEGREE);
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
                               pcl::PointIndices::Ptr objects_pointcloud_indices,
                               std::vector<pcl::PointIndices>* object_indices)
    {
        ROS_INFO("There are %ld points above the table", objects_pointcloud_indices->indices.size());

        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 300);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setIndices(objects_pointcloud_indices);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*object_indices);

        // Find the size of the smallest and the largest object,
        // where size = number of points in the cluster
        size_t min_size = std::numeric_limits<size_t>::max();
        size_t max_size = std::numeric_limits<size_t>::min();

        for (size_t i=0 ; i < object_indices->size() ; i++)
        {
            size_t cluster_size = object_indices->at(i).indices.size();
            if (cluster_size < min_size)
            {
                min_size = cluster_size;
            }
            if (cluster_size > max_size)
            {
                max_size = cluster_size;
            }
        }

        ROS_INFO("Found %ld objects, min size: %ld, max_size: %ld", object_indices->size(), min_size, max_size);
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

    void SegmentTabletopScene(PointCloudC::Ptr cloud, std::vector<Object>* objects)
    {
        // Extract the indices corresponding to the surface
        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        pcl::PointIndices::Ptr table_above(new pcl::PointIndices());
        SegmentSurface(cloud, table_inliers, table_above);
        std::vector<pcl::PointIndices> object_indices;
        SegmentSurfaceObjects(cloud, table_above, &object_indices);

        // Fill the output Object struct
        for (size_t i = 0 ; i < object_indices.size() ; i++)
        {
            Object curr_object = {};
            curr_object.name = "object_" + std::to_string(i);
            PointCloudC::Ptr object_cloud(new PointCloudC());

            pcl::PointIndices::Ptr indices(new pcl::PointIndices());
            *indices = object_indices[i];
            pcl::ExtractIndices<PointC> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(indices);
            extract.filter(*object_cloud);
            GetAxisAlignedBoundingBox(object_cloud, &curr_object.pose, &curr_object.dimensions);

            curr_object.cloud = object_cloud;
            objects->push_back(curr_object);
        }

        
    }

    Segmenter::Segmenter(const ros::Publisher& marker_pub, const ObjectRecognizer& recognizer)
        : marker_pub_(marker_pub), recognizer_(recognizer) 
    {}
    
    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg)
    {
        PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud_unfiltered);
        PointCloudC::Ptr cloud(new PointCloudC());
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

        std::vector<Object> objects;
        SegmentTabletopScene(cloud, &objects);

        for (size_t i=0 ; i < objects.size() ; i++)
        {
            const Object& object = objects[i];

            // Publish a bounding box around the object
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            object_marker.pose = object.pose;
            object_marker.scale = object.dimensions;
            object_marker.color.g = 1;
            object_marker.color.a = 0.3;
            marker_pub_.publish(object_marker);

            // Recognize the object
            std::string name;
            double confidence;
            recognizer_.Recognize(object, &name, &confidence);
            confidence = round(1000 * confidence) / 1000;
            
            std::stringstream ss;
            ss << name << " (" << confidence << ")";

            // Publish the recognition result
            visualization_msgs::Marker name_marker;
            name_marker.ns = "recognition";
            name_marker.id = i;
            name_marker.header.frame_id = "base_link";
            name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            name_marker.pose.position = object.pose.position;
            name_marker.pose.position.z += 0.1;
            name_marker.pose.orientation.w = 1;
            name_marker.scale.x = 0.025;
            name_marker.scale.y = 0.025;
            name_marker.scale.z = 0.025;
            name_marker.color.r = 0;
            name_marker.color.g = 0;
            name_marker.color.b = 1.0;
            name_marker.color.a = 1.0;
            name_marker.text = ss.str();
            marker_pub_.publish(name_marker);
        }
    }
} //namespace perception