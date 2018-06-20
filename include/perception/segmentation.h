#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "perception/object.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                    pcl::PointIndices::Ptr, pcl::PointIndices::Ptr);

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);
// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: the point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just 
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.

void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>:: Ptr cloud, std::vector<Object>* objects);
// Does a complete tabletop segmentation pipeline.
//
// Args:
//   cloud: The point cloud with the surface and the objects above it.
//   objects: The output objects.

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices>* objects_indices);

class Segmenter {
 public:
  Segmenter(const ros::Publisher&);
  void Callback(const sensor_msgs::PointCloud2&);

 private:
  ros::Publisher marker_pub_;
};
}  // namespace perception