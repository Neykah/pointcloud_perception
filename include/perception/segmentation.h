#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                    pcl::PointIndices::Ptr);

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

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* objects_indices);

class Segmenter {
 public:
  Segmenter(const ros::Publisher&, const ros::Publisher&);
  void Callback(const sensor_msgs::PointCloud2&);

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher marker_pub_;
};
}  // namespace perception