#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

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

class Segmenter {
 public:
  Segmenter(const ros::Publisher&);
  void Callback(const sensor_msgs::PointCloud2&);

 private:
  ros::Publisher surface_points_pub_;
};
}  // namespace perception