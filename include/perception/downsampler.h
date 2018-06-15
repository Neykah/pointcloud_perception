#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
class Downsampler {
    public:
      Downsampler();
      void Callback(const sensor_msgs::PointCloud2& msg);
    private:
};
} // namespace perception