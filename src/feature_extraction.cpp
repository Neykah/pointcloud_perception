#include "perception/feature_extraction.h"

#include <algorithm> // std::min and std::max

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {

void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features)
{
    // "x" dimension is always the smallest of x and y to account for rotations.
    // z always points up.

    features->names.push_back("box_dim_x");
    features->names.push_back("box_dim_y");
    if (object.dimensions.x <= object.dimensions.y)
    {
        features->values.push_back(object.dimensions.x);
        features->values.push_back(object.dimensions.y);
    }
    else
    {
        features->values.push_back(object.dimensions.y);
        features->values.push_back(object.dimensions.x);
    }
    features->names.push_back("box_dim_z");
    features->values.push_back(object.dimensions.z);
}

void ExtractColorFeatures(const Object& object,
                          perception_msgs::ObjectFeatures* features)
{
    std::vector<double> color_features;
    color_features.resize(125);
    for (size_t i=0 ; i < object.cloud->size() ; i++)
    {
        const pcl::PointXYZRGB& pt = object.cloud->at(i);
        // Reduce the color space to just 5 values (255 / 51) per channel.
        int r = std::min(pt.r / 51, 4);
        int g = std::min(pt.g / 51, 4);
        int b = std::min(pt.b / 51, 4);
        int index = r * 25 + g * 5 + b;
        color_features[index] += 1;
    }

    // Normalize to get a distribution
    for (size_t i=0 ; i < color_features.size() ; i++)
    {
        color_features[i] /= object.cloud->size();
    }
    features->values.insert(features->values.end(), color_features.begin(), color_features.end());
}
} // namespace perception