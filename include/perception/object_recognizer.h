#ifndef _OBJECT_RECOGNIZER_H_
#define _OBJECT_RECOGNIZER_H_

#include <string>
#include <vector>

#include "pcl/kdtree/flann.h"

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void LoadData(const std::string& data_dir,
              std::vector<perception_msgs::ObjectFeatures>* dataset);

class ObjectRecognizer
{
    public:
        explicit ObjectRecognizer(const std::vector<perception_msgs::ObjectFeatures>& dataset);
        void Recognize(const Object& object, std::string* name, double* confidence);
    
    private:
        std::vector<perception_msgs::ObjectFeatures> dataset_;
};

} // namespace perception

#endif // _OBJECT_RECOGNIZER_H_