
#ifndef SENSOR_DATA_CLOUD_DATA_HPP_
#define SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "global_definition/global_definition.h"

class CloudData {
public:
    using POINT = pcl::PointXYZRGB;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData()
            : cloud_ptr(new CLOUD()) {
        cloud_ptr->clear();
    }

public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};

#endif