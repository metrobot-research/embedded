#ifndef SENSOR_DATA_DUAL_IMG_STAMPED_HPP
#define SENSOR_DATA_DUAL_IMG_STAMPED_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

struct DualImgStamped{
    ros::Time time;
    cv_bridge::CvImageConstPtr img1_ptr;
    cv_bridge::CvImageConstPtr img2_ptr;
};

#endif //SENSOR_DATA_DUAL_IMG_STAMPED_HPP
