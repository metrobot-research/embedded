#ifndef SUBSCRIBER_IMG_SAVER_HPP_
#define SUBSCRIBER_IMG_SAVER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class ImgSaver {
public:
    ImgSaver(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size, std::string save_directory);

    ImgSaver() = default;

private:
    void msg_callback(const sensor_msgs::ImageConstPtr &msgImg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<cv::Mat> imgBuffer;
    std::deque<double> timestampBuf;
    std::string save_directory;

    std::mutex buff_mutex_;
};

#endif