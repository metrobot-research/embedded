#ifndef SUBSCRIBER_IMG_SUBSCRIBER_HPP_
#define SUBSCRIBER_IMG_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class ImgSubscriber {
public:
    ImgSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);

    ImgSubscriber() = default;

    void ParseData(std::deque<cv_bridge::CvImageConstPtr> &img_buffer);

private:
    void msg_callback(const sensor_msgs::ImageConstPtr &msgImg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<cv_bridge::CvImageConstPtr> imgBuffer;

    std::mutex buff_mutex_;
};

#endif