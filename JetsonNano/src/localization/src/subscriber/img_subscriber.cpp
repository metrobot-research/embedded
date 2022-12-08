#include "subscriber/img_subscriber.hpp"
//#include "glog/logging.h"

ImgSubscriber::ImgSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &ImgSubscriber::msg_callback, this);
    std::cout<<"ImgSubscriber: "<<topic_name<<std::endl;
}

void ImgSubscriber::msg_callback(const sensor_msgs::ImageConstPtr &msgImg) {
    buff_mutex_.lock();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrImg;
    try {
        cv_ptrImg = cv_bridge::toCvShare(msgImg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptrImg->image.type() == CV_8UC3) // only for RGB image, convert BGR to RGB, grey img is of type CV_8UC1
        cv::cvtColor(cv_ptrImg->image, cv_ptrImg->image, cv::COLOR_BGR2RGB);
    imgBuffer.push_back(cv_ptrImg);
    buff_mutex_.unlock();
}

void ImgSubscriber::ParseData(std::deque<cv_bridge::CvImageConstPtr> &img_receiver) {
    buff_mutex_.lock();
    if (!imgBuffer.empty()) {
        img_receiver.insert(img_receiver.end(), imgBuffer.begin(), imgBuffer.end());
        imgBuffer.clear();
    }
    buff_mutex_.unlock();
}