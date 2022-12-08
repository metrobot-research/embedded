#include "subscriber/img_saver.hpp"
//#include "glog/logging.h"
#include "global_definition/global_definition.h"
#include <opencv2/highgui/highgui.hpp>
#include <utility>

ImgSaver::ImgSaver(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size, std::string save_directory)
        : nh_(nh), save_directory(std::move(save_directory)) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &ImgSaver::msg_callback, this);
    std::cout<<"ImgSaver: "<<topic_name<<std::endl;
}

void ImgSaver::msg_callback(const sensor_msgs::ImageConstPtr &msgImg) {
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

    cv::Mat img = cv_ptrImg->image; // BGR
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // RGB
    double timestamp = cv_ptrImg->header.stamp.toSec();
    std::string image_path = SAVE_PATH + save_directory + "/" + std::to_string(uint64_t(timestamp * 1e9)) + ".bmp";
    cv::imwrite(image_path, img);
//        ROS_INFO("Receive images.");
//        LOG(INFO) << std::fixed << "Saved img with time: "<<cv_ptrImg->header.stamp.toSec();
    buff_mutex_.unlock();
}