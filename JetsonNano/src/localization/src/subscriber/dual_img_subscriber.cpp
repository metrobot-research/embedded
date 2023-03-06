#include "subscriber/dual_img_subscriber.hpp"
#include <boost/bind.hpp>

DualImgSubscriber::DualImgSubscriber(ros::NodeHandle &nh, std::string topic1, std::string topic2, size_t buff_size)
        : nh_(nh),
          sub_1(nh_, topic1, buff_size),
          sub_2(nh_, topic2, buff_size),
          sync_(sub_1, sub_2, buff_size*2){
    sync_.registerCallback(boost::bind(&DualImgSubscriber::msg_callback, this, _1, _2));
}

void DualImgSubscriber::ParseData(
        std::deque<DualImgStamped> &target_imgBuffer) {
    buff_mutex_.lock();
    if(!imgStampedBuffer.empty()){
        target_imgBuffer.insert(target_imgBuffer.end(), imgStampedBuffer.begin(), imgStampedBuffer.end());
        imgStampedBuffer.clear();
    }
    buff_mutex_.unlock();
}

void DualImgSubscriber::msg_callback(const sensor_msgs::ImageConstPtr &msg1,
                                     const sensor_msgs::ImageConstPtr &msg2) {
    buff_mutex_.lock();
// Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ImgPtr1, cv_ImgPtr2;
    try {
        cv_ImgPtr1 = cv_bridge::toCvShare(msg1);
        cv_ImgPtr2 = cv_bridge::toCvShare(msg2);
    }
    catch (cv_bridge::Exception &e) {
        std::cout << "cv_bridge exception: " <<  e.what() << std::endl;
        return;
    }
    // only when saving imgs, use the following color converter
//    if (cv_ImgPtr1->image.type() == CV_8UC3) // only for RGB image, convert BGR to RGB, grey img is of type CV_8UC1
//        cv::cvtColor(cv_ImgPtr1->image, cv_ImgPtr1->image, cv::COLOR_BGR2RGB);
    DualImgStamped new_dual_img_stamped = {msg1->header.stamp, cv_ImgPtr1, cv_ImgPtr2};
    imgStampedBuffer.push_back(new_dual_img_stamped);
    buff_mutex_.unlock();
}