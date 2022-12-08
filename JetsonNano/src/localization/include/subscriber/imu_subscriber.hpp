
#ifndef SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

struct IMUData{
    ros::Time timestamp;
    Eigen::Vector3f linear_acceleration;
    Eigen::Vector3f angular_velocity;
    Eigen::Quaternionf orientation;
};

class IMUSubscriber {
public:
    IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size): nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
        std::cout<<"IMUSubscriber: "<<topic_name<<std::endl;
    };

    IMUSubscriber() = default;

    void ParseData(std::deque<IMUData> &imu_data_buff){
        buff_mutex_.lock();
        if (!new_imu_data_.empty()) {
            imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
            new_imu_data_.clear();
        }
        buff_mutex_.unlock();
    };

private:
    void msg_callback(const sensor_msgs::ImuConstPtr &imu_msg_ptr){
        buff_mutex_.lock();
        IMUData imu_data;
        imu_data.timestamp = imu_msg_ptr->header.stamp;

        imu_data.linear_acceleration.x() = imu_msg_ptr->linear_acceleration.x;
        imu_data.linear_acceleration.y() = imu_msg_ptr->linear_acceleration.y;
        imu_data.linear_acceleration.z() = imu_msg_ptr->linear_acceleration.z;

        imu_data.angular_velocity.x() = imu_msg_ptr->angular_velocity.x;
        imu_data.angular_velocity.y() = imu_msg_ptr->angular_velocity.y;
        imu_data.angular_velocity.z() = imu_msg_ptr->angular_velocity.z;

        imu_data.orientation.x() = imu_msg_ptr->orientation.x;
        imu_data.orientation.y() = imu_msg_ptr->orientation.y;
        imu_data.orientation.z() = imu_msg_ptr->orientation.z;
        imu_data.orientation.w() = imu_msg_ptr->orientation.w;

//        LOG(INFO) << std::fixed << "imu time: "<<imu_msg_ptr->header.stamp.toSec();
        new_imu_data_.push_back(imu_data);
        buff_mutex_.unlock();
    };

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_;

    std::mutex buff_mutex_;
};

#endif // SUBSCRIBER_IMU_SUBSCRIBER_HPP_