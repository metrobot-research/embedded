//
// Created by warren on 2022/12/8.
//

#ifndef CATKIN_CHICKEN_ROBOT_SENSORDATA_SUBSCRIBER_H
#define CATKIN_CHICKEN_ROBOT_SENSORDATA_SUBSCRIBER_H

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "customized_msgs/SensorData.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

struct SensorDataPtrStamped{
    customized_msgs::SensorDataConstPtr data_ptr;
    ros::Time timestamp;
};

class Esp32Subscriber{
public:
    Esp32Subscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size): nh_(nh) {
            subscriber_ = nh_.subscribe(topic_name, buff_size, &Esp32Subscriber::msg_callback, this);
            std::cout << "Esp32Subscriber: " << topic_name << std::endl;
    };

    void ParseData(std::deque<SensorDataPtrStamped> &target_buff){
        buff_mutex_.lock();
        if (!new_SensorDataPtrStamped_.empty()) {
            target_buff.insert(target_buff.end(), new_SensorDataPtrStamped_.begin(), new_SensorDataPtrStamped_.end());
            new_SensorDataPtrStamped_.clear();
        }
        buff_mutex_.unlock();
    };

private:
    void msg_callback(const customized_msgs::SensorDataConstPtr &SensorData_ptr){
        buff_mutex_.lock();
        SensorDataPtrStamped data_stamped = {SensorData_ptr, ros::Time::now()};
        new_SensorDataPtrStamped_.push_back(data_stamped);
        buff_mutex_.unlock();
    };
    
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<SensorDataPtrStamped> new_SensorDataPtrStamped_;

    std::mutex buff_mutex_;
};

#endif //CATKIN_CHICKEN_ROBOT_SENSORDATA_SUBSCRIBER_H
