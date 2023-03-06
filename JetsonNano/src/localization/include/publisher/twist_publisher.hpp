//
// Created by warren on 2022/11/28.
//

#ifndef PUBLISHER_TWIST_PUBLISHER_H
#define PUBLISHER_TWIST_PUBLISHER_H

#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/TwistStamped.h>


class TwistPublisher {
public:
    TwistPublisher(ros::NodeHandle &nh,
                  const std::string& topic_name,
                  const std::string& frame_id,
                  int buff_size) : nh_(nh), frame_id_(frame_id){
        publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_name, buff_size);
    };

    TwistPublisher() = default;

    void Publish(const Eigen::Vector3f &lin_vel, const Eigen::Vector3f &ang_vel, const ros::Time &timestamp){
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id_;
        msg.twist.linear.x = lin_vel.x();
        msg.twist.linear.y = lin_vel.y();
        msg.twist.linear.z = lin_vel.z();
        msg.twist.angular.x = ang_vel.x();
        msg.twist.angular.y = ang_vel.y();
        msg.twist.angular.z = ang_vel.z();
        publisher_.publish(msg);
    };

    void Publish(const Eigen::Vector3f &lin_vel, const Eigen::Vector3f &ang_vel){
        Publish(lin_vel, ang_vel, ros::Time::now());
    };

    bool HasSubscribers(){return publisher_.getNumSubscribers() != 0;};

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

#endif //PUBLISHER_TWIST_PUBLISHER_H
