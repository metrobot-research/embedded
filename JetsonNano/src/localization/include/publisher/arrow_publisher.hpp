//
// Created by warren on 2022/11/28.
//

#ifndef PUBLISHER_ARROW_PUBLISHER_H
#define PUBLISHER_ARROW_PUBLISHER_H

#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <visualization_msgs//Marker.h>


class ArrowPublisher {
public:
    ArrowPublisher(ros::NodeHandle &nh,
                   const std::string& topic_name,
                   const std::string& frame_id,
                   int buff_size) : nh_(nh), frame_id_(frame_id){
        publisher_ = nh_.advertise<visualization_msgs::Marker>(topic_name, buff_size);
    };

    ArrowPublisher() = default;

    void Publish(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos, const ros::Time &timestamp){
        visualization_msgs::Marker marker;
        marker.header.stamp = timestamp;
        marker.header.frame_id = frame_id_;
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.points.resize(2);
        marker.points[0].x = start_pos.x();
        marker.points[0].y = start_pos.y();
        marker.points[0].z = start_pos.z();
        marker.points[1].x = end_pos.x();
        marker.points[1].y = end_pos.y();
        marker.points[1].z = end_pos.z();
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = 1.;
        marker.scale.x = 0.01;
        marker.color.a = 1.;
        marker.color.r = 1.;
        marker.color.g = 0.;
        marker.color.b = 0.;
        publisher_.publish(marker);
    };

    void Publish(const Eigen::Vector3f &start_pos, const Eigen::Vector3f &end_pos){
        Publish(start_pos, end_pos, ros::Time::now());
    };

    bool HasSubscribers(){return publisher_.getNumSubscribers() != 0;};

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

#endif //PUBLISHER_ARROW_PUBLISHER_H
