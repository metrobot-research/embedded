#include "publisher/pose_publisher.hpp"

#include <Eigen/Dense>
#include <utility>
//#include "glog/logging.h"

PosePublisher::PosePublisher(ros::NodeHandle &nh,
                             const std::string& topic_name,
                             const std::string& frame_id,
                             int buff_size)
        : nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
}

void PosePublisher::Publish(const Eigen::Vector3f &position, double orientation, double time) {
    ros::Time ros_time(time);
    PublishData(position, orientation, ros_time);

}

void PosePublisher::Publish(const Eigen::Vector3f &position, double orientation) {
    PublishData(position, orientation, ros::Time::now());
}

void PosePublisher::PublishData(const Eigen::Vector3f &position, double orientation, ros::Time time) {
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);

    pose_stamped.pose.orientation.x = 0; // q = cos(phi/2) + u * sin(phi/2) = qw + qv
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = sin(0.5 * orientation);
    pose_stamped.pose.orientation.w = cos(0.5 * orientation);

    publisher_.publish(pose_stamped);
}

bool PosePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}