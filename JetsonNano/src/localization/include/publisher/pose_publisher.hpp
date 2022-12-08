#ifndef PUBLISHER_POSE_PUBLISHER_HPP_
#define PUBLISHER_POSE_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>


class PosePublisher {
public:
    PosePublisher(ros::NodeHandle &nh,
                  const std::string& topic_name,
                  const std::string& frame_id,
                  int buff_size);

    PosePublisher() = default;

    void Publish(const Eigen::Vector3f &position, double orientation, double time);

    void Publish(const Eigen::Vector3f &position, double orientation);

    void PublishData(const Eigen::Vector3f &position, double orientation, ros::Time time);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

#endif