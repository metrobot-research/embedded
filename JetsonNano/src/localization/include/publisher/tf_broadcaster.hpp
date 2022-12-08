#ifndef PUBLISHER_TF_BROADCASTER_HPP_
#define PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TFBroadCaster {
public:
    TFBroadCaster() = default;

    void SendTransform(const std::string parent_frame, const std::string child_frame,
                       const Eigen::Vector3f &p, const Eigen::Quaternionf &q, ros::Time time){
        transform_.frame_id_ = parent_frame;
        transform_.child_frame_id_ = child_frame;
        transform_.stamp_ = time;
        transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()).normalize());
        transform_.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
        broadcaster_.sendTransform(transform_);
    };

    void SendTransformMatrix(const std::string parent_frame, const std::string child_frame, Eigen::Matrix4f T, ros::Time time){
        Eigen::Vector3f p = T.block<3,1>(0,3);
        Eigen::Quaternionf q = Eigen::Quaternionf(T.block<3,3>(0,0));
        SendTransform(parent_frame, child_frame, p, q, time);
    };

protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};

#endif