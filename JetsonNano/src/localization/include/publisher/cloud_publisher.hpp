#ifndef PUBLISHER_CLOUD_PUBLISHER_HPP_
#define PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/cloud_data.hpp"

class CloudPublisher {
public:
    CloudPublisher(ros::NodeHandle &nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size): nh_(nh), frame_id_(frame_id) {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    };

    CloudPublisher() = default;

    template<typename Tpoint> // can take pcl::PointCloud<pcl::Pointxxxxx>::ConstPtr as input
    void Publish(boost::shared_ptr<pcl::PointCloud<Tpoint>> &cloud_ptr_input, ros::Time time){
        PublishData(cloud_ptr_input, time);
    };

    template<typename Tpoint>
    void Publish(boost::shared_ptr<pcl::PointCloud<Tpoint>> &cloud_ptr_input){
        PublishData(cloud_ptr_input, ros::Time::now());
    };

    bool HasSubscribers(){
        return publisher_.getNumSubscribers() != 0;
    };

private:
    template<typename Tpoint>
    void PublishData(boost::shared_ptr<pcl::PointCloud<Tpoint>> &cloud_ptr_input, ros::Time time){
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    };

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
#endif