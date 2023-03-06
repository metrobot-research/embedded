//
// Created by warren on 2022/12/3.
//

#ifndef CATKIN_CHICKEN_ROBOT_D435I_LIN_VEL_ESTIMATOR_HPP
#define CATKIN_CHICKEN_ROBOT_D435I_LIN_VEL_ESTIMATOR_HPP

#include <iostream>
#include <cassert>
#include <deque>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

struct PosStamped{
    Eigen::Vector3f position;
    ros::Time timestamp;
};

class D435iLinVelEstimator{
public:
    D435iLinVelEstimator(ros::NodeHandle &nh): // params not set to const since we may need real-time tuning
            nh_(nh), cur_lin_vel_w(0,0,0)
//            ofs("/home/warren/Documents/code/ME206A_Proj/catkin_chicken_robot/src/localization/data/vel/vel.csv")
    {
        nh_.getParam("d435i_lin_vel_est_max_his_time", max_his_time);

        float freq;
        nh_.getParam("camera2_color_fps", freq);
        frame_dt = 1. / freq;

        pos_stamped_his.clear();
    };

    // Add Measurement and then Update Estimation
    void addPos(const ros::Time &timestamp, const Eigen::Vector3f &new_pos_w){
//        std::cout << "In addPos" << std::endl;
        PosStamped newPosStamped = {new_pos_w, timestamp};
        pos_stamped_his.push_back(newPosStamped);

        popOutdatedHistory(timestamp);

        float his_length = (pos_stamped_his.back().timestamp - pos_stamped_his.front().timestamp).toSec();

        if( his_length > max_his_time - frame_dt){
            cur_lin_vel_w = (pos_stamped_his.back().position - pos_stamped_his.front().position) / his_length;
        }else{
            cur_lin_vel_w.setZero();
        }
    };

    Eigen::Vector3f getCurVel(){return cur_lin_vel_w;};

private:
    void popOutdatedHistory(const ros::Time &timestamp){
        while(!pos_stamped_his.empty()){
            if( ((timestamp - pos_stamped_his.front().timestamp).toSec() > max_his_time) ){
                pos_stamped_his.pop_front();
            }else
                break;
        }
    };


    ros::NodeHandle nh_;
    float frame_dt;
    float max_his_time;
    std::deque<PosStamped> pos_stamped_his;
    Eigen::Vector3f cur_lin_vel_w;
};

#endif //CATKIN_CHICKEN_ROBOT_D435I_LIN_VEL_ESTIMATOR_HPP
