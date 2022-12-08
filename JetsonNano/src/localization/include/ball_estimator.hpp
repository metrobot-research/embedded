//
// Created by warren on 2022/11/27.
//

#ifndef CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H
#define CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H

#include <iostream>
#include <cassert>
#include <deque>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

struct BallPosStamped{
    Eigen::Vector3f position;
    ros::Time timestamp;
};

class BallEstimator{
public:
    BallEstimator(ros::NodeHandle &nh): // params not set to const since we may need real-time tuning
            nh_(nh), give_up(true), lost_time(0), has_inited(false), step_time(0), recent_his_time(0),
            cur_ball_vel_w(0,0,0)
//            ofs("/home/warren/Documents/code/ME206A_Proj/catkin_chicken_robot/src/localization/data/vel/vel.csv")
    {

        float freq;
        nh_.getParam("camera2_color_fps", freq);
        frame_dt = 1. / freq;

        nh_.getParam("ball_est_pred_mode", pred_mode);
        nh_.getParam("decay_mode_coeff", decay_mode_coeff);
        nh_.getParam("ball_est_max_recent_his_time", max_recent_his_time);
        nh_.getParam("ball_est_max_lost_time", max_lost_time);
        nh_.getParam("ball_est_noise_overcoming_vel", noise_overcoming_vel);
        float ball_est_short_window_time;
        nh_.getParam("ball_est_short_window_time", ball_est_short_window_time);
        short_window_size = ball_est_short_window_time / frame_dt + 1;
//        nh_.getParam("ball_est_max_valid_z_vel", max_valid_z_vel);
//        nh_.getParam("ball_est_max_valid_xy_vel", max_valid_xy_vel);
        nh_.getParam("ball_est_bounce_lost_coeff", bounce_lost_coeff);

        recent_ball_pos_stamped_his.clear();
        old_ball_pos_stamped_his.clear();
    };

    // Add Measurement and then Update Estimation
    void addPos(const ros::Time &timestamp, const Eigen::Vector3f &new_ball_pos_w, const Eigen::Vector3f &new_wheel_center_w){
//        std::cout << "In addPos" << std::endl;
        if(has_inited){
            step_time = (timestamp - last_time).toSec();

            // check whether the measurement is a spike, if yes, then this frame should be considered as lost
            // can only help if there's no distraction
//            if( (timestamp - latest_measurement.timestamp).toSec() < 0.2 ){
//                Eigen::Vector3f measured_vel = (new_ball_pos_w - latest_measurement.position) / (timestamp - latest_measurement.timestamp).toSec();
//                if((measured_vel.z() > max_valid_z_vel) || (measured_vel.block(0,0,2,1).norm() > max_valid_xy_vel)){
//                    addLost(timestamp, new_wheel_center_w);
//                    return;
//                }
//            }
        }
        last_time = timestamp;

        if(give_up){
            give_up = false;
            std::cout << "Redetected ball-----------" << std::endl;
        }
        lost_time = 0;



        BallPosStamped newPosStamped = {new_ball_pos_w, timestamp};
        recent_ball_pos_stamped_his.push_back(newPosStamped);

        popOutdatedHistory(timestamp);

        recent_his_time = (timestamp - recent_ball_pos_stamped_his.front().timestamp).toSec();

        cur_wheel_center_w = new_wheel_center_w;
        estimateCurBallState();

        latest_measurement = newPosStamped;
        has_inited = true;
//        std::cout << "Exit addPos" << std::endl;
    };

    void addLost(const ros::Time &timestamp, const Eigen::Vector3f &new_wheel_center_w){
//        std::cout << "In addLost" << std::endl;
        if(!has_inited){
//            std::cout << "Give up-------------" << std::endl;
            return;
        }
        step_time = (timestamp - last_time).toSec();
        last_time = timestamp;

        popOutdatedHistory(timestamp);

        if(!recent_ball_pos_stamped_his.empty())
            recent_his_time = (timestamp - recent_ball_pos_stamped_his.front().timestamp).toSec();
        else
            recent_his_time = 0;

        if(lost_time < max_lost_time){
            lost_time += step_time;
            if(lost_time >= max_lost_time){
                give_up = true;
                old_ball_pos_stamped_his.clear(); // fixme: if want to clear here, max lost time should > max old history time
                recent_ball_pos_stamped_his.clear();
//                std::cout << "Give up-------------" << std::endl;
            }else{
                cur_wheel_center_w = new_wheel_center_w;
                estimateCurBallState();
            }
        }else{
//            std::cout << "Give up-------------" << std::endl;
        }
//        std::cout << "Exit addPos" << std::endl;
    };

    // Get Estimation Result
    bool isGiveup(){return give_up;};
    Eigen::Vector3f getCurBallPos(){return cur_ball_pos_w;}; // always check isLost before querying for cur ball pos & vel
    Eigen::Vector3f getCurBallVel(){return cur_ball_vel_w;};

private:
    void popOutdatedHistory(const ros::Time &timestamp){
        while(!recent_ball_pos_stamped_his.empty()){
            if( ((timestamp - recent_ball_pos_stamped_his.front().timestamp).toSec() > max_recent_his_time) ){
                old_ball_pos_stamped_his.push_back(recent_ball_pos_stamped_his.front());
                recent_ball_pos_stamped_his.pop_front();
            }else
                break;
        }

        while(!old_ball_pos_stamped_his.empty()){
            if( (timestamp - old_ball_pos_stamped_his.front().timestamp).toSec() > max_lost_time )
                old_ball_pos_stamped_his.pop_front();
            else
                break;
        }
    };

    void estimateCurBallState(){
//        std::cout << "In estimateCurBallState" << std::endl;
        if(!give_up){
            if(lost_time > 0){ // lost but not give up, do pred
                updateBallPred(); // always has prev ball state if give_up = false && ball_position_his.empty()
                                  // the only case leading to this is lost for cur frame, but has previous estimate
            }else{ // lost_time = 0, cur frame not lost, recent_ball_pos_stamped_his.size() >= 1
                // cur ball pos is definitely the latest measurement
                cur_ball_pos_w = recent_ball_pos_stamped_his.back().position;
                // cur ball vel estimate is handled case by case
                if(recent_his_time > max_recent_his_time - frame_dt){ // moving window long enough
                    cur_ball_vel_w = (recent_ball_pos_stamped_his.back().position - recent_ball_pos_stamped_his.front().position) / recent_his_time;
                }else if(!old_ball_pos_stamped_his.empty()){ // moving window not long enough, lengthen the window
                    float vel_dt = (recent_ball_pos_stamped_his.back().timestamp - old_ball_pos_stamped_his.back().timestamp).toSec();
                    cur_ball_vel_w = (recent_ball_pos_stamped_his.back().position - old_ball_pos_stamped_his.back().position) / vel_dt;
                }else{ // not enough info for unnoisy vel estimate
                    cur_ball_vel_w.setZero();
                }

                // reduce window length in one dim if the vel in that dim is large
                if(recent_ball_pos_stamped_his.size() >= short_window_size){ // if recent history too short, we would have already used old history -> dt cannot be shortened
                    float dt = (recent_ball_pos_stamped_his.back().timestamp - recent_ball_pos_stamped_his[recent_ball_pos_stamped_his.size()-short_window_size].timestamp).toSec();
                    Eigen::Vector3f dp = recent_ball_pos_stamped_his.back().position - recent_ball_pos_stamped_his[recent_ball_pos_stamped_his.size()-short_window_size].position;
                    for(int i=0; i<3; i++){
                        if(abs(cur_ball_vel_w(i)) > noise_overcoming_vel){ // then it can't be that both history are empty, otherwise vel = 0
                            cur_ball_vel_w(i) = dp(i) / dt;
                        }
                    }
                }
            }

//            ofs << cur_ball_vel_w.z() << std::endl;

            prev_ball_pos_w = cur_ball_pos_w;
            prev_ball_vel_w = cur_ball_vel_w;
            prev_ball_height = cur_ball_pos_w.z() - cur_wheel_center_w.z();
        }
//        std::cout << "cur ball vel: " << cur_ball_vel_w << std::endl;
//        std::cout << "Exit estimateCurBallState" << std::endl;
    };

    void updateBallPred(){
//        std::cout << "In updateBallPred" << std::endl;
        if(pred_mode == "uniform"){
            cur_ball_pos_w = prev_ball_pos_w + step_time * prev_ball_vel_w;
            cur_ball_vel_w = prev_ball_vel_w;
        }else if(pred_mode == "static"){
            cur_ball_pos_w = prev_ball_pos_w;
            cur_ball_vel_w.setZero();
        }else if(pred_mode == "decay"){
            cur_ball_pos_w = prev_ball_pos_w + step_time * prev_ball_vel_w;
            cur_ball_vel_w = prev_ball_vel_w * decay_mode_coeff;
        }else if(pred_mode == "gravity"){
            // Assume uniform velocity in x,y
            cur_ball_pos_w.block(0,0,2,1) = prev_ball_pos_w.block(0,0,2,1)
                                            + step_time * prev_ball_vel_w.block(0,0,2,1);
            cur_ball_vel_w.block(0,0,2,1) = prev_ball_vel_w.block(0,0,2,1);
            // z pos & vel pred is different whether previously on gnd
            if(prev_ball_height < 0.1 && abs(prev_ball_vel_w.z()) < 0.05){ // previously on gnd, with tolerance in pos & vel meas
                cur_ball_vel_w.z() = 0;
                cur_ball_pos_w.z() = cur_wheel_center_w.z();
            }else{ // previously above gnd
                float t_hit_gnd = (prev_ball_vel_w.z() + sqrt(prev_ball_vel_w.z()*prev_ball_vel_w.z() + 2*9.8*prev_ball_height)) / 9.8;
                if(t_hit_gnd > step_time){ // ball does not hit gnd
                    cur_ball_pos_w.z() = prev_ball_pos_w.z() + step_time * prev_ball_vel_w.z() - 0.5 * step_time * step_time * 9.8; // assume acc is g
                    cur_ball_vel_w.z() = prev_ball_vel_w.z() - step_time * 9.8;
                }else{ // ball hits gnd, and bounce back
                    float v_bounce_back = -bounce_lost_coeff * (prev_ball_vel_w.z() - t_hit_gnd * 9.8);
                    float rise_time = step_time - t_hit_gnd;
                    if(rise_time > 2 * v_bounce_back / 9.8){
                        // will bounce again. indicating z vel is already small enough, simplify calc by setting z vel to 0, and ball to gnd
                        cur_ball_vel_w.z() = 0;
                        cur_ball_pos_w.z() = cur_wheel_center_w.z();
                    }else{ // large bouncing back vel, won't bounce again
                        cur_ball_vel_w.z() = v_bounce_back - 9.8 * rise_time;
                        cur_ball_pos_w.z() = cur_wheel_center_w.z() + v_bounce_back * rise_time - 0.5 * rise_time * rise_time * 9.8;
                    }

                }
            }
        }else
            std::cerr << "Invalid pred mode in ball_estimator (valid opts: uniform, gravity)" << std::endl;
//        std::cout << "Exit updateBallPred" << std::endl;
    };
    
    //fixme: just for debug
//    std::ofstream ofs;

    ros::NodeHandle nh_;
    std::string pred_mode;
    float decay_mode_coeff;
    float max_recent_his_time;
    float max_lost_time;
    float noise_overcoming_vel;
    int short_window_size;
//    float max_valid_z_vel, max_valid_xy_vel;
    float bounce_lost_coeff;

    float lost_time;
    bool give_up;
    bool has_inited;
    ros::Time last_time;
    float recent_his_time;
    float frame_dt; // default frame dt specified in launch
    float step_time; // dt between the cur and prev measurement (lost is also a measurement)
    std::deque<BallPosStamped> recent_ball_pos_stamped_his;
    std::deque<BallPosStamped> old_ball_pos_stamped_his;
    Eigen::Vector3f prev_ball_pos_w;
    BallPosStamped latest_measurement;
    Eigen::Vector3f prev_ball_vel_w;
    float prev_ball_height;
    Eigen::Vector3f cur_ball_pos_w;
    Eigen::Vector3f cur_ball_vel_w;
    Eigen::Vector3f cur_wheel_center_w;
};

#endif //CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H
