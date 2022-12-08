//
// Created by warren on 2021/10/26.
//

#include <ros/ros.h>
#include "localization_flow.h"
//#include "glog/logging.h"
#include "global_definition/global_definition.h"
#include <iostream>


int main(int argc, char *argv[]){
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh("~");

//    bool screen = false;
//    nh.param<bool>("screen", screen, "true");
//
//    google::InitGoogleLogging(argv[0]);
//    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
//    FLAGS_alsologtostderr = screen;
//    FLAGS_colorlogtostderr = true;
//    FLAGS_log_prefix = true;
//    FLAGS_logbufsecs = 0;
//
//    LOG(INFO) << "localization_node/screen: " << screen;

    std::shared_ptr<LocalizationFlow> localization_flow_ptr = std::make_shared<LocalizationFlow>(nh);

    double fps;
    nh.getParam("camera2_color_fps", fps);
    ros::Rate rate(fps);
    while (ros::ok()) {
        ros::spinOnce();
        localization_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}