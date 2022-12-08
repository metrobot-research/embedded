#include <ros/ros.h>
//#include "glog/logging.h"
#include "global_definition/global_definition.h"
#include "subscriber/img_saver.hpp"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "img_save_node");
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

//    Config::readConfig();

    std::shared_ptr<ImgSaver> rgb_save_ptr = std::make_shared<ImgSaver>(nh, "/d435i/color/image_raw", 10, "rgb");
//    std::shared_ptr<ImgSaver> depth_save_ptr = std::make_shared<ImgSaver>(nh, "/d435i/aligned_depth_to_color/image_raw", 10, "depth");

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}