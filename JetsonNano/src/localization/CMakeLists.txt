#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.5)
project(localization)
message("start make localization")

# Must add "-march=native"，or we'll get errors while using some pcl func.s
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)
#add_definitions(-DCPP11)

find_package(catkin REQUIRED COMPONENTS
        rospy
        rosbag
        roscpp
        cv_bridge
        message_filters
        image_transport
        tf
        pcl_ros
        pcl_conversions
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        customized_msgs
        message_filters # Topic Synchronization
        )

set(ALL_TARGET_LIBRARIES "")
include(cmake/eigen.cmake)
#include(cmake/glog.cmake)
include(cmake/global_definition.cmake)
include(cmake/PCL.cmake)
include(cmake/OpenCV.cmake)
include(cmake/boost.cmake)

catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "src/*.c*")
list(REMOVE_ITEM ALL_SRCS
        src/publisher/pose_publisher.cpp
        src/subscriber/cloud_subscriber.cpp
        src/subscriber/img_saver.cpp
        src/subscriber/img_subscriber.cpp)
#MESSAGE("ALL_SRCS:")
#MESSAGE(${ALL_SRCS})

add_executable(localization_node node/localization_node.cpp ${ALL_SRCS})
target_link_libraries(localization_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#add_executable(img_save_node node/img_save.cpp src/subscriber/img_saver.cpp)
#target_link_libraries(img_save_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})