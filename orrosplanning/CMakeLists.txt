cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
add_definitions(-Wall)

rosbuild_add_boost_directories()
rosbuild_gensrv()

rosbuild_add_library(orrosplanning src/orrosplanningmain.cpp)
rosbuild_link_boost(orrosplanning thread)
