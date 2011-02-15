cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)#RelWithDebInfo)
add_definitions(-Wall)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_library(rossensors src/rossensors.cpp)
rosbuild_link_boost(rossensors thread)
