cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

rosbuild_add_boost_directories()
rosbuild_genmsg()
rosbuild_gensrv()

add_subdirectory(src)
