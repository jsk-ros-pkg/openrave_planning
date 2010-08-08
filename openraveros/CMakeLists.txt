cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_executable(openraveros src/openraveros.cpp)
target_link_libraries (openraveros openrave-core)
rosbuild_link_boost(openraveros thread)

# add sessions dependency
rosbuild_find_ros_package(roscpp_sessions)
set_source_files_properties(src/openraveros.cpp PROPERTIES OBJECT_DEPENDS ${roscpp_sessions_PACKAGE_PATH}/include/ros/session.h)
