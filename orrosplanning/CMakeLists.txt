cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
add_definitions(-Wall)

rosbuild_add_boost_directories()
rosbuild_gensrv()

rosbuild_add_library(orrosplanning src/orrosplanningmain.cpp src/rospassivecontroller.cpp)
rosbuild_link_boost(orrosplanning thread)

rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)
