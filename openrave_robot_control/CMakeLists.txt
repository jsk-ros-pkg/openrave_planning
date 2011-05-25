cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

rosbuild_add_boost_directories()
rosbuild_genmsg()
rosbuild_gensrv()

add_subdirectory(src)

# roslaunch documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} ./${jsk_tools_PACKAGE_PATH}/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)
