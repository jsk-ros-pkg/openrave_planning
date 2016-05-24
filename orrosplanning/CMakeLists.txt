# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(orrosplanning)
find_package(catkin REQUIRED COMPONENTS roscpp openrave openraveros sensor_msgs trajectory_msgs arm_navigation_msgs message_generation)

add_service_files(FILES
  IK.srv MoveManipulator.srv 	MoveToHandPosition.srv 	SetGraspParameters.srv SetJointState.srv
)

generate_messages(DEPENDENCIES sensor_msgs trajectory_msgs arm_navigation_msgs)

catkin_package()

add_library(orrosplanning src/orrosplanningmain.cpp src/rospassivecontroller.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(orrosplanning ${Boost_LIBRARIES} ${catkin_LIBRARIES})

