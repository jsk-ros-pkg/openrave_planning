# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(collada_robots)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED )

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required (VERSION 2.6.0) # using import/export targets
project(collada_robots)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

set(ROBOTS_DIR data/robots)
file(GLOB robots "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOTS_DIR}/*.zae")
set(robots_new)
foreach(robot ${robots})
  get_filename_component(robot_base ${robot} NAME_WE)
  set(robot_new "${robot_base}.dae")
  add_custom_command(OUTPUT ${robot_new}
    COMMAND unzip -o -d "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOTS_DIR}" ${robot}
    DEPENDS ${robot})
  set(robots_new ${robots_new} ${robot_new})
endforeach()

add_custom_target(collada_robots ALL DEPENDS ${robots_new})
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)