cmake_minimum_required(VERSION 2.8.3)
project(collada_robots)

find_package(catkin REQUIRED)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/data/robots/README.rst
  COMMAND git clone git://github.com/rdiankov/collada_robots.git data/robots
  COMMAND find data/robots/ -iname '*.zae'  -print | xargs -n1 unzip -o -d data/robots/
  DEPENDS CMakeLists.txt
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_target(download_collada_robots ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/data/robots/README.rst)

catkin_package()

install(DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
