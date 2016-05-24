cmake_minimum_required(VERSION 2.8.3)
project(openrave)

find_package(catkin REQUIRED)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/openrave
  COMMAND git clone https://github.com/rdiankov/openrave.git
  COMMAND cd openrave && git reset --hard cdc681631c1ab0dab57d4201475ae02659fa4412 # latest_sable of May 7 2016
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_custom_target(download_openrave
  DEPENDS ${CMAKE_BINARY_DIR}/openrave)
add_custom_target(compile_openrave ALL
  COMMAND mkdir -p build_openrave
  COMMAND cd build_openrave && cmake ${CMAKE_BINARY_DIR}/openrave -DOPT_FCL_COLLISION=OFF -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}
  COMMAND make -C ${CMAKE_BINARY_DIR}/build_openrave
  COMMAND make -C ${CMAKE_BINARY_DIR}/build_openrave install
  DEPENDS download_openrave)

catkin_package(
  CFG_EXTRAS openrave-extras.cmake
  )

catkin_add_env_hooks(99.openrave SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY share bin lib include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


