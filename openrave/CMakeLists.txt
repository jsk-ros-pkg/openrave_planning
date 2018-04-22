cmake_minimum_required(VERSION 2.8.3)
project(openrave)

find_package(catkin REQUIRED)

set(OPENRAVE_SOURCE_DIR ${CMAKE_BINARY_DIR}/source_openrave)
set(OPENRAVE_BINARY_DIR ${CMAKE_BINARY_DIR}/build_openrave)

#set(OPENRAVE_COMPILE_OPT "-DOPT_VIDEORECORDING=OFF -DOPT_PLUGINS=OFF -DOPT_ACCURATEMATH=OFF -DOPT_PYTHON=OFF -DOPT_OCTAVE=OFF -DOPT_MATLAB=OFF -DOPT_STATIC=OFF -DOPT_COLLADA=OFF -DOPT_BULLET=OFF -DOPT_ODE_COLLISION=OFF -DOPT_QTCOIN_VIEWER=OFF -DOPT_QTOSG_VIEWER=OFF -DOPT_EXTRA_ROBOTS=OFF -DOPT_CBINDINGS=OFF                   -DOPT_FCL_COLLISION=OFF")
set(OPENRAVE_COMPILE_OPT "-DOPT_FCL_COLLISION=OFF")

catkin_package(CFG_EXTRAS openrave-extras.cmake)
catkin_add_env_hooks(99.openrave SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

list(GET CATKIN_WORKSPACES 0 CATKIN_WORKSPACES_0)
if("${CATKIN_WORKSPACES_0}" STREQUAL "${CMAKE_INSTALL_PREFIX}") # since CMAKE_INSTAL_PREFIX is always point to /install so install layout returns true
  message(STATUS "compiled with install layout")
  set(OPENRAVE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
else()
  message(STATUS "compiled with devel layout")
  set(OPENRAVE_INSTALL_PREFIX ${CATKIN_DEVEL_PREFIX})
  add_custom_target(compile_openrave_devel ALL DEPENDS compile_openrave)
endif()

add_custom_command(OUTPUT ${OPENRAVE_SOURCE_DIR}
  COMMAND git clone https://github.com/rdiankov/openrave.git ${OPENRAVE_SOURCE_DIR}
  COMMAND cd ${OPENRAVE_SOURCE_DIR} && git reset --hard 7c5f5e27eec2b2ef10aa63fbc519a998c276f908 # production release of Feb 19 2018
  COMMAND cd ${OPENRAVE_SOURCE_DIR} && patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/python-path.patch # use dist-packages isntead of site-packages
  COMMAND cd ${OPENRAVE_SOURCE_DIR} && patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/no-warn.patch # disable -Wall
  )
add_custom_target(download_openrave
  DEPENDS ${OPENRAVE_SOURCE_DIR}
  )
add_custom_target(compile_openrave ## need to compile everythin for installed target
  COMMAND mkdir -p ${OPENRAVE_BINARY_DIR}
  COMMAND cd ${OPENRAVE_BINARY_DIR} && cmake ${OPENRAVE_SOURCE_DIR} ${OPENRAVE_COMPILE_OPT} -DCMAKE_INSTALL_PREFIX=${OPENRAVE_INSTALL_PREFIX}
  COMMAND make -C ${OPENRAVE_BINARY_DIR}
  COMMAND make -C ${OPENRAVE_BINARY_DIR} install
  DEPENDS download_openrave
  )

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --target compile_openrave)")
