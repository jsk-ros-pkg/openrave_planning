# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(openrave)
find_package(catkin REQUIRED COMPONENTS std_msgs)

# build openrave
add_custom_target(compile_openrave ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/Makefile
  COMMAND export EUSDIR=${PROJECT_SOURCE_DIR} lt_cv_sys_lib_dlsearch_path_spec=${lt_cv_sys_lib_dlsearch_path_spec} && \$\(MAKE\) -C ${PROJECT_SOURCE_DIR} -f Makefile)

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build . --target compile_openrave)")

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS collada_robots libxml2 python-sympy bullet collada-dom qt4-dev-tools libqt4-dev libqt4-opengl-dev libcoin60-dev libsoqt4-dev python-scipy mayavi
    CATKIN_DEPENDS std_msgs# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)