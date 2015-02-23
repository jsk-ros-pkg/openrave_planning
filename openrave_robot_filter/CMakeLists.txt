# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(openrave_robot_filter)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf tf2 filters sensor_msgs resource_retriever openrave bullet qhull)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)



# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

add_executable(robotlinks_filter_node src/robotlinks_filter_node.cpp)
target_link_libraries (robotlinks_filter_node ${OPENRAVE_LIBRARIES} qhull)

find_package(Boost REQUIRED COMPONENTS signals)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(robotlinks_filter_node ${Boost_LIBRARIES})

# check for OpenMP
include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)

if( WIN32 )
  CHECK_INCLUDE_FILE(omp.h HAVE_OMP_H)
  if( HAVE_OMP_H )
    message(STATUS "Using OpenMP")
    check_cxx_compiler_flag(/openmp HAVE_OPENMP)

    if( HAVE_OPENMP )
      add_definitions("/openmp")
    endif( HAVE_OPENMP )
  endif( HAVE_OMP_H )
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

  # check if compilers supports -fopenmp
  check_cxx_compiler_flag(-fopenmp HAVE_OPENMP)
  check_library_exists(gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if( HAVE_OPENMP AND HAS_GOMP_LIB )
    add_definitions("-fopenmp")
    target_link_libraries(robotlinks_filter_node gomp)
    set(OPENMP_LFLAGS "-lgomp")
  endif( HAVE_OPENMP AND HAS_GOMP_LIB )
endif( WIN32 )
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf tf2 filters sensor_msgs resource_retriever openrave bullet qhull
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)