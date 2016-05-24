cmake_minimum_required(VERSION 2.8.3)
project(openraveros)
find_package(catkin REQUIRED COMPONENTS roscpp openrave roscpp_sessions std_msgs sensor_msgs message_generation)

add_message_files(FILES
  AABB.msg 	ActiveDOFs.msg
  AffineTransformMatrix.msg 	AttachedSensor.msg
  BodyInfo.msg 	ConfigurationSpecification.msg
  ConfigurationSpecificationGroup.msg 	Contact.msg
  Manipulator.msg 	PlannerParameters.msg
  Ray.msg 	RobotInfo.msg
  Trajectory.msg
)
add_service_files(FILES
  body_destroy.srv 	body_enable.srv
  body_getaabb.srv 	body_getaabbs.srv
  body_getdof.srv 	body_getjointvalues.srv
  body_setjointvalues.srv 	body_settransform.srv
  env_checkcollision.srv 	env_closefigures.srv
  env_createbody.srv 	env_createmodule.srv
  env_createplanner.srv 	env_createrobot.srv
  env_destroymodule.srv 	env_getbodies.srv
  env_getbody.srv 	env_getrobots.srv
  env_loadplugin.srv 	env_loadscene.srv
  env_plot.srv 	env_raycollision.srv
  env_set.srv 	env_triangulate.srv
  env_wait.srv 	module_sendcommand.srv
  openrave_session.srv 	planner_init.srv
  planner_plan.srv 	robot_controllersend.srv
  robot_controllerset.srv 	robot_getactivevalues.srv
  robot_sensorgetdata.srv 	robot_sensorsend.srv
  robot_setactivedofs.srv 	robot_setactivemanipulator.srv
  robot_setactivevalues.srv 	robot_starttrajectory.srv
)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(openraveros src/rosserver.cpp src/openraveros.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
target_link_libraries(openraveros ${Boost_LIBRARIES} ${catkin_LIBRARIES})
