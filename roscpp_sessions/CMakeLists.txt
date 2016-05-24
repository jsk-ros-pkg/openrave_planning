cmake_minimum_required(VERSION 2.8.3)
project(roscpp_sessions)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

add_service_files(FILES
  add_variables.srv get_variable.srv 	set_variable.srv 	simple_session.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

