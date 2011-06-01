cmake_minimum_required (VERSION 2.6.0) # using import/export targets
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
