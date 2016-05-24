# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(arm_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs actionlib_msgs sensor_msgs geometry_msgs trajectory_msgs tf message_generation)

add_message_Files(
  FILES AllowedCollisionEntry.msg AllowedCollisionMatrix.msg AllowedContactSpecification.msg ArmNavigationErrorCodes.msg AttachedCollisionObject.msg CollisionMap.msg CollisionObject.msg CollisionObjectOperation.msg CollisionOperation.msg Constraints.msg ContactInformation.msg DisplayTrajectory.msg JointConstraint.msg JointLimits.msg JointTrajectoryWithLimits.msg LinkPadding.msg MotionPlanRequest.msg MoveArmStatistics.msg MultiDOFJointState.msg MultiDOFJointTrajectory.msg MultiDOFJointTrajectoryPoint.msg OrderedCollisionOperations.msg OrientationConstraint.msg OrientedBoundingBox.msg PlanningScene.msg PositionConstraint.msg RobotState.msg RobotTrajectory.msg Shape.msg SimplePoseConstraint.msg VisibilityConstraint.msg WorkspaceParameters.msg
  DIRECTORY msg)

add_service_Files(
  FILES FilterJointTrajectory.srv FilterJointTrajectoryWithConstraints.srv GetCollisionObjects.srv GetJointTrajectoryValidity.srv GetMotionPlan.srv GetPlanningScene.srv GetRobotState.srv GetRobotTrajectoryValidity.srv GetStateValidity.srv SetPlanningSceneDiff.srv
  DIRECTORY srv)

add_action_files(
  FILES
  MakeStaticCollisionMap.action
  MoveArm.action
  SyncPlanningScene.action
  DIRECTORY action)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs sensor_msgs geometry_msgs trajectory_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs actionlib_msgs sensor_msgs geometry_msgs trajectory_msgs tf message_runtime
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
