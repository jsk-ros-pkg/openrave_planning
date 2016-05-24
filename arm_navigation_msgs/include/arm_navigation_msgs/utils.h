/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <algorithm>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/MoveArmGoal.h>

namespace arm_navigation_msgs
{

/** @brief Add a goal constraint to the move arm action goal.
    @param A reference to a simple pose constraint.
    @param A reference to a move arm goal message. The pose constraint will be added to the goal message as a position and orientation constraint.
*/
void addGoalConstraintToMoveArmGoal(const arm_navigation_msgs::SimplePoseConstraint &pose_constraint, arm_navigation_msgs::MoveArmGoal &move_arm_goal)
{
  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  poseConstraintToPositionOrientationConstraints(pose_constraint,position_constraint,orientation_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);
}

/** @brief Generate ordered collision operates that disable all collisions in a vector of names except
           for a specied vector of names
    @param All names
    @param Names to exclude
    @param Vector for appending collision_operations
*/

inline void generateDisableAllowedCollisionsWithExclusions(const std::vector<std::string>& all_names,
                                                           const std::vector<std::string>& exclude_names,
                                                           std::vector<arm_navigation_msgs::CollisionOperation>& collision_operations) 
{
  for(std::vector<std::string>::const_iterator it = all_names.begin();
      it != all_names.end();
      it++) {
    if(std::find(exclude_names.begin(), exclude_names.end(), *it) == exclude_names.end()) {
      arm_navigation_msgs::CollisionOperation coll;
      coll.object1 = *it;
      coll.object2 = coll.COLLISION_SET_OBJECTS;
      coll.operation = coll.DISABLE;
      collision_operations.insert(collision_operations.end(),coll);
      coll.object2 = coll.COLLISION_SET_ATTACHED_OBJECTS;
      collision_operations.insert(collision_operations.end(),coll);
      for(std::vector<std::string>::const_iterator it2 = all_names.begin();
          it2 != all_names.end();
          it2++) {
        if(*it != *it2 && std::find(exclude_names.begin(), exclude_names.end(), *it2) == exclude_names.end()) {
          coll.object1 = *it;
          coll.object2 = *it2;
          collision_operations.insert(collision_operations.end(),coll);
        }
      }
    }
  }
}
}
