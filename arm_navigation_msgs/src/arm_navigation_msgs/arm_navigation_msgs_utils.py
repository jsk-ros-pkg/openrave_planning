#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
from arm_navigation_msgs.msg import CollisionOperation

# This function returns a bunch of ordered collision operations
# to disable collisions with all the links not in exclude
def make_disable_allowed_collisions_with_exclusions(all, exclude):
    
    ret = []

    for i in all:
        if i not in exclude:
            coll = CollisionOperation()
            coll.object1 = i
            coll.object2 = coll.COLLISION_SET_OBJECTS
            coll.operation = coll.DISABLE
            ret.append(coll)
            coll2 = CollisionOperation()
            coll2.object1 = i
            coll2.object2 = coll.COLLISION_SET_ATTACHED_OBJECTS
            coll2.operation = coll.DISABLE
            ret.append(coll2)
            for j in all:
                if j != i and j not in exclude:
                    coll3 = CollisionOperation()
                    coll3.object1 = i
                    coll3.object2 = j
                    coll3.operation = coll.DISABLE
                    ret.append(coll3)
    return ret
