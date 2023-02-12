#!/usr/bin/env python3
import rospy # Python library for ROS
from ffa_msgs.msg import ArucoFrame
import moveit_commander
import sys
import copy
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
from std_msgs.msg import UInt8
import numpy as np
import quaternion
from tf import transformations as t
import tf2_geometry_msgs.tf2_geometry_msgs as tf_gm
import math



rospy.init_node('hardset_motions', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
grab_commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
arm_commander = moveit_commander.MoveGroupCommander("xarm6")

def getPose(position, orientation):
  goal_pose = geometry_msgs.msg.Pose()
  goal_pose.position.x = position[0]
  goal_pose.position.y = position[1]
  goal_pose.position.z = position[2]
  goal_pose.orientation.x = orientation[0]
  goal_pose.orientation.y = orientation[1]
  goal_pose.orientation.z = orientation[2]
  goal_pose.orientation.w = orientation[3]
  return goal_pose
def moveArm(arm_commander, target_pose):
  arm_commander.set_pose_target(target_pose)
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()



print("values initialized")
goal_pose = geometry_msgs.msg.Pose()
left_pose = arm_commander.get_current_pose()
print(left_pose)
# position = [left_pose.position.x + 0.1, left_pose.position.y, left_pose.position.z]
# orientation = [left_pose.orientation.x, left_pose.orientation.y, left_pose.orientation.z, left_pose.orientation.w]
# target_pose = getPose(position, orientation)
# moveArm(arm_commander,target_pose)
# # grab_commander.set_named_target('close')
# # grab_commander.go(wait=True)
# # grab_commander.clear_pose_targets()
goal_pose.position.x = 0.6205737395860738
goal_pose.position.y = 0.07737037078351983
goal_pose.position.z = 0.14485234781784814
goal_pose.orientation.x = -0.31419790328833525
goal_pose.orientation.y = 0.8036317048344582
goal_pose.orientation.z = 0.47026131785865344
goal_pose.orientation.w = 0.18522973163053247




arm_commander.set_pose_target(goal_pose)
arm_commander.go(wait=True)
arm_commander.clear_pose_targets()

# # print('99 closing gripper')
# # grab_commander.set_named_target('open')
# # grab_commander.go(wait=True)
# # grab_commander.clear_pose_targets()
# # print(arm_commander.get_current_pose())
# # rospy.sleep(1)


# moveArm(arm_commander,left_pose)
