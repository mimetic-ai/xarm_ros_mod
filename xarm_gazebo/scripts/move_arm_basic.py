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

global aruco_map 
aruco_map = {}

def callback(data):
  global aruco_map
  aruco_map[int(data.label)] = [data.aruco_center, data.aruco_x_axis, data.aruco_z_axis]

def int_callback(data, args):
  global aruco_map
  print(aruco_map.keys())
  arm_commander = args[0]
  grab_commander = args[1]
  center, x_axis, z_axis = aruco_map[data.data]
  # print(type(center), type(x_axis), type(z_axis))
  x_axis = np.array([x_axis.x, x_axis.y, x_axis.z])
  z_axis = np.array([z_axis.x, z_axis.y, z_axis.z])
  y_axis = np.cross(z_axis, x_axis)

  center = np.array([center.x, center.y, center.z])
  mod_grab_pos = center + 0.1 * z_axis

  M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
  q = quaternion.from_rotation_matrix(M)

  goal_pose = geometry_msgs.msg.Pose()
  goal_pose.position.x = mod_grab_pos[0]
  goal_pose.position.y = mod_grab_pos[1]
  goal_pose.position.z = mod_grab_pos[2]

  goal_pose.orientation.w = q.w
  goal_pose.orientation.x = q.x
  goal_pose.orientation.y = q.y
  goal_pose.orientation.z = q.z

  arm_commander.set_pose_target(goal_pose)
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()
  # rospy.logwarn('finished moving, going to ready position after 2 sec')
  # rospy.sleep(2)

  grab_commander.set_named_target('close')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()

  rospy.sleep(5)

  arm_commander.set_named_target('hold-up')
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('move_arm_basic', anonymous=True)
  moveit_commander.roscpp_initialize(sys.argv)
  grab_commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
  arm_commander = moveit_commander.MoveGroupCommander("xarm6")
  rospy.Subscriber('/spoon_aruco_frame', ArucoFrame, callback)
  rospy.Subscriber('/which_aruco', UInt8, int_callback, (arm_commander, grab_commander))

  arm_commander.set_max_velocity_scaling_factor(1.0)
  arm_commander.set_max_acceleration_scaling_factor(1.0)

  grab_commander.set_max_velocity_scaling_factor(1.0)
  grab_commander.set_max_acceleration_scaling_factor(1.0)
  # Node is subscribing to the video_frames topic
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
if __name__ == '__main__':
  receive_message()

