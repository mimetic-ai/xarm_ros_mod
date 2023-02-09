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

global aruco_map 
aruco_map = {}
bowl_pos = [0, 0, 0]

def bowl_callback(data):
    global bowl_pos
    bowl_pos = np.array([data.x, data.y, data.z])

def aruco_callback(data):
  global aruco_map
  aruco_map[int(data.label)] = [data.aruco_center, data.aruco_x_axis, data.aruco_z_axis]

def int_callback(data, args):
  global aruco_map, bowl_pos
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



  T1 = np.array([0, 0, 0.1]) # Translation from AruCo to grab
  R1 = t.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3] # Rotation from AruCo to grab
  # aruco_to_grab = t.concatenate_matrices(T, R)
  # grab_to_aruco = t.inverse_matrix(transform)
  # print(inv_tf)

  T2 = np.array([0, 0, 0.2]) # Translation from AruCo to spoon-end
  R2 = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) # Rotation from AruCo to spoon-end
  # aruco_to_scoop = t.concatenate_matrices(T, R)
  # scoop_to_aruco = t.inverse_matrix(transform)
  # print(inv_tf)

  R12 = R2@R1.T
  t12 = R1@np.array([T2 - T1]).T

  temp = np.eye(4)
  temp[:3,:3] = R12
  R12 = temp
  del temp
  T12 = t.translation_matrix((t12.T)[0])
  grab_to_end = t.concatenate_matrices(R12, T12)
  end_to_grab = t.inverse_matrix(grab_to_end)
  T = t.translation_from_matrix(end_to_grab)
  R = t.quaternion_from_matrix(end_to_grab)

  end_to_grab_tf = geometry_msgs.msg.TransformStamped()
  end_to_grab_tf.transform.translation.x = T[0]
  end_to_grab_tf.transform.translation.y = T[1]
  end_to_grab_tf.transform.translation.z = T[2]
  end_to_grab_tf.transform.rotation.x = R[0]
  end_to_grab_tf.transform.rotation.y = R[1]
  end_to_grab_tf.transform.rotation.z = R[2]
  end_to_grab_tf.transform.rotation.w = R[3]


  arm_commander.set_pose_target(goal_pose)
  print("96 moving to spoon")
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()
  # rospy.logwarn('finished moving, going to ready position after 2 sec')
  rospy.sleep(1) 

  print('99 closing gripper')
  grab_commander.set_named_target('close')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()

  rospy.sleep(1)

  over_bowl_spoon_pose = geometry_msgs.msg.PoseStamped()
  over_bowl_spoon_pose.pose.position.x = bowl_pos[0]
  over_bowl_spoon_pose.pose.position.y = bowl_pos[1]
  over_bowl_spoon_pose.pose.position.z = bowl_pos[2]
  over_bowl_spoon_pose.pose.orientation.w = np.cos(3*np.pi/4)
  over_bowl_spoon_pose.pose.orientation.x = 0
  over_bowl_spoon_pose.pose.orientation.y = 0
  over_bowl_spoon_pose.pose.orientation.z = np.sin(3*np.pi/4)

  over_bowl_hand_pose = tf_gm.do_transform_pose(over_bowl_spoon_pose, end_to_grab_tf)

  # traj_constraints = moveit_msgs.msg.Constraints()
  # ocm = moveit_msgs.msg.OrientationConstraint()
  # ocm.link_name = "xarm_gripper"
  # ocm.header.frame_id = "xarm_gripper"
  # ocm.orientation.x = 0
  # ocm.orientation.y = 0
  # traj_constraints.orientation_constraints.append(ocm)
  print('124 moving spoon over bowl')
  arm_commander.set_pose_target(over_bowl_hand_pose)
  # arm_commander.set_path_constraints(traj_constraints)
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()

  grab_commander.set_named_target('open')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()

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
  rospy.Subscriber('/spoon_aruco_frame', ArucoFrame, aruco_callback)
  rospy.Subscriber('/which_aruco', UInt8, int_callback, (arm_commander, grab_commander))
  rospy.Subscriber('/filtered_bowl_pos', geometry_msgs.msg.Vector3, bowl_callback)

  arm_commander.set_max_velocity_scaling_factor(1.0)
  arm_commander.set_max_acceleration_scaling_factor(1.0)

  grab_commander.set_max_velocity_scaling_factor(1.0)
  grab_commander.set_max_acceleration_scaling_factor(1.0)

  arm_commander.set_named_target('hold-up')
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()

  grab_commander.set_named_target('open')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()

  # Node is subscribing to the video_frames topic
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
if __name__ == '__main__':
  receive_message()

