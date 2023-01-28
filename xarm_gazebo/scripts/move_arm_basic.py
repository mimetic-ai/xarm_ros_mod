#!/usr/bin/env python3
import rospy # Python library for ROS
from ffa_msgs.msg import ArucoFrame
import moveit_commander
import sys
import copy
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
import moveit_commander






def callback(data, args):
  arm_commander = args[0]
  target_x = data.aruco_center.x
  target_y = data.aruco_center.y
  target_z = data.aruco_center.z
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x =  0.00281352
  pose_goal.orientation.y =  0.932559
  pose_goal.orientation.z =  -0.0804606
  pose_goal.orientation.w =  0.351927
  pose_goal.position.x = target_x
  pose_goal.position.y = target_y
  pose_goal.position.z = target_z
  arm_commander.set_pose_target(pose_goal)
  success = arm_commander.go(wait=True)
  arm_commander.stop()
  arm_commander.clear_pose_targets()




      
   

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  moveit_commander.roscpp_initialize(sys.argv)
  gripper_commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
  arm_commander = moveit_commander.MoveGroupCommander("xarm6")
  rospy.init_node('move_arm_basic', anonymous=True)
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/spoon_aruco_frame', ArucoFrame, callback, (arm_commander, gripper_commander))
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
if __name__ == '__main__':
  receive_message()

