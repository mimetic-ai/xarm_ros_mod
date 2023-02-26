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

global aruco_map, bowl_pos 
aruco_map = {}
bowl_pos = [0, 0, 0]

def roll(phi):
    q = quaternion.as_quat_array([np.cos(phi/2), np.sin(phi/2), 0, 0])
    return q

def pitch(theta):
    q = quaternion.as_quat_array([np.cos(theta/2), 0, np.sin(theta/2), 0])
    return q

def yaw(psi):
    q = quaternion.as_quat_array([np.cos(psi/2), 0, 0, np.sin(psi/2)])
    return q

def bowl_callback(data):
    global bowl_pos
    bowl_pos = np.array([data.x, data.y, data.z])

def aruco_callback(data):
  global aruco_map
  aruco_map[int(data.label)] = [data.aruco_center, data.aruco_x_axis, data.aruco_z_axis]

# Takes in a quaternion and returns a np.ndarray in the format [x, y, z, w]
# If quaternion is already indexable, ensures proper format: [x, y, z, w]
def quat_to_npy(q):
  if type(q) == quaternion.quaternion or geometry_msgs.msg._Quaternion.Quaternion:
    return np.array([q.x, q.y, q.z, q.w])
  else:
    raise TypeError('Variable of type {} should not be used.'.format(type(q)))
  

def msg_to_quat(q):
  if type(q) == geometry_msgs.msg._Quaternion.Quaternion:
    return quaternion.quaternion(q.w, q.x, q.y, q.z)
  else:
    raise TypeError('Variable of type {} should not be used.'.format(type(q)))
  
def pos_to_npy(pos):
  if type(pos) == geometry_msgs.msg._Vector3.Vector3 or geometry_msgs.msg._Point.Point:
    return np.array([pos.x, pos.y, pos.z])
  else:
    raise TypeError('Variable of type {} should not be used'.format(type(pos)))  

'''
so i have the position of the scoop in the gripper frame
i need to get the position of the scoop in the world frame
position of gripper in world frame plus rotation from gripper frame to world frame multiplied spoon in gripper frame
so i get the orientation of the spoon, convert to rotation matrix, transpose it and multiply by (0, 0, spn_len)
'''


# def moveSpoonOverBowl(arm_commander, spoon_len):
#   global bowl_pos
#   curr_pose = arm_commander.get_current_pose()
#   gripper_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(quat_to_npy(curr_pose.pose.orientation)))

#   # Calculating the position of the spoon's scooper in the world frame
#   temp = (gripper_orientation.T @ np.array([[0], [0], [spoon_len]])).reshape((-1))
#   print(temp)
#   spoon_pos_world = pos_to_npy(curr_pose.pose.position) + temp
#   print(spoon_pos_world)

#   # Moving the gripper to place the spoon above the bowl
#   gripper_to_spoon_vec = spoon_pos_world - pos_to_npy(curr_pose.pose.position)
#   print("gripper to spoon vec", gripper_to_spoon_vec)
#   print("bowl pos", bowl_pos)
#   # bowl_pos_2d = bowl_pos[:-1]
#   # gripper_xy = bowl_pos_2d - gripper_to_spoon_vec[:-1]
#   gripper_tgt_pos = bowl_pos - gripper_to_spoon_vec
#   print(gripper_tgt_pos)

#   tgt_pose = getPoseQ(gripper_tgt_pos, quaternion.from_rotation_matrix(gripper_orientation))
#   moveArm(arm_commander, tgt_pose)
#   tgt_pose.position.z = bowl_pos[-1] + 0.1
#   moveArm(arm_commander, tgt_pose)
    
# Takes in two quaternions and returns the smallest angle of rotation between them
def quat_distance(q1, q2):
  q1 = quat_to_npy(q1)
  q2 = quat_to_npy(q2)
  inner = np.inner(q1, q2)
  return np.arccos(2*inner**2 - 1)


##takes in position array and orientation quaternion and returns corresponding pose
def getPoseQ(position, orientation):
  goal_pose = geometry_msgs.msg.Pose()
  goal_pose.position.x = position[0]
  goal_pose.position.y = position[1]
  goal_pose.position.z = position[2]
  goal_pose.orientation.x = orientation.x
  goal_pose.orientation.y = orientation.y
  goal_pose.orientation.z = orientation.z
  goal_pose.orientation.w = orientation.w
  return goal_pose


##takes in position array and orientation values and returns corresponding pose
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


##moves to target pose
def moveArm(arm_commander, target_pose):
  arm_commander.set_pose_target(target_pose)
  arm_commander.go(wait=True)
  arm_commander.clear_pose_targets()

# def getWaypointDiag(start_pose, dest_point):
#  distance = 0
#  waypoints = []
#  distance_x = dest_point[0] - start_pose.position.x
#  distance_y = dest_point[0] - start_pose.position.y
#  increments_x = abs(distance_x)//0.1
#  increments_y = abs(distance_y)//0.1
#  for points in range(min(int(abs(increments_x)), int(abs(increments_y)))):
#    if distance_x >= 0.1:
#      start_pose.position.x += 0.1
#      if distance_y >= 0.1:
#        start_pose.position.y += 0.1
#      elif distance_y <= -0.1:
#        start_pose.position.y -= 0.1
#      waypoints.append(copy.deepcopy(start_pose))
#    elif distance_x <= -0.1:
#      start_pose.position.x -= 0.1
#      if distance_y >= 0.1:
#        start_pose.position.y += 0.1
#      elif distance_y <= -0.1:
#        start_pose.position.y -= 0.1
#      waypoints.append(copy.deepcopy(start_pose))
#  if distance_x >= 0:
#    start_pose.position.x += (abs(distance_x)%0.1)
#    if distance_y >= 0:
#     start_pose.position.y += (abs(distance_y)%0.1)
#    elif distance_y <= 0:
#     start_pose.position.y -= (abs(distance_y)%0.1)
#    waypoints.append(copy.deepcopy(start_pose))
#  elif distance_x <= 0:
#    start_pose.position.x -= (abs(distance_x)%0.1)
#    waypoints.append(copy.deepcopy(start_pose))
#  return waypoints

##get waypoints for the x direction between start and destination
def getWaypointX(start_pose, dest_point):
 distance = 0
 waypoints = []
 distance = dest_point[0] - start_pose.position.x
 increments = abs(distance)//0.1
 for points in range(int(abs(increments))):
   if distance >= 0.1:
     start_pose.position.x += 0.1
     waypoints.append(copy.deepcopy(start_pose))
   elif distance <= -0.1:
     start_pose.position.x -= 0.1
     waypoints.append(copy.deepcopy(start_pose))
 if distance >= 0:
   start_pose.position.x += (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 elif distance <= -0:
   start_pose.position.x -= (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 return waypoints

##get waypoints for the y direction between start and destination
def getWaypointY(start_pose, dest_point):
 distance = 0
 waypoints = []
 distance = dest_point[1] - start_pose.position.y
 increments = abs(distance)//0.1
 for points in range(int(abs(increments))):
   if distance >= 0.1:
     start_pose.position.y += 0.1
     waypoints.append(copy.deepcopy(start_pose))
   elif distance <= -0.1:
     start_pose.position.y -= 0.1
     waypoints.append(copy.deepcopy(start_pose))
 if distance >= 0:
   start_pose.position.y += (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 elif distance <= -0:
   start_pose.position.y -= (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 return waypoints

##get waypoint in z direction between start and destination
def getWaypointZ(start_pose, dest_point):
 distance = 0
 waypoints = []
 distance = dest_point[2] - start_pose.position.z
 increments = abs(distance)//0.1
 for points in range(int(abs(increments))):
   if distance >= 0.1:
     start_pose.position.z += 0.1
     waypoints.append(copy.deepcopy(start_pose))
   elif distance <= -0.1:
     start_pose.position.z -= 0.1
     waypoints.append(copy.deepcopy(start_pose))
 if distance >= 0:
   start_pose.position.z += (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 elif distance <= -0:
   start_pose.position.z -= (abs(distance)%0.1)
   waypoints.append(copy.deepcopy(start_pose))
 return waypoints
  
##moving spoon over bowl using waypoints
def moveSpoonOverBowlWaypoint(arm_commander, spoon_len, spoon_angle):
 global bowl_pos
 print("called go over bowl")
 curr_pose = arm_commander.get_current_pose().pose
 target_loc = [bowl_pos[0] - 0.6, bowl_pos[1], bowl_pos[2]]
 waypointsY = getWaypointY(curr_pose, target_loc)
 waypointsX = getWaypointX(curr_pose, target_loc)
 waypoints = waypointsY + waypointsX
 (plan, fraction) = arm_commander.compute_cartesian_path(
   waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold
 arm_commander.execute(plan, wait=True)
 arm_commander.clear_pose_targets()
 curr_pose = arm_commander.get_current_pose().pose
 return curr_pose

##grab spoon using waypoints 
def grabSpoonWaypoint(arm_commander, grab_commander, aruco_center):
 print("called grab spoon one")
 curr_pose = arm_commander.get_current_pose().pose
 target_loc = [aruco_center[0], aruco_center[1], aruco_center[2] - 0.02]
 waypointsY = getWaypointY(curr_pose, target_loc)
 waypointsX = getWaypointX(curr_pose, target_loc)
 waypointsZ = getWaypointZ(curr_pose, target_loc)
 waypoints = waypointsY + waypointsX + waypointsZ
#  waypoints = diagonalWaypoint + waypointsZ
 (plan, fraction) = arm_commander.compute_cartesian_path(
   waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold
 arm_commander.execute(plan, wait=True)
 arm_commander.clear_pose_targets()
 grab_commander.set_named_target('close')
 grab_commander.go(wait=True)
 grab_commander.clear_pose_targets()
 new_waypoint = []
 ##move up and tilt up
 curr_pose = arm_commander.get_current_pose().pose
 curr_pose.position.z += 0.06
 desired_angle = pitch(-1 * np.pi/30)
 ##aborted error
 # #desired_angle = pitch(np.pi/2)
 # ##aborted error
 # #desired_angle = yaw(np.pi/2)
 current_orientation_msg = curr_pose.orientation
 current_orientation = msg_to_quat(current_orientation_msg)
 new_orientation = desired_angle * current_orientation
 current_loc = [curr_pose.position.x, curr_pose.position.y, curr_pose.position.z]
 target_pose = getPoseQ(current_loc, new_orientation)
 new_waypoint.append(target_pose)
 (plan, fraction) = arm_commander.compute_cartesian_path(
   new_waypoint, 0.01, 0.0  # waypoints to follow  # eef_step
   )  # jump_threshold
 arm_commander.execute(plan, wait=True)
 arm_commander.clear_pose_targets()
 return arm_commander.get_current_pose()








##put spoon back using waypoints takes in home_pose
def goHomeWaypoint(arm_commander, grab_commander, home_pose):
  print("going home")
  curr_pose = arm_commander.get_current_pose().pose
  target_loc = [home_pose.position.x, home_pose.position.y, home_pose.position.z]
  waypointsY = getWaypointY(curr_pose, target_loc)
  waypointsX = getWaypointX(curr_pose, target_loc)
  waypoints = waypointsY + waypointsX
  (plan, fraction) = arm_commander.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
  arm_commander.execute(plan, wait=True)
  arm_commander.clear_pose_targets()
  new_waypoint = []
  curr_pose = arm_commander.get_current_pose().pose
  ##put down and tilt down
  curr_pose.position.z -= 0.06
  desired_angle = pitch(np.pi/30)
  ##aborted error
  #desired_angle = pitch(np.pi/2)

  ##aborted error
  #desired_angle = yaw(np.pi/2)
  current_orientation_msg = curr_pose.orientation
  current_orientation = msg_to_quat(current_orientation_msg)
  new_orientation = desired_angle * current_orientation
  current_loc = [curr_pose.position.x, curr_pose.position.y, curr_pose.position.z]
  target_pose = getPoseQ(current_loc, new_orientation)
  new_waypoint.append(target_pose)
  (plan, fraction) = arm_commander.compute_cartesian_path(
    new_waypoint, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
  arm_commander.execute(plan, wait=True)
  arm_commander.clear_pose_targets()
  grab_commander.set_named_target('open')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()
  new_waypoint_2 = []
  curr_pose = arm_commander.get_current_pose().pose
  curr_pose.position.z += 0.06
  new_waypoint_2.append(curr_pose)
  (plan, fraction) = arm_commander.compute_cartesian_path(
    new_waypoint_2, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
  arm_commander.execute(plan, wait=True)
  arm_commander.clear_pose_targets()



def dispense(arm_commander):
  print("called dispense")
  current_pose = arm_commander.get_current_pose().pose
  ##goes to wrong place
  desired_angle = roll(np.pi/3)
  ##aborted error
  #desired_angle = pitch(np.pi/2)

  ##aborted error
  #desired_angle = yaw(np.pi/2)
  current_orientation_msg = current_pose.orientation
  current_orientation = msg_to_quat(current_orientation_msg)
  new_orientation = desired_angle * current_orientation
  current_loc = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
  target_pose = getPoseQ(current_loc, new_orientation)
  moveArm(arm_commander=arm_commander, target_pose=target_pose)
  current_pose = arm_commander.get_current_pose().pose
  ##goes to wrong place
  desired_angle = roll(-1 * np.pi/3)
  ##aborted error
  #desired_angle = pitch(np.pi/2)

  ##aborted error
  #desired_angle = yaw(np.pi/2)
  current_orientation_msg = current_pose.orientation
  current_orientation = msg_to_quat(current_orientation_msg)
  new_orientation = desired_angle * current_orientation
  current_loc = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
  target_pose = getPoseQ(current_loc, new_orientation)
  moveArm(arm_commander=arm_commander, target_pose=target_pose)
  
  





# ##moves robotic arm from current position to the position in front of spoon1, takes in arm commander and aruco location info
# ##position: x, y, z
# ##orientation: x, y, z, w
# def frontOfSpoonOne(arm_commander, grab_commander, grab_pos, x_axis, y_axis, z_axis):
#   #grab_pos = grab_pos + (-0.01 * y_axis) # + (0.005 * x_axis)
#   M = np.array([y_axis, z_axis, -x_axis]).reshape((3,3))
#   #M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
#   q = quaternion.from_rotation_matrix(M)
#   goal_pose = getPoseQ(grab_pos, q)
#   moveArm(arm_commander, goal_pose)
#   current_pose = arm_commander.get_current_pose()
#   ##adjusting pose to grab position
#   current_pose.pose.position.x += 0.02
#   current_pose.pose.position.z -= 0.03
#   moveArm(arm_commander, current_pose)
#   ##grabbing and moving up to final pose
#   grab_commander.set_named_target('close')
#   grab_commander.go(wait=True)
#   grab_commander.clear_pose_targets()
  
#   # Creating a rotation matrix where spoon is horizontal to the table
#   M = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]).reshape((3,3))
#   q = quaternion.from_rotation_matrix(M)
#   q = pitch(np.pi/8) * q
  
#   up_pose = copy.deepcopy(current_pose)
#   up_pose.pose.position.z += 0.15
#   up_pose.pose.orientation.w = q.w
#   up_pose.pose.orientation.x = q.x
#   up_pose.pose.orientation.y = q.y
#   up_pose.pose.orientation.z = q.z
  
#   moveArm(arm_commander, up_pose)
#   return current_pose

# def frontOfSpoonTwo(arm_commander, grab_commander, grab_pos, x_axis, y_axis, z_axis):
#   #grab_pos = grab_pos + (-0.01 * y_axis) # + (0.005 * x_axis)
#   M = np.array([y_axis, z_axis, x_axis]).reshape((3,3))
#   #M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
#   q = quaternion.from_rotation_matrix(M)
#   goal_pose = getPoseQ(grab_pos - 0.05*z_axis - 0.03*y_axis, q)
#   moveArm(arm_commander, goal_pose)
#   current_pose = arm_commander.get_current_pose()
#   ##adjusting pose to grab position
#   current_pose.pose.position.x += 0.05*z_axis[0]
#   current_pose.pose.position.y += 0.05*z_axis[1]
#   current_pose.pose.position.z += 0.05*z_axis[2]
#   moveArm(arm_commander, current_pose)
#   ##grabbing and moving up to final pose
#   grab_commander.set_named_target('close')
#   grab_commander.go(wait=True)
#   grab_commander.clear_pose_targets()
  
#   # Creating a rotation matrix where spoon is horizontal to the table
#   M = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]).reshape((3,3))
#   q = quaternion.from_rotation_matrix(M)
#   q = pitch(np.pi/8) * q
  
#   up_pose = copy.deepcopy(current_pose)
#   up_pose.pose.position.z += 0.15
#   up_pose.pose.orientation.w = q.w
#   up_pose.pose.orientation.x = q.x
#   up_pose.pose.orientation.y = q.y
#   up_pose.pose.orientation.z = q.z
  
#   moveArm(arm_commander, up_pose)
#   return current_pose

def int_callback(data, args):
  global aruco_map, bowl_pos
  # print(aruco_map.keys())
  arm_commander = args[0]
  grab_commander = args[1]
  #print(arm_commander.get_current_pose())
  
  center, x_axis, z_axis = aruco_map[data.data]
  # x_axis corresponds to vector pointing from corner 1 to corner 0 of aruco code
  # z_axis corresponds to vector pointing from corner 2 to corner 1 of aruco code

  # print(type(center), type(x_axis), type(z_axis)) # I don't remember what this is; Believed geometry_msgs.msg._Vector3.Vector3
  x_axis = np.array([x_axis.x, x_axis.y, x_axis.z])
  z_axis = np.array([z_axis.x, z_axis.y, z_axis.z])
  y_axis = np.cross(z_axis, x_axis) # y_axis comes from z (cross) x

  spoon_len = 0.6
  spoon_angle = np.arctan2(z_axis[0], z_axis[1])
  bowl_pos = [0.798, 0, 0.608]

  center = np.array([center.x, center.y, center.z])
  #mod_grab_pos = center + (-0.035 * y_axis) + (0.01 * z_axis)  # desired grab position of the spoon
  mod_grab_pos = center  # desired grab position of the spoon
  # modifies gripper grabbing location in the aruco frame; center is used as origin
  spoon_grab_pose = grabSpoonWaypoint(arm_commander=arm_commander, grab_commander=grab_commander, aruco_center=center)
  moveSpoonOverBowlWaypoint(arm_commander=arm_commander, spoon_len=spoon_len, spoon_angle=spoon_angle)
  dispense(arm_commander = arm_commander)
  goHomeWaypoint(arm_commander=arm_commander, home_pose=spoon_grab_pose.pose, grab_commander=grab_commander)

  # spoon_grab_pose = [] # When dropping, gripper will return to where spoon was grabbed
  # if data.data == '1':
  #   #spoon_grab_pose = frontOfSpoonOne(arm_commander=arm_commander, grab_commander=grab_commander, grab_pos=mod_grab_pos, x_axis=x_axis, y_axis= y_axis, z_axis = z_axis)
  #   spoon_grab_pose = grabSpoonWaypoint(arm_commander=arm_commander, grab_commander=grab_commander, aruco_center=center)
  #   moveSpoonOverBowlWaypoint(arm_commander=arm_commander, spoon_len=spoon_len, spoon_angle=spoon_angle)
  #   goHomeWaypoint(arm_commander=arm_commander, home_pose=spoon_grab_pose.pose)
  # else:
  #   spoon_grab_pose = frontOfSpoonTwo(arm_commander=arm_commander, grab_commander=grab_commander, grab_pos=mod_grab_pos, x_axis=x_axis, y_axis= y_axis, z_axis = z_axis)
  #   moveSpoonOverBowlWaypoint(arm_commander=arm_commander, spoon_len=spoon_len, spoon_angle=spoon_angle)
  #   goHomeWaypoint(arm_commander=arm_commander, home_pose=spoon_grab_pose.pose)


 

def receive_message():

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('move_arm_basic', anonymous=True)
  moveit_commander.roscpp_initialize(sys.argv)
  grab_commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
  arm_commander = moveit_commander.MoveGroupCommander("xarm6")
  scene = moveit_commander.PlanningSceneInterface()
  rospy.sleep(1)
  #print(arm_commander.get_current_pose())
  rospy.Subscriber('/spoon_aruco_frame', ArucoFrame, aruco_callback)
  rospy.Subscriber('/which_aruco', UInt8, int_callback, (arm_commander, grab_commander))
  rospy.Subscriber('/filtered_bowl_pos', geometry_msgs.msg.Vector3, bowl_callback)

  p = geometry_msgs.msg.PoseStamped()
  p.header.frame_id = "world"
  p.pose.position.x = 0.7
  p.pose.position.y = 0
  p.pose.position.z = -0.03
  scene.add_box("table", p, (0.7, 0.5, 0.07))
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
 
  # # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
if __name__ == '__main__':
  receive_message()