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


def moveSpoonOverBowl(arm_commander, spoon_len):
  global bowl_pos
  curr_pose = arm_commander.get_current_pose()
  gripper_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(quat_to_npy(curr_pose.pose.orientation)))

  # Calculating the position of the spoon's scooper in the world frame
  temp = (gripper_orientation.T @ np.array([[0], [0], [spoon_len]])).reshape((-1))
  print(temp)
  spoon_pos_world = pos_to_npy(curr_pose.pose.position) + temp
  print(spoon_pos_world)

  # Moving the gripper to place the spoon above the bowl
  gripper_to_spoon_vec = spoon_pos_world - pos_to_npy(curr_pose.pose.position)
  print("gripper to spoon vec", gripper_to_spoon_vec)
  print("bowl pos", bowl_pos)
  # bowl_pos_2d = bowl_pos[:-1]
  # gripper_xy = bowl_pos_2d - gripper_to_spoon_vec[:-1]
  gripper_tgt_pos = bowl_pos - gripper_to_spoon_vec
  print(gripper_tgt_pos)

  tgt_pose = getPoseQ(gripper_tgt_pos, quaternion.from_rotation_matrix(gripper_orientation))
  moveArm(arm_commander, tgt_pose)
  tgt_pose.position.z = bowl_pos[-1] + 0.1
  moveArm(arm_commander, tgt_pose)
    
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


##moves robotic arm from current position to the position in front of spoon1, takes in arm commander and aruco location info
##position: x, y, z
##orientation: x, y, z, w
def frontOfSpoonOne(arm_commander, grab_commander, grab_pos, x_axis, y_axis, z_axis):
  #grab_pos = grab_pos + (-0.01 * y_axis) # + (0.005 * x_axis)
  M = np.array([y_axis, z_axis, -x_axis]).reshape((3,3))
  #M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
  q = quaternion.from_rotation_matrix(M)
  goal_pose = getPoseQ(grab_pos, q)
  moveArm(arm_commander, goal_pose)
  current_pose = arm_commander.get_current_pose()
  ##adjusting pose to grab position
  current_pose.pose.position.x += 0.02
  current_pose.pose.position.z -= 0.03
  moveArm(arm_commander, current_pose)
  ##grabbing and moving up to final pose
  grab_commander.set_named_target('close')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()
  
  # Creating a rotation matrix where spoon is horizontal to the table
  M = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]).reshape((3,3))
  q = quaternion.from_rotation_matrix(M)
  q = pitch(np.pi/8) * q
  
  up_pose = copy.deepcopy(current_pose)
  up_pose.pose.position.z += 0.15
  up_pose.pose.orientation.w = q.w
  up_pose.pose.orientation.x = q.x
  up_pose.pose.orientation.y = q.y
  up_pose.pose.orientation.z = q.z
  
  moveArm(arm_commander, up_pose)
  return current_pose

def frontOfSpoonTwo(arm_commander, grab_commander, grab_pos, x_axis, y_axis, z_axis):
  #grab_pos = grab_pos + (-0.01 * y_axis) # + (0.005 * x_axis)
  M = np.array([y_axis, z_axis, -x_axis]).reshape((3,3))
  #M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
  q = quaternion.from_rotation_matrix(M)
  goal_pose = getPoseQ(grab_pos - 0.05*z_axis - 0.03*y_axis, q)
  moveArm(arm_commander, goal_pose)
  current_pose = arm_commander.get_current_pose()
  ##adjusting pose to grab position
  current_pose.pose.position.x += 0.05*z_axis[0]
  current_pose.pose.position.y += 0.05*z_axis[1]
  current_pose.pose.position.z += 0.05*z_axis[2]
  moveArm(arm_commander, current_pose)
  ##grabbing and moving up to final pose
  grab_commander.set_named_target('close')
  grab_commander.go(wait=True)
  grab_commander.clear_pose_targets()
  
  # Creating a rotation matrix where spoon is horizontal to the table
  M = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]).reshape((3,3))
  q = quaternion.from_rotation_matrix(M)
  q = pitch(np.pi/8) * q
  
  up_pose = copy.deepcopy(current_pose)
  up_pose.pose.position.z += 0.15
  up_pose.pose.orientation.w = q.w
  up_pose.pose.orientation.x = q.x
  up_pose.pose.orientation.y = q.y
  up_pose.pose.orientation.z = q.z
  
  moveArm(arm_commander, up_pose)
  return current_pose


# def tweakPosition(arm_commander, grab_pos, x_axis, y_axis, z_axis):
#   M = np.array([-z_axis, x_axis, y_axis]).reshape((3,3))
#   q = quaternion.from_rotation_matrix(M)
#   goal_pose = getPoseQ(grab_pos, q)
#   moveArm(arm_commander, goal_pose)
#   current_pose = arm_commander.get_current_pose()
#   return current_pose


# ##moves robotic arm from current posotion to the position to the left of spoon1, takes in arm commander and aruco location info
# def leftOfSpoonOne(arm_commander, grab_pos, x_axis, y_axis, z_axis):
#   # This code was for the left or right approach
#   M1 = np.array([-z_axis, y_axis, x_axis]).reshape((3,3))
#   M2 = np.array([z_axis, y_axis, -x_axis]).reshape((3,3))
#   # Rows of rotation matrix match to axes of gripper
#   # First row is gripper x axis, second is y_axis, and third is z_axis
#   # in our case:
#   # gripper's z_axis should be the aruco's z_axis
#   # gripper's x_axis shoule be the aruco's y_axis
#   # gripper's y_axis should be the aruco's -x_axis
#   curr_q = arm_commander.get_current_pose().pose.orientation
#   q1 = quaternion.from_rotation_matrix(M1)
#   q2 = quaternion.from_rotation_matrix(M2) 
#   goal_pose = []
#   relativePos = None
#   # need to get rid of the two quaternion check
#   if quat_distance(curr_q, q1) < quat_distance(curr_q, q2): # checks min. dist. b/w 2 quats 
#     goal_pose = getPoseQ(grab_pos + 0.05*x_axis, q1)
#     relativePos = 'left'
#   else:
#     goal_pose = getPoseQ(grab_pos - 0.05*x_axis, q2)
#     relativePos = 'right'
#   moveArm(arm_commander, goal_pose)
#   current_pose = arm_commander.get_current_pose()
#   return current_pose, relativePos


###takes in string specifying relative position, grabs spoon
##CASE SENSITIVE
# def grabSpoon(arm_commander, grab_commander, relativePos, x_axis, y_axis, z_axis):
#   current_pos = arm_commander.get_current_pose().pose.position
#   position = np.array([current_pos.x, current_pos.y, current_pos.z])
#   if relativePos == 'straight':
#     grab_commander.set_named_target('close')
#     grab_commander.go(wait=True)
#     grab_commander.clear_pose_targets()
#     return
#   elif relativePos == 'left':
#         # tweakPosition(arm_commander=arm_commander, grab_pos = position, x_axis=x_axis, y_axis=y_axis, z_axis=z_axis)
#     #slightly adjusts gripper position so that spoon is grabbable
#     #TO-do: change approach so that pose is generalizable not hardset
#     position -= (0.05*x_axis)
#     orientation = arm_commander.get_current_pose().pose.orientation
#     target_pose = getPoseQ(position, orientation)
#     moveArm(arm_commander,target_pose)
#     grab_commander.set_named_target('close')
#     grab_commander.go(wait=True)
#     grab_commander.clear_pose_targets()
#     return
#   elif relativePos == 'right':
#     position += (0.05*x_axis)
#     orientation = arm_commander.get_current_pose().pose.orientation
#     target_pose = getPoseQ(position, orientation)
#     moveArm(arm_commander,target_pose)
#     grab_commander.set_named_target('close')
#     grab_commander.go(wait=True)
#     grab_commander.clear_pose_targets()
#     return
  

# def goHome(arm_commander, relativePos, intermediatePos):
#   if relativePos == 'straight':
#     arm_commander.set_named_target('hold-up')
#     arm_commander.go(wait=True)
#     arm_commander.clear_pose_targets()
#     return 
#   elif relativePos == 'left':
#     moveArm(arm_commander=arm_commander, target_pose=intermediatePos)
#     arm_commander.set_named_target('hold-up')
#     arm_commander.go(wait=True)
#     arm_commander.clear_pose_targets()
#     return



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

  center = np.array([center.x, center.y, center.z])
  #mod_grab_pos = center + (-0.035 * y_axis) + (0.01 * z_axis)  # desired grab position of the spoon
  mod_grab_pos = center  # desired grab position of the spoon
  # modifies gripper grabbing location in the aruco frame; center is used as origin

  spoon_grab_pose = [] # When dropping, gripper will return to where spoon was grabbed
  if data.data == '1':
    spoon_grab_pose = frontOfSpoonOne(arm_commander=arm_commander, grab_commander=grab_commander, grab_pos=mod_grab_pos, x_axis=x_axis, y_axis= y_axis, z_axis = z_axis)
  else:
    spoon_grab_pose = frontOfSpoonTwo(arm_commander=arm_commander, grab_commander=grab_commander, grab_pos=mod_grab_pos, x_axis=x_axis, y_axis= y_axis, z_axis = z_axis)
  moveSpoonOverBowl(arm_commander, 0.3)
  # Assuming we have grabbed the spoon and moved up 15 cm now
  # if we save the spoon_grab position, we can use this as the spoon drop position
  # grabSpoon(arm_commander= arm_commander, grab_commander= grab_commander, relativePos=relativePos, x_axis=x_axis, y_axis = y_axis, z_axis=z_axis)
  # goHome(arm_commander = arm_commander, relativePos = relativePos, intermediatePos = intermediate_pos)

  # Next steps:
  # the transform from the gripper frame to the scoop frame is:
    # the scoop's x_axis is the gripper's z_axis
    # the scoop's y_axis is the gripper's -y_axis
    # the scoop's z_axis is the gripper's x_axis
    # the scoop's origin is /alpha (cm) down the spoon from the aruco code's center 
    # alpha depends on the length of the spoon; don't know off the top of my head
    # this info is given in the scoop frame, but we need in the gripper frame
    # a rotation matrix's inverse is it's transpose
    # think about the grippers position with respect to the position of the scoop

  # we get the bowl position from the bowl callback
  # bowl_detection.py node is not auto-run
  # make sure to launch manually
  # bowl position subscriber should be created already

  # for right now, just try identity quaternion as the target pose for scoop
  # could need to be tweaked later
  # need to find gripper's position from scoop as described above
  # a rotation about the scoop's x_axis is equivalent to rotation around the gripper's y_axis

  # once scoop over bowl, scoop needs to be rotated around gripper z_axis by -pi/2 rad
  # then we return gripper to drop position as saved earlier

  # this is a high level outline
  # there is a lot to change
  # this should also help us communicate where we're running into issues
  # text if you run into something but don't expect an immediate reply
  # we can work through more stuff together when i start working in the evening
  

  # #M = np.array([-z_axis, x_axis, -y_axis]).reshape((3,3))
  # M = np.array([x_axis, z_axis, -y_axis]).reshape((3,3))
  # q = quaternion.from_rotation_matrix(M)

  # goal_pose = geometry_msgs.msg.Pose()
  # goal_pose.position.x = mod_grab_pos[0]
  # goal_pose.position.y = mod_grab_pos[1]
  # goal_pose.position.z = mod_grab_pos[2]

  # goal_pose.orientation.w = q.w
  # goal_pose.orientation.x = q.x
  # goal_pose.orientation.y = q.y
  # #goal_pose.orientation.z = (q.z * 3 * math.pi/2)
  # goal_pose.orientation.z = q.z



  # T1 = np.array([0, 0, 0.1]) # Translation from AruCo to grab
  # R1 = t.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3] # Rotation from AruCo to grab
  # # aruco_to_grab = t.concatenate_matrices(T, R)
  # # grab_to_aruco = t.inverse_matrix(transform)
  # # print(inv_tf)

  # T2 = np.array([0, 0, 0.2]) # Translation from AruCo to spoon-end
  # R2 = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) # Rotation from AruCo to spoon-end
  # # aruco_to_scoop = t.concatenate_matrices(T, R)
  # # scoop_to_aruco = t.inverse_matrix(transform)
  # # print(inv_tf)

  # R12 = R2@R1.T
  # t12 = R1@np.array([T2 - T1]).T

  # temp = np.eye(4)
  # temp[:3,:3] = R12
  # R12 = temp
  # del temp
  # T12 = t.translation_matrix((t12.T)[0])
  # grab_to_end = t.concatenate_matrices(R12, T12)
  # end_to_grab = t.inverse_matrix(grab_to_end)
  # T = t.translation_from_matrix(end_to_grab)
  # R = t.quaternion_from_matrix(end_to_grab)

  # end_to_grab_tf = geometry_msgs.msg.TransformStamped()
  # end_to_grab_tf.transform.translation.x = T[0]
  # end_to_grab_tf.transform.translation.y = T[1]
  # end_to_grab_tf.transform.translation.z = T[2]
  # end_to_grab_tf.transform.rotation.x = R[0]
  # end_to_grab_tf.transform.rotation.y = R[1]
  # end_to_grab_tf.transform.rotation.z = R[2]
  # end_to_grab_tf.transform.rotation.w = R[3]


  
  # arm_commander.set_pose_target(goal_pose)
  # print("96 moving to spoon")
  # arm_commander.go(wait=True)
  # arm_commander.clear_pose_targets()
  # # rospy.logwarn('finished moving, going to ready position after 2 sec')
  # rospy.sleep(1) 


  # # print('99 closing gripper')
  # # grab_commander.set_named_target('close')
  # # grab_commander.go(wait=True)
  # # grab_commander.clear_pose_targets()
  # # print(arm_commander.get_current_pose())
  # # # rospy.sleep(1)

  # # # over_bowl_spoon_pose = geometry_msgs.msg.PoseStamped()
  # # # over_bowl_spoon_pose.pose.position.x = bowl_pos[0]
  # # # over_bowl_spoon_pose.pose.position.y = bowl_pos[1]
  # # # over_bowl_spoon_pose.pose.position.z = bowl_pos[2]
  # # # over_bowl_spoon_pose.pose.orientation.w = np.cos(3*np.pi/4)
  # # # over_bowl_spoon_pose.pose.orientation.x = 0
  # # # over_bowl_spoon_pose.pose.orientation.y = 0
  # # # over_bowl_spoon_pose.pose.orientation.z = np.sin(3*np.pi/4)

  # # # over_bowl_hand_pose = tf_gm.do_transform_pose(over_bowl_spoon_pose, end_to_grab_tf)

  # # # # traj_constraints = moveit_msgs.msg.Constraints()
  # # # # ocm = moveit_msgs.msg.OrientationConstraint()
  # # # # ocm.link_name = "xarm_gripper"
  # # # # ocm.header.frame_id = "xarm_gripper"
  # # # # ocm.orientation.x = 0
  # # # # ocm.orientation.y = 0
  # # # # traj_constraints.orientation_constraints.append(ocm)
  # # # print('124 moving spoon over bowl')
  # # # arm_commander.set_pose_target(over_bowl_hand_pose)
  # # # # arm_commander.set_path_constraints(traj_constraints)
  # # # arm_commander.go(wait=True)
  # # # arm_commander.clear_pose_targets()

  # # # grab_commander.set_named_target('open')
  # # # grab_commander.go(wait=True)
  # # # grab_commander.clear_pose_targets()

  # # # arm_commander.set_named_target('hold-up')
  # # # arm_commander.go(wait=True)
  # # # arm_commander.clear_pose_targets()

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

