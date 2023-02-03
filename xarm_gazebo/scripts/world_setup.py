from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
import rospy

if __name__ == '__main__':
    set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    pause_physics()

    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_values = [-0.12628707305706666, -0.05495823463710536, -0.08447349672759508, -0.15534568276844762, -1.4345624350176536, 0.03765355624090638]
    arm_joint_resp = set_model_config('xarm6', 'robot_description', joint_names, joint_values)

    if not arm_joint_resp.success:
        print('setting arm joint angles failed')
        print(arm_joint_resp.state_message)
        exit(-1)

    model_state_resp = get_model_state('cafe_table', "world")

    if not model_state_resp.success:
        print('Could not get cafe_table state')
        print(model_state_resp.state_message)

    desired_state = ModelState()
    desired_state.model_name = "cafe_table"
    desired_state.pose = model_state_resp.pose
    desired_state.twist = model_state_resp.twist
    desired_state.pose.position.x += 0.05

    table_state_set = set_model_state(desired_state)
    if not table_state_set.success:
        print('Could not set cafe_table state')
        print(table_state_set.state_message)

    play_physics = unpause_physics()