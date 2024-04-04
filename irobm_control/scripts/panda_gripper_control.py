#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from franka_gripper.msg import GraspAction, GraspGoal
from sensor_msgs.msg import JointState
from irobm_control.srv import CloseGripper, CloseGripperResponse, OpenGripper, OpenGripperResponse
from irobm_control.srv import SetGripperWidth, SetGripperWidthResponse, Grasp, GraspResponse
from irobm_control.srv import GripperWidth, GripperWidthResponse

class PandaGripperNode:
    def __init__(self):

        # Determine if the environment is Gazebo or real robot
        self.is_gazebo = rospy.get_param('/use_sim_time', False)

        self.curr_pos = 0.0
        self.curr_velocity = 0.0

        # Connect to the gripper action server
        if self.is_gazebo:
            # Gazebo gripper action server
            print("Gazebo environment")
            gripper_action_server = '/franka_gripper/gripper_action'
            grasp_action_server = '/franka_gripper/grasp'
        else:
            # Real robot gripper action server
            print("Real environment")
            gripper_action_server = '/franka_gripper/gripper_action'
            grasp_action_server = '/franka_gripper/grasp'

        self.gripper_client = actionlib.SimpleActionClient(gripper_action_server, GripperCommandAction)
        self.grasp_client = actionlib.SimpleActionClient(grasp_action_server, GraspAction)
        t_out = self.gripper_client.wait_for_server()
        g_out = self.grasp_client.wait_for_server()

        rospy.Subscriber('/franka_gripper/joint_states', JointState, self.curr_pos_callback)

        self.open_gripper_service = rospy.Service('/irobm_control/open_gripper', OpenGripper, self.open_gripper_handler)
        self.close_gripper_service = rospy.Service('/irobm_control/close_gripper', CloseGripper, self.close_gripper_handler)
        self.set_gripper_width = rospy.Service('/irobm_control/set_gripper_width', SetGripperWidth, self.set_gripper_width_handler)
        self.grasp_handler_act = rospy.Service('irobm_control/grasp_obj', Grasp, self.grasp_handler)
        self.gripper_state_service = rospy.Service('/irobm_control/gripper_state', GripperWidth, self.gripper_state_handler)


    def curr_pos_callback(self, data):
        self.curr_pos = data.position[0] + data.position[1]
        self.curr_velocity = data.velocity[0] + data.velocity[1]
        # print(f'Positions width is {data.position}')

    # returns the current state of the gripper
    def gripper_state_handler(self, req):
        res = GripperWidthResponse()
        res.width = self.curr_pos
        res.velocity = self.curr_velocity
        return res
    
    def open_gripper_handler(self, req):
        # Position for the fully opened gripper is 0.04(0.04 in both directions, so fully open it is 0.08)
        self.set_gripper_position(0.04)

        response = OpenGripperResponse()
        response.success = True
        return response

    def close_gripper_handler(self, req):
        effort = req.effort
        if effort is None:
            self.set_gripper_position(0.0)
        else:
            self.set_gripper_position(0.0, effort=effort)

        response = CloseGripperResponse()
        response.success = True
        return response
    
    def grasp_handler(self, req):
        width = req.width
        force = req.force

        if force is None:
            self.grasp(width)
        else:
            self.grasp(width, force=force)

        response = GraspResponse()
        response.success = True
        return response
        
    # controls the gripper width. The gripper width can be assigned freely
    def set_gripper_width_handler(self, req):
        width = req.width
        effort = req.effort

        if effort is None:
            self.set_gripper_position(width)
        else:
            self.set_gripper_position(width, effort=effort)

        response = SetGripperWidthResponse()
        response.success = True
        return response

    # used as a general function to control the gripper(opening, closing, controlling width)
    def set_gripper_position(self, position, effort=10):
        # Use MoveIt! action to set gripper position
        print("Set Position")
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort  # Adjust the max effort as needed

        self.gripper_client.send_goal(goal)
        # send the goal
        self.gripper_client.wait_for_result()
        # result returned

    # grasps with a certain force, used to pick up objects
    def grasp(self, width, force, epsilon=0.05, speed=0.05):
        print(f'Width:{width}, Speed:{speed}, Force:{force}')
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon
        goal.epsilon.outer = epsilon
        goal.speed = speed
        goal.force = force

        self.grasp_client.send_goal(goal)

        self.grasp_client.wait_for_result()

        result = self.grasp_client.get_result()
        print(f'Results: {result}')



    def cleanup(self):
        # Shutdown the ROS node
        rospy.signal_shutdown('Shutdown requested')


if __name__ == '__main__':
    rospy.init_node('panda_gripper_controller_node', anonymous=True, log_level=rospy.DEBUG)
    panda_gripper_controller = PandaGripperNode()
    rospy.spin()

    #Example Gripper positions:
    # 
    # 