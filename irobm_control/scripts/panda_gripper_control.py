#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from franka_gripper.msg import GraspAction, GraspGoal
from irobm_control.srv import CloseGripper, CloseGripperResponse, OpenGripper, OpenGripperResponse
from irobm_control.srv import SetGripperWidth, SetGripperWidthResponse, Grasp, GraspResponse

class PandaGripperNode:
    def __init__(self):

        # Determine if the environment is Gazebo or real robot
        self.is_gazebo = rospy.get_param('/use_sim_time', False)

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

        self.open_gripper_service = rospy.Service('/irobm_control/open_gripper', OpenGripper, self.open_gripper_handler)
        self.close_gripper_service = rospy.Service('/irobm_control/close_gripper', CloseGripper, self.close_gripper_handler)
        self.set_gripper_width = rospy.Service('/irobm_control/set_gripper_width', SetGripperWidth, self.set_gripper_width_handler)
        self.grasp_handler = rospy.Service('irobm_control/grasp_obj', Grasp, self.grasp_handler)


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