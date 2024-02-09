#!/usr/bin/env python3

import rospy
import sys
import tf
import moveit_msgs.msg
import geometry_msgs.msg
import math
import numpy as np
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point

from irobm_control.srv import MoveTo, MoveToRequest, BasicTraj, BasicTrajRequest
from irobm_control.srv import OpenGripper, OpenGripperRequest, CloseGripper, CloseGripperRequest, Grasp, GraspRequest

class ExampleControllerNode:
    def __init__(self):
        rospy.init_node('example_controller_node', log_level=rospy.DEBUG)

        self.is_simulation = True

        # height of the desk in the simulation

        if self.is_simulation:
            # Initialize Gazebo service and relevant parameters
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.desk_h = np.array([0.0, 0.0, 0.787])
        else:
            self.desk_h = np.array([0.0, 0.0, 0.0])

        self.move_to_client = rospy.ServiceProxy('/irobm_control/move_to', MoveTo)
        self.basic_traj_client = rospy.ServiceProxy('/irobm_control/basic_traj', BasicTraj)

        self.open_gripper_client = rospy.ServiceProxy('/irobm_control/open_gripper', OpenGripper)
        self.close_gripper_client = rospy.ServiceProxy('/irobm_control/close_gripper', CloseGripper)
        self.grasp_client = rospy.ServiceProxy('/irobm_control/grasp_obj', Grasp)

    def extract_model_state(self, model_name):
        # Create a request object
        request = GetModelStateRequest()
        request.model_name = model_name

        try:
            # Call the GetModelState service
            response = self.get_model_state_service(request)

            # Extract and print the position and orientation
            if response.success:
                position = response.pose.position
                orientation = response.pose.orientation
                rospy.loginfo(f"Model '{model_name}' - Position: {position}, Orientation: {orientation}")
                return position, orientation
            else:
                rospy.logwarn(f"Failed to get state for model '{model_name}'")
                return None, None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
            return None, None

    def run(self):
        pos0 = [0.5, 0.0, 0.5]
        pos1 = [0.5, 0.0, 0.13]
        pos2 = [0.5, 0.0, 0.33]
        pos3 = [0.3, 0.3, 0.1315]
        posA = [0.3, 0.3, 0.32]
        pos4 = [0.5, -0.1, 0.13]
        pos5 = [0.3, 0.3, 0.185]
        posB = [0.3, 0.3, 0.374]
        posC = [0.5, -0.1, 0.33]
        
        req = MoveToRequest()
        req.position = Point(*pos0)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {pos0}')
        
        req = OpenGripperRequest()
        response = self.open_gripper_client(req)
        print('Executed open gripper')

        req = MoveToRequest()
        req.position = Point(*pos1)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {pos1}')

        req = GraspRequest()
        # req.width = 0.01125 # in sim
        req.width = 0.01075 #0.01075 since real cube is 0.43
        req.force = 20.0
        response = self.grasp_client(req)
        print('Executed Gripper width')

        req = MoveToRequest()
        req.position = Point(*pos2)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos2: {pos2}')

        req = MoveToRequest()
        req.position = Point(*posA)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to posA: {posA}')

        req = MoveToRequest()
        req.position = Point(*pos3)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos2: {pos3}')

        req = OpenGripperRequest()
        response = self.open_gripper_client(req)
        print('Executed open gripper')

        req = MoveToRequest()
        req.position = Point(*posA)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {posA}')

        req = MoveToRequest()
        req.position = Point(*posC)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {posC}')

        req = MoveToRequest()
        req.position = Point(*pos4)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {pos4}')

        req = GraspRequest()
        # req.width = 0.01125 # in sim
        req.width = 0.01075 #0.01075 since real cube is 0.43
        req.force = 20.0
        response = self.grasp_client(req)
        print('Executed Gripper width')

        req = MoveToRequest()
        req.position = Point(*posB)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {posB}')

        req = MoveToRequest()
        req.position = Point(*pos5)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {pos5}')

        req = OpenGripperRequest()
        response = self.open_gripper_client(req)
        print('Executed open gripper')

        req = MoveToRequest()
        req.position = Point(*posB)
        req.orientation = [math.pi, 0.0, -math.pi / 4]
        req.w_orient = True
        response = self.move_to_client(req)
        print(f'Executed move to pos1: {posB}')

    def run1(self):
        x = self.extract_model_state('cube_0')
        print(f'This is the position of x: {x}')





if __name__ == '__main__':
    ex_controller = ExampleControllerNode()
    print("Initialized")
    ex_controller.run()
    # pos, orient = ex_controller.extract_model_state('cube_0')
    # print(pos, orient)

    
