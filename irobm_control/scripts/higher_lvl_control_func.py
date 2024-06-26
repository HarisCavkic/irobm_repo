#!/usr/bin/env python3

import rospy
import numpy as np
import math
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point

from irobm_control.srv import MoveTo, MoveToRequest, BasicTraj
from irobm_control.srv import OpenGripper, OpenGripperRequest, Grasp, GraspRequest
from irobm_control.srv import PickNPlace, PickNPlaceResponse
from irobm_control.srv import GripperWidth, GripperWidthRequest

class HigherLvLControlNode:
    def __init__(self):
        self.is_simulation = rospy.get_param('is_sim', True)

        # height of the desk in the simulation

        if self.is_simulation:
            # Initialize Gazebo service and relevant parameters
            print('Higher lvl control is in sim')
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.desk_h = np.array([0.0, 0.0, 0.787])
            self.gripper_offset = np.array([0.0, 0.0, 0.11])
        else:
            print('Higher lvl control is in real')
            self.desk_h = np.array([0.0, 0.0, 0.0])
            self.gripper_offset = np.array([0.0, 0.0, 0.12])

        self.cube_dim = 0.045
        self.default_force = 20.0
        # maybe tweak offset a bit more
        self.gripper_offset = np.array([0.0, 0.0, 0.11])

        # Service Servers
        self.pick_n_place_srv = rospy.Service('/irobm_control/pick_n_place', PickNPlace, self.pick_n_place_handler)

        # Service Clients
        self.move_to_client = rospy.ServiceProxy('/irobm_control/move_to', MoveTo)
        self.basic_traj_client = rospy.ServiceProxy('/irobm_control/basic_traj', BasicTraj)

        self.open_gripper_client = rospy.ServiceProxy('/irobm_control/open_gripper', OpenGripper)
        self.grasp_client = rospy.ServiceProxy('/irobm_control/grasp_obj', Grasp)
        self.gripper_status_client = rospy.ServiceProxy('/irobm_control/gripper_state', GripperWidth)

    def pick_n_place_handler(self, req):
        pos = [req.cube_centroid.x, req.cube_centroid.y, req.cube_centroid.z]
        orient = [req.orientation.x, req.orientation.y, req.orientation.z]
        task = req.task

        response = PickNPlaceResponse()
        response.success = self.pick_n_place(pos, orient, task)

        return response


    # this is a utility function to handle picking and placing cubes with a given position and orientation
    # task: decides if it is about picking or placing
    def pick_n_place(self, pos, orient, task):
        init_dist = np.array([0.0, 0.0, 0.15])

        pos_np = np.array(pos)
        orient_np = np.array(orient)

        # define positions to get a clean pick or place done with an initial position above the actual pick or place position
        init_pos= (pos_np + self.gripper_offset + init_dist).tolist()
        print(f'Init Pose: {init_pos}')

        pick_pos_np = pos_np + self.gripper_offset
        print(f'Pick Pose: {pick_pos_np}')
        place_pos_np = pick_pos_np + np.array([0.0, 0.0, 0.005])
        print(f'Place Pose: {place_pos_np}')
        print(f'Orient: {orient}')

        pick_pos = pick_pos_np.tolist()
        place_pos = place_pos_np.tolist()

        # Execute the picking or placing
        req = MoveToRequest()
        req.position = Point(*init_pos)
        req.orientation = orient
        req.w_orient = True
        req.z_constraint = 0.2
        response = self.move_to_client(req)

        if task == 'pick':
            req = OpenGripperRequest()
            response = self.open_gripper_client(req)

            req = MoveToRequest()
            req.position = Point(*pick_pos)
            req.orientation = orient
            req.w_orient = True
            req.z_constraint = 0.0
            response = self.move_to_client(req)

            req = GraspRequest()
            req.width = self.cube_dim / 4
            req.force = 20.0
            response = self.grasp_client(req)

            req = GripperWidthRequest()
            response = self.gripper_status_client(req)
            # check if the cube was missed
            if response.width <= 0.025:
                return False

        elif task == 'place':
            req = GripperWidthRequest()
            response = self.gripper_status_client(req)
            if response.width <= 0.025:
                return False

            req = MoveToRequest()
            req.position = Point(*place_pos)
            req.orientation = orient
            req.w_orient = True
            req.z_constraint = 0.0
            response = self.move_to_client(req)

            req = GripperWidthRequest()
            response = self.gripper_status_client(req)
            # check if the cube got dropped inbetween
            if response.width <= 0.025:
                return False

            req = OpenGripperRequest()
            response = self.open_gripper_client(req)

        req = MoveToRequest()
        req.position = Point(*init_pos)
        req.orientation = orient
        req.w_orient = True
        req.z_constraint = 0.0
        response = self.move_to_client(req)

        return True


if __name__ == '__main__':
    # rospy.init_node('panda_move_node', log_level=rospy.DEBUG) #node with log set to debug
    rospy.init_node('higher_lvl_control_node')
    position_class = HigherLvLControlNode()
    rospy.spin()

    
