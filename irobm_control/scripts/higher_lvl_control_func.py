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
from irobm_control.srv import PickNPlace, PickNPlaceResponse

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
        else:
            print('Higher lvl control is in real')
            self.desk_h = np.array([0.0, 0.0, 0.0])

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

    def pick_n_place_handler(self, req):
        pos = [req.cube_centroid.x, req.cube_centroid.y, req.cube_centroid.z]
        orient = [req.orientation.x, req.orientation.y, req.orientation.z]
        task = req.task

        response = PickNPlaceResponse()
        response.success = self.pick_n_place(pos, orient, task)

        return response



    def pick_n_place(self, pos, orient, task):
        init_dist = np.array([0.0, 0.0, 0.15])

        pos_np = np.array(pos)
        orient_np = np.array(orient)

        init_pos= (pos_np + self.gripper_offset + init_dist).tolist()
        print(f'Init Pose: {init_pos}')

        pick_pos_np = pos_np + self.gripper_offset
        print(f'Pick Pose: {pick_pos_np}')
        place_pos_np = pick_pos_np + np.array([0.0, 0.0, 0.01])
        print(f'Place Pose: {place_pos_np}')
        print(f'Orient: {orient}')

        pick_pos = pick_pos_np.tolist()
        place_pos = place_pos_np.tolist()

        req = MoveToRequest()
        req.position = Point(*init_pos)
        req.orientation = orient
        req.w_orient = True
        response = self.move_to_client(req)

        if task == 'pick':
            req = OpenGripperRequest()
            response = self.open_gripper_client(req)

            req = MoveToRequest()
            req.position = Point(*pick_pos)
            req.orientation = orient
            req.w_orient = True
            response = self.move_to_client(req)

            req = GraspRequest()
            req.width = self.cube_dim / 4
            req.force = 20.0
            response = self.grasp_client(req)
        elif task == 'place':
            req = MoveToRequest()
            req.position = Point(*place_pos)
            req.orientation = orient
            req.w_orient = True
            response = self.move_to_client(req)

            req = OpenGripperRequest()
            response = self.open_gripper_client(req)

        req = MoveToRequest()
        req.position = Point(*init_pos)
        req.orientation = orient
        req.w_orient = True
        response = self.move_to_client(req)

        return True


if __name__ == '__main__':
    # rospy.init_node('panda_move_node', log_level=rospy.DEBUG) #node with log set to debug
    rospy.init_node('higher_lvl_control_node')
    position_class = HigherLvLControlNode()
    rospy.spin()

    
