#!/usr/bin/env python3

import rospy
import sys
import tf
import tf.transformations as tft
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
from irobm_control.srv import PickNPlace, PickNPlaceRequest, ArcPath, ArcPathRequest, Homing, HomingRequest
from irobm_perception.srv import CubeCentroids, CubeCentroidsRequest

class CubeTowerNode:
    def __init__(self):
        rospy.init_node('cube_tower_node', log_level=rospy.DEBUG)

        self.is_simulation = True

        self.num_of_cubes = 7
        self.tower_pos = np.array([0.45, -0.45, 0.0])
        self.cube_dim = 0.045

        self.desk_h = 0.787 if self.is_simulation else 0.

        self.default_orient = [math.pi, 0.0, -math.pi / 4]

        self.origin_joint_pose = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]

        if self.is_simulation:
            print('Cube Tower is in sim')
            # Initialize Gazebo service and relevant parameters
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Service Clients
        self.move_to_client = rospy.ServiceProxy('/irobm_control/move_to', MoveTo)
        self.basic_traj_client = rospy.ServiceProxy('/irobm_control/basic_traj', BasicTraj)
        self.arc_path_client = rospy.ServiceProxy('/irobm_control/arc_path', ArcPath)
        self.homing_client = rospy.ServiceProxy('/irobm_control/homing', Homing)

        self.open_gripper_client = rospy.ServiceProxy('/irobm_control/open_gripper', OpenGripper)
        self.grasp_client = rospy.ServiceProxy('/irobm_control/grasp_obj', Grasp)

        self.pick_n_place_client = rospy.ServiceProxy('/irobm_control/pick_n_place', PickNPlace)

        if not self.is_simulation:
            self.pc_handler = rospy.ServiceProxy('/irobm_perception/cube_centroids', CubeCentroids)
            rospy.wait_for_service('/irobm_perception/cube_centroids')

    def extract_model_state(self, model_name):
        # Create a request object
        request = GetModelStateRequest()
        request.model_name = model_name

        try:
            # Call the GetModelState service
            response = self.get_model_state_service(request)

            # Extract and print the position and orientation
            if response.success:
                # position as a geometry_msgs/Point
                position_point = response.pose.position
                pos = [position_point.x, position_point.y, position_point.z]
                # orientation as a geometry_msgs/Quaternion
                orientation = response.pose.orientation
                orient_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
                orient_rad = self.quaternion_to_radians(orient_quat)
                rospy.loginfo(f"Model '{model_name}' - Position: {pos}, Orientation: {orient_rad}")
                orient_deg = [math.degrees(angle) for angle in orient_rad]
                print(f'Orientation Deg: {orient_deg}')
                return pos, orient_rad
            else:
                rospy.logwarn(f"Failed to get state for model '{model_name}'")
                return None, None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
            return None, None
        
    def quaternion_to_radians(self, quaternion):
        # Convert quaternion to rotation matrix
        rotation_matrix = tft.quaternion_matrix(quaternion)
           
        # Extract roll, pitch, and yaw angles from rotation matrix
        roll, pitch, yaw = tft.euler_from_matrix(rotation_matrix, 'rzyx')
            
        # Convert angles from radians to degrees and return as a list
        return [roll, pitch, yaw]


    
    def build_tower(self):
        print('Start the build')
        #Follows an arc path around the centrer
        req = ArcPathRequest()
        req.center_of_circle = [0.5, 0.0, 0.0]
        req.radius = 0.2
        req.times = 8
        req.height = 0.5
        print("Starting the arc movement")
        response = self.arc_path_client(req)
        print("Done with the arc movement")
        cube_counter = self.num_of_cubes

        req = HomingRequest()
        response = self.homing_client(req)

        # only uses the perception service for the camera in the real world
        if not self.is_simulation:
            print("Calling the point cloud handler")
            req = CubeCentroidsRequest()
            responsePC = self.pc_handler(req)
            print("Done with the point cloud handler")
            rospy.sleep(1)
            cube_counter = len(responsePC.position)
            print(f'Found Cubes: {cube_counter}')

        #iterates over the predefined or extracted cubes from the scene
        for i in range(cube_counter):
            if self.is_simulation:
                cube_name = 'cube_' + str(i)
                cube_pos, cube_orient = self.extract_model_state(cube_name)
                cube_z_orient = cube_orient[0]         
            else:
                position_point = responsePC.position[i]
                cube_pos = [position_point.x, position_point.y, position_point.z]
                rotation_point = responsePC.orientation[i]
                cube_orient = [rotation_point.x, rotation_point.y, rotation_point.z]
                cube_z_orient = cube_orient[2]

                
            # calculate the correct gripper orientation to get cubes
            cube_pos[2] = 0.0225
            cube_yaw = (cube_z_orient + 2*math.pi) % (math.pi/2)
            print("HEEEEY here is the apperent position and rotations: ", cube_pos, cube_orient)
            if cube_yaw >= math.pi / 4:
                opposit_rot = math.pi/2 - cube_yaw
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, -opposit_rot])).tolist()
            else:
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, cube_yaw])).tolist()
            print(f'Gripper Orient: {gripper_orient}')
            
            # execute the picking
            print(f'Cube Pos: {cube_pos}')
            req = PickNPlaceRequest()
            req.cube_centroid = Point(*cube_pos)
            req.orientation = Point(*gripper_orient)
            req.task = 'pick'
            response = self.pick_n_place_client(req)
            
            # execute the placing of a cube
            curr_height = i * self.cube_dim + self.cube_dim / 2
            place_pos = (self.tower_pos + np.array([0.0, 0.0, curr_height])).tolist()
            req = PickNPlaceRequest()
            req.cube_centroid = Point(*place_pos)
            req.orientation = Point(*self.default_orient)
            req.task = 'place'
            response = self.pick_n_place_client(req)

            # get to pre defined home position to avoid joint limits
            req = HomingRequest()
            response = self.homing_client(req)


if __name__ == '__main__':
    cube_tower_controller = CubeTowerNode()
    print("Initialized")
    cube_tower_controller.build_tower()