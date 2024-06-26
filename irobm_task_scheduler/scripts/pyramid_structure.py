#!/usr/bin/env python3
import rospy
import sys
import tf
import tf.transformations as tft
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
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

class PyramidNode:
    def __init__(self):
        rospy.init_node('cube_tower_node', log_level=rospy.DEBUG)

        # important parametersto control the scene and robot
        self.is_simulation = True

        self.pyramid_center_pos = np.array([0.45, -0.45, 0.0])
        self.cube_dim = 0.045

        self.pick_zone = [[0.22, 0.69], [-0.3, 0.3]]

        self.desk_h = 0.787 if self.is_simulation else 0.

        self.default_orient = [math.pi, 0.0, -math.pi / 4]

        self.origin_joint_pose = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]

        if self.is_simulation:
            print('Cube Pyramid is in sim')
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

    # extracts the position and orientation of a given model from the simulation environment
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
                orient_quat = [orientation.z, orientation.y, orientation.x, orientation.w]
                orient_rad = self.quaternion_to_radians(orient_quat)
                rospy.loginfo(f"Model '{model_name}' - Position: {pos}, Orientation: {orient_rad}")
                orient_deg = [math.degrees(angle) for angle in orient_rad]
                print(f'Orientation Deg: {orient_deg}')
                return pos, orient_rad, True
            else:
                rospy.logwarn(f"Failed to get state for model '{model_name}'")
                return None, None, False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
            return None, None, False
        
    # Takes the given model name(substring) and iterates over all names with the name inside + an iteration number
    # Returns two lists of lists, for orientations and positions
    def model_name_finder(self, model_name):
        model_pos_l = []
        model_orient_l = []
        i = 0

        while True:
            model = 'cube' + '_' + str(i)
            pos, orient, found = self.extract_model_state(model)
            if not found: 
                break
            model_pos_l.append(pos)
            model_orient_l.append(orient)
            i = i + 1

        return model_pos_l, model_orient_l
     
        
    def quaternion_to_radians(self, quaternion):
        # Convert quaternion to rotation matrix
        rotation_matrix = tft.quaternion_matrix(quaternion)
           
        # Extract roll, pitch, and yaw angles from rotation matrix
        roll, pitch, yaw = tft.euler_from_matrix(rotation_matrix, 'rzyx')
            
        # Convert angles from radians to degrees and return as a list
        return [roll, pitch, yaw]


    # creates a list of goal centroids for the pyramid with the right placing order
    def pyramid_goal_centroids(self, center_pos, height):
        padding = 0.008 # space between cubes
        
        dist_between_cubes = padding + self.cube_dim
        x_offset_odd = math.cos(math.pi/4) * dist_between_cubes
        y_offset_odd = math.sin(math.pi/4) * dist_between_cubes
        x_offset_even = math.cos(math.pi/4) * (dist_between_cubes / 2)
        y_offset_even = math.sin(math.pi/4) * (dist_between_cubes / 2)
        z_coord = 0.0225
        centroid_list = []

        # iterates from down, but builds the layers from the bottom up
        for i in range(height, 0, -1):
            # half the cubes of the current layer
            half_l_cubes = math.floor(i / 2)
            for j in range(i):
                if i % 2 == 1:
                    centroid = [center_pos[0] + (-half_l_cubes + j) * x_offset_odd,
                                center_pos[1] + (-half_l_cubes + j) * y_offset_odd,
                                z_coord]
                    centroid_list.append(centroid)
                else:
                    if j == 0:
                        edge_centroid = [center_pos[0] + x_offset_even * -1 + x_offset_odd * -(half_l_cubes - 1),
                                         center_pos[1] + y_offset_even * -1 + y_offset_odd * -(half_l_cubes - 1)]
                    centroid = [edge_centroid[0] + x_offset_odd * j, edge_centroid[1] + y_offset_odd * j, z_coord]
                    centroid_list.append(centroid)
            z_coord = z_coord + self.cube_dim

        return centroid_list
    
    # Builds a tower a the given center position with the given number of cubes:
    # center_pos: The position of the first cube for the tower
    # num_cubes: The number of cubes that shall be in the tower
    def tower_goal_centroids(self, center_pos, num_cubes):
        centroid_list = []
        for i in range(num_cubes):
            centroid = [center_pos[0], center_pos[1], center_pos[2] + self.cube_dim * i]
            centroid_list.append(centroid)

        return centroid_list
    
    # chooses the best order for the cube to be picked in while also getting the correct orientation to pick it,
    # even when another cube is touching it

    def choose_best_pick(self, ref_base_l, pick_pos_l):
        best_pick_order_l = []
        smallest_dist_l = []
        dist_too_low_l = []
        dist_to_cube = 0.0
        smallest_dist = math.inf

        # is there enough distance to the base structure for picking
        enough_dist_base = True

        #the list of the structure base, but with numpy arrays as elements
        ref_base_l_np = []
        for i in range(len(ref_base_l)):
            ref_base_l_np.append(np.array(ref_base_l[i]))

        # evaluates each cube if it is the best possible pick by distances
        for i in range(len(pick_pos_l)):
            dist_to_cube = 0.0
            pick_pos_np = np.array((pick_pos_l[i])[0])
            in_zone = self.in_pick_zone((pick_pos_l[i])[0])
            if not in_zone:
                continue

            # check if there is enough space to the structure
            for j in range(len(ref_base_l)):
                dist = np.linalg.norm(ref_base_l[j] - pick_pos_np)
                if dist < 0.095:
                    enough_dist_base = False
                    break
            
            if not enough_dist_base:
                dist_too_low_l.append(pick_pos_np)
                continue

            smallest_dist = math.inf
            con_vec = None
            closest_cube = None
            # check for smallest distance to all other remaining cubes
            for k in range(len(pick_pos_l)):
                if k == i:
                    continue
                comp_cube = pick_pos_l[k]
                comp_pos_np = np.array(comp_cube[0])
                dist_to_cube = np.linalg.norm(comp_pos_np - pick_pos_np)

                if smallest_dist > dist_to_cube:
                    smallest_dist = dist_to_cube
                    con_vec = comp_pos_np - pick_pos_np
                    closest_cube = comp_cube
            # sorts the cubes, so the cubes with the biggest distance   s to its closest neighbour gets picked first
            if smallest_dist >= 0.1:
                # adds the best gripper orientation
                orient = list((pick_pos_l[i])[1])
                gripper_orient = self.best_orientation(orient)
                pick = pick_pos_l[i] + (gripper_orient,)

                if len(smallest_dist_l) == 0:
                    smallest_dist_l.append(smallest_dist)
                    best_pick_order_l.append(pick)
                else:
                    for l in range(len(smallest_dist_l)):
                        if smallest_dist >= smallest_dist_l[l]:
                            smallest_dist_l.insert(l, smallest_dist)
                            best_pick_order_l.insert(l, pick)
                            break
                        elif smallest_dist < smallest_dist_l[l] and not l == (len(smallest_dist_l) - 1):
                            continue
                        elif l == (len(smallest_dist_l) - 1):
                            smallest_dist_l.append(smallest_dist)
                            best_pick_order_l.append(pick)
                            break
                                
            else:
                # adds the best gripper orientation, only important if another cube is too close. Else default choosing with smallest angle
                orient = list((pick_pos_l[i])[1])
                gripper_orient = self.best_orientation(orient, connection_vec=con_vec)

                pick = pick_pos_l[i] + (gripper_orient,)

                if len(smallest_dist_l) == 0:
                    smallest_dist_l.append(smallest_dist)
                    best_pick_order_l.append(pick)
                else:
                    for l in range(len(smallest_dist_l)):
                        if smallest_dist >= smallest_dist_l[l]:
                            print(f'I got inserted at {l}')
                            smallest_dist_l.insert(l, smallest_dist)
                            best_pick_order_l.insert(l, pick)
                            break
                        elif smallest_dist < smallest_dist_l[l] and not l == (len(smallest_dist_l) - 1):
                            continue
                        elif l == (len(smallest_dist_l) - 1):
                            smallest_dist_l.append(smallest_dist)
                            best_pick_order_l.append(pick)
                            break
        return best_pick_order_l
    

    #finds the best orientation for the gripper which won't collide with a cube that is close
    def best_orientation(self, orient1, connection_vec=None):
        cube_yaw = (orient1[2] + 2*math.pi) % (math.pi/2)
        print(f'Yaw: {(cube_yaw * 180)/ math.pi}')
        if connection_vec is None:
            if cube_yaw >= math.pi / 4:
                opposit_rot = math.pi/2 - cube_yaw
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, -opposit_rot])).tolist()
            else:
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, cube_yaw])).tolist()
        else:
            # calculates the angle between the face and the connection vector
            # this vector would face along the y-axis in default orientation
            face_vec = np.array([math.sin(cube_yaw), -math.cos(cube_yaw), 0.0])
            cross_prod = np.cross(connection_vec, face_vec)
            magn_prod = np.linalg.norm(connection_vec) * np.linalg.norm(face_vec)
            face_con_angle = math.asin(np.linalg.norm(cross_prod) / magn_prod)

            # calculate the angle between the x-axis and the connection vector
            y_axis = np.array([0.0, 1.0, 0.0])
            cross_prod = np.cross(connection_vec, y_axis)
            magn_prod = np.linalg.norm(connection_vec) * np.linalg.norm(y_axis)
            y_con_angle = math.asin(np.linalg.norm(cross_prod) / magn_prod)

            # choose how the gripper should rotate
            if (face_con_angle <= (math.pi / 4) and cube_yaw <= (math.pi / 4)) or \
                (face_con_angle <= (math.pi / 4) and cube_yaw > (math.pi / 4)):
                rot = math.pi/2 - cube_yaw
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, -rot])).tolist()
            elif (face_con_angle > (math.pi / 4) and cube_yaw <= (math.pi / 4)) or \
                (face_con_angle > (math.pi / 4) and cube_yaw > (math.pi / 4)):
                rot = cube_yaw
                gripper_orient = (np.array(self.default_orient) + np.array([0.0, 0.0, rot])).tolist()
        return gripper_orient
            
    # checks if a given centroid is in a predefined pick zone. Only in this zone, cubes can be picked
    def in_pick_zone(self, centroid):
        if centroid[0] > (self.pick_zone[0])[0] and centroid[0] < (self.pick_zone[0])[1] \
            and centroid[1] > (self.pick_zone[1])[0] and centroid[1] < (self.pick_zone[1])[1]:
            return True
        return False              
    
    def build_pyramid(self):
        print('Start the build')

        req = HomingRequest()
        response = self.homing_client(req)
        
        # perpendicular orientation towards the tower base
        perp_orient = [self.default_orient[0], self.default_orient[1],
                       self.default_orient[2] - math.pi/2 + math.atan2(self.pyramid_center_pos[1], self.pyramid_center_pos[0])]

        # creating the desired structure with the given functions
        pyramid_height = 6
        goal_cube_centroids = self.pyramid_goal_centroids(self.pyramid_center_pos, pyramid_height)
        tower_pos = [self.pyramid_center_pos[0], self.pyramid_center_pos[1], 0.0225 + self.cube_dim * pyramid_height]
        goal_cube_centroids.extend(self.tower_goal_centroids(tower_pos, 2))
        print(f'The center goals are: {goal_cube_centroids}')
        # A copy to iterate over and remove placed cubs
        remaining_cubes = goal_cube_centroids.copy()

        # Information about the structure size
        structure_size = len(goal_cube_centroids)
        print(f'The structure needs {structure_size} cubes')

        i = 0
        while len(remaining_cubes) > 0:
            # scan environment
            
            # if in the real world, the service for the perception is called
            # ensures, that the simulation and real environemt work with the same data structure of detected cubes
            if not self.is_simulation:
                model_pos_l = []
                model_orient_l = []
                req = CubeCentroidsRequest()
                responsePC = self.pc_handler(req)
                rospy.sleep(1)
                cube_counter = len(responsePC.position)
                for i in range(len(responsePC.position)):
                    model_pos_l.append([responsePC.position[i].x, responsePC.position[i].y, responsePC.position[i].z])
                    model_orient_l.append([responsePC.orientation[i].x, responsePC.orientation[i].y, responsePC.orientation[i].z])
                model_SE_l = list(zip(model_pos_l, model_orient_l))
                print('Received centroids from perception')
            else:
                model_pos_l, model_orient_l = self.model_name_finder('cube')
                model_SE_l = list(zip(model_pos_l, model_orient_l))
                print('Reiceived centroids from gazebo models')
            

            print('The ordered cubes start')
            ordered_cubes = self.choose_best_pick(goal_cube_centroids[0:2], model_SE_l)
            print(f'Ordered List Len: {len(ordered_cubes)}')
            

            # Check if there are pickable cubes
            if len(ordered_cubes) == 0:
                print('No more pickable cubes around')
                print('Please place new cubes and press Enter afterwards.')
                key_press = input()
                continue
            
            # sets gripper orientation and cube positions to reduce errors in picking
            cube_pos = (ordered_cubes[0])[0]
            cube_pos[2] = 0.0225
            cube_orient = (ordered_cubes[0])[1]
            gripper_orient = list((ordered_cubes[0])[2])

            # cube_yaw = (cube_z_orient + 2*math.pi) % (math.pi/2)
            print(f"Current goal position: {remaining_cubes[0]}")
            print(f'Gripper Orient: {gripper_orient}')
            
            print(f'Cube Pos: {cube_pos}')

            # Executes the picking and placing to fill a goal position
            req = PickNPlaceRequest()
            req.cube_centroid = Point(*cube_pos)
            req.orientation = Point(*gripper_orient)
            req.task = 'pick'
            response = self.pick_n_place_client(req)
            if not response:
                continue
            
            req = PickNPlaceRequest()
            req.cube_centroid = Point(*remaining_cubes[0])
            req.orientation = Point(*perp_orient)
            req.task = 'place'
            response = self.pick_n_place_client(req)
            if not response:
                continue

            req = HomingRequest()
            response = self.homing_client(req)

            print("Placed Cube correctly")
            # if cube is placed correctly, it gets removed from the goal list
            del remaining_cubes[0]
            i = i+1
        
        print('The structure has been fully build')



if __name__ == '__main__':
    pyramid_controller = PyramidNode()
    print("Initialized")
    pyramid_controller.build_pyramid()