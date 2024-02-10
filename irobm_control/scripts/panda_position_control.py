#!/usr/bin/env python3

import rospy
import sys
import numpy as np
import tf
import tf.transformations as tft
import moveit_commander
import moveit_commander.robot as mr
import moveit_msgs.msg
import geometry_msgs.msg
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point
from irobm_control.srv import MoveTo, MoveToResponse, BasicTraj, BasicTrajResponse, ArcPath, ArcPathResponse
from irobm_control.srv import Homing, HomingResponse
from copy import deepcopy

class PandaMoveNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize MoveIt!
        self.robot = mr.RobotCommander()
        self.group = mr.MoveGroupCommander("panda_arm")


        # Check if running in simulation
        # in the simulation all z-values have to be increased by 0.787 due to the table
        self.is_simulation = rospy.get_param('is_sim', True)

        velocity = rospy.get_param('/robot_description_planning/default_velocity_scaling_factor', True)
        print(f'VElocity: {velocity}')
        rospy.set_param('/robot_description_planning/default_velocity_scaling_factor', 0.3)

        if self.is_simulation:
            # Initialize Gazebo service
            print('Position Control in sim')
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.desk_h = np.array([0.0, 0.0, 0.787])
        else:
            print('Position Control in real')
            self.desk_h = np.array([0.0, 0.0, 0.0])

        self.origin_joint_pose = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
        self.homing_configuration = [-0.003883130299572653, -0.18531659259774813, 0.0034981294616078205, -2.1012125574986595, 0.00035676014991842123, 1.913004710985454, 0.7990935928072485]

        self.move_to = rospy.Service('/irobm_control/move_to', MoveTo, self.move_to_handler)
        self.basic_traj = rospy.Service('/irobm_control/basic_traj', BasicTraj, self.basic_traj_handler)
        self.arc_path_srv = rospy.Service('/irobm_control/arc_path', ArcPath, self.arc_path_handler)
        self.homing = rospy.Service('irobm_control/homing', Homing, self.home_handler)

    
    def home_handler(self, req):
        print("Going HOME")
        self.move_panda_to_joint_position(self.homing_configuration)
        response = HomingResponse()
        response.success = True
        return response

    def move_to_handler(self, req):
        pos_np = np.array([req.position.x, req.position.y, req.position.z])
        position = (pos_np + self.desk_h).tolist()

        if not req.w_orient:
            orientation = [math.pi, 0.0, -math.pi / 4]
        else:
            orientation = [req.orientation[0], req.orientation[1], req.orientation[2]]

        self.move_panda_to_position(position, orientation)

        response = MoveToResponse()
        response.success = True
        
        return response

    def basic_traj_handler(self, req:BasicTraj._request_class):
        position = []
        orientation = []

        for i in range(len(req.position_list)):
            temp_pos_np = np.array([req.position_list[i].x, req.position_list[i].y, req.position_list[i].z])
            temp_pos = (temp_pos_np + self.desk_h).tolist()

            if not req.w_orient:
                temp_orient = [math.pi, 0.0, -math.pi / 4]
            else:
                temp_orient = [req.orientation[i].x, req.orientation[i].y, req.orientation[i].z]

            
            position.append(temp_pos)
            orientation.append(temp_orient)

        if not(len(position) == len(orientation)):
            print("Position and Orientation length are not the same")

        self.move_to_positions(position, orientation)

        response = BasicTrajResponse()
        response.success = True

        return response
    
    def arc_path_handler(self, req:ArcPath._request_class):
        if req.radius == 0 and req.times == 0:
            self.arc_path(req.center_of_circle, req.height)
        else:
            self.arc_path(req.center_of_circle, req.height, req.radius, req.times)
        response = ArcPathResponse()
        response.success = True
        return response
        pass

    def euler_to_quaternion(self, roll, pitch, yaw, model = 'sxyz'):
        quaternion = tft.quaternion_from_euler(roll, pitch, yaw, model)
        return geometry_msgs.msg.Quaternion(*quaternion)
    
    def quaternion_to_euler(self, q:geometry_msgs.msg.Quaternion, model = 'sxyz'):
        q_l = [q.x, q.y, q.z, q.w]
        euler = list(tft.euler_from_quaternion(q_l, model))
        return euler

    def set_model_state(self, pose, twist):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = 'panda'

            # Check if pose is provided and set it
            if pose is not None:
                model_state.pose = pose

            # Check if twist is provided and set it
            if twist is not None:
                model_state.twist = twist

            response = self.set_model_state_service(model_state)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def move_to_positions(self, positions, common_orientation_euler):
        for i, position in enumerate(positions):
            # Use common orientation for all positions except the last one
            orientation_euler = common_orientation_euler[i]
            self.move_panda_to_position(position, orientation_euler)

    def move_panda_to_position(self, position, euler_angles, model = 'sxyz'):
        # Set the target pose for the end-effector
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = position[0]
        target_pose.position.y = position[1]
        target_pose.position.z = position[2]

        if euler_angles == None:
            euler_angles = [math.pi, 0.0, -math.pi/4]
        
        # Set the orientation using Euler angles if provided
        if euler_angles is not None:
            quaternion = self.euler_to_quaternion(*euler_angles, model)
            target_pose.orientation = quaternion
        else:
            target_pose.orientation.w = 1.0  # Default to identity quaternion if not provided

        self.group.set_pose_target(target_pose)

        # Plan and execute the motion
        plan = self.group.go(wait=True)

    def execute_traj(self, positions_list, orientations_euler_list):
        if len(positions_list) != len(orientations_euler_list):
            print("Positions and orientation list must have the same length")
            return
        waypoints = []
        for i in range(len(positions_list)):
            position = positions_list[i]
            orientation_euler = orientations_euler_list[i] 
            waypoints.append(self._create_pose(position, orientation_euler))

        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def _create_pose(self, position, orientation_euler):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = position[0]
        target_pose.position.y = position[1]
        target_pose.position.z = position[2]

        if orientation_euler is not None:
            quaternion = self.euler_to_quaternion(*orientation_euler)
            target_pose.orientation = quaternion
        else:
            target_pose.orientation.w = 1.0  # Default to identity quaternion if not provided

        return target_pose

    def run(self):
        # Set the initial position of the Panda arm in Gazebo
        initial_position = [0.3, 0.0, 1.3]  # Adjust as needed
        if self.is_simulation:
            self.set_model_state(None, None)  # Clear any previous state
            self.set_model_state(None, None)  # Set initial state

        # Move the Panda arm to a new position using MoveIt!
        if self.is_simulation:
            target_position = [0.7, 0.0, 1.3]  # Adjust as needed [y, x, z]
            target_orientation = [math.pi, 0.0, -math.pi / 4] # [roll, pitch, yaw]
        else:
            target_position = [0.7, 0.0, 0.5] # right robot axis is y away from the robot, x is left of robots view and z is upwards
            target_orientation = [math.pi, 0.0, -math.pi / 4]
        self.move_panda_to_position(target_position, target_orientation)

        if self.is_simulation:
            target_pos_ls = [[0.3, 0.0, 1.3], [0.5, 0.0, 1.1], [0.5, 0.3, 1.4]]
            target_orient = [target_orientation, target_orientation, target_orientation]
        else:
            target_pos_ls = [[0.4, 0.0, 0.2], [0.4, 0.3, 0.4], [0.5, 0.3, 0.5]]
            target_orient = [target_orientation, target_orientation, target_orientation]
        self.move_to_positions(target_pos_ls, target_orient)
        # self.set_origin_position()
    
    def move_panda_to_joint_position(self, joint_group_position:list):
        move_group = self.group
        try:
            move_group.go(joint_group_position, wait=True)
        except joint_group_position.__len__() != 7:
            print('The data is not enough!')
    
    def set_origin_position(self):
        self.move_panda_to_joint_position(self.origin_joint_pose)

    def arc_path(self, center_of_circle:list, height, radius = 0.12, times=14):
        if center_of_circle.__len__() != 3:
            print("The input of center_of_circle is false!")
            return
        if times > 24:
            print("The euler is too large!")
            return
        self.set_origin_position()
        pose = self.group.get_current_pose().pose
        pose.position.z = center_of_circle[2] + 0.115 + height + self.desk_h[2]
        pose.position.y = center_of_circle[1] + radius*math.sin(math.pi/24*times)
        pose.position.x = center_of_circle[0] + radius*math.cos(math.pi - math.pi/24*times)
        # x = pose.position.x
        # y = pose.position.y
        q = self.quaternion_to_euler(pose.orientation)
        pose.orientation = self.euler_to_quaternion(q[0], q[1], q[2]-math.pi/12/2*times)
        q2 = q[2]-math.pi/12/2*times
        waypoints = []
        waypoints.append(deepcopy(pose))
        for i in range(1, times+1):
            pose.orientation = self.euler_to_quaternion(q[0], q[1], q2+math.pi/12*i)
            pose.position.x = center_of_circle[0] + radius * math.cos(math.pi - math.pi/12/2*times + math.pi/12*i)
            pose.position.y = center_of_circle[1] + radius * math.sin(math.pi - math.pi/12/2*times + math.pi/12*i)
            waypoints.append(deepcopy(pose))
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0)
        self.group.execute(plan)
        rospy.sleep(2)
        pass

    def grasp_position_generation(self, pose_of_cube:geometry_msgs.msg.Pose):
        euler_angle = self.quaternion_to_euler(pose_of_cube.orientation)
        pose = deepcopy(pose_of_cube)
        grasp_positions_dict = dict()
        n_euler_angle = [euler_angle[0] + math.pi, euler_angle[1], euler_angle[2] - math.pi/4]
        pose.orientation = self.euler_to_quaternion(*n_euler_angle)
        pose.position.z = pose.position.z + 0.115
        grasp_positions_dict.update({'topm':deepcopy(pose)})
        n_euler_angle = [euler_angle[0] + math.pi, euler_angle[1], euler_angle[2] - 3*math.pi/4]
        pose.orientation = self.euler_to_quaternion(*n_euler_angle)
        grasp_positions_dict.update({'tops':deepcopy(pose)})
        pose.position.z = pose_of_cube.position.z
        theta = [euler_angle[2] + i*math.pi/2 for i in range(-2, 2)]
        orien_list = ['right', 'front', 'left', 'back']
        for i in range(len(theta)):
            n_euler_angle = [euler_angle[0] + math.pi/2, euler_angle[1]+theta[i], - math.pi/4]
            pose.orientation = self.euler_to_quaternion(*n_euler_angle, 'rxyz')
            pose.position.x = pose_of_cube.position.x + 0.115*math.cos(theta[i])
            pose.position.y = pose_of_cube.position.y + 0.115*math.sin(theta[i])
            grasp_positions_dict.update({orien_list[i]:deepcopy(pose)})
        return grasp_positions_dict
        pass

    def print_current_pose(self):
        pose = self.group.get_current_pose().pose
        euler_angles = [angle*180/math.pi for angle in self.quaternion_to_euler(pose.orientation)]
        print(f"euler angle(xyz): {euler_angles}\nposition(xyz): ({pose.position.x}, {pose.position.y}, {pose.position.z})")

if __name__ == '__main__':
    # rospy.init_node('panda_move_node', log_level=rospy.DEBUG) #node with log set to debug
    rospy.init_node('panda_move_node')
    position_class = PandaMoveNode()
    rospy.spin()



