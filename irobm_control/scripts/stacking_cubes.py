#!/usr/bin/env python3

import rospy
import sys
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class PandaMoveNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_move_node', anonymous=True)

        # Initialize MoveIt!
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")

        # Check if running in simulation
        is_simulation = False

        if is_simulation:
            # Initialize Gazebo service
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        else:
            # Additional initialization for the real robot, if needed
            pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return geometry_msgs.msg.Quaternion(*quaternion)

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
            orientation_euler = common_orientation_euler if i == len(positions) - 1 else None
            self.move_panda_to_position(position, orientation_euler)

    def move_panda_to_position(self, position, euler_angles):
        # Set the target pose for the end-effector
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = position[0]
        target_pose.position.y = position[1]
        target_pose.position.z = position[2]

        if euler_angles == None:
            euler_angles = [3.1415, 0.0, 0.0]
        
        # Set the orientation using Euler angles if provided
        if euler_angles is not None:
            quaternion = self.euler_to_quaternion(*euler_angles)
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
        self.set_model_state(None, None)  # Clear any previous state
        self.set_model_state(None, None)  # Set initial state

        # Move the Panda arm to a new position using MoveIt!
        target_position = [0.7, 0.0, 1.3]  # Adjust as needed [y, x, z]
        target_orientation = [3.1415, 0.0, 0.0] # [roll, pitch, yaw]
        self.move_panda_to_position(target_position, target_orientation)

        target_pos_ls = [[0.3, 0.0, 1.3], [0.5, 0.0, 1.1], [0.5, 0.0, 1.4]]
        target_orient = [target_orientation, target_orientation, target_orientation]
        self.execute_traj(target_pos_ls, target_orient)

if __name__ == '__main__':
    try:
        panda_move_node = PandaMoveNode()
        panda_move_node.run()
    except rospy.ROSInterruptException:
        pass
