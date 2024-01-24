#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class PandaGripperController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('panda_gripper_controller_node', anonymous=True)

        # Determine if the environment is Gazebo or real robot
        self.is_gazebo = rospy.get_param('/use_sim_time', False)

        # Connect to the gripper action server
        if self.is_gazebo:
            # Gazebo gripper action server
            print("Gazeb")
            gripper_action_server = '/franka_gripper/gripper_action'
        else:
            # Real robot gripper action server
            print("Real")
            gripper_action_server = '/franka_gripper/gripper_action'

        self.gripper_client = actionlib.SimpleActionClient(gripper_action_server, GripperCommandAction)
        t_out = self.gripper_client.wait_for_server()

    def open_gripper(self):
        # Open the gripper fully
        self.set_gripper_position(0.04)  # Adjust the value based on your gripper's open position

    def close_gripper(self):
        # Close the gripper fully
        print("Close Gripper")
        self.set_gripper_position(0.0)

    def set_gripper_distance(self, distance):
        # Set the gripper to a specific distance
        print("Set Distance")
        self.set_gripper_position(distance)

    def set_gripper_position(self, position):
        # Use MoveIt! action to set gripper position
        print("Set Position")
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = 0.0  # Adjust the max effort as needed

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def cleanup(self):
        # Shutdown the ROS node
        rospy.signal_shutdown('Shutdown requested')

if __name__ == '__main__':
    try:
        panda_gripper_controller = PandaGripperController()
        print("Finish setup")

        panda_gripper_controller.set_gripper_distance(0.01)  # Adjust the distance as needed
        print("specific distance")
        rospy.sleep(5)

        # Open the gripper
        panda_gripper_controller.open_gripper()

        # Wait for a moment
        print("Open")
        rospy.sleep(5)

        # Set the gripper to a specific distance
        panda_gripper_controller.set_gripper_distance(0.01)  # Adjust the distance as needed
        print("specific distance")
        rospy.sleep(5)
        
        # Wait for a moment
        print("Close")
        rospy.sleep(5)

        # Close the gripper
        # panda_gripper_controller.close_gripper()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Clean up
        panda_gripper_controller.cleanup()