#!/usr/bin/env python3
import rospy
import numpy as np
import random
import math
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel

def spawn_cubes(num_cubes, start_position):
    rospy.init_node('spawn_cubes', anonymous=True)
    Spawning = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    cube_sdf="""
    <?xml version="1.0" ?>
    <sdf version="1.4">
    <model name='%NAME%'>
      <static>0</static>
      <link name='%NAME%'>
        <inertial>
            <mass>0.066</mass>
            <inertia> <!-- inertias are tricky to compute -->
              <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
              <ixx>0.0000221859</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
              <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
              <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
              <iyy>0.0000221859</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
              <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
              <izz>0.0000221859</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
            </inertia>
          </inertial>
        <collision name='collision'>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode>
                <max_vel>0</max_vel>
                <min_depth>0.003</min_depth>
              </ode>
            </contact>
            <!--NOTE: Uses dynamic friction of brick on a wood surface
            see https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
            -->
            <friction>
              <ode>
                <mu>10</mu>
                <mu2>10</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
          </surface>
          <geometry>
            <box>
              <size> 0.045 0.045 0.045 </size>
            </box>
          </geometry>
        </collision>
        <visual name='%NAME%'>
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://%NAME%.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
      <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
        <frameName>world</frameName>
        <bodyName>%NAME%</bodyName>
        <topicName>%NAME%_odom</topicName>
        <updateRate>30.0</updateRate>
      </plugin>
    </model>
    """

    cube_urdf="""
    <?xml version="1.0" ?>
    <robot name="%NAME%" xmlns:xacro="http://ros.org/wiki/xacro">
        <link name="%NAME%">
            <inertial>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <mass value="0.066" />
              <inertia ixx="0.0000221859" ixy="0.0" ixz="0.0" iyy="0.0000221859" iyz="0.0" izz="0.0000221859" />
            </inertial>
            <collision>
                <geometry>
                  <box size="0.045 0.045 0.045" />
                </geometry>
            </collision>
            <visual>
                <geometry>
                  <mesh filename="package://franka_zed_gazebo/meshes/%NAME%.dae" scale='1 1 1'/>
                </geometry>
            </visual>
        </link>
        <gazebo>
          <collision name="%NAME%_collision">
          <max_contacts>10</max_contacts>
            <surface>
              <contact>
                <ode>
                  <max_vel>0</max_vel>
                  <min_depth>0.003</min_depth>
                </ode>
              </contact>
              <!--NOTE: Uses dynamic friction of brick on a wood surface
              see https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
              -->
              <friction>
                <ode>
                  <mu>10</mu>
                  <mu2>10</mu2>
                  <fdir1>0 0 0</fdir1>
                  <slip1>0</slip1>
                  <slip2>0</slip2>
                </ode>
                <torsional>
                  <coefficient>1</coefficient>
                  <patch_radius>0</patch_radius>
                  <surface_radius>0</surface_radius>
                  <use_patch_radius>1</use_patch_radius>
                  <ode>
                    <slip>0</slip>
                  </ode>
                </torsional>
              </friction>
              <bounce>
                <restitution_coefficient>0</restitution_coefficient>
                <threshold>1e+06</threshold>
              </bounce>
            </surface>
          </collision>
          <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
            <frameName>world</frameName>
            <bodyName>%NAME%</bodyName>
            <topicName>%NAME%_odom</topicName>
            <updateRate>30.0</updateRate>
          </plugin>
        </gazebo>
    </robot>
    """

    xpose, ypose, zpose = start_position
    distance_between_cubes = 0.1

    def spawn(id, position, orientation):
        model_name = 'cube_{0}'.format(id)
        model_xml = cube_sdf.replace('%NAME%', model_name)
        cube_pose = Pose(Point(*position), Quaternion(*quaternion_from_euler(*orientation)))
        Spawning(model_name, model_xml, "", cube_pose, "world")
        rospy.loginfo("%s was spawned in Gazebo", model_name)

    cube_id = 0
    for i in range(num_cubes):
        row = i // 3  # Assuming 5 columns in the grid
        col = i % 3
        position = [xpose + col * distance_between_cubes,
                    ypose + row * distance_between_cubes,
                    zpose]
        position = [random.uniform(0.25, 0.69), random.uniform(-0.3, 0.3), zpose]
        print(f'X: {position[0]}  Y: {position[1]}  Z: {position[2]}')
        orientation = [0, 0, random.uniform(-math.pi, math.pi)] # You can modify the orientation if needed
        print(f'Orientation: {orientation}')
        spawn(cube_id, position, orientation)
        cube_id = cube_id + 1

if __name__ == "__main__":
    # Specify the number of cubes to spawn and the starting position
    num_cubes_to_spawn = 8
    starting_position = [0.5, 0, 0.84]
    spawn_cubes(num_cubes_to_spawn, starting_position)