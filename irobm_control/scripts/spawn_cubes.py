#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel

def spawn_cubes(num_cubes, start_position):
    rospy.init_node('spawn_cubes', anonymous=True)
    Spawning = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    cube_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.4">
    <model name='%NAME%'>
      <static>0</static>
      <link name='%NAME%'>
        <inertial>
            <mass>0.066</mass>
            <inertia>
              <ixx>0.0000221859</ixx>
              <ixy>0.0</ixy>
              <ixz>0.0</ixz>
              <iyy>0.0000221859</iyy>
              <iyz>0.0</iyz>
              <izz>0.0000221859</izz>
            </inertia>
          </inertial>
        <collision name='collision'>
          <surface>
            <friction>
              <ode>
              <mu>0.6</mu> 
              <mu2>0.6</mu2>
              </ode>
            </friction>
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
    </sdf>
    """

    xpose, ypose, zpose = start_position
    distance_between_cubes = 0.1

    def spawn(id, position, orientation):
        model_name = 'cube_{0}'.format(id)
        model_xml = cube_sdf.replace('%NAME%', model_name)
        cube_pose = Pose(Point(*position), Quaternion(*quaternion_from_euler(*orientation)))
        Spawning(model_name, model_xml, "", cube_pose, "world")
        rospy.loginfo("%s was spawned in Gazebo", model_name)

    for i in range(num_cubes):
        row = i // 3  # Assuming 5 columns in the grid
        col = i % 3
        position = [xpose + col * distance_between_cubes,
                    ypose + row * distance_between_cubes,
                    zpose]
        print(f'X: {position[0]}  Y: {position[1]}  Z: {position[2]}')
        orientation = [0, 0, 0]  # You can modify the orientation if needed
        spawn(i, position, orientation)

if __name__ == "__main__":
    # Specify the number of cubes to spawn and the starting position
    num_cubes_to_spawn = 9
    starting_position = [0.5, 0, 0.8]
    spawn_cubes(num_cubes_to_spawn, starting_position)