#!/usr/bin/env python3.8
import math
import os
import subprocess
from pathlib import Path
import copy
import time
import yaml 

from sensor_msgs.msg import PointCloud2, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy
import tf2_ros

from utils import visualize, transform_to_base


DATA_PATH = Path(__file__).parent.parent / "data"



class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self, cloudpoints_topic):
        rospy.init_node('decision_maker')

        sub = rospy.Subscriber(cloudpoints_topic, PointCloud2, self.callback)
        #self.publisher_filtered = rospy.Publisher("perception/point_cloud/filtered_cloud")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.on_shutdown(self.shutdown_procedure)
        self.save_signal = False
        #todo: need current positions of the robot to calculate the offset to the objects
        self.index = 0
        self.transform_index =0

    def shutdown_procedure(self):
        pass


    def save(self, pc2_msg, transform = False):
        if transform:
            transform_stamped = self.tf_buffer.lookup_transform("panda_link0", pc2_msg.header.frame_id, rospy.Time())
            pc2_msg = do_transform_cloud(pc2_msg, transform_stamped)
            """try:
                transform_stamped = self.tf_buffer.lookup_transform("panda_link0", pc2_msg.header.frame_id,
                                                                    rospy.Time(), rospy.Duration(1.0))
                pc2_msg = do_transform_cloud(pc2_msg, transform_stamped)
            except Exception as exc:
                print(exc)
                frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
                print(list(frames_dict.keys()))
                exit()"""

        points = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True) 

        # Convert points to a numpy array
        points_array = np.array(list(points))
        if transform:
            mask1 = np.where(points_array[:, 0] > 0.2, True, False)
            mask2 = np.where(points_array[:, 0] < 0.82, True, False)
            mask3 = np.where(points_array[:, 1] < 0.4, True, False)
            mask4 = np.where(points_array[:, 1] > -0.4, True, False)
            mask5 = np.where(points_array[:, 2] > 0.01, True, False)
            mask6 = np.where(points_array[:, 2] < 0.08, True, False)
            mask = np.logical_and(mask6, 
                                    np.logical_and(mask5, 
                                                    np.logical_and(mask4, 
                                                    np.logical_and(mask3, 
                                                    np.logical_and(mask1, mask2)))))
            #mask = np.logical_and(mask4, np.logical_and(mask, np.logical_and(mask2, mask3)))
            points_array_filtered = points_array[mask]
            file_path = str(DATA_PATH / f'point_cloud_transformed_test{self.transform_index}.npy')
            self.transform_index += 1
        else:
            points_array_filtered = points_array
            file_path = str(DATA_PATH / f'point_cloud_test{self.index}.npy')
            self.index += 1
        
        print("Its good writing")
        #write

        
        np.save(file_path, points_array_filtered.astype(np.float16)) #save float16 => smaller memory footprint

        return points_array_filtered, points_array

    def callback(self, pc2_msg):

        if not pc2_msg.is_dense:
            pass
            #rospy.logwarn('invalid points in Pointcloud!')
        
        if not self.save_signal:
            #if not hit save dont do anything just return
            return
        print("saving pc")
        #self.save(pc2_msg)
        self.save(pc2_msg, transform=False)
        print("saving transformed pc")
        points_array_filtered, points_array = self.save(pc2_msg, transform=True)
        
        print("Visualizing")
        # Visualize the point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points_array)
        point_cloud.paint_uniform_color([1, 0, 0])
        point_cloud_filtered = o3d.geometry.PointCloud()
        point_cloud_filtered.points = o3d.utility.Vector3dVector(points_array_filtered)
        point_cloud_filtered.paint_uniform_color([0, 1 ,0])
        coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([point_cloud, point_cloud_filtered, coord_axes], zoom=0.3,
                                              front=[-1, 0, 0],
                                              lookat=[0, 1, 0],
                                              up=[0., 0, 1])
        self.save_signal = False
        exit(0)
        """
        import time
        import yaml 
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(5.0)

        frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())
        #end test
        
        stamp = pc2_msg.header.stamp
        data = transform_to_base(pc2_msg, stamp, self.tf_buffer)
        """

if __name__ == '__main__':
    try:
        simulation_topic = "/zed2/point_cloud/cloud_registered"
        real_robot_topic = "/zed2/zed_node/point_cloud/cloud_registered"
        pch = PCHandler(cloudpoints_topic=real_robot_topic)
        #rospy.spin()
        print("starting")
        while True:
            if pch.save_signal:
                continue

            user_input = input("Press Enter: ")

            if user_input.strip() == '':
                pch.save_signal = True
            else:
                print("Input is not blank")

            
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
        exit()