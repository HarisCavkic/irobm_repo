#!/usr/bin/env python3.8
import math
import os
import subprocess
from pathlib import Path
import copy
import actionlib
from irobm_control.srv import MoveTo, MoveToResponse, BasicTraj, BasicTrajResponse

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import struct
import ctypes
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy
import tf2_ros
import pcl
from pcl import PointCloud

from utils import visualize, transform_to_base

DATA_PATH = Path(__file__).parent.parent / "data"


def euclidean_clustering(segmented_cloud, cloud):
    """
    Perform Euclidean clustering on a PointCloud_PointXYZRGB object.

    Parameters:
    - segmented_cloud: pcl._pcl.PointCloud_PointXYZRGB object.

    Returns:
    - clusters: List of numpy arrays, each representing a cluster.
    """

    # Convert PCL PointCloud to numpy array
    cloud_np = np.asarray(segmented_cloud)

    # Create a PCL EuclideanClusterExtraction object
    ec = cloud.make_EuclideanClusterExtraction()

    # Set the cluster tolerance (adjust based on your data)
    ec.set_ClusterTolerance(0.02)

    # Set the minimum and maximum cluster size (adjust based on your data)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(30)

    cluster_indices = ec.Extract()

    # Extract clusters from the original cloud using indices
    clusters = []
    for indices in cluster_indices:
        cluster_points = np.asarray(cloud.extract(indices))
        clusters.append(cluster_points)

    return clusters


class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self):
        rospy.init_node('decision_maker')
        self.sub = rospy.Subscriber("/zed2/point_cloud/cloud_registered", PointCloud2, self.callback)
        self.service = rospy.ServiceProxy('/move_to', MoveTo)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.on_shutdown(self.shutdown_procedure)

        self.positions = [...]
        self.current_cloud = None
        self.transform_index = 0
        # initial setup
        self.check_service_and_process_positions()
        # todo: need current positions of the robot to calculate the offset to the objects

    def shutdown_procedure(self):
        pass

    def save_cloud(self, pc2_msg):
        pc2_msg = transform_to_base(pc2_msg, pc2_msg.header.stamp, self.tf_buffer)
        points = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert points to a numpy array
        points_array = np.array(list(points))
        # todo check weather this all is necessary
        mask = np.where(points_array[:, 2] < 1.3, True, False)
        mask3 = np.where(points_array[:, 1] < 1.3, True, False)
        mask2 = np.where(points_array[:, 0] < 1.3, True, False)
        mask = np.logical_and(mask, np.logical_and(mask2, mask3))
        points_array = points_array[mask]

        file_path = str(DATA_PATH / f'point_cloud_transformed{self.transform_index}.npy')
        self.transform_index += 1

        print("Its good writing")
        # write
        np.save(file_path, points_array.astype(np.float16))  # save float16 => smaller memory footprint
        return points_array

    def do_cloud_preproc(self):
        pass

    def check_service_and_process_positions(self):
        rospy.wait_for_service('/move_to')
        for position in self.positions:
            self.move_to_position_and_wait(position)
            self.process_received_cloud()

        self.do_cloud_preproc()

    def move_to_position_and_wait(self, position):
        # Make sure to define the MoveTo service message format
        rospy.wait_for_service('/move_to')
        try:
            # todo adjust
            move_to_request = MoveTo()  # Modify with actual request format
            move_to_request.position = position  # Modify according to the actual service message fields
            response = self.service(move_to_request)  # this is blocking operation

            # Wait for the service to complete
            # Implement any specific waiting logic here if required
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def process_received_cloud(self):
        if self.current_cloud:
            self.save_cloud(self.current_cloud)
            self.current_cloud = None

    def callback(self, pc2_msg):
        self.current_cloud = pc2_msg



if __name__ == '__main__':
    try:
        pch = PCHandler()
        # dm.create_voronoi()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
