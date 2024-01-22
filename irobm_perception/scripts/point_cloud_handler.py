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

        if not pc2_msg.is_dense:
            rospy.logwarn('invalid points in Pointcloud!')

        # test
        import time
        import yaml
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(5.0)

        frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())
        # end test

        stamp = pc2_msg.header.stamp
        data = transform_to_base(pc2_msg, stamp, tf_buffer)
        if data is None:
            print("No transformation")
            return
        # visualize(data)
        points_converted = self.pointCloud2_to_PointXYZRGB(data)

        # why not just numpy.where?
        fil = points_converted.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(0, 1.3)
        cloud_filtered = fil.filter()
        print("Filtered the z axis")

        original_size = cloud_filtered.size
        # visualize(self.pointXYZRGB_to_pointCloud2(cloud_filtered)

        segmented_clusters = euclidean_clustering(cloud_filtered, self.pointXYZRGB_to_pointCloud2(cloud_filtered))
        for i, cluster in enumerate(segmented_clusters):
            print(f"Cluster {i + 1}: {cluster.shape[0]} points")

        while (cloud_filtered.size > (original_size * 40) / 100):
            indices, plane_coeff = self.segment_pcl(cloud_filtered,
                                                    pcl.SACMODEL_PLANE)
            cloud_table = cloud_filtered.extract(indices, negative=False)
            cloud_filtered = cloud_filtered.extract(indices, negative=True)

        pc2_filtered = self.pointXYZRGB_to_pointCloud2(cloud_filtered)
        print("Visualising")
        visualize(pc2_filtered)
        # Visualize the point cloud
        print("Done one point")

    def segment_pcl(self, cloud_pcl, model_type, threshold=0.006):
        seg = cloud_pcl.make_segmenter_normals(ksearch=50)
        print("Filtered the z axis2")
        seg.set_optimize_coefficients(True)
        seg.set_model_type(model_type)
        seg.set_normal_distance_weight(0.15)
        print("Filtered the z axis")
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(200)
        seg.set_distance_threshold(threshold)  # 0.006
        indices, coefficients = seg.segment()
        print("Filtered the z axis3")
        # https://pointclouds.org/documentation/group__sample__consensus.html
        # describes the coeffici
        if len(indices) == 0:
            print('Could not estimate a planar model for the given dataset.')
            exit(0)

        return indices, coefficients

    def pointCloud2_to_PointXYZRGB(self, point_cloud):
        points_list = []
        for data in pc2.read_points(point_cloud, skip_nans=False):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data

    def pointXYZRGB_to_pointCloud2(self, pcl_cloud):
        ros_msg = PointCloud2()

        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "base_footprint"

        ros_msg.height = 1
        ros_msg.width = pcl_cloud.size

        ros_msg.fields.append(
            PointField(name="x",
                       offset=0,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="y",
                       offset=4,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="z",
                       offset=8,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="rgb",
                       offset=16,
                       datatype=PointField.FLOAT32,
                       count=1))

        ros_msg.is_bigendian = False
        ros_msg.point_step = 32
        ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
        ros_msg.is_dense = False
        buffer = []

        for data in pcl_cloud:
            s = struct.pack('>f', data[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            buffer.append(
                struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b,
                            g, r, 0, 0, 0, 0))

        ros_msg.data = b"".join(buffer)

        return ros_msg


if __name__ == '__main__':
    try:
        pch = PCHandler()
        # dm.create_voronoi()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
