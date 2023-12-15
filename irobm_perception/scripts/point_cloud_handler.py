#!/usr/bin/env python3.8
import math
import os
import subprocess
from pathlib import Path
import copy


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


DATA_PATH = Path(__file__).parent.parent / "data"

def visualize(xyz):
     # generate some neat n times 3 matrix using a variant of sync function
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    if not isinstance(xyz, np.ndarray):
        xyz = point_cloud2.read_points(xyz, field_names=("x", "y", "z"), skip_nans=True)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(xyz)

    # Update the visualization
    o3d.visualization.draw_geometries([point_cloud])
    
    """pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    o3d.visualization.draw_geometries([pcd])"""

    """
    # convert Open3D.o3d.geometry.PointCloud to numpy array
    xyz_load = np.asarray(pcd_load.points)
    print('xyz_load')
    print(xyz_load)

    o3d.io.write_point_cloud(str(DATA_PATH / "sync.ply"), pcd)

    # Load saved point cloud and visualize it
    pcd_load = o3d.io.read_point_cloud(str(DATA_PATH / "sync.ply"))

    # save z_norm as an image (change [0,1] range to [0,255] range with uint8 type)
    img = o3d.geometry.Image((z_norm * 255).astype(np.uint8))
    o3d.io.write_image(str(DATA_PATH / "sync.png"), img)
    o3d.visualization.draw_geometries([img])
    """


class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self):
        rospy.init_node('decision_maker')

        sub = rospy.Subscriber("/zed2/point_cloud/cloud_registered", PointCloud2, self.callback)
        #self.publisher_filtered = rospy.Publisher("perception/point_cloud/filtered_cloud")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
        rospy.on_shutdown(self.shutdown_procedure)

        #todo: need current positions of the robot to calculate the offset to the objects
    
    def shutdown_procedure(self):
        pass


    def callback(self, pc2_msg):

        if not pc2_msg.is_dense:
            rospy.logwarn('invalid points in Pointcloud!')

        #test
        import time
        import yaml 
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(5.0)

        frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())
        #end test

        stamp = pc2_msg.header.stamp
        data = self.transform_to_base(pc2_msg, stamp)

        points_converted = self.pointCloud2_to_PointXYZRGB(data)

        #why not just numpy.where?
        fil = points_converted.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(0.46, 1.5)
        cloud_filtered = fil.filter()

        print("Filtered the z axis")
        original_size = cloud_filtered.size
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

    
    def transform_to_base(self, pc_ros, stamp):

        lookup_time = rospy.get_rostime()
        # end_time = stamp + rospy.Duration(10)
        target_frame = "panda_link0"  # base_link
        source_frame = "camera_link"

        trans = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                lookup_time, rospy.Duration(1))

        cloud_out = do_transform_cloud(pc_ros, trans)
        return cloud_out
    
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