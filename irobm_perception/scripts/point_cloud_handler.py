#!/usr/bin/env python3.8
import math
import os
import subprocess
from pathlib import Path
import copy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
import ros_numpy
import numpy as np
import open3d as o3d
import rospy
import pcl


DATA_PATH = Path(__file__).parent.parent / "data"

def visualize(xyz):
     # generate some neat n times 3 matrix using a variant of sync function
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
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

        rospy.on_shutdown(self.shutdown_procedure)

        #todo: need current positions of the robot to calculate the offset to the objects
    

    def callback(self, pc2_msg):

        if not pc2_msg.is_dense:
            rospy.logwarn('invalid points in Pointcloud!')

        points_converted = self.pointCloud2_to_PointXYZRGB(pc2_msg)

        points = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)

        fil = points.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(0.46, 1.5)
        cloud_filtered = fil.filter()

        # Visualize the point cloud
        print("Done one point")

    def shutdown_procedure(self):
        pass


    def pointCloud2_to_PointXYZRGB(self, point_cloud):
        points_list = []
        for data in pc2.read_points(point_cloud, skip_nans=False):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data


if __name__ == '__main__':
    try:
        pch = PCHandler()
        # dm.create_voronoi()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)