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
        # numpify returns numpy ndarray that has 4 entries ['x','y','z','rgb'], where each has dim (720,1280)
        # this is due to fact that we are processing 2d image and stereo camera calculates (using two cameras) pointcloud
        # this means that for every pixel, i.e., (x,y)-coordinate of the image we have corresponding x,y,z relative coordinates 
        # (x,y,z) - position in space
        """
        #half version (half the nr of pixels)
        pc = ros_numpy.numpify(pc2_msg) 
        height = pc.shape[0]
        width = pc.shape[1]#
        half_width = int(width / 1.2)
        half_height = int(height / 1.5)
        l_w = half_width // 2
        r_w = width - (half_width //2)
        
        
        l_h = half_height // 2
        r_h = half_height * 2 - (half_height //2)
        
        np_points = np.zeros((half_height * half_width, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'][:half_height, :half_width], half_height * half_width) #flattened real-world (relative to camera) x coordinate of all pixels
        np_points[:, 1] = np.resize(pc['y'][:half_height,  :half_width], half_height * half_width) #flattened real-world (relative to camera) y coordinate of all pixels
        np_points[:, 2] = np.resize(pc['z'][:half_height,  :half_width], half_height * half_width) #flattened real-world (relative to camera) z coordinate of all pixels
        visualize(np_points)
        """

        points = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True) 
        
        # Convert points to a numpy array
        points_array = np.array(list(points))
        
        # Visualize the point cloud
        visualize(points_array)

        print("Done one point")

    def shutdown_procedure(self):
        pass





if __name__ == '__main__':
    try:
        pch = PCHandler()
        # dm.create_voronoi()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)