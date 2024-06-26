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

from utils import visualize

DATA_PATH = Path(__file__).parent.parent / "data"


class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self):
        rospy.init_node('decision_maker')
        self.target_point_cloud = None

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

        if self.target_point_cloud is None:
            # Initialize the target point cloud if not done already
            self.target_point_cloud = copy.deepcopy(points_array)
            visualize(self.target_point_cloud, "Target Point Cloud")
        else:
            # Apply ICP registration
            current_point_cloud = copy.deepcopy(points_array)
            transformation = self.icp_registration(current_point_cloud, self.target_point_cloud)

            # Transform the current point cloud using the estimated transformation
            current_point_cloud_transformed = self.apply_transformation(current_point_cloud, transformation)

        # Visualize the point cloud
        visualize(points_array)

        # Calculate object coordinates in camera frame
        object_coordinates_camera_frame = np.mean(current_point_cloud_transformed, axis=0)
        print("Object Coordinates (Camera Frame):", object_coordinates_camera_frame)

        # Calculate object offset in camera frame
        offset_to_object_camera_frame = np.mean(current_point_cloud_transformed - self.target_point_cloud, axis=0)
        print("Object Offset (Camera Frame):", offset_to_object_camera_frame)

        print("Done one point")

    def icp_registration(self, source, target):
        source_o3d = o3d.geometry.PointCloud()
        source_o3d.points = o3d.utility.Vector3dVector(source)

        target_o3d = o3d.geometry.PointCloud()
        target_o3d.points = o3d.utility.Vector3dVector(target)

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
        reg_p2l = o3d.pipelines.registration.ergistration_icp(
            source_o3d, target_o3d, 0.02, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=criteria
        )

        return reg_p2l.transformation

    def apply_transformation(self, points, transformation):
        points_homogeneous = np.column_stack((points, np.ones(len(points))))
        transformed_points = np.dot(transformation, points_homogeneous.T).T[:, :3]
        return transformed_points

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