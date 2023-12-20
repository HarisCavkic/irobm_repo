#!/usr/bin/env python3.8
from pathlib import Path
import copy

import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy

from utils import visualize, combine_pointclouds

DATA_PATH = Path(__file__).parent.parent / "data/backup"

def draw_registration_results(source, target, transformation = None):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    #source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


    

def load_and_view(): 
    pc1 = np.load(str(DATA_PATH / f'point_cloud_transformed1.npy'))
    #pc2 = np.load(str(DATA_PATH / f'point_cloud2.npy'))
    pc2 = np.load(str(DATA_PATH / f'point_cloud_transformed2.npy'))
    pc3 = np.load(str(DATA_PATH / f'point_cloud_transformed3.npy'))
    point_cloud1 = o3d.geometry.PointCloud()
    point_cloud1.points = o3d.utility.Vector3dVector(pc1)
    point_cloud2 = o3d.geometry.PointCloud()
    point_cloud2.points = o3d.utility.Vector3dVector(pc2)
    point_cloud3 = o3d.geometry.PointCloud()
    point_cloud3.points = o3d.utility.Vector3dVector(pc2)
    draw_registration_results(point_cloud2, point_cloud3)
    combine_pointclouds([point_cloud1, point_cloud2])
    combine_pointclouds([point_cloud1, point_cloud2, point_cloud3])



if __name__ == '__main__':
    try:
        load_and_view()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)

    #visualize image:
    #rosrun image_view image_view image:=/zed2/zed_node/rgb/image_rect_color