#!/usr/bin/env python3.8
from pathlib import Path
import copy

import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy


DATA_PATH = Path(__file__).parent.parent / "data/backup"

def draw_registration_results(source, target, transformation = None):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    #source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def visualize(xyz):
     # generate some neat n times 3 matrix using a variant of sync function
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    if isinstance(xyz, o3d.cpu.pybind.geometry.PointCloud):
        point_cloud = xyz
    elif isinstance(xyz, np.ndarray):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
    elif not isinstance(xyz, np.ndarray):
        xyz = point_cloud2.read_points(xyz, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.array(list(xyz))
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(xyz)

    # Update the visualization
    o3d.visualization.draw_geometries([point_cloud])
    
def combine(pcds, voxel_size = .001): #voxel_size = 0.001  implies averaging points in radius of 1mm (millimeter)
    pcds_new = list()
    if isinstance(pcds[0], np.ndarray):
        for pcd in pcds:
            pcds_new.append(o3d.utility.Vector3dVector(pcd))
    else:
        pcds_new = pcds

    pose_graph = o3d.pipelines.registration.PoseGraph()
    pcd_combined = o3d.geometry.PointCloud()
    """max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down,
                                    max_correspondence_distance_coarse,
                                    max_correspondence_distance_fine)
                                    
    #this part of the code should be there if registraion is used, i.e., there is translation and rotation between Point Clouds that is then written in pose graph
    """
    for point_id in range(len(pcds_new)):
        #pcds[point_id].transform(pose_graph.nodes[point_id].pose) #should be used if code above (with pose graph) is used, i.e., there is transformation
        pcd_combined += pcds_new[point_id]
    
    #visualize(pcd_combined)
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    #o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
    visualize(pcd_combined_down)

def load_and_view(): 
    pc1 = np.load(str(DATA_PATH / f'point_cloud_transformed1.npy'))
    #pc2 = np.load(str(DATA_PATH / f'point_cloud2.npy'))
    pc2 = np.load(str(DATA_PATH / f'point_cloud_transformed2.npy'))
    point_cloud1 = o3d.geometry.PointCloud()
    point_cloud1.points = o3d.utility.Vector3dVector(pc1)
    point_cloud2 = o3d.geometry.PointCloud()
    point_cloud2.points = o3d.utility.Vector3dVector(pc2)
    #draw_registration_results(point_cloud2, point_cloud1)
    combine([point_cloud1, point_cloud2])


if __name__ == '__main__':
    try:
        load_and_view()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)

    #visualize image:
    #rosrun image_view image_view image:=/zed2/zed_node/rgb/image_rect_color