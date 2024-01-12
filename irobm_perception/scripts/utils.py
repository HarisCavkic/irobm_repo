from pathlib import Path
import copy
import yaml

import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros


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


def combine_pointclouds(pcds, voxel_size = .001): #voxel_size = 0.001  implies averaging points in radius of 1mm (millimeter)
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


def transform_to_base(pc_ros, stamp, tf_buffer):
        
        lookup_time = rospy.get_rostime()
        # end_time = stamp + rospy.Duration(10)
        target_frame = "panda_link0"  # base_link
        try:
            trans = tf_buffer.lookup_transform(target_frame, pc_ros.header.frame_id,
                                                    lookup_time, rospy.Duration(1))

            cloud_out = do_transform_cloud(pc_ros, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform exception: %s", str(e))
            frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
            print(list(frames_dict.keys()))
            cloud_out = None
        return cloud_out