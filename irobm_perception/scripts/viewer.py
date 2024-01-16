#!/usr/bin/env python3.8
from pathlib import Path
import copy

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

# Uncomment for ros version
# from sensor_msgs import point_cloud2
# import rospy
# from utils import visualize, combine_pointclouds


DATA_PATH = Path(__file__).parent.parent / "data/backup"


# NO ROS VERSION
# cant pull these from utils because utils uses ros
# i.e., this part can be ran on systems without ros

def visualize(xyz):
    # generate some neat n times 3 matrix using a variant of sync function
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    if isinstance(xyz, o3d.cpu.pybind.geometry.PointCloud):
        point_cloud = xyz
    elif isinstance(xyz, np.ndarray):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
    else:
        raise NotImplementedError
    # Update the visualization
    o3d.visualization.draw_geometries([point_cloud])


def combine_pointclouds(pcds,
                        voxel_size=.001,
                        visualise=False):  # voxel_size = 0.001  implies averaging points in radius of 1mm (millimeter)
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
        # pcds[point_id].transform(pose_graph.nodes[point_id].pose) #should be used if code above (with pose graph) is used, i.e., there is transformation
        pcd_combined += pcds_new[point_id]

    # visualize(pcd_combined)
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    # o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
    if visualise:
        visualize(pcd_combined_down)
    return pcd_combined_down


def draw_registration_results(pc1, pc2=None, transformation=None):
    if not isinstance(pc1, list):
        assert pc2 is not None
        pc1_temp = copy.deepcopy(pc1)
        pc2_temp = copy.deepcopy(pc2)
        pc1_temp.paint_uniform_color([1, 0.706, 0])
        pc2_temp.paint_uniform_color([0, 0.651, 0.929])
        # source_temp.transform(transformation)
        o3d.visualization.draw_geometries([pc1_temp, pc2_temp])
    else:
        point_clouds = pc1
        colors = [[1, 0.706, 0], [0, 0.651, 0.929]]
        # Create a list of deepcopy of point clouds
        point_clouds_temp = [copy.deepcopy(pc) for pc in point_clouds]

        # Set random colors for visualization
        for i, pc_temp in enumerate(point_clouds_temp):
            color = np.random.rand(3, ) if len(colors) == 0 else colors.pop()
            pc_temp.paint_uniform_color(color)

        # Visualize the point clouds
        o3d.visualization.draw_geometries(point_clouds_temp)


def load_and_view():
    pc1 = np.load(str(DATA_PATH / f'point_cloud_transformed1.npy'))
    # pc2 = np.load(str(DATA_PATH / f'point_cloud2.npy'))
    pc2 = np.load(str(DATA_PATH / f'point_cloud_transformed2.npy'))
    pc3 = np.load(str(DATA_PATH / f'point_cloud_transformed3.npy'))
    point_cloud1 = o3d.geometry.PointCloud()
    point_cloud1.points = o3d.utility.Vector3dVector(pc1)
    point_cloud2 = o3d.geometry.PointCloud()
    point_cloud2.points = o3d.utility.Vector3dVector(pc2)
    point_cloud3 = o3d.geometry.PointCloud()
    point_cloud3.points = o3d.utility.Vector3dVector(pc3)
    # draw_registration_results([point_cloud1, point_cloud2, point_cloud3])
    # combine_pointclouds([point_cloud1, point_cloud2])
    combined_pcd = combine_pointclouds([point_cloud1, point_cloud2, point_cloud3])
    assert isinstance(combined_pcd, o3d.geometry.PointCloud)
    # o3d.visualization.draw_geometries([combined_pcd])


    # filtering
    # todo test this part
    # probably not needed
    filtered_pcd = combined_pcd.remove_statistical_outlier(nb_neighbors=16, std_ratio=10)
    outliers = combined_pcd.select_by_index(filtered_pcd[1], invert=True)
    outliers.paint_uniform_color([1, 0, 0])
    filtered_pcd = filtered_pcd[0]
    # o3d.visualization.draw_geometries([filtered_pcd])
    # end filtering

    # normals
    nn_distance = np.mean(combined_pcd.compute_nearest_neighbor_distance())
    radius_normals = 4 * nn_distance
    combined_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals, max_nn=16),
                                  fast_normal_computation=True)
    combined_pcd.paint_uniform_color([0.6, .6, .6])
    # o3d.visualization.draw_geometries([combined_pcd])

    # end normals

    # RANSAC Planar Segmentatio
    pcd = combined_pcd
    pt_to_plane_dist = .0005
    plane_model, inliners = pcd.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
    inlier_cloud = pcd.select_by_index(inliners)
    cubes_cloud = pcd.select_by_index(inliners, invert=True)

    inlier_cloud.paint_uniform_color([1., 0., 0.])
    cubes_cloud.paint_uniform_color([0.6, .6, .6])
    o3d.visualization.draw_geometries([cubes_cloud])

    # end of RANSAC

    # generate cube model
    assert isinstance(cubes_cloud, o3d.geometry.PointCloud)
    cube_model = create_cube_model(0.05)
    cube_model_cloud = mesh_to_pcd(cube_model)
    #cube_model_cloud.translate(cubes_cloud.get_center())
    result = multi_start_icp(cube_model_cloud, cubes_cloud)

    # cube_model_cloud.paint_uniform_color([1., 0., 0.])
    # o3d.visualization.draw_geometries([cube_model_cloud, cubes_cloud])

    exit()

    labels = np.array(cubes_cloud.cluster_dbscan(eps=.005, min_points=40))
    max_labels = labels.max()
    print(max_labels)
    for i in range(max_labels + 1):
        to_visualise = cubes_cloud.select_by_index(np.where(labels == i)[0])
        o3d.visualization.draw_geometries([to_visualise])

    exit()

    colors = plt.get_cmap("tab10")(labels / (max_labels if max_labels > 0 else 1))
    colors[labels < 0] = 0
    cubes_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([cubes_cloud])
    print("")


def get_icp_start(cubes_cloud):
    """
    Can be used to adjust the start of the icp
    Return has to be np.array of length 3
    """
    start = cubes_cloud.get_center()  # just center of the point cloud
    return start


def multi_start_icp(cube_model_cloud, cubes_cloud, num_attempts=100, distance_threshold=0.05):
    """
    Perform multi-start ICP to align a cube model with a point cloud containing multiple cubes,
    and return the corresponding points from the cubes_cloud for each successful alignment.

    Parameters:
    cube_model_cloud (open3d.geometry.PointCloud): The point cloud of the cube model.
    cubes_cloud (open3d.geometry.PointCloud): The point cloud containing multiple cubes.
    num_attempts (int): Number of initial poses to try.
    distance_threshold (float): Maximum distance threshold for ICP.

    Returns:
    list of tuples: Each tuple contains the transformed point cloud, the ICP fitness score,
                    and the corresponding points from cubes_cloud.
    """
    best_alignments = []
    used_custom_sample = False
    for _ in range(num_attempts):
        # Generate a random initial pose for the cube model
        initial_pose = np.eye(4)
        initial_pose[:3, :3] = np.random.uniform(-0.5, 0.5, size=(3, 3))  # Random rotation
        if not used_custom_sample:
            initial_pose[:3, 3] = get_icp_start(cubes_cloud) # Random translation
            used_custom_sample = True
        else:
            initial_pose[:3, 3] = np.random.uniform(-0.5, 0.5, size=3)  # Random translation

        # Copy the cube model and apply the initial pose
        cube_model_copy = copy.deepcopy(cube_model_cloud)
        cube_model_copy.transform(initial_pose)

        # Apply ICP
        icp_result = o3d.pipelines.registration.registration_icp(
            cube_model_copy,
            cubes_cloud,
            distance_threshold,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Store the result if the alignment is good
        if icp_result.fitness > 0.5:  # You can adjust this threshold based on your requirements
            # Find the corresponding points in cubes_cloud
            corresponding_points = find_corresponding_points(icp_result.transformation, cube_model_cloud, cubes_cloud,
                                                             distance_threshold)
            best_alignments.append((icp_result.transformation, icp_result.fitness, corresponding_points))
            #used_custom_sample = False


    # Sort the results based on fitness score (higher is better)
    best_alignments.sort(key=lambda x: x[1], reverse=True)

    return best_alignments


def find_corresponding_points(transformation, source_cloud, target_cloud, distance_threshold):
    """
    Find points in the target cloud that correspond to the aligned source cloud.

    Parameters:
    transformation (np.array): The transformation matrix obtained from ICP.
    source_cloud (open3d.geometry.PointCloud): The source
    Returns:
    open3d.geometry.PointCloud: The corresponding points in the target cloud.
    """
    # Apply the transformation to the source cloud
    transformed_source = copy.deepcopy(source_cloud)
    transformed_source.transform(transformation)

    # KDTree for efficient nearest neighbor search
    target_kdtree = o3d.geometry.KDTreeFlann(target_cloud)

    # Find corresponding points
    corresponding_indices = set()
    for point in transformed_source.points:
        [_, idx, _] = target_kdtree.search_radius_vector_3d(point, distance_threshold)
        corresponding_indices.update(idx)

    # Extract corresponding points from target cloud
    corresponding_points = np.asarray(target_cloud.points)[list(corresponding_indices)]

    # Create a point cloud for visualization
    corresponding_cloud = o3d.geometry.PointCloud()
    corresponding_cloud.points = o3d.utility.Vector3dVector(corresponding_points)

    return corresponding_cloud


def mesh_to_pcd(mesh, v2=False):
    # todo test v1 agains v2 (v2=False vs v2 = True)
    if v2:
        nr_points = 9000  # todo test lower number
        return mesh.sample_points_uniformly(number_of_points=nr_points)

    # Define the dimensions and spacing for your point grid
    width = .05  # Width of the cube
    height = .05  # Height of the cube
    depth = .05  # Depth of the cube
    spacing = 0.0025  # Spacing between points todo: test bigger value (bigger value => less points)

    # Calculate the number of points in each dimension
    num_points_x = int(width / spacing)
    num_points_y = int(height / spacing)
    num_points_z = int(depth / spacing)

    # Create a grid of points
    points = []
    for i in range(num_points_x):
        for j in range(num_points_y):
            for k in range(num_points_z):
                x = i * spacing - width / 2
                y = j * spacing - height / 2
                z = k * spacing - depth / 2
                points.append([x, y, z])

    # Convert the list of points to a NumPy array
    points_np = np.array(points, dtype=np.float32)

    # Create a point cloud from the NumPy array
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_np)
    return point_cloud


def create_cube_model(side_length):
    """
    Creates a 3D model of a cube with a given side length.

    Parameters:
    side_length (float): The length of each side of the cube.

    Returns:
    open3d.geometry.TriangleMesh: The 3D model of the cube.
    """
    # Create a cube mesh
    cube_mesh = o3d.geometry.TriangleMesh.create_box(width=side_length,
                                                     height=side_length,
                                                     depth=side_length)

    # Translate the cube to center it at the origin
    cube_mesh.translate(-np.array(cube_mesh.get_center()))

    return cube_mesh


if __name__ == '__main__':
    try:
        load_and_view()
    except Exception as exc:
        print("Something went wront")
        print(exc)

    """
    ROS VERSION
    except rospy.ROSInterruptException as exc:
    print("Something went wront")
    print(exc)
    """

    # visualize image:
    # rosrun image_view image_view image:=/zed2/zed_node/rgb/image_rect_color
