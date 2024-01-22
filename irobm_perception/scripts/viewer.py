#!/usr/bin/env python3.8
from pathlib import Path
import copy
import random

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

# Uncomment for ros version
# from sensor_msgs import point_cloud2
# import rospy
# from utils import visualize, combine_pointclouds


# DATA_PATH = Path(__file__).parent.parent / "data/backup"
DATA_PATH = Path(__file__).parent.parent / "data"


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


def draw_registration_results(source, target, transformation=None):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def load_and_view():
    pc1 = np.load(str(DATA_PATH / f'point_cloud_transformed0.npy'))
    # pc2 = np.load(str(DATA_PATH / f'point_cloud2.npy'))
    pc2 = np.load(str(DATA_PATH / f'point_cloud_transformed1.npy'))
    pc3 = np.load(str(DATA_PATH / f'point_cloud_transformed2.npy'))
    point_cloud1 = o3d.geometry.PointCloud()
    point_cloud1.points = o3d.utility.Vector3dVector(pc1)
    point_cloud2 = o3d.geometry.PointCloud()
    point_cloud2.points = o3d.utility.Vector3dVector(pc2)
    point_cloud3 = o3d.geometry.PointCloud()
    point_cloud3.points = o3d.utility.Vector3dVector(pc3)
    # o3d.visualization.draw_geometries([point_cloud1, point_cloud2, point_cloud3])
    # combine_pointclouds([point_cloud1, point_cloud2])
    combined_pcd = combine_pointclouds([point_cloud1, point_cloud2, point_cloud3], visualise=False)
    assert isinstance(combined_pcd, o3d.geometry.PointCloud)
    #o3d.visualization.draw_geometries([combined_pcd])

    ###
    # recorded pc has outliers this is brute force to remove these
    points_array = np.array(list(combined_pcd.points))
    # todo check weather this all is necessary
    mask = np.where(points_array[:, 2] < 1.3, True, False)
    mask3 = np.where(points_array[:, 1] < 0.8, True, False)
    mask2 = np.where(points_array[:, 0] < 1.3, True, False)
    mask = np.logical_and(mask, np.logical_and(mask2, mask3))
    points_array = points_array[mask]
    points_pcd = o3d.geometry.PointCloud()
    points_pcd.points = o3d.utility.Vector3dVector(points_array)
    combined_pcd.paint_uniform_color([1, 0, 0])
    points_pcd.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw_geometries([combined_pcd, points_pcd])
    #####

    # filtering
    # todo test this part
    # probably not needed
    combined_pcd = points_pcd
    filtered_pcd = combined_pcd.remove_statistical_outlier(nb_neighbors=16, std_ratio=10)
    outliers = combined_pcd.select_by_index(filtered_pcd[1], invert=True)
    outliers.paint_uniform_color([1, 0, 0])
    filtered_pcd = filtered_pcd[0]
    # o3d.visualization.draw_geometries([filtered_pcd])
    # end filtering

    # normals
    combined_pcd = filtered_pcd
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
    #o3d.visualization.draw_geometries([cubes_cloud])

    # end of RANSAC

    # dbscan for rest
    # works only if cubes not near each other (really near)
    assert isinstance(cubes_cloud, o3d.geometry.PointCloud)
    labels = np.array(cubes_cloud.cluster_dbscan(eps=0.005, min_points=5))
    max_labels = labels.max()

    colors = plt.get_cmap("tab10")(labels / max_labels if max_labels > 0 else 1)
    colors[labels < 0] = 0
    cubes_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # o3d.visualization.draw_geometries([cubes_cloud])

    segmented_cubes = list()

    for i in range(0, max_labels + 1):
        print("LABEL: ", i)
        indices_to_extract = np.where(labels == i)

        # Convert PointCloud to a NumPy array
        cubes_points_ndarray = np.asarray(cubes_cloud.points)

        # Extract points based on indices
        segment = cubes_points_ndarray[indices_to_extract]

        # Create a new PointCloud from the selected points
        selected_cloud = o3d.geometry.PointCloud()
        selected_cloud.points = o3d.utility.Vector3dVector(segment)
        segmented_cubes.append(selected_cloud)
        #o3d.visualization.draw_geometries([selected_cloud])

    # Global alignment
    all_transformations = list()
    cubes_list = list()
    for segment in segmented_cubes:
        cube_model = create_cube_model(0.046)
        cube_model_cloud = mesh_to_pcd(cube_model, v2=False, hollow_cube=True, size=0.046)
        # set cube to 0 position
        cube_model_cloud.translate(-cube_model_cloud.get_center(), relative = False)

        initial_transformation = np.eye(4)
        initial_transformation[:3, 3] = get_icp_start(segment) #push near target cloud
        cube_model_cloud.transform(initial_transformation)

        threshold = 10
        result = do_icp(cube_model_cloud, segment, distance_threshold=threshold)
        transformation, fitness = result
        ##test visual
        cube_model_cloud.transform(transformation)
        cube_model_cloud.paint_uniform_color([1., 0., 0.])
        segment.paint_uniform_color([0.6, .6, .6])
        cubes_list.append(cube_model_cloud)
        #o3d.visualization.draw_geometries([segment, cube_model_cloud])
        #o3d.visualization.draw_geometries([cubes_cloud, cube_model_cloud])

        all_transformations.append(np.dot(transformation, initial_transformation))

        """cube_model_new = create_cube_model(0.046)
        cube_model_cloud_new = mesh_to_pcd(cube_model_new, v2=False, hollow_cube=True, size=0.046)
        cube_model_cloud_new.translate(-cube_model_cloud_new.get_center(), relative=False)
        cube_model_cloud_new.transform(final_trafo)
        o3d.visualization.draw_geometries([segment, cube_model_cloud_new])"""



        """for iter_nr in range(2):
            threshold = 10 if iter_nr == 0 else 0.1
            result = do_icp(cube_model_cloud, segment, distance_threshold=threshold)
            transformation, fitness = result
            ##test visual
            cube_model_cloud.transform(transformation)
            cube_model_cloud.paint_uniform_color([1., 0., 0.])
            segment.paint_uniform_color([0.6, .6, .6])
            o3d.visualization.draw_geometries([segment, cube_model_cloud])
            o3d.visualization.draw_geometries([cubes_cloud, cube_model_cloud])

        voxel_size = 0.0005  # test
        source_down, source_fpfh = preprocess_point_cloud(cube_model_cloud, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(segment, voxel_size)
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    voxel_size)
       """
    all_clouds = [cubes_cloud]
    all_clouds.extend(cubes_list)
    o3d.visualization.draw_geometries(all_clouds)
    exit()
    # generate cube model

    cube_model = create_cube_model(0.05)
    cube_model_cloud = mesh_to_pcd(cube_model)
    # cube_model_cloud.translate(cubes_cloud.get_center())
    result = multi_start_icp(cube_model_cloud, cubes_cloud)

    # cube_model_cloud.paint_uniform_color([1., 0., 0.])
    # o3d.visualization.draw_geometries([cube_model_cloud, cubes_cloud])

    labels = np.array(cubes_cloud.cluster_dbscan(eps=.005, min_points=40))
    max_labels = labels.max()
    print(max_labels)
    for i in range(max_labels + 1):
        to_visualise = cubes_cloud.select_by_index(np.where(labels == i)[0])
        o3d.visualization.draw_geometries([to_visualise])

    colors = plt.get_cmap("tab10")(labels / (max_labels if max_labels > 0 else 1))
    colors[labels < 0] = 0
    cubes_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([cubes_cloud])
    print("")


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.05),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def get_icp_start(cubes_cloud):
    """
    Can be used to adjust the start of the icp
    Return has to be np.array of length 3
    """
    start = cubes_cloud.get_center()  # just center of the point cloud
    return start


def do_icp(cube_model_cloud, segmented_cloud, distance_threshold=10):
    # initial_pose = np.eye(4)
    # initial_pose[:3, :3] = np.random.uniform(-0.5, 0.5, size=(3, 3))
    # initial_pose[:3, 3] = get_icp_start(segmented_cloud)

    cube_model_copy = copy.deepcopy(cube_model_cloud)
    # cube_model_copy.transform(initial_pose)

    # Apply ICP
    icp_result = o3d.pipelines.registration.registration_icp(
        cube_model_copy,  # source
        segmented_cloud,  # target
        distance_threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    if icp_result.fitness < 0.5:
        # if normal icp did not work do multi start
        print("Didnt work. Doing multi start icp")
        raise NotImplementedError
        result = multi_start_icp(cube_model_cloud, segmented_cloud)[0]
    else:
        result = (icp_result.transformation, icp_result.fitness)

    return result


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
            initial_pose[:3, 3] = get_icp_start(cubes_cloud)  # Random translation
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
            best_alignments.append((icp_result.transformation, icp_result.fitness))
            # used_custom_sample = False

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


def mesh_to_pcd(mesh, v2=False, hollow_cube=True, size=0.05, randomness_factor = 0.001):
    # todo test v1 agains v2 (v2=False vs v2 = True)

    # Define the dimensions and spacing for your point grid
    width = size  # Width of the cube
    height = size  # Height of the cube
    depth = size  # Depth of the cube
    spacing = 0.0025  # Spacing between points todo: test bigger value (bigger value => less points)

    # Calculate the number of points in each dimension
    num_points_x = int(width / spacing)
    num_points_y = int(height / spacing)
    num_points_z = int(depth / spacing)
    get_randomness = lambda: 0

    if v2:
        # Maximum random offset for each point
        nr_points = 6000  # todo test lower number
        get_randomness = lambda: random.uniform(-randomness_factor, randomness_factor)
        return mesh.sample_points_uniformly(number_of_points=nr_points)

    # Create a grid of points
    points = []
    if not hollow_cube:

        for i in range(num_points_x):
            for j in range(num_points_y):
                for k in range(num_points_z):
                    x = i * spacing - width / 2
                    y = j * spacing - height / 2
                    z = k * spacing - depth / 2
                    points.append([x, y, z])

    else:
        for i in range(num_points_x):
            for j in range(num_points_y):
                # Add points on the top and bottom faces
                # points.append([i * spacing - width / 2 + get_randomness(), j * spacing - height / 2 + get_randomness(), -depth / 2 + get_randomness()]) #comment this line for not generating the bottom side
                points.append([i * spacing - width / 2 + get_randomness(), j * spacing - height / 2 + get_randomness(),
                               depth / 2+ get_randomness()] )

        for i in range(num_points_x):
            for k in range(num_points_z):
                # Add points on the front and back faces
                # points.append([i * spacing - width / 2+ get_randomness(), -height / 2+ get_randomness(), k * spacing - depth / 2+ get_randomness()])#comment this line for not generating the back side
                points.append([i * spacing - width / 2 + get_randomness(), height / 2 + get_randomness(),
                               k * spacing - depth / 2 + get_randomness()])

        for j in range(num_points_y):
            for k in range(num_points_z):
                # Add points on the left and right faces
                points.append([-width / 2 + get_randomness(), j * spacing - height / 2 + get_randomness(),
                               k * spacing - depth / 2 + get_randomness()])
                #points.append([width / 2 + get_randomness(), j * spacing - height / 2 + get_randomness(),
                #               k * spacing - depth / 2 + get_randomness()])

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

    #cube_mesh.remove_vertices_by_index([1])
    # Translate the cube to center it at the origin
    cube_mesh.translate(-np.array(cube_mesh.get_center()))

    return cube_mesh


if __name__ == '__main__':
    load_and_view()
    """try:
        load_and_view()
    except Exception as exc:
        print("Something went wront")
        print(exc)"""

    """
    ROS VERSION
    except rospy.ROSInterruptException as exc:
    print("Something went wront")
    print(exc)
    """

    # visualize image:
    # rosrun image_view image_view image:=/zed2/zed_node/rgb/image_rect_color
