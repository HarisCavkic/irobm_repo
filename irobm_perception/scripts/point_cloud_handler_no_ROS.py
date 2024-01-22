#!/usr/bin/env python3.8
from pathlib import Path
import copy

import numpy as np
import open3d as o3d

import matplotlib.pyplot as plt


DATA_PATH = Path(__file__).parent.parent / "data"


class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self):
        self.current_cloud = None
        self.transform_index = 3
        self.combined_pcd = None
        self.transformations = None
        # initial setup
        self.check_service_and_process_positions(visualize = True)
        # todo: need current positions of the robot to calculate the offset to the objects

    def shutdown_procedure(self):
        pass

    def callback(self, pc2_msg):
        self.current_cloud = pc2_msg

    def check_service_and_process_positions(self, visualize = False):
        self.do_cloud_preproc(visualize)

    def do_cloud_preproc(self, visualize=False):
        self.combined_pcd = self.combine_pointclouds(
            visualise=True)  # visualize only for debugging otherwise its blocking

        self.combined_pcd.paint_uniform_color([0.6, .6, .6])

        # remove outliers, i.e., do filtering
        self.remove_outliers(visualize)

        # estimate the normals
        self.estimate_normals(visualize)

        # RANSAC Planar Segmentation, i.e., remove the desk
        self.run_RANSAC_plane(visualize)
        self.run_RANSAC_plane(visualize)
        # db scan rest of the cloud, i.e., segment the cubes
        segmented_cubes = self.do_dbscan(visualize)

        self.transformations = self.get_transformations(segmented_cubes, visualize)

    def combine_pointclouds(self,
                            voxel_size=.001,
                            visualise=False):  # voxel_size = 0.001  implies averaging points in radius of 1mm (millimeter)
        pcds = list()
        nr_loaded_clouds = 0  # if there is problem with loading one of the pcds we dont want index out of bounds
        # load the data
        for i in range(self.transform_index):
            try:
                pc = np.load(str(DATA_PATH / f'point_cloud_transformed{i}.npy'))
                pcds.append(o3d.geometry.PointCloud())
                pcds[nr_loaded_clouds].points = o3d.utility.Vector3dVector(pc)
                nr_loaded_clouds += 1
            except Exception as exc:
                print(f"Something went wront when loading the pointcloud number {i}")
                print(exc)

        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds)):
            # pcds[point_id].transform(pose_graph.nodes[point_id].pose) #should be used if code above (with pose graph) is used, i.e., there is transformation
            pcd_combined += pcds[point_id]

        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        if visualise:
            o3d.visualization.draw_geometries([pcd_combined_down])  # todo check weather 1.3 is good or should be lower
        return pcd_combined_down

    def remove_outliers(self, visualize=False):
        """
            remove outliers, i.e., do filtering
        """
        filtered_pcd = self.combined_pcd.remove_statistical_outlier(nb_neighbors=16, std_ratio=10)
        final_cloud = filtered_pcd[0]
        if visualize:
            outliers = self.combined_pcd.select_by_index(filtered_pcd[1], invert=True)
            outliers.paint_uniform_color([1, 0, 0])
            o3d.visualization.draw_geometries([final_cloud])

        self.combined_pcd = final_cloud

    def estimate_normals(self, visualize=False):
        nn_distance = np.mean(self.combined_pcd.compute_nearest_neighbor_distance())
        radius_normals = 4 * nn_distance  # just some approximation, rule of the thumb
        self.combined_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals, max_nn=16),
            fast_normal_computation=True)
        if visualize:
            self.combined_pcd.paint_uniform_color([0.6, .6, .6])
            o3d.visualization.draw_geometries([self.combined_pcd])

    def run_RANSAC_plane(self, visualize=False):
        pt_to_plane_dist = .0005
        plane_model, inliners = self.combined_pcd.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3,
                                                                num_iterations=1000)
        inlier_cloud = self.combined_pcd.select_by_index(inliners)
        cubes_cloud = self.combined_pcd.select_by_index(inliners, invert=True)

        if visualize:
            inlier_cloud.paint_uniform_color([1., 0., 0.])
            cubes_cloud.paint_uniform_color([0.6, .6, .6])
            o3d.visualization.draw_geometries([cubes_cloud, inlier_cloud])

        self.combined_pcd = cubes_cloud

    def do_dbscan(self, visualize=False):
        labels = np.array(self.combined_pcd.cluster_dbscan(eps=0.005, min_points=5))
        max_labels = labels.max()

        segmented_cubes = list()
        cubes_points_ndarray = np.asarray(self.combined_pcd.points)
        for i in range(0, max_labels + 1):
            print("LABEL: ", i)
            indices_to_extract = np.where(labels == i)

            # Extract points based on indices
            segment = cubes_points_ndarray[indices_to_extract]

            # Create a new PointCloud from the selected points
            selected_cloud = o3d.geometry.PointCloud()
            selected_cloud.points = o3d.utility.Vector3dVector(segment)
            segmented_cubes.append(selected_cloud)

        if visualize:
            colors = plt.get_cmap("tab10")(labels / max_labels if max_labels > 0 else 1)
            colors[labels < 0] = 0
            self.combined_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
            o3d.visualization.draw_geometries([self.combined_pcd])

        return segmented_cubes

    def get_transformations(self, segmented_cubes, visualize=False):
        all_transformations = list()

        for segment in segmented_cubes:
            # for safety create cube each time
            cube_model = self.create_cube_model(0.046)
            cube_model_cloud = cube_model.sample_points_uniformly(number_of_points=6000)  # mesh to pcd

            initial_transformation = np.eye(4)
            initial_transformation[:3, 3] = segment.get_center()  # push near target cloud
            cube_model_cloud.transform(initial_transformation)

            threshold = 10
            result = self.do_icp(cube_model_cloud, segment, distance_threshold=threshold)
            transformation, _ = result
            final_transformation = np.dot(transformation, initial_transformation)

            if visualize:
                cube_model_cloud.translate(-np.array(cube_model_cloud.get_center()))  # back to origin
                cube_model_cloud.transform(final_transformation)
                cube_model_cloud.paint_uniform_color([1., 0., 0.])
                segment.paint_uniform_color([0.6, .6, .6])
                o3d.visualization.draw_geometries([segment, cube_model_cloud])

            all_transformations.append(final_transformation)

        return all_transformations

    def create_cube_model(self, side_length):
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

        # only use 3 sides of the cube (not full cube is generated)
        # this is due to fact that the cubes in the scanned
        # point cloud are not full and often are lacking sides
        cube_mesh.remove_vertices_by_index([1])

        # Translate the cube to center it at the origin
        # this is important for icp, as we want to get position and orientation relative to base
        cube_mesh.translate(-np.array(cube_mesh.get_center()))

        return cube_mesh

    def do_icp(self, cube_model_cloud, segmented_cloud, distance_threshold=10):
        cube_model_copy = copy.deepcopy(cube_model_cloud)

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
            # result = multi_start_icp(cube_model_cloud, segmented_cloud)[0] # mby needed mby not
        else:
            result = (icp_result.transformation, icp_result.fitness)

        return result


if __name__ == '__main__':
    pch = PCHandler()