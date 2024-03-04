#!/usr/bin/env python3.8
from pathlib import Path
import copy
import math

from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
import rospy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from irobm_control.srv import MoveTo, MoveToRequest
import tf.transformations as tf_trans
import pyzed.sl as sl
from dynamic_reconfigure.client import Client
        

from irobm_perception.srv import CubeCentroids, CubeCentroidsResponse
from irobm_control.srv import PickNPlace, PickNPlaceRequest, ArcPath, ArcPathRequest, Homing, HomingRequest

DATA_PATH = Path(__file__).parent.parent / "data"

TEST = False

class PCHandler():
    """
    has to be started with required="true"; this enshures when this task is done whole ros shuts down
    #object_type

    """

    def __init__(self, pc_topic):
        rospy.init_node('decision_maker')
        self.sub = rospy.Subscriber(pc_topic, PointCloud2, self.callback)
        self.service = rospy.ServiceProxy('/irobm_control/move_to', MoveTo)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.on_shutdown(self.shutdown_procedure)
        self.origin_joint_pose = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
        self.home_position = [.5, 0,  0.5, [3.1415, 0, -0.8], True]
        self.positions = [[.5, 0, 0.5, [3.1415, 0, -0.8], True],
                        [.5, -0.1, 0.5, [3.1415, 0, .8], True],
                        [.5, 0.1, 0.5, [3.1415, 0, -1.6], True],
            #[.5,.4, 0.5, [3.1415, -0.7, -math.pi/2 - 0.15], True],
                  #        [.5,-.4, .4, [3.1415, -0.4, 1], True],
                          
                          ]

        self.current_cloud = None
        self.transform_index = 0
        self.combined_pcd = None
        self.transformations = None
        self.save_signal = False
        
        self.move_to = rospy.Service('/irobm_perception/cube_centroids', CubeCentroids, self.point_cloud_handle)
        self.homing_client = rospy.ServiceProxy('/irobm_control/homing', Homing)

        self.reconfig_client = Client('zed2/zed_node', timeout=30)
        self.reconfigured_clien = False
        
        # initial setup
        #self.check_service_and_process_positions(visualize=True)
        # todo: need current positions of the robot to calculate the offset to the objects

    def shutdown_procedure(self):
        pass

    def callback(self, pc2_msg):
        # self.current_cloud = pc2_msg
        if not self.reconfigured_clien:
            self.reconfig_client.update_configuration({"auto_exposure_gain" : False})
            self.reconfig_client.update_configuration({"gain" : 1})
            self.reconfig_client.update_configuration({"exposure" : 50})
            self.reconfig_client.update_configuration({"brightness" : 6})
            self.reconfigured_clien = True

        if self.save_signal:
            try:
                # print("Frame id: ", pc2_msg.header.frame_id)
                transform_stamped = self.tf_buffer.lookup_transform("panda_link0", pc2_msg.header.frame_id, rospy.Time())
                self.current_cloud = do_transform_cloud(pc2_msg, transform_stamped)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Transform exception: %s", str(e))
            finally:
                self.save_signal = False

    def point_cloud_handle(self, req):
        self.check_service_and_process_positions(visualize=False)

        print("Trying to go to approximated positions")

        response = CubeCentroidsResponse()
        positions = list()
        rotations = list()

        for transformation in self.transformations:
            position = list(transformation[:3, 3])
            rotation_euler = tf_trans.euler_from_matrix(list(transformation[:3,:3]))
            
            positions.append(Point(*position))
            rotations.append(Point(*rotation_euler))

        print("Found positions: ")
        print(positions)
        print("Found orientations: ")
        print(rotations)

        response.position = positions
        response.orientation = rotations

        return response

    def check_service_and_process_positions(self, visualize=False):
        print("Waiting for service move to")
        rospy.wait_for_service('/irobm_control/move_to')
        print("Move to online")
        for position in self.positions:
            # todo undo
            self.move_to_position_and_wait(position)
            #exit()
            # print("Should be there")
            rospy.sleep(1)
            self.save_signal = True
            self.process_received_cloud()

        self.do_cloud_preproc(visualize)

    def move_to_position_and_wait(self, position):
        # Make sure to define the MoveTo service message format
        print("Moving to position: ", position)
        x, y, z, orientation, use_orientation = position
        try:
            # todo adjust
            point = Point(x, y, z)
            move_to_request = MoveToRequest()  # Modify with actual request format
            move_to_request.position = point  # Modify according to the actual service message fields
            move_to_request.orientation = orientation
            move_to_request.w_orient = use_orientation
            response = self.service(move_to_request)  # this is blocking operation
            return response
            # Wait for the service to complete
            # Implement any specific waiting logic here if required
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def process_received_cloud(self):
        while not self.current_cloud:
            rospy.sleep(.1)

        self.save_cloud(self.current_cloud)
        self.current_cloud = None

    def save_cloud(self, pc2_msg, test=TEST):
        print("Saving cloud")
        points = point_cloud2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert points to a numpy array
        points_array = np.array(list(points))
        if test:
            print("Len of array: ", len(points_array))
            print("Before filtering")
            test = o3d.geometry.PointCloud()
            test.points = o3d.utility.Vector3dVector(points_array)
            coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
            o3d.visualization.draw_geometries([test, coord_axes], zoom=0.3,
                                              front=[-1, 0, 0],
                                              lookat=[0, 1, 0],
                                              up=[0., 0, 1])
        # todo check weather this all is necessary
        mask1 = np.where(points_array[:, 0] > 0.2, True, False)
        mask2 = np.where(points_array[:, 0] < 0.82, True, False)
        mask3 = np.where(points_array[:, 1] < 0.4, True, False)
        mask4 = np.where(points_array[:, 1] > -0.4, True, False)
        mask5 = np.where(points_array[:, 2] > 0.02, True, False)
        mask6 = np.where(points_array[:, 2] < 0.15, True, False)
        mask = np.logical_and(mask6, 
                                np.logical_and(mask5, 
                                                np.logical_and(mask4, 
                                                np.logical_and(mask3,
                                                np.logical_and(mask1, mask2)))))
        points_array = points_array[mask]

        ###TEST
        if test:
            print("After filtering")
            test = o3d.geometry.PointCloud()
            test.points = o3d.utility.Vector3dVector(points_array)
            o3d.visualization.draw_geometries([test, coord_axes], zoom=0.2,
                                              front=[-1, 0, 0],
                                              lookat=[0, 1, 0],
                                              up=[0., 0, 1])
            # todo check weather this all is necessary
            ###END
        file_path = str(DATA_PATH / f'point_cloud_transformed{self.transform_index}.npy')
        self.transform_index += 1

        print("Its good writing")
        # write
        np.save(file_path, points_array.astype(np.float16))  # save float16 => smaller memory footprint
        return points_array

    def do_cloud_preproc(self, visualize=False):
        print("Combined clouds")
        self.combined_pcd = self.combine_pointclouds(
            visualise=True)  # visualize only for debugging otherwise its blocking

        self.combined_pcd.paint_uniform_color([0.6, .6, .6])

        print("Removing outliers")
        # remove outliers, i.e., do filtering
        self.remove_outliers(visualize, nb_neighbors=10, std_ratio=1)

        # estimate the normals
        print("estimate the normals")
        self.estimate_normals(visualize)

        # RANSAC Planar Segmentation, i.e., remove the desk
        #print("RANSAC")
        #self.run_RANSAC_plane(visualize)

        #print("Removing outliers")
        # remove outliers, i.e., do filtering
        #self.remove_outliers(visualize, nb_neighbors=50, std_ratio=3)

        # db scan rest of the cloud, i.e., segment the cubes
        print("DB SCAN")
        segmented_cubes = self.do_dbscan(True)
        print(f"Found {len(segmented_cubes)} cubes")

        self.transformations = self.get_transformations(segmented_cubes, visualize)

    def combine_pointclouds(self,
                            voxel_size=.001,
                            visualise=False):  # voxel_size = 0.001  implies averaging points in radius of 1mm (millimeter)
        pcds = list()
        nr_loaded_clouds = 0  # if there is problem with loading one of the pcds we dont want index out of bounds
        # load the data
        coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        for i in range(self.transform_index):
            try:
                pc = np.load(str(DATA_PATH / f'point_cloud_transformed{i}.npy'))
                pcds.append(o3d.geometry.PointCloud())
                pcds[nr_loaded_clouds].points = o3d.utility.Vector3dVector(pc)
                """o3d.visualization.draw_geometries([pcds[nr_loaded_clouds], coord_axes], zoom=0.3412,
                                  front=[-1, 0, 0],
                                  lookat=[0, 1, 0],
                                  up=[0., 0, 1])"""
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

    def remove_outliers(self, visualize=False, nb_neighbors=10, std_ratio=5):
        """
            remove outliers, i.e., do filtering
        """
        filtered_pcd = self.combined_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        final_cloud = filtered_pcd[0]
        if visualize:
            outliers = self.combined_pcd.select_by_index(filtered_pcd[1], invert=True)
            outliers.paint_uniform_color([1, 0, 0])
            o3d.visualization.draw_geometries([final_cloud, outliers])

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
        pt_to_plane_dist = .013
        plane_model, inliners = self.combined_pcd.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=5,
                                                                num_iterations=100)
        inlier_cloud = self.combined_pcd.select_by_index(inliners)
        cubes_cloud = self.combined_pcd.select_by_index(inliners, invert=True)

        if visualize:
            inlier_cloud.paint_uniform_color([1., 0., 0.])
            cubes_cloud.paint_uniform_color([0.6, .6, .6])
            o3d.visualization.draw_geometries([cubes_cloud, inlier_cloud])

        self.combined_pcd = cubes_cloud

    def do_dbscan(self, visualize=False):
        # todo remove small clouds
        labels = np.array(self.combined_pcd.cluster_dbscan(eps=0.005, min_points=5))
        max_labels = labels.max()

        segmented_cubes = list()
        cubes_points_ndarray = np.asarray(self.combined_pcd.points)
        for i in range(0, max_labels + 1):
            indices_to_extract = np.where(labels == i)
            print("LABEL: ", i, "NR points: ", len(indices_to_extract[0]))
            if len(indices_to_extract[0]) < 1000:
                # cant be cube must be outliers
                continue
            # Extract points based on indices
            segment = cubes_points_ndarray[indices_to_extract]

            # Create a new PointCloud from the selected points
            selected_cloud = o3d.geometry.PointCloud()
            selected_cloud.points = o3d.utility.Vector3dVector(segment)
            segmented_cubes.append(selected_cloud)

        max_labels = len(segmented_cubes)
        if visualize:
            colors = plt.get_cmap("tab10")(labels / max_labels if max_labels > 0 else 1)
            colors[labels < 0] = 0
            self.combined_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
            o3d.visualization.draw_geometries([self.combined_pcd])

        return segmented_cubes

    def get_transformations(self, segmented_cubes, visualize=False):
        all_transformations = list()
        coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

        for segment in segmented_cubes:
            # for safety create cube each time
            cube_model = self.create_cube_model(0.04)
            cube_model_cloud = cube_model.sample_points_uniformly(number_of_points=4000)  # mesh to pcd
            cube_np_array = np.asarray(cube_model_cloud.points)
            mask = np.where(cube_np_array[:, 1] < 0.015, True, False)
            cube_array = cube_np_array[mask]
            cube_model_cloud = o3d.geometry.PointCloud()
            cube_model_cloud.points = o3d.utility.Vector3dVector(cube_array)


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
                coord_axes2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1,
                                                                                origin=final_transformation[:3, 3])
                o3d.visualization.draw_geometries([coord_axes2, segment, cube_model_cloud, coord_axes])

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
    try:
        simulation_topic = "/zed2/point_cloud/cloud_registered"
        real_robot_topic = "/zed2/zed_node/point_cloud/cloud_registered"
        pch = PCHandler(real_robot_topic)
        # dm.create_voronoi()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
