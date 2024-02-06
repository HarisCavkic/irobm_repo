#!/usr/bin/env python3.8
import tf2_ros
import rospy
import yaml
import numpy as np


def get_T_matrix(rotation, translation, quaternion):
    """
    rotation: numpy.ndarray of size (4,1) representing rotation in quaternions if quaternion==True otherwise (3,1) RPY
    translate: numpy.ndarray of size (3,1) representing translation
    quaternion: bools representing weather rotation is expressed in quaternions
    """
    if quaternion:
        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        R = np.array([[1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
                      [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qx * qw],
                      [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx ** 2 - 2 * qy ** 2]])
    else:
        # Convert RPY to rotation matrix
        roll, pitch, yaw = rotation.flatten()
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = T[:3, 3] = translation

    return T

import numpy as np

def get_rot_and_translation(T):
    """
    T: numpy.ndarray of size (4x4) representing homogeneous transformation matrix
    return: rotation in quaternions and translation vector
    """
    # Extract translation vector
    translation = T[0:3, 3]

    # Extract rotation matrix
    R = T[0:3, 0:3]

    # Convert rotation matrix to quaternion
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    quaternion = np.array([qw, qx, qy, qz])

    return quaternion, translation

def do():
    rospy.init_node("Test")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        transform_stamped = tf_buffer.lookup_transform('zed2_left_camera_frame', "map",
                                                       rospy.Time(), rospy.Duration(1.0))

        rotation = transform_stamped.transform.rotation
        translation = transform_stamped.transform.translation

        # Print the transformation matrix
        print("Transform matrix:\n", transform_stamped.transform)
        print("Rotation:\n", rotation)
        print("Translation:\n", translation)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform exception: %s", str(e))
        frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
        print(list(frames_dict.keys()))
        cloud_out = None

    z2_T_map = get_T_matrix(rotation, np.array([translation.x, translation.y, translation.z]), True)

    ph_rot_z2 = np.array([-0.097397, 0, 0.0274111, ])
    ph_t_z2 = np.array([0, -0.824473, 0])

    ph_T_z2 = get_T_matrix(ph_rot_z2, ph_t_z2, False)


    """map_R_z2 = np.array([[0.0387373, 0, 0],
                         [0, 0.9083464, 0],
                         [0, 0, -0.0316367]])
    map_t_z2 = np.array([-0.0191342, 0.119998, 0.0312461])

    z2_R_map = map_R_z2.T
    z2_t_map = -map_R_z2 @ map_t_z2
    z2_T_map = np.eye(4)
    z2_T_map[:3, :3] = z2_R_map
    z2_T_map[:3, 3] = z2_t_map
    print("Rot: ", z2_R_map)
    print("Transl: ", z2_t_map)
    print("T: ", z2_T_map)

    ph_R_z2 = np.array([[0, 0, 0],
                        [0, 0.824473, 0],
                        [0, 0, 0]])
    ph_t_z2 = np.array([-0.097397, 0, 0.0274111])

    ph_T_z2 = np.eye(4)
    ph_T_z2[:3, :3] = ph_R_z2
    ph_T_z2[:3, 3] = ph_t_z2"""

    ph_T_map = ph_T_z2 @ z2_T_map

    print("T: ", ph_T_map)

    quatern, trans = get_rot_and_translation(ph_T_map)
    print("Rot: ", quatern)
    print("Transl: ", trans)

if __name__ == '__main__':
    try:

        do()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
