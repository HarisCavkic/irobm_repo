#!/usr/bin/env python3.8
import tf2_ros
import rospy
import yaml
import numpy as np

def do():
    rospy.init_node("Test")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        transform_stamped = tf_buffer.lookup_transform("map", 'zed2_left_camera_frame',
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

    map_R_z2 = np.array([[0.0387373, 0, 0],
                        [0, 0.9083464,0],
                        [0, 0, -0.0316367]])
    map_t_z2 = np.array([-0.0191342, 0.119998, 0.0312461])

    z2_R_map = map_R_z2.T
    z2_t_map = -map_R_z2@map_t_z2
    z2_T_map = np.eye(4)
    z2_T_map[:3,:3] = z2_R_map
    z2_T_map[:3,3] = z2_t_map
    print("Rot: ", z2_R_map)
    print("Transl: ", z2_t_map)
    print("T: ", z2_T_map)


    ph_R_z2 = np.array([[0, 0, 0],
                        [0, 0.824473,0],
                        [0, 0, 0]])
    ph_t_z2 = np.array([-0.097397, 0, 0.0274111])

    ph_T_z2 = np.eye(4)
    ph_T_z2[:3,:3] = ph_R_z2
    ph_T_z2[:3,3] = ph_t_z2

    ph_T_map = ph_T_z2 @ z2_T_map

    print("T: ", ph_T_map)

if __name__ == '__main__':
    try:

        do()
        rospy.spin()
    except rospy.ROSInterruptException as exc:
        print("Something went wront")
        print(exc)
