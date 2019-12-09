#!/usr/bin/env python
import os
import rospy
import tf
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
import numpy as np
import copy
from tf.transformations import quaternion_matrix, quaternion_from_matrix, translation_matrix
from pam.msg import feedback
from tf import transformations as t


def read_aruco_transformation():

    with open('/home/roya/catkin_ws/src/pam_manipulation_planning/config/aruco_camera.yaml') as f:
        aruco = yaml.safe_load(f)

    kinect_T_aruco_rot =quaternion_matrix([aruco["kinect_orientation_x"], aruco["kinect_orientation_y"], aruco["kinect_orientation_z"], aruco["kinect_orientation_w"] ])
    kinect_T_aruco_trans= translation_matrix([aruco["kinect_position_x"], aruco["kinect_position_y"], aruco["kinect_position_z"]])
    transform = t.concatenate_matrices(kinect_T_aruco_trans, kinect_T_aruco_rot)

    return transform

def read_mocap_T_frame0():

    with open('/home/amir/catkin_ws/src/ergo-hri/ll4ma_posture_estimation/config/study_config_S19.yaml') as f:
        stool = yaml.safe_load(f)
    mocap_T_frame0=np.array([[-1, 0, 0, stool["seat_x"]+.37],
    [0, 0, 1, stool["seat_y"]-stool["seat_height"]],
    [0, 1, 0, stool["seat_z"]+.23],
    [0, 0, 0, 1]])
    return mocap_T_frame0

def calculate_transformation():
    kinect_T_topic= read_aruco_transformation()
    mocap_T_aruco=np.array([[0, 1, 0, 3.833],
                            [0, 0, 1, 0.421],
                            [1, 0, 0, -3.039],
                            [0, 0, 0, 1]])
    frame0_T_mocap=np.array([[1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])
    topic_T_aruco=np.array([[-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

    kinect_T_aruco = np.dot(kinect_T_topic, topic_T_aruco)
    aruco_T_kinect = t.inverse_matrix(kinect_T_aruco)
    frame0_T_aruco = np.dot(frame0_T_mocap, mocap_T_aruco)
    frame0_T_camera = np.dot(frame0_T_aruco, aruco_T_kinect)
    # mocap_T_frame0=read_mocap_T_frame0()
    # camera_T_frame0=np.dot(camera_T_frame0, mocap_T_frame0)

    frame0_T_camera=frame0_T_camera.tolist()
    quat_frame0_T_camera=quaternion_from_matrix(frame0_T_camera)
    quat_frame0_T_camera=quat_frame0_T_camera.tolist()
    return quat_frame0_T_camera, frame0_T_camera



def callback(data):
    parent_link = "frame_0"
    child_link  = "kinect2_rgb_optical_frame"
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    #
    quat_frame0_T_camera, frame0_T_camera = calculate_transformation()
    #
    #
    name = "{}__to__{}".format(parent_link, child_link)
    base_tf_stmp = TransformStamped()
    base_tf_stmp.header.stamp            = rospy.Time.now()
    base_tf_stmp.header.frame_id         = parent_link
    base_tf_stmp.child_frame_id          = child_link
    base_tf_stmp.transform.translation.x = frame0_T_camera[0][3]
    base_tf_stmp.transform.translation.y = frame0_T_camera[1][3]
    base_tf_stmp.transform.translation.z = frame0_T_camera[2][3]
    base_tf_stmp.transform.rotation.x    = quat_frame0_T_camera[0]
    base_tf_stmp.transform.rotation.y    = quat_frame0_T_camera[1]
    base_tf_stmp.transform.rotation.z    = quat_frame0_T_camera[2]
    base_tf_stmp.transform.rotation.w    = quat_frame0_T_camera[3]
    rospy.loginfo(base_tf_stmp)
    rospy.loginfo("Sending base transform...")
    broadcaster.sendTransform(base_tf_stmp)


if __name__ == '__main__':

    rospy.init_node("aruco_calibration_tf_publisher")


    rospy.Subscriber('feedback', feedback, callback, queue_size=1, buff_size=2**24)

    rospy.spin()
