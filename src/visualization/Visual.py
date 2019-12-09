#!/usr/bin/env python

import roslib

import rospy
import tf
import math
from pam.msg import Roomba_cmd_vel, Gripper_mode, controlInput, feedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState


def callback(data):

    pub = rospy.Publisher('/joint_states1', JointState, queue_size=10)
    pose=JointState()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'frame_0'
    pose.name =  ["robot_x", "robot_y", "robot_phi"]

    pose.position = len(pose.name) * [0.0]

    pose.position[0]=data.PAM_state.x
    pose.position[1]=data.PAM_state.y
    pose.position[2]=data.PAM_state.phi

    pub.publish(pose)


if __name__ == '__main__':
        rospy.init_node('Visual')
        rospy.Subscriber('feedback', feedback, callback, queue_size=1, buff_size=2**24)
        rospy.spin()
