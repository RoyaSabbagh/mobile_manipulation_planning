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
        # br = tf.TransformBroadcaster()
        # rate = rospy.Rate(10.0)
        # br.sendTransform((data.walker_state.x, data.walker_state.y, 0.0), tf.transformations.quaternion_from_euler(0, 0, data.walker_state.phi),
        #                  rospy.Time.now(),
        #                  "base_link_w",
        #                  "frame_0")
        # rate.sleep()

    pub = rospy.Publisher('/joint_states2', JointState, queue_size=10)
    pose=JointState()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'frame_0'
    pose.name =  ["walker_x", "walker_y", "walker_phi"]

    pose.position = len(pose.name) * [0.0]

    pose.position[0]=data.walker_state.x
    pose.position[1]=data.walker_state.y
    pose.position[2]=data.walker_state.phi

    pub.publish(pose)

if __name__ == '__main__':

        rospy.init_node('Visual_walker')
        rospy.Subscriber('feedback', feedback, callback, queue_size=1, buff_size=2**24)
        rospy.spin()
