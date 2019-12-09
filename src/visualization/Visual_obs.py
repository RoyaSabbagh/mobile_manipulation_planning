#!/usr/bin/env python

import roslib

import rospy
import tf
import math
from pam.msg import feedback
from tf.transformations import quaternion_from_euler


def callback(data):
        br = tf.TransformBroadcaster()
        br.sendTransform((data.bed_state.x, data.bed_state.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.bed_state.phi),
                         rospy.Time.now(),
                         "bed_base",
                         "frame_0")

        br.sendTransform((float(room_length)/2, 0, 1.5), tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "wall_1",
                         "frame_0")

        br.sendTransform((room_length, float(room_width)/2, 1.5), tf.transformations.quaternion_from_euler(0, 0, 1.57),
                         rospy.Time.now(),
                         "wall_2",
                         "frame_0")

        br.sendTransform((float(room_length)/2, room_width, 1.5), tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "wall_3",
                         "frame_0")

        br.sendTransform((data.blue_chair_state.x, data.blue_chair_state.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.blue_chair_state.phi),
                         rospy.Time.now(),
                         "base_link_bc",
                         "frame_0")

        br.sendTransform((data.gray_chair_state.x, data.gray_chair_state.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.gray_chair_state.phi),
                         rospy.Time.now(),
                         "base_link_gc",
                         "frame_0")

        br.sendTransform((data.cart_state.x, data.cart_state.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.cart_state.phi),
                         rospy.Time.now(),
                         "base_link_c",
                         "frame_0")
        if target_object == 'walker':
            br.sendTransform((data.goal.x, data.goal.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.goal.phi),
                         rospy.Time.now(),
                         "base_link_wg",
                         "frame_0")
        elif target_object == 'blue_chair':
            br.sendTransform((data.goal.x, data.goal.y, 0), tf.transformations.quaternion_from_euler(0, 0, data.goal.phi),
                         rospy.Time.now(),
                         "base_link_bcg",
                         "frame_0")

room_length = rospy.get_param("room_length")
room_width = rospy.get_param("room_width")
target_object = rospy.get_param("target_object")

if __name__ == '__main__':
        rospy.init_node('Visual_obs')
        rospy.Subscriber('feedback', feedback, callback, queue_size=1, buff_size=2**24)

        rospy.spin()
