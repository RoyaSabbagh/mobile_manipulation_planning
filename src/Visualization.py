#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import copy

class Visual(object):

        def __init__(self):
                self.planned_path_object_pub = rospy.Publisher("/planned_path_object", Path, queue_size=1)
                self.planned_path_PAM_pub = rospy.Publisher("/planned_path_PAM", Path, queue_size=1)
                self.actual_path_object_pub = rospy.Publisher("/actual_path_object", Path, queue_size=1)
                self.actual_path_PAM_pub = rospy.Publisher("/actual_path_PAM", Path, queue_size=1)
                self.predicted_path_PAM_pub = rospy.Publisher("/predicted_path_PAM", Path, queue_size=1)
                self.PAM_goal_marker_Pub = rospy.Publisher('PAMGoalMarker', Marker, queue_size=1)
                self.object_goal_marker_Pub = rospy.Publisher('objectGoalMarker', Marker, queue_size=1)
                self.desired_force_marker_Pub = rospy.Publisher('DesiredForceMarker', Marker, queue_size=1)


        def Set_Arrow_Marker(self, force, color):
                P = Marker()
                P.header.frame_id = "frame_0"
                P.header.stamp    = rospy.get_rostime()
                P.ns = "robot"
                P.id = 0
                P.type = 0 # arrow
                P.action = 0
                P.pose.position.x = force.pose.x
                P.pose.position.y = force.pose.y
                P.pose.position.z = 0.5
                quaternion = quaternion_from_euler(0, 0 ,force.pose.phi)
                P.pose.orientation.x = quaternion[0]
                P.pose.orientation.y = quaternion[1]
                P.pose.orientation.z = quaternion[2]
                P.pose.orientation.w = quaternion[3]
                P.scale.x = force.magnitude/float(1)
                P.scale.y = 0.02
                P.scale.z = 0.02

                P.color.r = color[0]
                P.color.g = color[1]
                P.color.b = color[2]
                P.color.a = 1.0

                P.lifetime =  rospy.Duration(0)
                return P

        def Set_Marker(self, point, color):

                state = Point()
                state.x = point.x
                state.y = point.y
                state.z = 0.1

                P = Marker()
                P.header.frame_id = "frame_0"
                P.header.stamp    = rospy.get_rostime()
                P.ns = "robot"
                P.id = 0
                P.type = 2 # point
                P.action = 0
		P.pose.position.x = point.x
                P.pose.position.y = point.y
                P.pose.position.z = 0.1
                P.pose.orientation.w = 1.0
                P.scale.x = 0.1
                P.scale.y = 0.1
                P.scale.z = 0.1

                P.color.r = color[0]
                P.color.g = color[1]
                P.color.b = color[2]
                P.color.a = 1.0

                P.lifetime =  rospy.Duration(0)
                return P


        def convert_path(self, path):
                viz_path = Path()
                viz_path.header.frame_id = 'frame_0'
		if len(path) > 0:
			for i in range(len(path)):
			        loc = Pose()
			        loc.position.x = path[i][0]
			        loc.position.y = path[i][1]
			        loc.position.z = 0
			        pose = PoseStamped()
			        pose.pose = loc
			        viz_path.poses.append(pose)
                return viz_path

        def plot(self, force, roomba_goal, object_goal, PAM_path_predicted, object_path_actual, object_path_planned, PAM_path_actual, PAM_path_planned):

		self.predicted_path_PAM_pub.publish(PAM_path_predicted)
                self.actual_path_object_pub.publish(object_path_actual)
                self.planned_path_object_pub.publish(object_path_planned)
		self.actual_path_PAM_pub.publish(PAM_path_actual)
                self.planned_path_PAM_pub.publish(PAM_path_planned)

                PAM_goal_marker = self.Set_Marker(roomba_goal, [0, 1, 0])
                self.PAM_goal_marker_Pub.publish(PAM_goal_marker)

                object_goal_marker = self.Set_Marker(object_goal, [0, 0, 1])
                self.object_goal_marker_Pub.publish(object_goal_marker)

		desired_force = self.Set_Arrow_Marker(force, [1, 0, 0])
                self.desired_force_marker_Pub.publish(desired_force)
