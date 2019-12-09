#!/usr/bin/env python

import rospy
'Setting the physical experiments examples '

class environment():

	def __init__(self):
		# Enviromental parameteres
		self.robot_state_initial = state()
		self.object_state_initial = state()
		self.bed_state = state()
		self.gray_chair_state = state()
		self.blue_chair_state = state()
		self.cart_state = state()
		self.PAM_goal = state()
		self.object_goal = state()
		self.walls = [0.5, 4, -0.5, 4.5]
		self.epsilon, self.epsilon_phi = 0.01, 0.005
		self.dt = 0.2
		self.phi_limit = [1, 4.5]
		self.v_max = [0.1, 0.1]
		self.force_max = 1
		self.n, self.n_p, self.n_g = 10, 15, 15
		self.PAM_r = 0.22  # radius of PAM
		self.r_gripper =0.26 # gripper radius
		self.r_grasp= 0.4 # min distance for grasp initiation
		room_length = rospy.get_param("room_length")
		room_width = rospy.get_param("room_width")
		self.walls = [0, room_length, 0, room_width]

	def example_1(self):
		self.manipulation_object = "walker"
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.5, 2, 0.2, 0, 0, 0]

	def example_2(self):
		self.manipulation_object = "walker"
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [2.7, 3.5, 1.5, 0, 0, 0]

	def example_3(self):
		self.manipulation_object = "walker"
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3, 2.6, 1.5, 0, 0, 0]
