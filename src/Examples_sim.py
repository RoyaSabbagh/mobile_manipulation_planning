#!/usr/bin/env python

import rospy
'Setting the simulation experiments examples '

class sim_environment():

	def __init__(self):

		self.robot_state_initial = state()
		self.object_state_initial = state()
		self.bed_state = state()
		self.walker_state = state()
		self.gray_chair_state = state()
		self.blue_chair_state = state()
		self.cart_state = state()
		self.PAM_goal = state()
		self.object_goal = state()
		self.walls = [-0.5, 4, -0.5, 4.5]
		self.manipulation_object = "walker"
		self.epsilon, self.epsilon_phi = 0.01, 0.005
		self.dt = 0.2
		self.phi_limit = [0.7, 4]
		self.v_max = [2, 10]
		self.force_max = 100
		self.n, self.n_p, self.n_g = 25, 20, 20
		self.PAM_r = 0.22  # radius of PAM
		self.r_gripper =0.26 # gripper radius
		self.r_grasp= 0.55 # min distance for grasp initiation
		room_length = rospy.get_param("room_length")
		room_width = rospy.get_param("room_width")
		self.walls = [0, room_length, 0, room_width]

	def example_1(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "walker"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,5.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2, 4, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [4.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 4, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.5, 2.5, 0.2, 0, 0, 0]

	def example_2(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "walker"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5.5,0.5, 3, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [1.6, 0.8, 0.3, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [3.2, 0.8, -1.5, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 0.7, 0.5, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.8, 2.2, -1.2, 0, 0, 0]

	def example_3(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "walker"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5,4.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2.2,  3.6, 1, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [30, 1, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [2.5, 0.8, 1.57, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [4.5, 2, 3.1, 0, 0, 0]

	def example_4(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "walker"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,0.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [3.5, 2, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [4.5, 3, 0.6, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 5, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [1, 3.2, 1.5, 0, 0, 0]

	def example_5(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "blue_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,5.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2, 4, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [4.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 4, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.5, 2.5, 0.2, 0, 0, 0]

	def example_6(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "blue_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5.5,0.5, 0.1, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [1.6, 0.8, 0.3, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [3.2, 0.8, -1.5, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 0.7, 0.5, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.8, 2.2, -1.2, 0, 0, 0]

	def example_7(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "blue_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5,4.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2.2,  3.6, 1, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [30, 1, -1.57, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [2.5, 0.8, 1.57, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [4.5, 2, 3.1, 0, 0, 0]

	def example_8(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "blue_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,0.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [3.5, 2, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [4.5, 3, 0.6, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 5, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [1, 3.2, 1.5, 0, 0, 0]

	def example_9(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "gray_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,5.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2, 4, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [4.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 4, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.5, 2.5, 0.2, 0, 0, 0]

	def example_10(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "gray_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5.5,0.5, 0.1, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [1.6, 0.8, 0.3, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [3.2, 0.8, -1.5, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 0.7, 0.5, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.8, 2.2, -1.2, 0, 0, 0]

	def example_11(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "gray_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5,4.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2.2,  3.6, 1, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [30, 1, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [2.5, 0.8, 1.57, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [4.5, 2, 3.1, 0, 0, 0]

	def example_12(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "gray_chair"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,0.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [3.5, 2, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [4.5, 3, 0.6, 0.2, 0, 0]
		self.cart_state.x, self.cart_state.y, self.cart_state.phi, self.cart_state.xdot, self.cart_state.ydot, self.cart_state.phidot = [4.5, 5, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [1, 3.2, 1.5, 0, 0, 0]

	def example_13(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "cart"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,5.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2, 4, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [4.7, 1.5, 0.01, 0.2, 0, 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [4.5, 4, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.5, 2.5, 0.2, 0, 0, 0]

	def example_14(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "cart"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5.5,0.5, 0.1, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [1.6, 0.8, 0.3, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [3.2, 0.8, -1.5, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2., 0.01, 0.2, 0, 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [4.5, 0.7, 0.5, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [3.8, 2.2, -1.2, 0, 0, 0]

	def example_15(self):
		# room_length = 6
		# room_width = 5
		self.manipulation_object = "cart"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [5,4.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [2.2,  3.6, 1, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [4.5, 3.5, 1.57, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [30, 1, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5.5, 2, 0.01, 0.2, 0, 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [2.5, 0.8, 1.57, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [4.5, 2, 3.1, 0, 0, 0]

	def example_16(self):
		# room_length = 5
		# room_width = 6
		self.manipulation_object = "cart"
		self.robot_state_initial.x, self.robot_state_initial.y, self.robot_state_initial.phi, self.robot_state_initial.xdot, self.robot_state_initial.ydot, self.robot_state_initial.phidot = [4.5,0.5, 3.2, 0, 0, 0]
		self.object_state_initial.x, self.object_state_initial.y, self.object_state_initial.phi, self.object_state_initial.xdot, self.object_state_initial.ydot, self.object_state_initial.phidot = [3.5, 2, 0.7, 0, 0, 0]
		self.bed_state.x, self.bed_state.y, self.bed_state.phi, self.bed_state.xdot, self.bed_state.ydot, self.bed_state.phidot = [2, 1.5, 0.001, 0, 0 , 0]
		self.gray_chair_state.x, self.gray_chair_state.y, self.gray_chair_state.phi, self.gray_chair_state.xdot, self.gray_chair_state.ydot, self.gray_chair_state.phidot = [1, 0.5, -1.57, 0, 0 , 0]
		self.blue_chair_state.x, self.blue_chair_state.y, self.blue_chair_state.phi, self.blue_chair_state.xdot, self.blue_chair_state.ydot, self.blue_chair_state.phidot = [5, 3.5, 0.6, 0.2, 0, 0]
		self.walker_state.x, self.walker_state.y, self.walker_state.phi, self.walker_state.xdot, self.walker_state.ydot, self.walker_state.phidot = [4.5, 5, 0.001, 0, 0 , 0]
		self.object_goal.x, self.object_goal.y, self.object_goal.phi, self.object_goal.xdot, self.object_goal.ydot, self.object_goal.phidot = [1, 3.2, 1.5, 0, 0, 0]
