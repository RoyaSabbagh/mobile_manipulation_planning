import math
import numpy as np
from numpy.linalg import inv
from LQR import State
from pam.msg import feedback
import rospy
'Simulation for dynamics of both the object and the robot based on the learned dynamics'

class simulation():

	def __init__(self, env):
		self.dt = 0.2
		if env.manipulation_object == 'walker':
			self.walker_state = env.object_state_initial
			self.blue_chair_state = env.blue_chair_state
			self.gray_chair_state = env.gray_chair_state
			self.cart_state = env.cart_state
		elif env.manipulation_object == 'blue_chair':
			self.blue_chair_state = env.object_state_initial
			self.walker_state = env.walker_state
			self.gray_chair_state = env.gray_chair_state
			self.cart_state = env.cart_state
		elif env.manipulation_object == 'gray_chair':
			self.gray_chair_state = env.object_state_initial
			self.walker_state = env.walker_state
			self.blue_chair_state = env.blue_chair_state
			self.cart_state = env.cart_state
		elif env.manipulation_object == 'cart':
			self.cart_state = env.object_state_initial
			self.blue_chair_state = env.blue_chair_state
			self.gray_chair_state = env.gray_chair_state
			self.walker_state = env.walker_state
		self.PAM_state = env.robot_state_initial
		self.bed_state = env.bed_state
		self.r_gripper = env.r_gripper

	def update_object(self, target_object, params, control_input, leg, leg_num):
		'''Updating object state based on the control input + noise'''

		noise = np.random.normal(0,1,len(params))
		max_noise = [0.1, 0.1, 0.01, 0.01, 0.5, 0.5, 0.01, 0.01, 0.01]
		params = [params[i] + noise[i]*max_noise[i] for i in range(len(params))]
		gripper_force = control_input[2]
		Rho = params[0:4]
		Omega = params[4:7]
		R = [[np.cos(target_object.phi), -np.sin(target_object.phi)], [np.sin(target_object.phi), np.cos(target_object.phi)]]
		R2 = [[np.cos(Omega[2]), -np.sin(Omega[2])], [np.sin(Omega[2]), np.cos(Omega[2])]]
		V_wheel = np.linalg.multi_dot([inv(R2),inv(R),[target_object.xdot, target_object.ydot]+ np.linalg.multi_dot([[[0, -target_object.phidot],[target_object.phidot, 0]], R, [Rho[2],Rho[3]]])])
		F_wheel = -np.dot(R,np.dot(R2,np.dot([[Omega[0], 0], [0, Omega[1]]], V_wheel)))
		r_F = [leg[0]-target_object.x, leg[1]-target_object.y]
		r_U = [r_F[0] - Rho[2], r_F[1] - Rho[3]]
		tau = np.cross(r_U, gripper_force)+ np.cross([-Rho[2], -Rho[3]], F_wheel)
		F_T = F_wheel+gripper_force
		target_object.phidot = target_object.phidot + tau/Rho[1]*self.dt
		target_object.xdot = target_object.xdot + F_T[0]/Rho[0]*self.dt
		target_object.ydot = target_object.ydot + F_T[1]/Rho[0]*self.dt

		gripper_pose = [self.PAM_state.x+self.r_gripper*np.cos(self.PAM_state.phi), self.PAM_state.y+self.r_gripper*np.sin(self.PAM_state.phi)]
		target_object.phi = target_object.phi + self.dt*target_object.phidot
		R = [[np.cos(target_object.phi), -np.sin(target_object.phi)], [np.sin(target_object.phi), np.cos(target_object.phi)]]
		r = [-(np.sign(leg_num/2)*2-1)*params[8]/2, (np.sign(leg_num% 3)*2-1)*params[7]/2]
		target_object.x, target_object.y = gripper_pose + np.dot(R,r)
		return target_object

	def update_PAM(self, control_input):
		'''Updating robot's state based on the control input'''

		if control_input[1] == 0:
			self.PAM_state.x = self.PAM_state.x + control_input[0] * np.cos(self.PAM_state.phi) * self.dt
			self.PAM_state.y = self.PAM_state.y + control_input[0] * np.sin(self.PAM_state.phi) * self.dt
		elif control_input[1] == 1:
			self.PAM_state.phi = self.PAM_state.phi + 0.5 * self.dt
		elif control_input[1] == -1:
			self.PAM_state.phi = self.PAM_state.phi - 0.5 * self.dt
		elif control_input[0] < 0:
			r = control_input[0]/control_input[1]
			dtheta = -control_input[1]*self.dt
			R = [[np.cos(dtheta), -np.sin(dtheta)], [np.sin(dtheta), np.cos(dtheta)]]
			ICC = [self.PAM_state.x + r*np.sin(self.PAM_state.phi), self.PAM_state.y - r*np.cos(self.PAM_state.phi)]
			diff = [self.PAM_state.x - ICC[0], self.PAM_state.y - ICC[1]]
			r_gripper =0.26
			r_g = np.sqrt(r**2+r_gripper**2)
			self.PAM_state.x, self.PAM_state.y = ICC + np.dot(R,diff)
			self.PAM_state.phi = self.PAM_state.phi - control_input[1] * self.dt
		elif control_input[0] == 0:
				self.PAM_state.phi = self.PAM_state.phi + control_input[1] * self.dt
		else:
			r = control_input[0]/control_input[1]
			self.PAM_state.x = self.PAM_state.x + r * (np.sin(self.PAM_state.phi + control_input[1]*self.dt) - np.sin(self.PAM_state.phi))
			self.PAM_state.y = self.PAM_state.y - r * (np.cos(self.PAM_state.phi + control_input[1]*self.dt) - np.cos(self.PAM_state.phi))
			self.PAM_state.phi = self.PAM_state.phi + control_input[1] * self.dt

		if self.PAM_state.phi>np.pi:
			self.PAM_state.phi = self.PAM_state.phi - 2*np.pi
		elif self.PAM_state.phi< -np.pi:
			self.PAM_state.phi = self.PAM_state.phi + 2*np.pi

	def feedback(self, target_object, params, control_input, mode, legs, current_leg, goal, manipulation_object):
		fb = feedback()
		self.update_PAM(control_input)
		if mode == 3:
			target_object_new = self.update_object(target_object, params, control_input, legs[current_leg], current_leg)
			if manipulation_object == 'walker':
				fb.walker_state = target_object_new
			elif manipulation_object == 'blue_chair':
				fb.blue_chair_state = target_object_new
			elif manipulation_object == 'gray_chair':
				fb.gray_chair_state = target_object_new
			elif manipulation_object == 'cart':
				fb.cart_state = target_object_new
		fb.PAM_state = self.PAM_state
		fb.walker_state = self.walker_state
		fb.blue_chair_state = self.blue_chair_state
		fb.bed_state = self.bed_state
		fb.gray_chair_state = self.gray_chair_state
		fb.cart_state = self.cart_state
		fb.goal = goal
		return (fb)
