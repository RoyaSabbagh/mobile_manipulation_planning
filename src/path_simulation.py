#!/usr/bin/env python

import rospy
import copy
import numpy as np
from numpy.linalg import inv
'Fining number of repositioning actions needed for a manipulation plan.'

def traj_simulation(PAM_state, object_state, desired_force, legs, leg, command):
	r_gripper =0.26
	dt = 0.2
	epsilon, epsilon_phi = 0.02, 0.02
	r_grasp= 0.4
	phi_limit = [0.7, 4.3]
	manipulation_object = "walker"

	# Walker parameters : Rho(4), Omega(3), width(1), length(1)   #Rho[0,1,2,3]=[mass, inertia, x_c, y_c]
	walker_params = [8.25, 4.31,  0.14, -0.02, 0.97, 0.84, -0.03, 0.56, 0.36]
	walker_r = np.sqrt(walker_params[7]**2 +walker_params[8]**2)/2
	blue_chair_params = [8.25, 4.31,  0.14, -0.02, 0.97, 0.84, -0.03, 0.54, 0.46]
	blue_chair_r = np.sqrt(blue_chair_params[7]**2 +blue_chair_params[8]**2)

	if manipulation_object == "walker":
		object_params = walker_params
	elif manipulation_object == "blue_chair":
		object_params = blue_chair_params

	transitions = 0

        for i in range(1, len(command)):
		phi_close = np.pi + PAM_state.phi - object_state.phi - leg*np.pi/2
		if phi_close < 0:
			phi_close += 2*np.pi
		if phi_close >= 2*np.pi:
			phi_close -= 2*np.pi
		if (phi_close >= phi_limit[1] or phi_close <= phi_limit[0]):
			transitions += 1
			phi_0 = np.arctan2(desired_force[i][1],desired_force[i][0])
			phi_force =  -np.pi/2 + phi_0 - object_state.phi + leg*np.pi/2
			if phi_force <= phi_limit[1] or phi_force >= phi_limit[0]:
				push_pull = 1
			else:
				push_pull = 0
			PAM_state.x, PAM_state.y, PAM_state.phi = [r_grasp * np.cos(phi_0) * np.sign(push_pull * 2 - 1) + legs[leg][0], r_grasp * np.sin(phi_0) * np.sign(push_pull * 2 - 1) + legs[leg][1], np.pi * push_pull + phi_0]

		else:
			# Finding robot commands in local frame because it is less confusing
			vel = command[i]
			phi_vel= np.arctan2(vel[1],vel[0])-PAM_state.phi
			if phi_vel>= -np.pi/2 and phi_vel< np.pi/2:
				vel_sign = 1
				if phi_vel >=0:
					omega_sign = 1
				else:
					omega_sign = -1
			else:
				vel_sign = -1
				if phi_vel >=np.pi/2:
					omega_sign = -1
				else:
					omega_sign = 1
			gripper_pose = [PAM_state.x+r_gripper*np.cos(PAM_state.phi), PAM_state.y+r_gripper*np.sin(PAM_state.phi)]
			phi_ICC = np.abs(np.pi/2 - np.abs(phi_vel))
			middle_point = [gripper_pose[0]+vel[0]*dt/2, gripper_pose[1]+vel[1]*dt/2]
			m1 = -1/(vel[1]/vel[0])
			b1 = middle_point[1]-m1*middle_point[0]
			m2 = -1/np.tan(PAM_state.phi)
			b2 = PAM_state.y-m2*PAM_state.x
			ICC = [(b2-b1)/(m1-m2), (m1*b2-m2*b1)/(m1-m2)]
			r_R = np.sqrt((PAM_state.x- ICC[0])**2 + (PAM_state.y- ICC[1])**2)
			theta_1 = np.arctan2(gripper_pose[1]-ICC[1], gripper_pose[0]-ICC[0])
			theta_2 = np.arctan2(gripper_pose[1]+vel[1]*dt-ICC[1], gripper_pose[0]+vel[0]*dt-ICC[0])
			omega_R = np.abs(theta_2-theta_1)/dt
			vel_R = omega_R *r_R
			desired = [vel_sign*vel_R, omega_sign*omega_R, vel]

			PAM_state = update_PAM(PAM_state, dt, desired)
			object_state = update_object(object_state, PAM_state, object_params, r_gripper, dt, desired, legs[leg], leg)
	return transitions


def update_object(object_state, PAM_state, params, r_gripper, dt, control_input, leg, leg_num):

	gripper_vel = control_input[2]
	Rho = params[0:4]
	Omega = params[4:7]
	R = [[np.cos(object_state.phi), -np.sin(object_state.phi)], [np.sin(object_state.phi), np.cos(object_state.phi)]]
	R2 = [[np.cos(Omega[2]), -np.sin(Omega[2])], [np.sin(Omega[2]), np.cos(Omega[2])]]
	V_wheel = np.linalg.multi_dot([inv(R2),inv(R),[object_state.xdot, object_state.ydot]+ np.linalg.multi_dot([[[0, -object_state.phidot],[object_state.phidot, 0]], R, [Rho[2],Rho[3]]])])
	F_wheel = -np.dot(R,np.dot(R2,np.dot([[Omega[0], 0], [0, Omega[1]]], V_wheel)))
	r_F = [leg[0]-object_state.x, leg[1]-object_state.y]
	r_U = [r_F[0] - Rho[2], r_F[1] - Rho[3]]
	tau_wheel = np.cross(r_U, F_wheel) * dt
	tau = Rho[0]*(r_U[0]*(gripper_vel[1]-object_state.ydot)-r_U[1]*(gripper_vel[0]-object_state.xdot)) + tau_wheel
	object_state.phidot = object_state.phidot + tau/Rho[1]
	object_state.xdot = gripper_vel[0] - r_U[0]*object_state.phidot
	object_state.ydot = gripper_vel[1] - r_U[1]*object_state.phidot

	gripper_pose = [PAM_state.x+r_gripper*np.cos(PAM_state.phi), PAM_state.y+r_gripper*np.sin(PAM_state.phi)]
	object_state.phi = object_state.phi + dt*object_state.phidot
	R = [[np.cos(object_state.phi), -np.sin(object_state.phi)], [np.sin(object_state.phi), np.cos(object_state.phi)]]
	r = [-(np.sign(leg_num/2)*2-1)*params[8]/2, (np.sign(leg_num% 3)*2-1)*params[7]/2]
	object_state.x, object_state.y = gripper_pose + np.dot(R,r)
	return object_state


def update_PAM(PAM_state, dt, control_input):
	if control_input[1] == 0:
		PAM_state.x = PAM_state.x + control_input[0] * np.cos(PAM_state.phi) * dt
		PAM_state.y = PAM_state.y + control_input[0] * np.sin(PAM_state.phi) * dt
	elif control_input[1] == 1:

		PAM_state.phi = PAM_state.phi + 0.1 * dt
	elif control_input[1] == -1:
		PAM_state.phi = PAM_state.phi - 0.1 * dt
	elif control_input[0] < 0:
		r = control_input[0]/control_input[1]
		dtheta = -control_input[1]*dt
		R = [[np.cos(dtheta), -np.sin(dtheta)], [np.sin(dtheta), np.cos(dtheta)]]
		ICC = [PAM_state.x + r*np.sin(PAM_state.phi), PAM_state.y - r*np.cos(PAM_state.phi)]
		diff = [PAM_state.x - ICC[0], PAM_state.y - ICC[1]]
		r_gripper =0.26
		r_g = np.sqrt(r**2+r_gripper**2)

		PAM_state.x, PAM_state.y = ICC + np.dot(R,diff)
		PAM_state.phi = PAM_state.phi - control_input[1] * dt
	else:
		r = control_input[0]/control_input[1]
		PAM_state.x = PAM_state.x + r * (np.sin(PAM_state.phi + control_input[1]*dt) - np.sin(PAM_state.phi))
		PAM_state.y = PAM_state.y - r * (np.cos(PAM_state.phi + control_input[1]*dt) - np.cos(PAM_state.phi))
		PAM_state.phi = PAM_state.phi + control_input[1] * dt
	if PAM_state.phi>np.pi:
		PAM_state.phi = PAM_state.phi - 2*np.pi
	elif PAM_state.phi< -np.pi:
		PAM_state.phi = PAM_state.phi + 2*np.pi
	return PAM_state
