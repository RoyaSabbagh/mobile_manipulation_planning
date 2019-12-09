#!/usr/bin/env python

import rospy
import copy
import numpy as np
from numpy.linalg import inv
import csv
from pam.msg import controlInput, feedback, state, force
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from Optimization import OptTraj_LQR, nominal_traj_LQR, OptPath
from utility import dynamicModelRoomba, define_obs, Find_phi_quarter, FindLeg, detectChange, find_corners, path_from_command
from Visualization import Visual
import rosbag
from Manipulation_Simulation import simulation
from path_simulation import traj_simulation
from Examples_sim import sim_environment
from LQR import lqr_control
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
'Main script to perform high level control of the system'


def Optimizer(r_grasp,PAM_r, PAM_s, object_s, object_f, object_params, phi, r_max, walls, obstacles, obstacles_PAM, current_leg, n, n_p, v_max, force_max, legs, dt):
    """ This function finds the best for both the target object and the robot. Here, we assign 4 different type of costs. (1) cost of changing legs if needed. (2) cost of robot motion to the starting state of the planned path. (3) cost of object manipulation, which is based on object's path. (4) cost of predicted re-positionings needed to execute the manipulation plan."""
    global action_push_pull, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned
    # assigning cost of changing from one leg to another based on the distance to the desired pose
    cost_ChangeLeg = 1
    dz_final = np.sqrt((object_s.x - object_f.x) ** 2 + (object_s.y - object_f.y) ** 2)
    if dz_final < 1:
        cost_ChangeLeg = 5
    elif dz_final < 2:
        cost_ChangeLeg = 10
    else:
        cost_ChangeLeg = 5

    # assigning weight for cost of predicted repositioning and cost of robot motion
    w_cost_reposition = 40
    w_cost_motion = 10

    # finding object's leg cordinates
    object_leg = find_corners(object_s.x, object_s.y, object_s.phi, object_params[7], object_params[8])

    # initialization (initializeing cost to infinity)
    cost = [float('inf'), float('inf'), float('inf'), float('inf')]
    cost_legchange = [0, 0, 0, 0]
    cost_PAM = [[0, 0],[0, 0],[0, 0],[0, 0]]
    cost_manipulation = [0, 0, 0, 0]
    cost_motion = [0, 0, 0, 0]
    force = [0, 0, 0, 0]
    path = [[[], []], [[], []], [[], []], [[], []]]
    planned_path_w = [[],[],[],[]]
    PAM_g = [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]]
    command = [[], [], [], []]
    des = [[], [], [], [], []]
    PAM_goal = state()

    # find the nominal trajectory for manipulation
    theta = nominal_traj_LQR([object_s.x,object_s.y,object_s.phi], [object_f.x,object_f.y,object_f.phi], v_max, walls, obstacles, n, dt)

    # itterate through each leg to find the leg with minimum cost
    for leg in range(4):
        phi_linear = theta
        psi_linear = [theta[k] + phi[leg] for k in range(len(theta))]
    	# find the cost and required force for manipulation for the leg
        force[leg], cost_manipulation[leg], planned_path_w[leg], command[leg], des[leg]= OptTraj_LQR([object_s.x, object_s.y, object_s.phi, object_s.xdot, object_s.ydot, object_s.phidot], [object_f.x, object_f.y, object_f.phi, object_f.xdot, object_f.ydot, object_f.phidot], v_max, walls, obstacles, object_params[0:4], object_params[4:7], phi_linear, psi_linear, force_max, r_max[leg], n, dt, object_leg[leg])
    	# adding cost of changing leg
        if leg != current_leg:
    		cost_legchange[leg] = cost_ChangeLeg
    		# adding cost of PAM motion to PAM goal pose
    		phi0 = np.arctan2(object_leg[leg][1]-object_s.y,object_leg[leg][0]-object_s.x)
    		# finding the better option between pulling and pushing for each leg, with the same manipulation plan
    		for push_pull in [0,1]:
				PAM_g[leg][push_pull] = [r_grasp * np.cos(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][0], r_grasp * np.sin(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][1], np.pi * push_pull + phi0]
				cost_PAM[leg][push_pull], path[leg][push_pull], command_pam, goal_orientation = OptPath([PAM_s.x, PAM_s.y, PAM_s.phi], PAM_g[leg][push_pull], walls, obstacles_PAM, n_p, dt)
				if cost_PAM[leg][push_pull]!= float("inf"):
					PAM_s_sim = copy.deepcopy(PAM_s)
					PAM_s_sim.x, PAM_s_sim.y, PAM_s_sim.phi = [PAM_r * np.cos(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][0], PAM_r * np.sin(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][1], np.pi * push_pull + phi0]
					# adding cost of predicted re-positionings
					n_transition = traj_simulation(copy.deepcopy(PAM_s_sim), copy.deepcopy(object_s), force[leg], legs, leg, command[leg])
					# print(n_transition)
					cost_PAM[leg][push_pull] += w_cost_reposition*n_transition
    		cost_motion[leg] += min(cost_PAM[leg])*w_cost_motion
    		action_push_pull[leg] = np.argmin(cost_PAM[leg])
    	else:
            phi0 = np.arctan2(force[leg][0][1], force[leg][0][0])
            for push_pull in [0,1]:
                PAM_g[leg][push_pull] = [r_grasp * np.cos(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][0], r_grasp * np.sin(phi0) * np.sign(push_pull * 2 - 1) + object_leg[leg][1], np.pi * push_pull + phi0]
    cost = [cost_legchange[leg] + cost_motion[leg] + cost_manipulation[leg] for leg in range(4)]

    if min(cost) < float("inf"):
    	[min_index, min_value] = [np.argmin(cost), min(cost)]
    	# Finding the grasping goal pose based on the selected plan
    	phi0 = np.arctan2(object_leg[min_index][1]-object_s.y,object_leg[min_index][0]-object_s.x)
    	grasping_goal = [PAM_r * np.cos(phi0) * np.sign(action_push_pull[min_index] * 2 - 1) + object_leg[min_index][0], PAM_r * np.sin(phi0) * np.sign(action_push_pull[min_index] * 2 - 1) + object_leg[min_index][1], np.pi * action_push_pull[min_index] + phi0]
    	PAM_goal = state()
    	PAM_goal.x, PAM_goal.y, PAM_goal.phi = PAM_g[min_index][action_push_pull[min_index]]
    	object_path_planned = Path()
    	object_path_planned.header.frame_id = 'frame_0'
    	for i in range(len(planned_path_w[min_index])):
    		pose = PoseStamped()
    		pose.pose.position.x = planned_path_w[min_index][i][0]
    		pose.pose.position.y = planned_path_w[min_index][i][1]
    		pose.pose.position.z = 0
    		object_path_planned.poses.append(pose)

    	PAM_path_planned = Path()
    	PAM_path_planned.header.frame_id = 'frame_0'
    	if min_index != current_leg:
    		for i in range(len(path[min_index][action_push_pull[min_index]])):
    			pose = PoseStamped()
    			pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z = path[min_index][action_push_pull[min_index]][i]
    			PAM_path_planned.poses.append(pose)
    else:
    	min_index = 5
    	min_value = float("inf")
    if 0 < min_index and min_index < 4:
       force_d = force[min_index][0]
    else:
       force_d = [0,0,0]

    return  cost ,min_index, force_d, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned, des[min_index]

def run_simulation(system):
    """ This is the main function that runs the simulation. Based on the state of the robot and target object and the goal state for the object, it determines which mode it is and finds the proper command in each mode as follows: (0) stay put. (1) robot motion to the manipulation pre-grasp state. (2) robot motion to grasp the object. (3) manipulation of the object. (4) re-position. (5) success."""
    r = rospy.Rate(5)
    global  time_run_start, target_object, total_error_position, total_error_orientation, total_time, env, object_params, object_params_sim, obss, r_max, phi, PAM_path_actual, object_path_actual, sys_mode, vis, bed_state, old_object_state, grasping_goal, PAM_goal, object_path_planned, found_solution, object_r, PAM_path_planned, desired_force, boolean, best_leg,  transition_mode, transition, blue_chair_state, des, ind

    pose_aux_p = PoseStamped()
    pose_aux_p.pose.position.x, pose_aux_p.pose.position.y, pose_aux_p.pose.position.z = [system.PAM_state.x, system.PAM_state.y, 0]
    PAM_path_actual.poses.append(pose_aux_p)

    gripper_pose = [system.PAM_state.x+env.r_gripper*np.cos(system.PAM_state.phi), system.PAM_state.y+env.r_gripper*np.sin(system.PAM_state.phi)]  # calculates the gripper pose from PAM pose
    phi_quarter = Find_phi_quarter(system.PAM_state.phi)  #defines in which quarter is the PAM orientation

    newObs = [env.bed_state.x, env.bed_state.y, env.bed_state.phi, 2, 1]  #adds the bed as obstacle for the target object
    obss = [newObs]
    newObs = [env.cart_state.x, env.cart_state.y, env.cart_state.phi, 1, 0.3]
    obss.append(newObs)
    newObs = [env.gray_chair_state.x, env.gray_chair_state.y, env.gray_chair_state.phi, 0.4, 0.4]
    obss.append(newObs)

    if env.manipulation_object == "walker":
        target_object = system.walker_state
        obss.append([system.blue_chair_state.x, system.blue_chair_state.y, system.blue_chair_state.phi, 0.6, 0.55])  #adds the blue_chair as an obstacle for target object
    elif env.manipulation_object == "blue_chair":
        target_object = system.blue_chair_state
        obss.append([system.walker_state.x, system.walker_state.y, system.walker_state.phi, 0.54, 0.36])  #adds the walker as an obstacle for target object
    elif env.manipulation_object == "gray_chair":
		target_object = system.gray_chair_state
		obss.append([system.blue_chair_state.x, system.blue_chair_state.y, system.blue_chair_state.phi, 0.6, 0.55])  #adds the blue_chair as an obstacle for target object
    elif env.manipulation_object == "cart":
		target_object = system.cart_state
		obss.append([system.blue_chair_state.x, system.blue_chair_state.y, system.blue_chair_state.phi, 0.6, 0.55])  #adds the walker as an obstacle for target object

    obss_PAM = copy.copy(obss)
    obss_PAM.append([target_object.x, target_object.y, target_object.phi, 1.001, 0.3])  #adds the target object as an obstacle for PAM
    legs, current_leg = FindLeg(gripper_pose, target_object, object_params)   #return the grasped leg
    obstacles = define_obs(obss, object_r+env.PAM_r)  #final obstacles for the object
    obstacles_PAM = define_obs(obss_PAM, env.PAM_r)   #final obstacles for PAM

    input_d = controlInput()  #input to the low-level controller
    vel = [0, 0]
    command = []
    change = detectChange(target_object, old_object_state)  # checks if the environment (currently only including the bed and walker position) has been changed

    if (target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2 <= env.epsilon and (target_object.phi-env.object_goal.phi)**2 <= env.epsilon_phi*10:
    	sys_mode = 5
    	desired = [0, 0]
    	total_error_position = np.sqrt((target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2)
    	total_error_orientation = np.sqrt((target_object.phi-env.object_goal.phi)**2)*180/np.pi
    	rospy.loginfo("Success :)")
    	rospy.loginfo("total_time")
    	rospy.loginfo(total_time)
    	rospy.loginfo("error:")
    	rospy.loginfo(total_error_position)
    	rospy.loginfo(total_error_orientation)

    elif transition:
    	rospy.loginfo("transition_mode:")
    	rospy.loginfo(transition_mode)
    	if transition_mode == 0: #move backward from current leg
    		target_object.phidot = 0
    		target_object.xdot = 0
    		target_object.ydot = 0
    		if ((legs[best_leg][0]-gripper_pose[0])*(legs[best_leg][0]-gripper_pose[0])+(legs[best_leg][1]-gripper_pose[1])*(legs[best_leg][1]-gripper_pose[1])) >= 0.15:
    			transition_mode = 1
    			command = [[0, 0, 1]]
    		else:
    			command = [[-0.04, 0, 1]]
    		desired = [command[0][0], command[0][1]]
    	elif transition_mode == 1: # move towards the new goal pose
    		if (system.PAM_state.x-PAM_goal.x)**2+ (system.PAM_state.y-PAM_goal.y)**2 >= 0.002:
    			cost_PAM, path, command, goal_orientation = OptPath([system.PAM_state.x, system.PAM_state.y, system.PAM_state.phi], [PAM_goal.x, PAM_goal.y, PAM_goal.phi], env.walls, obstacles_PAM, env.n_p, env.dt)
    			if command != []:
    				PAM_path_planned = Path()
    				PAM_path_planned.header.frame_id = 'frame_0'
    				for i in range(len(path)):
    					pose = PoseStamped()
    					pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z = path[i]
    					PAM_path_planned.poses.append(pose)

    				if len(command) == 0:
    					desired = [0, 0]
    				else:
    					dz_final = np.sqrt((PAM_path_planned.poses[-1].pose.position.x-PAM_goal.x)**2+ (PAM_path_planned.poses[-1].pose.position.y-PAM_goal.y)**2)
    					if dz_final < 0.05 and (PAM_path_planned.poses[-1].pose.orientation.z-PAM_goal.phi)**2 < 0.01 and env.n_p > 3:
    						env.n_p -= 1  #recude the horizon when it is close enough to the goal. this removes the lower velocity effect of being close to the goal with high n_p
    					elif env.n_p <= 20:
    						env.n_p += 1 # increase n_p if it moves far from the goal and due to low n_p it cannot plan well
    					if command[0][0]**2 <= 0.01 and command[0][1]**2 <= 0.01:
    						desired = [command[1][0], command[1][1]]
    					else:
    						desired = [command[0][0], command[0][1]] #v and w velocity of gripper
    			else:
    				boolean = boolean +1
    				rospy.loginfo(sys_mode+0.5)
    				if boolean < 3:
    					rospy.loginfo("goal_orientation:")
    					rospy.loginfo(goal_orientation)
    					rospy.loginfo("system.PAM_state.phi:")
    					rospy.loginfo(system.PAM_state.phi)
    					if goal_orientation - system.PAM_state.phi > 0:
    						desired = [0.0001, 1]
    					else:
    						desired = [0.0001, -1]
    				else:
    					desired = [0, 0]
    					boolean = 1
    		else:
    			transition_mode = 2
    			desired = [0, 0]

    	elif transition_mode == 2: #turn towards the leg
    		phi_grasp = np.arctan2(legs[best_leg][1] - system.PAM_state.y, legs[best_leg][0] - system.PAM_state.x)
    		if (phi_grasp - system.PAM_state.phi)*(phi_grasp - system.PAM_state.phi) >= 0.005:
    			boolean = boolean +1
    			if boolean <3 :
    				if PAM_goal.phi - system.PAM_state.phi > 0:
    					desired = [0.0001, 1]
    				else:
    					desired = [0.0001, -1]
    			else:
    				desired = [0, 0]
    				boolean = 1
    		else:
    			transition_mode = 3
    			desired = [0, 0]
    	else: #re-grasp
    		if current_leg == best_leg:
    			transition = False
    			transition_mode = 0
    			desired = [0, 0]
    		else:
    			command =  [[0.04, 0,1]]
    			desired = [command[0][0], command[0][1]]
    else:
        if (found_solution == 0) and not transition:
            rospy.loginfo("change")

            pose_aux = PoseStamped()
            pose_aux.pose.position.x, pose_aux.pose.position.y, pose_aux.pose.position.z= [target_object.x, target_object.y, 0]
            object_path_actual.poses.append(pose_aux)

            env.n_p = 20
            min_value, best_leg, force, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned, des = Optimizer(env.r_grasp, env.PAM_r, system.PAM_state, target_object, env.object_goal, object_params, phi, r_max, env.walls, obstacles, obstacles_PAM, current_leg, env.n, env.n_p, env.v_max, env.force_max, legs, env.dt) #runs the whole optimization again both for object and PAM to find the best manipulation plan
            ind = 0
            if min(min_value) == float("inf"):  #inf means the infeasible optimization
                found_solution = 0  #runs until it finds a soluion
                command = [[0, 0, 1]]
                desired = [command[0][0], command[0][1]]
            else:
                found_solution = 1
                desired_force.pose.x, desired_force.pose.y, desired_force.pose.phi, desired_force.magnitude = legs[best_leg][0], legs[best_leg][1], np.arctan2(force[1],force[0]), np.sqrt(force[0]**2+force[1]**2)
                desired = [0, 0]
        else:
            if current_leg == best_leg: #(system.PAM_state.x-grasping_goal[0])**2+(system.PAM_state.y-grasping_goal[1])**2 <= 0.01: #and (system.PAM_state.phi-grasping_goal[2])**2 <= 0.01:

            	phi_close = np.pi + system.PAM_state.phi - target_object.phi - best_leg*np.pi/2
            	if phi_close < 0:
            		phi_close += 2*np.pi
            	if phi_close >= 2*np.pi:
            		phi_close -= 2*np.pi
            	if (phi_close >= env.phi_limit[1] or phi_close <= env.phi_limit[0]):
            		sys_mode = 4
            		transition = True
            		phi_0 = np.arctan2(legs[best_leg][1]-target_object.y,legs[best_leg][0]-target_object.x)#desired_force.pose.phi
            		phi_force =  -np.pi/2 + phi_0 - target_object.phi + best_leg*np.pi/2
            		if phi_force <= env.phi_limit[1] or phi_force >= env.phi_limit[0]:
            			push_pull = 1
            		else:
            			push_pull = 0
            		PAM_goal = state()

            		PAM_goal.x, PAM_goal.y, PAM_goal.phi = [env.r_grasp * np.cos(phi_0) * np.sign(push_pull * 2 - 1) + legs[best_leg][0], env.r_grasp * np.sin(phi_0) * np.sign(push_pull * 2 - 1) + legs[best_leg][1], np.pi * push_pull + phi_0]
            		command = [[0, 0, 1]]
            		desired = [command[0][0], command[0][1]]
            	else:
            		sys_mode = 3
            		print "ind"
            		print ind
            		if ind >= env.n-1:
        		          sys_mode = 6
        		          desired = [0, 0, [0,0]]
            		else:

                		u_old = [target_object.xdot, target_object.ydot , target_object.phidot]
                		u = lqr_control(target_object, des, phi[current_leg], ind)
                		a = [(u[i] - u_old[i])/env.dt for i in range(3)]
                		F = [a[i]*object_params[0] for i in range(2)]
                		Rho = object_params[0:4]
                		Omega = object_params[4:7]
                		R = [[np.cos(target_object.phi), -np.sin(target_object.phi)], [np.sin(target_object.phi), np.cos(target_object.phi)]]
                		R2 = [[np.cos(Omega[2]), -np.sin(Omega[2])], [np.sin(Omega[2]), np.cos(Omega[2])]]
                		V_wheel = np.linalg.multi_dot([inv(R2),inv(R),[target_object.xdot, target_object.ydot]+ np.linalg.multi_dot([[[0, -target_object.phidot],[target_object.phidot, 0]], R, [Rho[2],Rho[3]]])])
                		F_wheel = -np.dot(R,np.dot(R2,np.dot([[Omega[0], 0], [0, Omega[1]]], V_wheel)))
                		F_des = [F[i]-F_wheel[i] for i in range(2)]

                		ind += 1

                		vel = [u[0]+u[2]*r_max[current_leg]*np.cos(phi[current_leg]+target_object.phi),u[1]+u[2]*r_max[current_leg]*np.sin(phi[current_leg]+target_object.phi)]

                		if vel == [0,0]:
                			command = [[0, 0, 1]]
                		else:
                			phi_vel= np.arctan2(vel[1],vel[0])-system.PAM_state.phi
                			if phi_vel < -np.pi:
                				phi_vel += 2*np.pi
                			if phi_vel >= np.pi:
                				phi_vel -= 2*np.pi
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
                			phi_ICC = np.abs(np.pi/2 - np.abs(phi_vel))
                			middle_point = [gripper_pose[0]+vel[0]*env.dt/2, gripper_pose[1]+vel[1]*env.dt/2]
                			m1 = -1/(vel[1]/vel[0])
                			b1 = middle_point[1]-m1*middle_point[0]
                			m2 = -1/np.tan(system.PAM_state.phi)
                			b2 = system.PAM_state.y-m2*system.PAM_state.x
                			ICC = [(b2-b1)/(m1-m2), (m1*b2-m2*b1)/(m1-m2)]
                			r_R = np.sqrt((system.PAM_state.x- ICC[0])**2 + (system.PAM_state.y- ICC[1])**2)
                			theta_1 = np.arctan2(gripper_pose[1]-ICC[1], gripper_pose[0]-ICC[0])
                			theta_2 = np.arctan2(gripper_pose[1]+vel[1]*env.dt-ICC[1], gripper_pose[0]+vel[0]*env.dt-ICC[0])
                			omega_R = np.abs(theta_2-theta_1)/env.dt
                			vel_R = omega_R *r_R
                			command = [[vel_sign*vel_R, omega_sign*omega_R, 1]]
                		desired = [command[0][0], command[0][1], F_des]
                		object_path_planned = Path()
                		object_path_planned.header.frame_id = 'frame_0'

            elif (system.PAM_state.x-grasping_goal[0])**2+(system.PAM_state.y-grasping_goal[1])**2 <= env.r_grasp**2+env.epsilon and (system.PAM_state.phi-grasping_goal[2])**2 <= env.epsilon_phi:
                sys_mode = 2
                command =  [[0.04, 0, 1]]
                desired = [command[0][0], command[0][1]]

            elif (system.PAM_state.x-PAM_goal.x)**2+(system.PAM_state.y-PAM_goal.y)**2 <= env.epsilon and (system.PAM_state.phi-PAM_goal.phi)**2 >= env.epsilon_phi:
                PAM_goal.phi = np.arctan2(legs[best_leg][1] - system.PAM_state.y, legs[best_leg][0] - system.PAM_state.x)
                grasping_goal[2] = np.arctan2(legs[best_leg][1] - system.PAM_state.y, legs[best_leg][0] - system.PAM_state.x)
                boolean = boolean +1
                sys_mode = 1
                rospy.loginfo(sys_mode+0.5)
                if boolean < 3:
                	if PAM_goal.phi - system.PAM_state.phi > 0:
                		desired = [0.00001, 1]
                	else:
                		desired = [0.00001, -1]
                else:
                	desired = [0, 0]
                	boolean = 1
            else:
                target_object.phidot = 0
                target_object.xdot = 0
                target_object.ydot = 0
                sys_mode = 1
                cost_PAM, path, command, goal_orientation = OptPath([system.PAM_state.x, system.PAM_state.y, system.PAM_state.phi], [PAM_goal.x, PAM_goal.y, PAM_goal.phi], env.walls, obstacles_PAM, env.n_p, env.dt)
                PAM_path_planned = Path()
                PAM_path_planned.header.frame_id = 'frame_0'
                for i in range(len(path)):
                    pose = PoseStamped()
                    pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z = path[i]
                    PAM_path_planned.poses.append(pose)

                if len(command) == 0:
                    desired = [0, 0]
                else:
                    dz_final = np.sqrt((PAM_path_planned.poses[-1].pose.position.x-PAM_goal.x)**2+ (PAM_path_planned.poses[-1].pose.position.y-PAM_goal.y)**2)
                    if dz_final < 0.01 and (PAM_path_planned.poses[-1].pose.orientation.z-PAM_goal.phi)**2 < 0.01 and env.n_p > 3:
                    	env.n_p -= 1  #recude the horizon when it is close enough to the goal. this removes the lower velocity effect of being close to the goal with high n_p
                    elif env.n_p <= 20:
                        env.n_p += 1 # increase n_p if it moves far from the goal and due to low n_p it cannot plan well
                    desired = [command[0][0], command[0][1]] #v and w velocity of gripper
    rospy.loginfo("sys_mode")
    rospy.loginfo(sys_mode)
    old_object_state = copy.copy(target_object)
    fb = system.feedback(target_object, object_params_sim, desired, sys_mode, legs, current_leg, env.object_goal, env.manipulation_object)
    total_time += system.dt
    rospy.loginfo("time:")
    rospy.loginfo(total_time)
    rospy.loginfo("time_run:")
    rospy.loginfo(rospy.get_time()-time_run_start)
    rospy.loginfo("***************************************************************************")
    pub.publish(fb)
    PAM_path_predicted = path_from_command(command, copy.copy(system.PAM_state))

    input_d.desired_input, input_d.mode = vel, sys_mode
    desiredInput.publish(input_d)
    vis.plot(desired_force, PAM_goal, env.object_goal, PAM_path_predicted, object_path_actual, object_path_planned, PAM_path_actual, PAM_path_planned)
    r.sleep()

if __name__ == '__main__':

	rospy.init_node('high_level_controller', anonymous=True)
	desiredInput = rospy.Publisher('controlInput', controlInput, queue_size=10)
	pub = rospy.Publisher('feedback', feedback, queue_size=1)
	plan_pub_robot = rospy.Publisher('traj_robot', DisplayTrajectory, queue_size=1)
	plan_pub_walker = rospy.Publisher('traj_obj', DisplayTrajectory, queue_size=1)
	r = rospy.Rate(1)
	# joint_traj_robot = JointTrajectory()
	# joint_traj_robot.header.frame_id = 'frame_0'
	# joint_traj_robot.joint_names = ["robot_x", "robot_y", "robot_phi"]
	# joint_traj_point_robot = JointTrajectoryPoint()
	# display_traj_robot = DisplayTrajectory()
	# display_traj_robot.model_id = 'AGV'
	# robot_traj_robot = RobotTrajectory()
    #
	# joint_traj_walker = JointTrajectory()
	# joint_traj_walker.header.frame_id = 'frame_0'
	# joint_traj_walker.joint_names = ["walker_x", "walker_y", "walker_phi"]
	# joint_traj_point_walker = JointTrajectoryPoint()
	# display_traj_walker = DisplayTrajectory()
	# display_traj_walker.model_id = 'walker'
	# robot_traj_walker = RobotTrajectory()

	vis = Visual()
	desired_force = force()
	object_path_actual = Path()
	object_path_actual.header.frame_id = 'frame_0'
	PAM_path_actual = Path()
	PAM_path_actual.header.frame_id = 'frame_0'
	object_path_planned = Path()
	PAM_path_planned = Path()
	old_object_state = state()


	# object parameters : Rho(4), Omega(3), width(1), length(1)   #Rho[0,1,2,3]=[mass, inertia, x_c, y_c]
	# sample from distribution
	walker_mu = [1.18129219e+02, 7.23564360e+01, 9.11049246e-02, 7.23271543e-02, 7.71361628e+01, 7.67173557e+01, 2.12292195e-02, 0.56, 0.36]
	walker_std = [5.20518267e+01, 1.41632722e+01, 3.91568512e-02, 4.78442496e-02, 16.07199159, 16.17982517,  1.85912597, 0.0001, 0.0001]
	walker_r = np.sqrt(walker_mu[7]**2 +walker_mu[8]**2)/2
	blue_chair_mu = [ 143.91837244, 120.24236598,  -0.052589144,   0.050633468, 77.48830679, 76.84306259,  0.097583331, 0.52, 0.48]
	blue_chair_std = [45.39123028, 21.61617779,  0.029367533,  0.014292341, 22.50928369, 21.38863341,  1.74230422, 0.0001, 0.0001]
	blue_chair_r = np.sqrt(blue_chair_mu[7]**2 +blue_chair_mu[8]**2)/2
	gray_chair_mu = [ 1.13589231e+02, 1.68936430e+02, 5.88371344e-02, 9.50501828e-02, 27.28969998, 26.93801089, -0.03027382, 0.47, 0.4]
	gray_chair_std = [49.50201962, 23.63512571,  0.18768593,  0.17460398, 14.85526759, 13.80347269,  1.76543045, 0.0001, 0.0001]
	gray_chair_r = np.sqrt(gray_chair_mu[7]**2 +gray_chair_mu[8]**2)/2
	cart_mu = [ 154.13239509, 183.27723826,   0.66815968,   0.39554305, 22.18162462, 23.85949298,  0.12067901, 1.001, 0.3]
	cart_std = [37.63730445, 15.23335471,  0.1987102 ,  0.16746027, 14.74183501, 16.90726448,  1.79245946, 0.0001, 0.0001]
	cart_r = np.sqrt(cart_mu[7]**2 +cart_mu[8]**2)/2

	num_trial = 1

	try:
		# initialization
		time_total = 0
		success = 0
		error_total_position = 0
		error_total_orientation = 0
		with open('/home/amir/Roya/sim_LQR_experiment3_final.csv', mode='a') as file:
		          writer = csv.writer(file)
		          writer.writerow(["success_status", "total_time_sim", "total_time_run", "total_error_position", "total_error_orientation"])
		          for n_t in range(num_trial):
		                    env = sim_environment()
		                    env.example_3()
		                    system = simulation(env)
		                    # display_traj_robot.trajectory_start.joint_state.header.frame_id = 'frame_0'
		                    # display_traj_robot.trajectory_start.joint_state.name = ["robot_x", "robot_y", "robot_phi"]
		                    # display_traj_robot.trajectory_start.joint_state.position = [system.PAM_state.x, system.PAM_state.y,system.PAM_state.phi]
		                    # display_traj_robot.trajectory_start.joint_state.velocity = [0, 0, 0]
                            #
		                    # display_traj_walker.trajectory_start.joint_state.header.frame_id = 'frame_0'
		                    # display_traj_walker.trajectory_start.joint_state.name = ["walker_x", "walker_y", "walker_phi"]
		                    # display_traj_walker.trajectory_start.joint_state.position = [system.walker_state.x, system.walker_state.y,system.walker_state.phi]
		                    # display_traj_walker.trajectory_start.joint_state.velocity = [0, 0, 0]
		                    if env.manipulation_object == "walker":
		                              mu = walker_mu
		                              std = walker_std
		                              object_r = walker_r
		                    elif env.manipulation_object == "blue_chair":
		                              mu = blue_chair_mu
		                              std = blue_chair_std
		                              object_r = blue_chair_r
		                    elif env.manipulation_object == "gray_chair":
		                              mu = gray_chair_mu
		                              std = gray_chair_std
		                              object_r = gray_chair_r
		                    elif env.manipulation_object == "cart":
		                              mu = cart_mu
		                              std = cart_std
		                              object_r = cart_r

		                    object_params_sim = [np.random.normal(mu[i],std[i]) for i in range(len(mu))]
		                    object_params = mu

		                    phi = [np.arctan2(-object_params[8] + object_params[3], - object_params[7] + object_params[2]) + np.pi,
		                              np.arctan2(-object_params[8] + object_params[3], object_params[2]), np.arctan2(-object_params[3], object_params[2]) + np.pi,
		                              np.arctan2(object_params[3], object_params[7] - object_params[2]) + np.pi]
		                    r_max = [0,0,0,0]
		                    for i in range(4):
		                              r_max[i] = np.sqrt(((np.sign(i% 3)*2-1)*object_params[7]/2 - object_params[2])**2 + ((np.sign(i%2)*2-1)*object_params[8]/2 - object_params[3])**2)

		                    rospy.loginfo("****************next trial****************")
		                    rospy.loginfo(n_t)
		                    sys_mode = 0
		                    boolean = 1
		                    transition = False
		                    best_leg = 5
		                    transition_mode = 0
		                    action_push_pull = [0, 0, 0, 0]
		                    grasping_goal = [100, 100, 100]
		                    found_solution = 0
		                    des = []
		                    ind = 0
		                    time_run_start = rospy.get_time()
		                    total_time = 0
		                    total_error_position = 0
		                    total_error_orientation = 0
		                    for i in range(2500):
		                              run_simulation(system)
		                              if sys_mode == 5:
		                                        success_status = 1
		                                        # robot_traj_robot.joint_trajectory = joint_traj_robot
		                                        # display_traj_robot.trajectory = [robot_traj_robot]
		                                        # robot_traj_walker.joint_trajectory = joint_traj_walker
		                                        # display_traj_walker.trajectory = [robot_traj_walker]
		                                        # plan_pub_walker.publish(display_traj_walker)
		                                        # plan_pub_robot.publish(display_traj_robot)
		                                        break
		                              elif total_time>=180 or sys_mode == 6:
		                                        success_status = 0
		                                        total_error_position = np.sqrt((target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2)
		                                        total_error_orientation = np.sqrt((target_object.phi-env.object_goal.phi)**2)*180/np.pi
		                                        rospy.loginfo("Fail :(")
		                                        rospy.loginfo("total_time")
		                                        rospy.loginfo(total_time)
		                                        # robot_traj_robot.joint_trajectory = joint_traj_robot
		                                        # display_traj_robot.trajectory = [robot_traj_robot]
		                                        # robot_traj_walker.joint_trajectory = joint_traj_walker
		                                        # display_traj_walker.trajectory = [robot_traj_walker]
		                                        # plan_pub_walker.publish(display_traj_walker)
		                                        # plan_pub_robot.publish(display_traj_robot)
		                                        break
		                              if i <5:
		                                        r.sleep()
		                    total_time_run = rospy.get_time()- time_run_start
		                    writer.writerow([success_status, total_time, total_time_run, total_error_position, total_error_orientation])
		rospy.spin()
	finally:
		print "finish"
