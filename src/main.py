#!/usr/bin/env python

import rospy
import copy
import numpy as np
import csv
from pam.msg import controlInput, feedback, state, force
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from Optimization import OptTraj, nominal_traj, OptPath
from utility import dynamicModelRoomba, define_obs, Find_phi_quarter, FindLeg, detectChange, find_corners, path_from_command
from Visualization import Visual
import rosbag
from path_simulation import traj_simulation
from Examples_real import environment
'Main script to perform high level control of the system'


def Optimizer(r_grasp,PAM_r, PAM_s, object_s, object_f, object_params, phi, r_max, walls, obstacles, obstacles_PAM, current_leg, n, n_p, v_max, force_max, legs, dt):
    """ This function finds the best for both the target object and the robot. Here, we assign 4 different type of costs. (1) cost of changing legs if needed. (2) cost of robot motion to the starting state of the planned path. (3) cost of object manipulation, which is based on object's path. (4) cost of predicted re-positionings needed to execute the manipulation plan."""
    global action_push_pull, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned
    # assigning cost of changing from one leg to another based on the distance to the desired pose
    cost_ChangeLeg = 1
    dz_final = np.sqrt((object_s.x - object_f.x) ** 2 + (object_s.y - object_f.y) ** 2)
    if dz_final < 1:
        cost_ChangeLeg = 4000
    elif dz_final < 2:
        cost_ChangeLeg = 10000
    else:
        cost_ChangeLeg = 4000

    # assigning weight for cost of predicted repositioning and cost of robot motion
    w_cost_reposition = 300
    w_cost_motion = 100

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
    theta = nominal_traj([object_s.x,object_s.y,object_s.phi], [object_f.x,object_f.y,object_f.phi], v_max, walls, obstacles, n, dt)

    # itterate through each leg to find the leg with minimum cost
    for leg in range(4):
        phi_linear = theta
        psi_linear = [theta[k] + phi[leg] for k in range(len(theta))]
    	# find the cost and required force for manipulation for the leg
        force[leg], cost_manipulation[leg], planned_path_w[leg], command[leg], des= OptTraj([object_s.x, object_s.y, object_s.phi, object_s.xdot, object_s.ydot, object_s.phidot], [object_f.x, object_f.y, object_f.phi, object_f.xdot, object_f.ydot, object_f.phidot], v_max, walls, obstacles, object_params[0:4], object_params[4:7], phi_linear, psi_linear, force_max, r_max[leg], n, dt, object_leg[leg])
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
    			pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z =path[min_index][action_push_pull[min_index]][i]
    			PAM_path_planned.poses.append(pose)
    else:
    	min_index = 5
    	min_value = float("inf")
    if 0 < min_index and min_index <= 4:
       force_d = force[min_index][0]
    else:
       force_d = [0,0,0]

    return  cost ,min_index, force_d, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned

def callback(data):
    """ This is the main function that runs the experiment. Based on the state of the robot and target object and the goal state for the object, it determines which mode it is in and finds the proper command in each mode as follows: (0) stay put. (1) robot motion to the manipulation pre-grasp state. (2) robot motion to grasp the object. (3) manipulation of the object. (4) re-position. (5) success."""
    r = rospy.Rate(10)
    global near_pam_goal, time_run_start, target_object, total_error_position, total_error_orientation, total_time, env, first_3, object_params, obss, r_max, phi, PAM_path_actual, object_path_actual, sys_mode, initials, vis, old_bed_state, old_walker_state, old_blue_chair_state, grasping_goal, PAM_goal, object_path_planned, found_solution, object_r, vis_bag, PAM_path_planned, desired_force, boolean, best_leg, transition_mode, transition, manipulation_object
    if sys_mode!=5:
    	pose_aux_p = PoseStamped()
    	pose_aux_p.pose.position.x, pose_aux_p.pose.position.y, pose_aux_p.pose.position.z = [data.PAM_state.x, data.PAM_state.y, 0]
    	PAM_path_actual.poses.append(pose_aux_p)

    	gripper_pose = [data.PAM_state.x+env.r_gripper*np.cos(data.PAM_state.phi), data.PAM_state.y+env.r_gripper*np.sin(data.PAM_state.phi)]  # calculates the gripper pose from PAM pose
    	phi_quarter = Find_phi_quarter(data.PAM_state.phi)  #defines in which quarter is the PAM orientation

    	newObs = [data.bed_state.x, data.bed_state.y, data.bed_state.phi, 2, 1]  #adds the bed as obstacle for the target object
    	obss = [newObs]
    	newObs = [data.cart_state.x, data.cart_state.y, data.cart_state.phi, 1, 0.3]
    	obss.append(newObs)
    	newObs = [data.gray_chair_state.x, data.gray_chair_state.y, data.gray_chair_state.phi, 0.4, 0.4]
    	obss.append(newObs)

    	if env.manipulation_object == "walker":
    		target_object = data.walker_state
    		obss.append([data.blue_chair_state.x, data.blue_chair_state.y, data.blue_chair_state.phi, 0.6, 0.55])  #adds the blue_chair as an obstacle for target object
    	elif env.manipulation_object == "blue_chair":
    		target_object = data.blue_chair_state
    		obss.append([data.walker_state.x, data.walker_state.y, data.walker_state.phi, 0.6, 0.5])  #adds the walker as an obstacle for target object
    	obss_PAM = copy.copy(obss)
    	obss_PAM.append([target_object.x, target_object.y, target_object.phi, 0.6, 0.5])  #adds the target object as an obstacle for PAM
    	legs, current_leg = FindLeg(gripper_pose, target_object, object_params)   #return the grasped leg
    	obstacles = define_obs(obss, object_r+env.PAM_r)  #final obstacles for the object
    	obstacles_PAM = define_obs(obss_PAM, env.PAM_r)   #final obstacles for PAM

    	input_d = controlInput()  #input to the low-level controller
    	vel = [0, 0]
    	command = []
    	nominal_path_p = []
    	change = detectChange(data.walker_state, data.bed_state, data.blue_chair_state, old_blue_chair_state, old_bed_state, old_walker_state)  # checks if the environemtn (currently only including the bed and walker position) has been changed
    	if (target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2 <= 3*env.epsilon and (target_object.phi-env.object_goal.phi)**2 <= 2*env.epsilon_phi*10:
    		sys_mode = 5
    		desired = [0, 0]

    	elif transition:
    		rospy.loginfo("transition:")
    		rospy.loginfo(transition)
    		if transition_mode == 0: #move backward from current leg
    			if ((legs[best_leg][0]-gripper_pose[0])*(legs[best_leg][0]-gripper_pose[0])+(legs[best_leg][1]-gripper_pose[1])*(legs[best_leg][1]-gripper_pose[1])) >= 0.1:
    				transition_mode = 1
    				command = [[0, 0, 1]]
    			else:
    				command = [[-0.04, 0, 1]]
    			desired = [command[0][0], command[0][1]]
    		elif transition_mode == 1: #turn towards the new goal pose
    			found_solution = 0
    			transition = False
    			command = [[0, 0, 1]]
    			desired = [command[0][0], command[0][1]]
    			transition_mode = 0

    	else:
    		if (change or found_solution == 0) and not transition:
    			pose_aux = PoseStamped()
    			pose_aux.pose.position.x, pose_aux.pose.position.y, pose_aux.pose.position.z= [target_object.x, target_object.y, 0]
    			object_path_actual.poses.append(pose_aux)

    			env.n_p = 15
    			min_value, best_leg, force, PAM_goal, grasping_goal, object_path_planned, PAM_path_planned = Optimizer(env.r_grasp, env.PAM_r, data.PAM_state, target_object, env.object_goal, object_params, phi, r_max, env.walls, obstacles, obstacles_PAM, current_leg, env.n, env.n_p, env.v_max, env.force_max, legs, env.dt) #runs the whole optimization again both for object and PAM to find the best manipulation plan
    			if min(min_value) == float("inf"):  #inf means the infeasible optimization
    				found_solution = 0  #runs until it finds a soluion
    				command = [[0, 0, 1]]
    				desired = [command[0][0], command[0][1]]
    			else:
    				found_solution = 1
    				desired_force.pose.x, desired_force.pose.y, desired_force.pose.phi, desired_force.magnitude = legs[best_leg][0], legs[best_leg][1], np.arctan2(force[1],force[0]), np.sqrt(force[0]**2+force[1]**2)
    				desired = [0, 0]
    		if found_solution == 1 and not transition:
    		          if current_leg == best_leg: #(data.PAM_state.x-grasping_goal[0])**2+(data.PAM_state.y-grasping_goal[1])**2 <= 0.01: #and (data.PAM_state.phi-grasping_goal[2])**2 <= 0.01:
    		                    phi_close = np.pi + data.PAM_state.phi - target_object.phi - best_leg*np.pi/2
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
    		                              if first_3:
    		                                        desired = [0, 0]
    		                                        first_3 = False
    		                              else:
    		                                        phi_linear = nominal_traj([target_object.x, target_object.y, target_object.phi], [env.object_goal.x,env.object_goal.y,env.object_goal.phi], env.v_max, env.walls, obstacles, env.n, env.dt)
    		                                        psi_linear = [phi_linear[k] + phi[current_leg]-np.pi/2 for k in range(len(phi_linear))]
    		                                        force, cost, planned_path_w, vel, des= OptTraj([target_object.x, target_object.y, target_object.phi, target_object.xdot, target_object.ydot, target_object.phidot], [env.object_goal.x, env.object_goal.y, env.object_goal.phi, env.object_goal.xdot, env.object_goal.ydot, env.object_goal.phidot], env.v_max, env.walls, obstacles, object_params[0:4], object_params[4:7], phi_linear, psi_linear, env.force_max, r_max[current_leg], env.n, env.dt, gripper_pose)
    		                                        # Finding robot commands in local frame because it is less confusing
    		                                        phi_vel= np.arctan2(vel[1][1],vel[1][0])-data.PAM_state.phi
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
    		                                                            omega_sign = 1
    		                                                  else:
    		                                                            omega_sign = -1
    		                                        phi_ICC = np.abs(np.pi/2 - np.abs(phi_vel))
    		                                        middle_point = [gripper_pose[0]+vel[1][0]*env.dt/2, gripper_pose[1]+vel[1][1]*env.dt/2]
    		                                        m1 = -1/(vel[1][1]/vel[1][0])
    		                                        b1 = middle_point[1]-m1*middle_point[0]
    		                                        m2 = -1/np.tan(data.PAM_state.phi)
    		                                        b2 = data.PAM_state.y-m2*data.PAM_state.x
    		                                        ICC = [(b2-b1)/(m1-m2), (m1*b2-m2*b1)/(m1-m2)]
    		                                        r_R = np.sqrt((data.PAM_state.x- ICC[0])**2 + (data.PAM_state.y- ICC[1])**2)
    		                                        theta_1 = np.arctan2(gripper_pose[1]-ICC[1], gripper_pose[0]-ICC[0])
    		                                        theta_2 = np.arctan2(gripper_pose[1]+vel[1][1]*env.dt-ICC[1], gripper_pose[0]+vel[1][0]*env.dt-ICC[0])
    		                                        omega_R = np.abs(theta_2-theta_1)/env.dt
    		                                        vel_R = omega_R *r_R
    		                                        command = [[vel_sign*vel_R, omega_sign*omega_R, 1]]
    		                                        desired = [command[0][0], command[0][1], vel[1]]
    		                                        object_path_planned = Path()
    		                                        object_path_planned.header.frame_id = 'frame_0'

    		                                        for i in range(len(planned_path_w)):
    		                                                  pose = PoseStamped()
    		                                                  pose.pose.position.x, pose.pose.position.y, pose.pose.position.z= [planned_path_w[i][0], planned_path_w[i][1], 0]
    		                                                  object_path_planned.poses.append(pose)
    		                                        desired_force.pose.x, desired_force.pose.y, desired_force.pose.phi, desired_force.magnitude = legs[best_leg][0], legs[best_leg][1], np.arctan2(force[0][1],force[0][0]), np.sqrt(force[0][0]**2+force[0][1]**2)
    		                              r.sleep()
    		          elif (data.PAM_state.x-grasping_goal[0])**2+(data.PAM_state.y-grasping_goal[1])**2 <= env.r_grasp**2+env.epsilon and (data.PAM_state.phi-grasping_goal[2])**2 <= env.epsilon_phi:
    		                    sys_mode = 2
    		                    command =  [[0.06, 0,1]]
    		                    desired = [command[0][0], command[0][1]]
    		                    first_3 = True

    		          elif near_pam_goal and (data.PAM_state.x-PAM_goal.x)**2+(data.PAM_state.y-PAM_goal.y)**2 <= env.epsilon and (data.PAM_state.phi-PAM_goal.phi)**2 >= env.epsilon_phi:
    		                    PAM_goal.phi = np.arctan2(legs[best_leg][1] - data.PAM_state.y, legs[best_leg][0] - data.PAM_state.x)
    		                    grasping_goal[2] = np.arctan2(legs[best_leg][1] - data.PAM_state.y, legs[best_leg][0] - data.PAM_state.x)
    		                    boolean = boolean +1
    		                    sys_mode = 1
    		                    rospy.loginfo(sys_mode+0.5)
    		                    if boolean < 10:
    		                              if PAM_goal.phi-data.PAM_state.phi > np.pi:
    		                                        dphi = PAM_goal.phi-data.PAM_state.phi - 2*np.pi
    		                              elif PAM_goal.phi-data.PAM_state.phi < -np.pi:
    		                                        dphi = 2 * np.pi + PAM_goal.phi-data.PAM_state.phi
    		                              else:
    		                                        dphi = PAM_goal.phi-data.PAM_state.phi
    		                              if dphi > 0:
    		                                        desired = [0.01, 1]
    		                              else:
    		                                        desired = [0.01, -1]
    		                    else:
    		                              desired = [0, 0]
    		                              boolean = 1

    		          else:
    		                    sys_mode = 1
    		                    if  (data.PAM_state.x-PAM_goal.x)**2+(data.PAM_state.y-PAM_goal.y)**2 <= env.epsilon/2:
    		                              near_pam_goal = True
    		                    else:
    		                              near_pam_goal = False
    		                    cost_PAM, path, command, goal_orientation = OptPath([data.PAM_state.x, data.PAM_state.y, data.PAM_state.phi], [PAM_goal.x, PAM_goal.y, PAM_goal.phi], env.walls, obstacles_PAM, env.n_p, env.dt)
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

    		if found_solution == 0:
    		          desired = [0, 0]
    		          sys_mode = 0

	rospy.loginfo("sys_mode")
	rospy.loginfo(sys_mode)
	vel = dynamicModelRoomba(desired) #converts v and w velocity of gripper into angular and translational velocity of Roomba
	rospy.loginfo("desired:")
	rospy.loginfo(desired)
	total_time_run = rospy.get_time()-time_run_start
	rospy.loginfo("time_run:")
	rospy.loginfo(total_time_run)
	rospy.loginfo("***************************************************************************")
	if sys_mode == 5:
	       success_status = 1
	       total_error_position = np.sqrt((target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2)
	       total_error_orientation = np.sqrt((target_object.phi-env.object_goal.phi)**2)*180/np.pi
	       rospy.loginfo("Success :)")
	       rospy.loginfo("error:")
	       rospy.loginfo(total_error_position)
	       rospy.loginfo(total_error_orientation)
	       rospy.loginfo("***************************************************************************")
	elif total_time_run>=450:
	       success_status = 0
	       total_error_position = np.sqrt((target_object.x-env.object_goal.x)**2+(target_object.y-env.object_goal.y)**2)
	       total_error_orientation = np.sqrt((target_object.phi-env.object_goal.phi)**2)*180/np.pi
	       rospy.loginfo("Fail :(")
	       rospy.loginfo("error:")
	       rospy.loginfo(total_error_position)
	       rospy.loginfo(total_error_orientation)
	       rospy.loginfo("***************************************************************************")

	PAM_path_predicted = path_from_command(command, data.PAM_state)
	old_bed_state = copy.copy(data.bed_state)
	old_walker_state = copy.copy(data.walker_state)
	old_blue_chair_state = copy.copy(data.blue_chair_state)

	input_d.desired_input, input_d.mode = vel, sys_mode
	desiredInput.publish(input_d)

	vis.plot(desired_force, PAM_goal, env.object_goal, PAM_path_predicted, object_path_actual, object_path_planned, PAM_path_actual, PAM_path_planned)


if __name__ == '__main__':

	rospy.init_node('high_level_controller', anonymous=True)
	desiredInput = rospy.Publisher('controlInput', controlInput, queue_size=10)
	rospy.Subscriber('feedback', feedback, callback, queue_size=1, buff_size=2**24)

	vis = Visual()
	desired_force = force()
	object_path_actual = Path()
	object_path_actual.header.frame_id = 'frame_0'
	PAM_path_actual = Path()
	PAM_path_actual.header.frame_id = 'frame_0'
	object_path_planned = Path()
	PAM_path_planned = Path()
	old_walker_state = state()
	old_bed_state = state()
	old_blue_chair_state = state()

	env = environment()
	env.example_2()
	# object parameters : Rho(4), Omega(3), width(1), length(1)   #Rho[0,1,2,3]=[mass, inertia, x_c, y_c]
	walker_params = [1.71238550e+01, 1.42854052e+02, -9.47588940e-01, -9.75868278e-02, 10.36983266, 92.26898084,  1.19346089, 0.54, 0.35]
	std = [8.47688602, 23.76900771,  0.12580533,  0.06097811, 2.70890194, 6.40448586, 0.08338584, 0, 0]
	walker_r = np.sqrt(walker_params[7]**2 +walker_params[8]**2)/2
	blue_chair_params = [143.91837244, 170.24236598,  -0.52589144,   0.50633468, 37.48830679, 36.84306259,  0.09758333, 0.54, 0.46]
	blue_chair_r = np.sqrt(blue_chair_params[7]**2 +blue_chair_params[8]**2)/2
	rospy.loginfo(env.manipulation_object)
	if env.manipulation_object == "walker":
		object_params = walker_params
		object_r = walker_r
	elif env.manipulation_object == "blue_chair":
		object_params = blue_chair_params
		object_r = blue_chair_r
	phi = [np.arctan2(-object_params[8] + object_params[3], - object_params[7] + object_params[2]) + np.pi,
			 np.arctan2(-object_params[8] + object_params[3], object_params[2]), np.arctan2(-object_params[3], object_params[2]) + np.pi,
			 np.arctan2(object_params[3], object_params[7] - object_params[2]) + np.pi]
	r_max = [0,0,0,0]
	for i in range(4):
		r_max[i] = np.sqrt((np.sign(i% 3)*object_params[7] - object_params[2])**2 + (np.sign(i/2)*object_params[8] - object_params[3])**2)

	# initialization
	time_total = 0
	success = 0
	error_total_position = 0
	error_total_orientation = 0

	near_pam_goal = False
	sys_mode = 0
	boolean = 1
	transition = False
	best_leg = 5
	transition_mode = 0
	action_push_pull = [0, 0, 0, 0]
	grasping_goal = [100, 100, 100]
	found_solution = 0
	time_run_start = rospy.get_time()
	total_time = 0
	total_error_position = 0
	total_error_orientation = 0
	first_3 = True

	try:
	       rospy.spin()
	finally:
	       print "finish"
