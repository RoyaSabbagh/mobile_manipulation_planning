#!/usr/bin/env python

import rospy
import copy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point


def dynamicModelRoomba(gripper_vel):
    ''' This function converts the gripper velocity to
    the robots command inputs
    '''
    if abs(gripper_vel[1]) <= 0.01:
        Roomba_Radius = 32767
        Roomba_Linear = gripper_vel[0] * 100
        if 0 < Roomba_Linear and Roomba_Linear <= 15:
            Roomba_Linear = 15
        if -15 <= Roomba_Linear and Roomba_Linear < 0:
            Roomba_Linear = -15
    else:
        Roomba_Radius = gripper_vel[0] / gripper_vel[1]
        Roomba_Linear = gripper_vel[0] * 1000

        if Roomba_Radius > 1.4:
            Roomba_Radius = 32767
            if 0 < Roomba_Linear < 15:
                Roomba_Linear = 15
            if -15 < Roomba_Linear < 0:
                Roomba_Linear = -15
        elif Roomba_Radius < -1.5:
            Roomba_Radius = 32767
            if 0 < Roomba_Linear < 15:
                Roomba_Linear = 15
            if -15 < Roomba_Linear < 0:
                Roomba_Linear = -15
        elif Roomba_Radius == 0:
            Roomba_Radius = 1*np.sign(gripper_vel[1])
            Roomba_Linear = 12
        elif 0 < Roomba_Radius < 0.05:
            Roomba_Radius = 1
            Roomba_Linear = 15
        elif -0.05 < Roomba_Radius < 0:
            Roomba_Radius = -1
            Roomba_Linear = 15
        else:
            Roomba_Radius = Roomba_Radius * 100
            if 0 < Roomba_Linear and Roomba_Linear < 15:
                Roomba_Linear = 15
            if -15 < Roomba_Linear and Roomba_Linear < 0:
                Roomba_Linear = -15

    if Roomba_Radius != 32767:
        Roomba_Radius = -Roomba_Radius
    Roomba_Linear = -Roomba_Linear
    return Roomba_Linear, Roomba_Radius

def path_from_command(command, point):
    ''' This function converts the PAM commands to predicted path'''
    m_path = 5
    pose = PoseStamped()
    pose.pose.position.x = point.x
    pose.pose.position.y = point.y
    pose.pose.position.z = 0
    pam_path = Path()
    pam_path.poses.append(pose)
    pam_path.header.frame_id = 'frame_0'
    for i in range(len(command)):
        v = command[i][0]
        w = command[i][1]
        dt = command[i][2]
        if w <= 0.001:
            for j in range(m_path):
                aux = [point.x+float(dt*v)/m_path*(np.cos(point.phi)), point.y+float(dt*v)/m_path*(np.sin(point.phi)), point.phi]
                point.x = aux[0]
                point.y = aux[1]
                point.phi = aux[2]
                pose = PoseStamped()
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.position.z = 0
                pam_path.poses.append(pose)
        else:
            r_w = float(v)/w
            for j in range(m_path):
                aux = [point.x+r_w*(np.cos((float(dt*w)/m_path)+point.phi-np.sign(w)*(np.pi/2))-np.cos(point.phi-np.sign(w)*(np.pi/2))), point.y+r_w*(np.sin((float(dt*w)/m_path)+point.phi-np.sign(w)*(np.pi/2))-np.sin(point.phi-np.sign(w)*(np.pi/2))), point.phi+(float(dt*w)/m_path)]
                point.x = aux[0]
                point.y = aux[1]
                point.phi = aux[2]
                pose = PoseStamped()
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.position.z = 0
                pam_path.poses.append(pose)
    return pam_path

def define_obs(obss, radius):
	''' This function returns parameters used for obstacle avoidance'''
        obstacles = []
        for obs in obss:
                m_box = [np.tan(obs[2]), np.tan(obs[2] + np.pi / 2), np.tan(obs[2]), np.tan(obs[2] + np.pi / 2)]
                corners = find_corners(obs[0], obs[1], obs[2],2*radius + obs[3], 2*radius + obs[4])
                b = [corners[0][1] - m_box[0] * corners[0][0], corners[1][1] - m_box[1] * corners[1][0],
                         corners[2][1] - m_box[2] * corners[2][0], corners[3][1] - m_box[3] * corners[3][0]]
                center_pose = [obs[0], obs[1]]
                obstacles.append([m_box, b, center_pose])
        return obstacles

def Find_phi_quarter(phi):
	''' This function returns which quarter robot's orientation is in '''
        phi_quarter = 0
        if 0 <= phi <= np.pi/2:
                phi_quarter = 1
        elif np.pi/2 <= phi <= np.pi:
                phi_quarter = 2
        elif 0 >= phi >= -np.pi/2:
                phi_quarter = 4
        elif -np.pi/2 >= phi >= -np.pi:
                phi_quarter = 3
        return phi_quarter

def FindLeg(gripper_pose, object_state, object_params):
	''' This function detects which leg is currently grasped by the robot '''
        current_leg = 10
        legs = find_corners(object_state.x, object_state.y, object_state.phi, object_params[7], object_params[8])
        for i in range(4):
                if ((legs[i][0]-gripper_pose[0])*(legs[i][0]-gripper_pose[0])+(legs[i][1]-gripper_pose[1])*(legs[i][1]-gripper_pose[1])) < 0.002:
                        current_leg = i

        return legs, current_leg

def detectChange(walker_state, obs_state, blue_chair_state, old_blue_chair_state, old_bed_state, old_walker_state):
	''' This function detects if the environment is changed '''
        change = False
        if (walker_state.x-old_walker_state.x)**2 + (walker_state.y-old_walker_state.y)**2 > 0.0001 or (walker_state.phi-old_walker_state.phi)**2 > 0.0001 or (obs_state.x-old_bed_state.x)**2 + (obs_state.y-old_bed_state.y)**2 > 0.0001 or (obs_state.phi-old_bed_state.phi)**2 > 0.0001:
                change = True
        return change

def find_corners(x, y, phi, width, length):
	''' This function finds corners of an object '''
	corners = []
	corners.append([x - (width/2) * np.sin(phi) - (length/2) * np.cos(phi), y + (width/2) * np.cos(phi) - (length/2) * np.sin(phi)])
	corners.append([x + (width/2) * np.sin(phi) - (length/2) * np.cos(phi), y - (width/2) * np.cos(phi) - (length/2) * np.sin(phi)])
	corners.append([x + (width/2) * np.sin(phi) + (length/2) * np.cos(phi), y - (width/2) * np.cos(phi) + (length/2) * np.sin(phi)])
	corners.append([x - (width/2) * np.sin(phi) + (length/2) * np.cos(phi), y + (width/2) * np.cos(phi) + (length/2) * np.sin(phi)])

	return corners
