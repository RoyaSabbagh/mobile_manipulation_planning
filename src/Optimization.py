from gurobipy import *
import rospy
import math
import numpy as np
import rospy
'This script includes all the optimization functions for motion and manipulation planning.'
'There are some minor differences between optimization implemention for LQR and our method,'
'and also between simulation and real experiments.'


def OptTraj(state_s, state_f, v_max, walls, obstacles, Rho, Omega, phi_linear, psi_linear, force_max, r_max, n, dt, gripper_pose):
# Create a new model
    dist = math.sqrt((state_f[0]-state_s[0])**2+(state_f[1]-state_s[1])**2)
    dt = 0.5
    big_M = 10000000000000
    w = [0.1*dt*dt, 5, 0.1/(dist*dist),0.01]
    cost = float('inf')
    desired_force = [[0,0,0]]
    path = []
    desired_force = []
    vel = []
    desired = []
    desired_force = [[0,0,0]]
    rospy.loginfo("Starting optimization...")
############## defining the problem for Gurobi #############
    ManipulationPlanner = Model("Manipulation")
    ManipulationPlanner.reset(0) #reset the model
# Create variables
    ManipulationPlanner.setParam('OutputFlag', 0)
    ManipulationPlanner.setParam("TimeLimit", 0.5)
    states = ManipulationPlanner.addVars(n+1, 6, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="states")
    force = ManipulationPlanner.addVars(n, 2, lb=-1000, ub=1000, vtype=GRB.CONTINUOUS, name="force")  #exerted force to the object from Roomba
    state_ff = ManipulationPlanner.addVars(6, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="final_step_state") #final state of each horizon, the goal is to make it close to the goal of object
    V_wheel = ManipulationPlanner.addVars(n, 2, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="v_wheel") #wheel is the model of the object
    F_wheel = ManipulationPlanner.addVars(n, 2, lb=-1000, ub=1000, vtype=GRB.CONTINUOUS, name="F_wheel") #it defers from the exerting force from the PAM, this one includes friction too.
    u = ManipulationPlanner.addVars(len(obstacles), n+1, 4, lb=-1, ub=1, vtype=GRB.BINARY, name="u") # for big-M method and obstacle avoidance
# Set objective (min path and move toward the goal in both position and orientation manner)
    ManipulationPlanner.setObjective((w[0]*sum([states[k,l]*states[k,l] for k in range(n+1) for l in range(3,5)]) + w[1]*sum([(states[k,l]-state_f[l])*(states[k,l]-state_f[l]) for k in range(n+1) for l in [0,1]]) + w[2]*sum([(states[k,l]-state_f[l])*(states[k,l]-state_f[l]) for k in range(n+1) for l in [2]])+w[3]*sum([(force[k, 0]-force[k-1, 0])*(force[k, 0]-force[k-1, 0])+(force[k, 1]-force[k-1, 1])*(force[k, 1]-force[k-1, 1]) for k in range(1,n)])), GRB.MINIMIZE)
# Add position, velocity and acceleration constraints
    for i in range(6):
        ManipulationPlanner.addConstr(states[0, i] == state_s[i], "c0_{}".format(i))  # define the start pose
    for j in range(1, n+1):
        for i in range(3):
            ManipulationPlanner.addConstr(states[j, i+3] == (states[j, i]-states[j-1, i])/dt, "c1_{}_{}".format(i,j))
            ManipulationPlanner.addConstr(states[j, i+3]*states[j, i+3] <= v_max[0]*v_max[0], "c2_{}_{}".format(i,j))
# Add dynamic and force limit constraints
    for i in range(n):              #Rho[0,1,2,3]=[mass, inertia, x_c, y_c]
        ManipulationPlanner.addConstr(V_wheel[i,0] == np.cos(phi_linear[i]+Omega[2])*(states[i, 3] - states[i, 5] * (Rho[2]*np.sin(phi_linear[i])+Rho[3]*np.cos(phi_linear[i]))) + np.sin(phi_linear[i]+Omega[2]) * (states[i, 4] + states[i, 5] * (Rho[2]*np.cos(phi_linear[i])-Rho[3]*np.sin(phi_linear[i]))), "c3_{}".format(i))
        ManipulationPlanner.addConstr(V_wheel[i,1] == -np.sin(phi_linear[i]+Omega[2])*(states[i, 3] - states[i, 5] * (Rho[2]*np.sin(phi_linear[i])+Rho[3]*np.cos(phi_linear[i]))) + np.cos(phi_linear[i]+Omega[2]) * (states[i, 4] + states[i, 5] * (Rho[2]*np.cos(phi_linear[i])-Rho[3]*np.sin(phi_linear[i]))), "c4_{}".format(i))
        ManipulationPlanner.addConstr(F_wheel[i,0] == -np.cos(phi_linear[i]+Omega[2])*(V_wheel[i,0] * Omega[0]) + np.sin(phi_linear[i]+Omega[2]) * (V_wheel[i,1] * Omega[1]), "c5_{}".format(i))
        ManipulationPlanner.addConstr(F_wheel[i,1] == -np.sin(phi_linear[i]+Omega[2])*(V_wheel[i,0] * Omega[0]) - np.cos(phi_linear[i]+Omega[2]) * (V_wheel[i,1] * Omega[1]), "c6_{}".format(i))
        ManipulationPlanner.addConstr((F_wheel[i,0] + force[i, 0] * math.cos(psi_linear[i]) + force[i, 1] * math.sin(psi_linear[i])) / Rho[0] * dt == states[i+1, 3] - states[i, 3], "c7_{}".format(i)) #Rho[0] =mass
        ManipulationPlanner.addConstr((F_wheel[i,1] + force[i, 0] * math.sin(psi_linear[i]) - force[i, 1] * math.cos(psi_linear[i])) / Rho[0] * dt == states[i+1, 4] - states[i, 4], "c8_{}".format(i))  #f=mass* acceleration
        ManipulationPlanner.addConstr(force[i, 1] * r_max /Rho[1] * dt == states[i+1, 5] - states[i, 5], "c9_{}".format(i))
    for i in range(n-1):
        ManipulationPlanner.addConstr(force[i, 0]*force[i, 0] + force[i, 1]*force[i, 1] <= force_max*force_max, "c10_{}".format(i))#limit the exerting force
# Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n+1):
            for k in range(4):
                ManipulationPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c11_{}_{}_{}".format(n_obs,i,j))
        for i in range(0,n):
            ManipulationPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c12_{}_{}".format(n_obs, i))
# Add walls constraints
    # for i in range(n):
        # ManipulationPlanner.addConstr(states[i, 0] >= walls[0]+0.5, "c13_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 0] <= walls[1], "c14_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 1] >= walls[2], "c15_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 1] <= walls[3], "c16_{}".format(i))

    ManipulationPlanner.optimize()

    rospy.loginfo("Finished optimization...")
    if ManipulationPlanner.Status == 2: # 2 means it is solved successfully
        cost = ManipulationPlanner.objVal
        desired_force = []
    	for i in range(n):
            force_r = [force[i,0].x, force[i,1].x]  #force{number of horizon, element} we only take the first time step
            new_force = [force_r[0] * math.cos(psi_linear[0]) + force_r[1] * math.sin(psi_linear[0]), force_r[0] * math.sin(psi_linear[0]) - force_r[1] * math.cos(psi_linear[0]), force_r[1] * r_max]
            desired_force.append(new_force)
            dx = states[i,3].x
            dy = states[i,4].x
            dphi = states[i,5].x
            desired.append([states[i,0].x,states[i,1].x,states[i,2].x,states[i,3].x,states[i,4].x,states[i,5].x])
            vel.append([(dx-dphi*r_max*np.cos(psi_linear[i])),(dy-dphi*r_max*np.sin(psi_linear[i]))])
        for i in range(n+1):
            path.append([states[i,0].x, states[i,1].x])  #we take the entire horizon because we want to show in Rviz and also for LQR

    # if ManipulationPlanner.Status == 4:
    #     ManipulationPlanner.computeIIS()
    #     print "infeasible"
    #     for c in ManipulationPlanner.getConstrs():
    #         if c.IISConstr:
    #             print('%s' % c.constrName)

    return desired_force, 10*cost, path, vel, desired

def nominal_traj(pos_s, pos_f, v_max, walls, obstacles, n, dt):
    # Create a new model
    big_M = 100000000000
    dt = 1
    w1 = 5
    NominalTrajPlanner = Model("NominalTraj")
    NominalTrajPlanner.setParam('OutputFlag', 0)
    NominalTrajPlanner.setParam("TimeLimit", 0.5)
    # Create variables
    pos = NominalTrajPlanner.addVars(n + 1, len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="pos")
    pos_d = NominalTrajPlanner.addVars(n, len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="pos_derivatives")
    pos_ff = NominalTrajPlanner.addVars(len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="final_step_pos")
    u = NominalTrajPlanner.addVars(len(obstacles), n + 1, 4, lb=-1, ub=1, vtype=GRB.BINARY, name="u")
    # Set objective
    NominalTrajPlanner.setObjective((sum([pos_d[k, l] * pos_d[k, l] for k in range(n) for l in range(len(pos_s))]) +
                                       w1*(sum([(pos_ff[l] - pos_f[l]) * (pos_ff[l] - pos_f[l]) for l in range(len(pos_s))]))), GRB.MINIMIZE)
    # Add constraint
    for i in range(len(pos_s)):
        NominalTrajPlanner.addConstr(pos[0, i] == pos_s[i], "c0_{}".format(i))
        NominalTrajPlanner.addConstr(pos[n, i] == pos_ff[i], "c1_{}".format(i))
        for j in range(n):
            NominalTrajPlanner.addConstr(pos_d[j, i] == pos[j + 1, i] - pos[j, i], "c2_{}_{}".format(i,j))
            NominalTrajPlanner.addConstr((pos_d[j, 0] * pos_d[j, 0]) + (pos_d[j, 1] * pos_d[j, 1]) <= v_max[0] *v_max[0] * dt * dt, "c3_{}_{}".format(i,j))
            NominalTrajPlanner.addConstr(pos_d[j, 2] * pos_d[j, 2] <= v_max[1] * v_max[1] * dt * dt, "c4_{}_{}".format(i,j))
    # Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n + 1):
            for k in range(4):
                NominalTrajPlanner.addConstr((pos[i, 1] - obstacles[n_obs][0][k]*pos[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c5_{}_{}_{}".format(n_obs,i,j))
        for i in range(n):
            NominalTrajPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c6_{}_{}".format(n_obs,i))

    # Add walls constraints
    # for i in range(n):
    #     NominalTrajPlanner.addConstr(pos[i, 0] >= walls[0], "c7_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 0] <= walls[1], "c8_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 1] >= walls[2], "c9_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 1] <= walls[3], "c10_{}".format(i))

    NominalTrajPlanner.optimize()


    # if NominalTrajPlanner.Status == 4:
    #     NominalTrajPlanner.computeIIS()
    #     print "infeasible"
    #     for c in NominalTrajPlanner.getConstrs():
    #         if c.IISConstr:
    #             print('%s' % c.constrName)
    theta = []
    for i in range(n+1):
        theta.append(pos[i,2].x)
    return theta

def OptPath(state_s, state_f, walls, obstacles, n, dt):
# Create a new model
    big_M = 1000000000000000
    v_max = [1, 3]
    w = [10, 2, 0.05]
    # state_bed = [2, 0.2]
    command = []
    phi = []
    path = []
    cost = float('inf')
    initial_orientation = 0

    for n_obs in range(len(obstacles)):
	    Safe = 0
	    for k in range(4):
	           if (state_f[1] - obstacles[n_obs][0][k]*state_f[0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k]) <= 0:
	                  Safe = 1
	    if Safe == 0 or state_f[0] <= walls[0] or state_f[0] >= walls[1] or state_f[1] <= walls[2]+0.5 or state_f[1] >= walls[3]:
	           return cost, [[],[]], [], []
    NominalPathPlanner = Model("NominalPath")
# Create variables
    NominalPathPlanner.setParam('OutputFlag', 0)
    NominalPathPlanner.setParam("TimeLimit", 0.5)
    states = NominalPathPlanner.addVars(n+1, 2, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="states")
    dstates = NominalPathPlanner.addVars(n, 2, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="dstates")
    v_abs = NominalPathPlanner.addVars(n, lb=0, ub=10, vtype=GRB.CONTINUOUS, name="v_abs")
    u =NominalPathPlanner.addVars(len(obstacles), n+1, 4, lb=-1, ub=1, vtype=GRB.BINARY, name="u")
# Set objective
    NominalPathPlanner.setObjective(w[0]*sum([dstates[k,l]*dstates[k,l] for k in range(n) for l in range(2)]) + w[1]*sum([(states[k,l]-state_f[l])*(states[k,l]-state_f[l]) for l in range(2) for k in range(n+1)]), GRB.MINIMIZE)
# Add position, velocity and acceleration constraints
    for i in range(2):
        NominalPathPlanner.addConstr(states[0, i] == state_s[i], "c0_{}".format(i))
    for j in range(n):
        for i in range(2):
            NominalPathPlanner.addConstr(dstates[j,i] == (states[j+1, i]-states[j, i]), "c1_{}_{}".format(i,j))
        NominalPathPlanner.addConstr(v_max[0] * dt * v_max[0] * dt >= (dstates[j,1] * dstates[j,1]) + (dstates[j,0] * dstates[j,0]), "c2_{}".format(j))
# Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n+1):
            for k in range(4):
                NominalPathPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c3_{}_{}_{}".format(n_obs,i,k))
        for i in range(0,n):
            NominalPathPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c4_{}_{}".format(n_obs, i))
    # # Add walls constraints
    # for i in range(n+1):
    #     # NominalPathPlanner.addConstr(states[i, 0] >= walls[0]+0.5, "c5_{}".format(i))
    #     # NominalPathPlanner.addConstr(states[i, 0] <= walls[1], "c6_{}".format(i))
    #     NominalPathPlanner.addConstr(states[i, 1] >= walls[2]+0.5, "c7_{}".format(i))
        # NominalPathPlanner.addConstr(states[i, 1] <= walls[3], "c8_{}".format(i))

    NominalPathPlanner.optimize()
    if NominalPathPlanner.Status == 2:
        cost = NominalPathPlanner.objVal
        phi = [state_s[2]]
        for i in range(n):
            x1 = states[i,0].x
            y1 = states[i,1].x
            x2 = states[i+1,0].x
            y2 = states[i+1,1].x
            new_phi = math.atan2(y2-y1,x2-x1)
            if i == 0:
                rospy.loginfo("new_phi")
                rospy.loginfo(new_phi)
            if (y2-y1)**2+(x2-x1)**2 >= 0.000001:
                if abs(abs(new_phi-phi[-1])-np.pi)<=0.001:
                    v = -math.sqrt((x2-x1)**2+(y2-y1)**2)/dt
                    w = 0
                elif (new_phi -phi[-1])*(new_phi -phi[-1]) <= 0.05:
                    v = math.sqrt((x2-x1)**2+(y2-y1)**2)/dt
                    w = 0
                else:
                    if new_phi-phi[-1] > np.pi:
                        dphi = new_phi-phi[-1] - 2*np.pi
                    elif new_phi-phi[-1] < -np.pi:
                        dphi = 2 * np.pi + new_phi-phi[-1]
                    else:
                        dphi = new_phi-phi[-1]
                    w = dphi/dt
                    v = 0
                phi.append(new_phi)
                command.append([v, w, dt])
                path.append([states[i,0].x, states[i,1].x, phi[-1]])
        initial_orientation = phi[1]

    # if NominalPathPlanner.Status == 4 or NominalPathPlanner.Status == 3:
    #     NominalPathPlanner.computeIIS()
    #     print "infeasible"
    #     for c in NominalPathPlanner.getConstrs():
    #         if c.IISConstr:
    #             print('%s' % c.constrName)

    return cost, path, command, initial_orientation

def OptPath_real(state_s, state_f, walls, obstacles, n, dt):
     cost = float('inf')
     dt =1
     initial_orientation = 0
     for n_obs in range(len(obstacles)):
         Safe = 0
         for k in range(4):
             if (state_f[1] - obstacles[n_obs][0][k]*state_f[0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k]) <= 0:
                 Safe = 1
         if Safe == 0:
             return cost, [[],[]], [], []
     phi, nominal_path = nominal_Path_real(state_s, state_f, walls, obstacles, n, dt)
     path = []
     command = []
     indicator = []
     # print nominal_path
     if len(phi)!=0:
         count = 1
         for k in range(1,len(phi)):
             if (phi[k] - phi[k-1])**2<=0.001:
                 count = - count
                 phi[k-1] = phi[k-1] + count * 0.001
         aux = []
         for j in range(n):
             aux.append((math.sin(phi[j+1]-np.sign(phi[j+1]-phi[j])*math.pi/2) - math.sin(phi[j]-np.sign(phi[j+1]-phi[j])*math.pi/2))/(math.cos(phi[j+1]-np.sign(phi[j+1]-phi[j])*math.pi/2) - math.cos(phi[j]-np.sign(phi[j+1]-phi[j])*math.pi/2)))
         v_max = [0.3, 1]
         big_M = 100000000
         w1 = 1
         w2 = 5
         w3 = 0
     # Create a new model
         PathPlanner = Model("path")
         PathPlanner.reset(0)
     # Create variables
         PathPlanner.setParam('OutputFlag', 0)
         PathPlanner.setParam("TimeLimit", 0.5)
         states = PathPlanner.addVars(n+1, 3, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="states")
         dstates = PathPlanner.addVars(n, 3, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dstates")
         state_ff = PathPlanner.addVars(3, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="state_ff")
         ind = PathPlanner.addVars(n, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="ind")
         ind2 = PathPlanner.addVars(n, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="ind")
         v_abs = PathPlanner.addVars(n, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="v_abs")
         w_abs = PathPlanner.addVars(n, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="w_abs")
         u = PathPlanner.addVars(len(obstacles), n+1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="u")
     	# Set objective
         PathPlanner.setObjective(w1*sum([dstates[k,l]*dstates[k,l] for k in range(n) for l in range(3)]) + w2*sum([(state_ff[k]-state_f[k])*(state_ff[k]-state_f[k]) for k in range(2)]) + w3*(state_ff[2]-state_f[2])*(state_ff[2]-state_f[2]), GRB.MINIMIZE)
     	# Add position, velocity and acceleration constraints
         for i in range(3):
             PathPlanner.addConstr(states[0, i] == state_s[i], "c0_{}".format(i))
             PathPlanner.addConstr(states[n, i] == state_ff[i], "c1_{}".format(i))
             for j in range(n):
                 PathPlanner.addConstr(dstates[j,i] == (states[j+1, i]-states[j, i]), "c2_{}_{}".format(i,j))
         for j in range(n):
            PathPlanner.addGenConstrIndicator(ind2[j], True, states[j,2] <= phi[j]+0.3)
            PathPlanner.addGenConstrIndicator(ind2[j], True, states[j,2] >= phi[j]-0.3)
            PathPlanner.addGenConstrIndicator(ind2[j], False, states[j,2] <= phi[j]+math.pi+0.3)
            PathPlanner.addGenConstrIndicator(ind2[j], False, states[j,2] >= phi[j]+math.pi-0.3)
            PathPlanner.addConstr(dstates[j,1] * np.sin(phi[j]) >= 0, "c7_{}".format(j))
            PathPlanner.addGenConstrAbs(w_abs[j], dstates[j,2])
            PathPlanner.addConstr(v_abs[j] * v_abs[j] >= (dstates[j,1] * dstates[j,1]) + (dstates[j,0] * dstates[j,0]), "c9_{}".format(j))
            PathPlanner.addConstr(v_abs[j] <= v_max[0] * dt, "c10_{}".format(j))
            PathPlanner.addConstr(w_abs[j] <= v_max[1] * dt, "c11_{}".format(j))
            PathPlanner.addGenConstrIndicator(ind[j], True, v_abs[j] - 1.3 * w_abs[j], GRB.LESS_EQUAL, 0.0)
            PathPlanner.addGenConstrIndicator(ind[j], False, w_abs[j] , GRB.LESS_EQUAL, 0.1)
            PathPlanner.addGenConstrIndicator(ind[j], False, dstates[j,1] == dstates[j, 0] * math.tan(phi[j]))
            PathPlanner.addGenConstrIndicator(ind[j], True, dstates[j,1] == dstates[j, 0] * aux[j])

     	# Add Obstacle constraints
         for n_obs in range(len(obstacles)):
             for i in range(0, n+1):
                 for k in range(4):
                     PathPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k]) <= - (u[n_obs, i, k]-1)*big_M, "c16_{}_{}_{}".format(n_obs, i , k))
             for i in range(0,n):
                 PathPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c17_{}_{}".format(n_obs,i))

         PathPlanner.optimize()
         if PathPlanner.Status == 2:
             cost = PathPlanner.objVal
             for i in range(n+1):
                 path.append([states[i,0].x, states[i,1].x, states[i,2].x])
             for i in range(n):
                 dx = dstates[i,0].x
                 dy = dstates[i,1].x
                 dphi = dstates[i,2].x
                 phi_d= math.atan2(dy,dx)
                 if (phi_d * phi[i] < 0):
                     v = -math.sqrt( dx**2 + dy**2 )/dt
                 else:
                     v = math.sqrt(dx**2 + dy**2)/dt
                 w =  dphi/dt
                 command.append([v, w, dt])
                 phi.append(phi_d)
                 path.append([states[i,0].x, states[i,1].x, phi[-1]])
         initial_orientation = phi[1]
         # if PathPlanner.Status == 4 or PathPlanner.Status == 3:
         #     PathPlanner.computeIIS()
         #     print "infeasible"
         #     for c in PathPlanner.getConstrs():
         #         if c.IISConstr:
         #             print('%s' % c.constrName)
     return cost, path, command, initial_orientation

def nominal_Path_real(state_s, state_f, walls, obstacles, n, dt):
 # Create a new model
     big_M = 1000000000
     v_max = [0.3, 1]
     w1 = 1
     w2 = 5
     NominalPathPlanner = Model("NominalPath")
 # Create variables
     NominalPathPlanner.setParam('OutputFlag', 0)
     NominalPathPlanner.setParam("TimeLimit", 0.5)
     states = NominalPathPlanner.addVars(n+1, 2, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="states")
     dstates = NominalPathPlanner.addVars(n, 2, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="dstates")
     state_ff = NominalPathPlanner.addVars(2, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="state_ff")
     v_abs = NominalPathPlanner.addVars(n, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="v_abs")
     u =NominalPathPlanner.addVars(len(obstacles), n+1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="u")
 # Set objective
     NominalPathPlanner.setObjective(w1*sum([dstates[k,l]*dstates[k,l] for k in range(n) for l in range(2)]) + w2*sum([(state_ff[k]-state_f[k])*(state_ff[k]-state_f[k]) for k in range(2)]), GRB.MINIMIZE)
 # Add position, velocity and acceleration constraints
     for i in range(2):
         NominalPathPlanner.addConstr(states[0, i] == state_s[i], "c0_{}".format(i))
         NominalPathPlanner.addConstr(states[n, i] == state_ff[i], "c1_{}".format(i))
     NominalPathPlanner.addConstr(dstates[0, 1] == dstates[0, 0] * np.tan(state_s[2]), "c2")

     for j in range(n):
         for i in range(2):
             NominalPathPlanner.addConstr(dstates[j,i] == (states[j+1, i]-states[j, i]), "c3_{}_{}".format(i,j))
         NominalPathPlanner.addConstr(v_max[0] * dt * v_max[0] * dt >= (dstates[j,1] * dstates[j,1]) + (dstates[j,0] * dstates[j,0]), "c4_{}".format(j))

 # Add Obstacle constraints
     for n_obs in range(len(obstacles)):
         for i in range(0, n+1):
             for k in range(4):
                 NominalPathPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c5_{}_{}".format(n_obs,i,k))
         for i in range(0,n):
             NominalPathPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c6_{}_{}".format(n_obs,i))

     NominalPathPlanner.optimize()
     phi = []
     path = []
     if NominalPathPlanner.Status == 2:
         for i in range(n):
             x1 = states[i,0].x
             y1 = states[i,1].x
             x2 = states[i+1,0].x
             y2 = states[i+1,1].x
             phi.append(math.atan2(y2-y1,x2-x1))
         phi.append(math.atan2(y2-y1,x2-x1))
         for i in range(n+1):
             path.append([states[i,0].x, states[i,1].x])
     # if NominalPathPlanner.Status == 4 or NominalPathPlanner.Status == 3:
     #     NominalPathPlanner.computeIIS()
     #     print "infeasible"
     #     for c in NominalPathPlanner.getConstrs():
     #         if c.IISConstr:
     #             print('%s' % c.constrName)

     return phi, path

def OptTraj_LQR(state_s, state_f, v_max, walls, obstacles, Rho, Omega, phi_linear, psi_linear, force_max, r_max, n, dt, gripper_pose):
# Create a new model
    dist = math.sqrt((state_f[0]-state_s[0])**2+(state_f[1]-state_s[1])**2)
    big_M = 10000000000000
    w = [1*dt*dt, 1000, 500]
    cost = float('inf')
    desired_force = [[0,0,0]]
    path = []
    desired_force = []
    vel = []
    desired = []
    desired_force = [[0,0,0]]

############## defining the problem for Gurobi #############
    ManipulationPlanner = Model("Manipulation")
    ManipulationPlanner.reset(0) #reset the model
# Create variables
    ManipulationPlanner.setParam('OutputFlag', 0)
    states = ManipulationPlanner.addVars(n+1, 6, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="states")
    force = ManipulationPlanner.addVars(n, 2, lb=-1000, ub=1000, vtype=GRB.CONTINUOUS, name="force")  #exerted force to the object from Roomba
    state_ff = ManipulationPlanner.addVars(6, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="final_step_state") #final state of each horizon, the goal is to make it close to the goal of object
    V_wheel = ManipulationPlanner.addVars(n, 2, lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="v_wheel") #wheel is the model of the object
    F_wheel = ManipulationPlanner.addVars(n, 2, lb=-1000, ub=1000, vtype=GRB.CONTINUOUS, name="F_wheel") #it defers from the exerting force from the PAM, this one includes friction too.
    u = ManipulationPlanner.addVars(len(obstacles), n+1, 4, lb=-1, ub=1, vtype=GRB.BINARY, name="u") # for big-M method and obstacle avoidance
# Set objective (min path and move toward the goal in both position and orientation manner)
    ManipulationPlanner.setObjective((w[0]*sum([states[k,l]*states[k,l] for k in range(n+1) for l in range(3,5)]) + w[1]*sum([(state_ff[l]-state_f[l])*(state_ff[l]-state_f[l]) for l in [0,1]]) + w[2]*sum([(state_ff[l]-state_f[l])*(state_ff[l]-state_f[l]) for l in [2]]) + 10*sum([(states[k,l]-state_f[l])*(states[k,l]-state_f[l]) for k in range(n+1) for l in [0,1]])), GRB.MINIMIZE)
# Add position, velocity and acceleration constraints
    for i in range(6):
        ManipulationPlanner.addConstr(states[0, i] == state_s[i], "c0_{}".format(i))  # define the start pose
    for i in range(3):
        ManipulationPlanner.addConstr(states[n, i] == state_ff[i], "c0_{}".format(i))
    for j in range(1, n+1):
        for i in range(3):
            ManipulationPlanner.addConstr(states[j, i+3] == (states[j, i]-states[j-1, i])/dt, "c1_{}_{}".format(i,j))
            ManipulationPlanner.addConstr(states[j, i+3]*states[j, i+3] <= v_max[0]*v_max[0], "c2_{}_{}".format(i,j))
# Add dynamic and force limit constraints
    for i in range(n):              #Rho[0,1,2,3]=[mass, inertia, x_c, y_c]
        ManipulationPlanner.addConstr(V_wheel[i,0] == np.cos(phi_linear[i]+Omega[2])*(states[i, 3] - states[i, 5] * (Rho[2]*np.sin(phi_linear[i])+Rho[3]*np.cos(phi_linear[i]))) + np.sin(phi_linear[i]+Omega[2]) * (states[i, 4] + states[i, 5] * (Rho[2]*np.cos(phi_linear[i])-Rho[3]*np.sin(phi_linear[i]))), "c3_{}".format(i))
        ManipulationPlanner.addConstr(V_wheel[i,1] == -np.sin(phi_linear[i]+Omega[2])*(states[i, 3] - states[i, 5] * (Rho[2]*np.sin(phi_linear[i])+Rho[3]*np.cos(phi_linear[i]))) + np.cos(phi_linear[i]+Omega[2]) * (states[i, 4] + states[i, 5] * (Rho[2]*np.cos(phi_linear[i])-Rho[3]*np.sin(phi_linear[i]))), "c4_{}".format(i))
        ManipulationPlanner.addConstr(F_wheel[i,0] == -np.cos(phi_linear[i]+Omega[2])*(V_wheel[i,0] * Omega[0]) + np.sin(phi_linear[i]+Omega[2]) * (V_wheel[i,1] * Omega[1]), "c5_{}".format(i))
        ManipulationPlanner.addConstr(F_wheel[i,1] == -np.sin(phi_linear[i]+Omega[2])*(V_wheel[i,0] * Omega[0]) - np.cos(phi_linear[i]+Omega[2]) * (V_wheel[i,1] * Omega[1]), "c6_{}".format(i))
        ManipulationPlanner.addConstr((F_wheel[i,0] + force[i, 0] * math.cos(psi_linear[i]) + force[i, 1] * math.sin(psi_linear[i])) / Rho[0] * dt == states[i+1, 3] - states[i, 3], "c7_{}".format(i)) #Rho[0] =mass
        ManipulationPlanner.addConstr((F_wheel[i,1] + force[i, 0] * math.sin(psi_linear[i]) - force[i, 1] * math.cos(psi_linear[i])) / Rho[0] * dt == states[i+1, 4] - states[i, 4], "c8_{}".format(i))  #f=mass* acceleration
        ManipulationPlanner.addConstr(force[i, 1] * r_max /Rho[1] * dt == states[i+1, 5] - states[i, 5], "c9_{}".format(i))
    for i in range(n-1):
        ManipulationPlanner.addConstr(force[i, 0]*force[i, 0] + force[i, 1]*force[i, 1] <= force_max*force_max, "c10_{}".format(i))#limit the exerting force
# Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n+1):
            for k in range(4):
                ManipulationPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c11_{}_{}_{}".format(n_obs,i,j))
        for i in range(0,n):
            ManipulationPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c12_{}_{}".format(n_obs, i))
# Add walls constraints
    # for i in range(n):
        # ManipulationPlanner.addConstr(states[i, 0] >= walls[0]+0.5, "c13_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 0] <= walls[1], "c14_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 1] >= walls[2], "c15_{}".format(i))
    #     ManipulationPlanner.addConstr(states[i, 1] <= walls[3], "c16_{}".format(i))

    ManipulationPlanner.optimize()
    if ManipulationPlanner.Status == 2: # 2 means it is solved successfully
        cost = ManipulationPlanner.objVal
        desired_force = []
    	for i in range(n):
            force_r = [force[i,0].x, force[i,1].x]  #force{number of horizon, element} we only take the first time step
            new_force = [force_r[0] * math.cos(psi_linear[0]) + force_r[1] * math.sin(psi_linear[0]), force_r[0] * math.sin(psi_linear[0]) - force_r[1] * math.cos(psi_linear[0]), force_r[1] * r_max]
            desired_force.append(new_force)
            dx = states[i,3].x
            dy = states[i,4].x
            dphi = states[i,5].x
            desired.append([states[i,0].x,states[i,1].x,states[i,2].x,states[i,3].x,states[i,4].x,states[i,5].x])
            vel.append([(dx-dphi*r_max*np.cos(psi_linear[i])),(dy-dphi*r_max*np.sin(psi_linear[i]))])
        for i in range(n+1):
            path.append([states[i,0].x, states[i,1].x])  #we take the entire horizon because we want to show in Rviz and also for LQR

    # if ManipulationPlanner.Status == 4 or ManipulationPlanner.Status == 3:
    #     ManipulationPlanner.computeIIS()
    #     print "infeasible"
    #     for c in ManipulationPlanner.getConstrs():
    #         if c.IISConstr:
    #             print('%s' % c.constrName)

    return desired_force, 10*cost, path, vel, desired

def nominal_traj_LQR(pos_s, pos_f, v_max, walls, obstacles, n, dt):
    # Create a new model
    big_M = 100000000000
    NominalTrajPlanner = Model("NominalTraj")
    NominalTrajPlanner.setParam('OutputFlag', 0)
    # Create variables
    pos = NominalTrajPlanner.addVars(n + 1, len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="pos")
    pos_d = NominalTrajPlanner.addVars(n, len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="pos_derivatives")
    pos_ff = NominalTrajPlanner.addVars(len(pos_s), lb=-10, ub=10, vtype=GRB.CONTINUOUS, name="final_step_pos")
    u = NominalTrajPlanner.addVars(len(obstacles), n + 1, 4, lb=-1, ub=1, vtype=GRB.BINARY, name="u")
    # Set objective
    NominalTrajPlanner.setObjective((sum([pos_d[k, l] * pos_d[k, l] for k in range(n) for l in range(len(pos_s))]) + sum([pos_ff[l] * pos_f[l] for l in range(2)])), GRB.MINIMIZE)
    # Add constraint
    for i in range(len(pos_s)):
        NominalTrajPlanner.addConstr(pos[0, i] == pos_s[i], "c0_{}".format(i))
        NominalTrajPlanner.addConstr(pos[n, i] == pos_ff[i], "c1_{}".format(i))
        for j in range(n):
            NominalTrajPlanner.addConstr(pos_d[j, i] == pos[j + 1, i] - pos[j, i], "c2_{}_{}".format(i,j))
            NominalTrajPlanner.addConstr((pos_d[j, 0] * pos_d[j, 0]) + (pos_d[j, 1] * pos_d[j, 1]) <= v_max[0] *v_max[0] * dt * dt, "c3_{}_{}".format(i,j))
            NominalTrajPlanner.addConstr(pos_d[j, 2] * pos_d[j, 2] <= v_max[1] * v_max[1] * dt * dt, "c4_{}_{}".format(i,j))
    # Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n + 1):
            for k in range(4):
                NominalTrajPlanner.addConstr((pos[i, 1] - obstacles[n_obs][0][k]*pos[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k] ) <= - (u[n_obs, i, k]-1)*big_M, "c5_{}_{}_{}".format(n_obs,i,j))
        for i in range(n):
            NominalTrajPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1, "c6_{}_{}".format(n_obs,i))

    # Add walls constraints
    # for i in range(n):
    #     NominalTrajPlanner.addConstr(pos[i, 0] >= walls[0], "c7_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 0] <= walls[1], "c8_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 1] >= walls[2], "c9_{}".format(i))
    #     NominalTrajPlanner.addConstr(pos[i, 1] <= walls[3], "c10_{}".format(i))

    NominalTrajPlanner.optimize()


    if NominalTrajPlanner.Status == 4 or NominalTrajPlanner.Status == 3:
        NominalTrajPlanner.computeIIS()
        print "infeasible"
        for c in NominalTrajPlanner.getConstrs():
            if c.IISConstr:
                print('%s' % c.constrName)
    theta = []
    for i in range(n+1):
        theta.append(pos[i,2].x)
    return theta
