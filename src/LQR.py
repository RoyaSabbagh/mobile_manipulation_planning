import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
"""
Path tracking with LQR
"""

# LQR parameter
Q = np.eye(3)
R = np.eye(2)

# parameters
dt = 0.5  # time tick[s]
L = 0.5  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

class State:
    def __init__(self, x=0.0, y=0.0, phi=0.0, v=0.0):
        self.x = x
        self.y = y
        self.phi = phi
        self.v = v

def update(state, u, w, psi):
    state.x = state.x + u * math.cos(state.phi+psi) * dt
    state.y = state.y + u * math.sin(state.phi+psi) * dt
    state.phi = state.phi + w * dt
    return state

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = np.linalg.multi_dot([A.T, X, A]) - np.linalg.multi_dot([np.linalg.multi_dot([A.T, X, B]), np.linalg.inv(R + np.linalg.multi_dot([B.T, X, B])), B.T, X, A]) + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn
    return Xn

def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """
    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.linalg.multi_dot([np.linalg.inv(np.linalg.multi_dot([B.T, X, B]) + R), B.T, X, A])
    eigVals, eigVecs = la.eig(A - np.dot(B, K))
    return K, X, eigVals

def lqr_control(state, desired, psi, ind): #x_d, y_d, phi_d, pe, pth_e,u_d,w_d):
    #ind, e = calc_nearest_index(state, desired)
    A = np.zeros((3, 3))
    dt =1
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0

    B = np.zeros((3, 2))
    B[0, 0] = math.cos(state.phi+psi) * dt
    B[1, 0] = math.sin(state.phi+psi) * dt
    B[2, 1] = dt
    K, _, _ = dlqr(A, B, Q, R)
    x = np.zeros((3, 1))
    x[0, 0] = state.x - desired[ind][0]
    x[1, 0] = state.y - desired[ind][1]
    x[2, 0] = pi_2_pi(state.phi - desired[ind][2])

    ustar = -np.dot(K, x)
    u_d = [desired[ind+1][3], desired[ind+1][4], desired[ind+1][5]]
    u = [u_d[0]+ ustar[0, 0]*math.cos(state.phi+psi), u_d[1]+ ustar[0, 0]*math.sin(state.phi+psi), u_d[2]+ ustar[1, 0]]
    return u

def calc_nearest_index(state, desired):
    dx = [state.x - desired[i][0] for i in range(len(desired))]
    dy = [state.y - desired[i][1] for i in range(len(desired))]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    min_d = min(d)
    ind = d.index(min_d)
    min_d = math.sqrt(min_d)
    dxl = desired[ind][0] - state.x
    dyl = desired[ind][1] - state.y
    angle = pi_2_pi(desired[ind][2] - math.atan2(dyl, dxl))
    if angle < 0:
        min_d *= -1
    return ind, min_d

def closed_loop_prediction(x_d, y_d, phi_d, goal,u_d,w_d):
    #****************************************************************************************************************************************
    # This part should be replaced by initialization with current object state
    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05
    state = State(x=-0.0, y=-0.0, phi=0.0, v=0.0)
    #****************************************************************************************************************************************
    time = 0.0
    x = [state.x]
    y = [state.y]
    phi = [state.phi]
    t = [0.0]
    e, e_th = 0.0, 0.0

    while T >= time:
        u, w = lqr_control(state, x_d, y_d, phi_d, e, e_th,u_d,w_d)
        state = update(state, u, w)
        time = time + dt
        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            break
        x.append(state.x)
        y.append(state.y)
        phi.append(state.phi)
        t.append(time)
    return t, x, y, phi

def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0
    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
        if switch:
            direction *= -1
        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
        if switch:
            speed_profile[i] = 0.0
    speed_profile[-1] = 0.0
    return speed_profile

def main():
    #****************************************************************************************************************************************
    # This part should be replaced by input from object path optimization plan
    print("LQR steering control tracking start!!")
    x_d = [0,6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
    y_d = [0,-3.0, -5.0, 6.5, 3.0, 7.0, 10.0]
    phi_d = [0.1,0.2, 0.3, 0.4, 0.4, 0.4, 0.4]
    for i in range(1,len(x_d)):
	dx = x_d[i]-x_d[i-1]
	dy = y_d[i]-y_d[i-1]
	if dx<0:
		if dy>=0:
			u_d = [np.sqrt((x_d[i]-x_d[i-1])**2+(y_d[i]-y_d[i-1])**2) for i in range(1,len(x_d))]
		else:
			u_d = [np.sqrt((x_d[i]-x_d[i-1])**2+(y_d[i]-y_d[i-1])**2) for i in range(1,len(x_d))]
	else:
		if dy>=0:
			u_d = [np.sqrt((x_d[i]-x_d[i-1])**2+(y_d[i]-y_d[i-1])**2) for i in range(1,len(x_d))]
		else:
			u_d = [np.sqrt((x_d[i]-x_d[i-1])**2+(y_d[i]-y_d[i-1])**2) for i in range(1,len(x_d))]
    w_d = [(phi_d[i]-phi_d[i-1]) for i in range(1,len(x_d))]
    goal = [x_d[-1], y_d[-1], phi_d[-1]]

    #****************************************************************************************************************************************
    t, x, y, phi = closed_loop_prediction(x_d, y_d, phi_d, goal,u_d,w_d)
    show_animation = True

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(x_d, y_d, "xb", label="input")
        #plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.show()

if __name__ == '__main__':
	main()
