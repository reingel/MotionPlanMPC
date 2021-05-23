# Car race along a track
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
plt.ion()

from casadi import *

from road import Road
from robot import Robot


g = 9.80665
deg = 3.14159/180

nx = 6

N = 40
dt = 0.05

obs_height = 0.08

# robot = Robot(0, 0.14, 0, np.zeros(3))
Lh = Robot.Lh
Hh = Robot.Hh
Ls = Robot.Ls
Hs = Robot.Hs
La = Robot.La
Rw = Robot.Rw
mr = Robot.mh + 3*Robot.mw
Wr = mr*g # Robot weight
Ia = Robot.mw*Robot.La**2

opti = Opti()  # Optimization problem

# ---- decision variables ---------
X = opti.variable(nx, N)
# U = opti.variable(nx, N)
K = opti.variable()
D = opti.variable()

xh = X[0,:]
yh = X[1,:]
thh = X[2,:]
tha1 = X[3,:]
tha2 = X[4,:]
tha3 = X[5,:]

# object function
energy = sqrt(sum2(
    (xh[1:N] - xh[0:N-1])**2 +
    (yh[1:N] - yh[0:N-1])**2 +
    (thh[1:N] - thh[0:N-1])**2 +
    (tha1[1:N] - tha1[0:N-1])**2 +
    (tha2[1:N] - tha2[0:N-1])**2 +
    (tha3[1:N] - tha3[0:N-1])**2
) / (nx*N))
opti.minimize(energy)

# ---- dynamic constraints --------
# dx/dt = f(x,u)
# def f(x, u):
#     return u

# for k in range(N):
#     x_next = X[:,k] + f(X[:,k], U[:,k]) * dt
#     opti.subject_to(X[:,k+1] == x_next)

# road profile
def sigmoid(x):
    return 1 / (1 + exp(-x))

def wheel_path(x,k,d):
    return obs_height*sigmoid(k*(x - d))

# approaching angle
aprc_angle = obs_height*K*exp(-K*D) / (1 + exp(-K*D))**2
opti.subject_to(aprc_angle < 5*deg)

# follow path
wheel1x = xh + Ls[0]*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha1)
wheel2x = xh + Ls[1]*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha2)
wheel3x = xh + Ls[2]*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha3)
wheel1y = yh + Ls[0]*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha1)
wheel2y = yh + Ls[1]*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha2)
wheel3y = yh + Ls[2]*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha3)

opti.subject_to(wheel1y == wheel_path(wheel1x,K,D) + Rw)
opti.subject_to(wheel2y == wheel_path(wheel2x,K,D) + Rw)
opti.subject_to(wheel3y == wheel_path(wheel3x,K,D) + Rw)

# contact constraints
# hull_gap = fmin(
#     fmin(
#         yh + Lh*sin(-thh) + Hh*cos(-thh), # uf
#         yh - Lh*sin(-thh) + Hh*cos(-thh)  # ur
#     ),
#     fmin(
#         yh + Lh*sin(-thh) - Hh*cos(-thh), # df
#         yh - Lh*sin(-thh) - Hh*cos(-thh)  # dr
#     )
# ) - road_height(xh)
# opti.subject_to(hull_gap > 0)

# wheel1_gap = wheel1y - Rw - road_height(xh)
# wheel2_gap = wheel2y - Rw - road_height(xh)
# wheel3_gap = wheel3y - Rw - road_height(xh)
# opti.subject_to(wheel1_gap > -Rw/10)
# opti.subject_to(wheel2_gap > -Rw/10)
# opti.subject_to(wheel3_gap > -Rw/10)

# quasi-static equilibrium constraints
# wheel1_x_wrt_hull = wheel1x - xh
# wheel2_x_wrt_hull = wheel2x - xh
# wheel3_x_wrt_hull = wheel3x - xh
# force_wh1 = robot.wheel_spring_force(wheel1_gap)
# force_wh2 = robot.wheel_spring_force(wheel2_gap)
# force_wh3 = robot.wheel_spring_force(wheel3_gap)
# Fy = force_wh1 + force_wh2 + force_wh3
# Mz = -(wheel1_x_wrt_hull*force_wh1 + wheel2_x_wrt_hull*force_wh2 + wheel3_x_wrt_hull*force_wh3)
# opti.subject_to(Fy == Wr)
# opti.subject_to(Mz == 0)

# force/moment limits
# for k in range(N-1):
#     opti.subject_to((vxh[k+1] - vxh[k])**2 < 0.1)
#     opti.subject_to((vyh[k+1] - vyh[k])**2 < 0.1)
#     opti.subject_to((dthh[k+1] - dthh[k])**2 < 0.1)
#     opti.subject_to((dtha1[k+1] - dtha1[k])**2 < 0.1)
#     opti.subject_to((dtha2[k+1] - dtha2[k])**2 < 0.1)
#     opti.subject_to((dtha3[k+1] - dtha3[k])**2 < 0.1)

# state constraints
opti.subject_to(opti.bounded(-0.5, xh, 3))
opti.subject_to(opti.bounded(0, yh, 10))
opti.subject_to(opti.bounded(-45*deg, thh, 45*deg))
opti.subject_to(opti.bounded(-90*deg, tha1, 0*deg))
opti.subject_to(opti.bounded(-90*deg, tha2, 90*deg))
opti.subject_to(opti.bounded(0*deg, tha3, 90*deg))

# control input constraits
# opti.subject_to(opti.bounded(0, vxh, 1))
# opti.subject_to(opti.bounded(-0.5, vyh, 0.5))
# opti.subject_to(opti.bounded(-10*deg, dthh,  10*deg))
# opti.subject_to(opti.bounded(-10*deg, dtha1, 10*deg))
# opti.subject_to(opti.bounded(-10*deg, dtha2, 10*deg))
# opti.subject_to(opti.bounded(-10*deg, dtha3, 10*deg))

# ---- boundary conditions --------
# opti.subject_to(xh[0] == 0)
# opti.subject_to(yh[0] == 0.14)
# opti.subject_to(thh[0] == 0)
# opti.subject_to(tha1[0] == -60*deg)
# opti.subject_to(tha2[0] == 60*deg)
# opti.subject_to(tha3[0] == 60*deg)

# ---- initial values for solver ---
opti.set_initial(xh, 0)
opti.set_initial(yh, 0.14)
opti.set_initial(thh, 0*deg)
opti.set_initial(tha1, -60*deg)
opti.set_initial(tha2, 60*deg)
opti.set_initial(tha3, 60*deg)
# opti.set_initial(U, 0)
opti.set_initial(K, 1)
opti.set_initial(D, 0.5)

# ---- solve NLP              ------
opti.solver("ipopt",{'expand':True},{'max_iter':10000})
sol = opti.solve()   # actual solve

# ---- post-processing        ------

