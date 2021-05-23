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
nu = 6

N = 40
dt = 0.05

# xh_tgt = 2.5

road_amp = 0.08
road_freq = 1

robot = Robot(0, 0.14, 0, np.zeros(3))
mr = robot.mh + 3*robot.mw
Wr = mr*g # robot weight
Ia = robot.mw*robot.La**2

opti = Opti()  # Optimization problem

# ---- decision variables ---------
X = opti.variable(nx, N+1)  # state trajectory
# robot.q
xh = X[0,:]
yh = X[1,:]
thh = X[2,:]
tha1 = X[3,:]
tha2 = X[4,:]
tha3 = X[5,:]
U = opti.variable(nu, N)   # control trajectory (throttle)
vxh = U[0,:]
vyh = U[1,:]
dthh = U[2,:]
dtha1 = U[3,:]
dtha2 = U[4,:]
dtha3 = U[5,:]
# T = opti.variable()      # final time
# dt = T/N  # length of a control interval

# road profile
def road_height(x):
    # return road_amp*sin(2*3.1415*road_freq*x)
    return road_amp*(tanh(10*(x-1)) + 1)/2

# road_height_tgt = road_height(xh_tgt)

# ---- objective          ---------
# mterm = (
#     (xh[-1] - xh_tgt)**2 +
#     (yh[-1] - (road_height_tgt + 0.14))**2 +
#     robot.Lh**2 * thh[-1]**2 +
#     robot.La**2 * (
#         (tha1[-1] + 60*deg)**2 +
#         (tha2[-1] - 60*deg)**2 +
#         (tha3[-1] - 60*deg)**2
#     )
# ) / nx
mterm = -xh[-1]
# lterm = sum1(sum2(
#     (vxh[1:N] - vxh[0:N-1])**2 +
#     (vyh[1:N] - vyh[0:N-1])**2 +
#     (dthh[1:N] - dthh[0:N-1])**2 +
#     (dtha1[1:N] - dtha1[0:N-1])**2 +
#     (dtha2[1:N] - dtha2[0:N-1])**2 +
#     (dtha3[1:N] - dtha3[0:N-1])**2
# )) / (N*nu)
# w = 1
# opti.minimize(w*mterm + lterm)
opti.minimize(mterm)


# ---- dynamic constraints --------
# dx/dt = f(x,u)
def f(x, u):
    return u


for k in range(N):  # loop over control intervals
    # Runge-Kutta 4 integration
    # k1 = f(X[:,k],         U[:,k])
    # k2 = f(X[:,k]+dt/2*k1, U[:,k])
    # k3 = f(X[:,k]+dt/2*k2, U[:,k])
    # k4 = f(X[:,k]+dt*k3,   U[:,k])
    # x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4)
    # Euler integration
    x_next = X[:,k] + f(X[:,k], U[:,k]) * dt
    opti.subject_to(X[:,k+1] == x_next)

# contact constraints
hull_gap = fmin(
    fmin(
        yh + robot.Lh*sin(-thh) + robot.Hh*cos(-thh), # uf
        yh - robot.Lh*sin(-thh) + robot.Hh*cos(-thh)  # ur
    ),
    fmin(
        yh + robot.Lh*sin(-thh) - robot.Hh*cos(-thh), # df
        yh - robot.Lh*sin(-thh) - robot.Hh*cos(-thh)  # dr
    )
) - road_height(xh)
opti.subject_to(hull_gap > 0)

wheel1x = xh + robot.Ls[0]*cos(-thh) - robot.Hs*sin(-thh) + robot.La*sin(-thh-tha1)
wheel2x = xh + robot.Ls[1]*cos(-thh) - robot.Hs*sin(-thh) + robot.La*sin(-thh-tha2)
wheel3x = xh + robot.Ls[2]*cos(-thh) - robot.Hs*sin(-thh) + robot.La*sin(-thh-tha3)
wheel1y = yh + robot.Ls[0]*sin(-thh) - robot.Hs*cos(-thh) - robot.La*cos(-thh-tha1)
wheel2y = yh + robot.Ls[1]*sin(-thh) - robot.Hs*cos(-thh) - robot.La*cos(-thh-tha2)
wheel3y = yh + robot.Ls[2]*sin(-thh) - robot.Hs*cos(-thh) - robot.La*cos(-thh-tha3)

wheel1_gap = wheel1y - robot.Rw - road_height(xh)
wheel2_gap = wheel2y - robot.Rw - road_height(xh)
wheel3_gap = wheel3y - robot.Rw - road_height(xh)
# opti.subject_to(wheel1_gap > -robot.Rw/10)
# opti.subject_to(wheel2_gap > -robot.Rw/10)
# opti.subject_to(wheel3_gap > -robot.Rw/10)

# quasi-static equilibrium constraints
wheel1_x_wrt_hull = wheel1x - xh
wheel2_x_wrt_hull = wheel2x - xh
wheel3_x_wrt_hull = wheel3x - xh
force_wh1 = robot.wheel_spring_force(wheel1_gap)
force_wh2 = robot.wheel_spring_force(wheel2_gap)
force_wh3 = robot.wheel_spring_force(wheel3_gap)
Fy = force_wh1 + force_wh2 + force_wh3
Mz = -(wheel1_x_wrt_hull*force_wh1 + wheel2_x_wrt_hull*force_wh2 + wheel3_x_wrt_hull*force_wh3)
opti.subject_to(Fy == Wr)
opti.subject_to(Mz == 0)

# force/moment limits
for k in range(N-1):
    opti.subject_to((vxh[k+1] - vxh[k])**2 < 0.1)
    opti.subject_to((vyh[k+1] - vyh[k])**2 < 0.1)
    opti.subject_to((dthh[k+1] - dthh[k])**2 < 0.1)
    opti.subject_to((dtha1[k+1] - dtha1[k])**2 < 0.1)
    opti.subject_to((dtha2[k+1] - dtha2[k])**2 < 0.1)
    opti.subject_to((dtha3[k+1] - dtha3[k])**2 < 0.1)

# state constraints
# opti.subject_to(xh > -0.5)
# opti.subject_to(opti.bounded(0.14 - 0.5, yh, 0.14 + 0.5))
opti.subject_to(opti.bounded(-45*deg, thh, 45*deg))
opti.subject_to(opti.bounded(-90*deg, tha1, 0*deg))
opti.subject_to(opti.bounded(-90*deg, tha2, 90*deg))
opti.subject_to(opti.bounded(0*deg, tha3, 90*deg))

# control input constraits
opti.subject_to(opti.bounded(0, vxh, 1))
opti.subject_to(opti.bounded(-0.5, vyh, 0.5))
opti.subject_to(opti.bounded(-10*deg, dthh,  10*deg))
opti.subject_to(opti.bounded(-10*deg, dtha1, 10*deg))
opti.subject_to(opti.bounded(-10*deg, dtha2, 10*deg))
opti.subject_to(opti.bounded(-10*deg, dtha3, 10*deg))

# ---- boundary conditions --------
opti.subject_to(xh[0] == 0)
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
opti.set_initial(U, 0)

# ---- solve NLP              ------
opti.solver("ipopt",{'expand':True},{'max_iter':10000})
sol = opti.solve()   # actual solve

# ---- post-processing        ------

