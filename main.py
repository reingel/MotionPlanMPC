# Car race along a track
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

import numpy as np
import matplotlib.pyplot as plt
plt.ion()

from casadi import *

from road import Road
from robot import Robot


N = 40
nx = 15
nu = 6
g = 9.80665
deg = 3.14159/180

road = Road()
road.add_node(-10, 0)
road.add_node(1, 0)
road.add_node(1, 0.1)
road.add_node(2, 0.1)
road.add_node(2, 0)
road.add_node(10, 0)

robot = Robot(0, 0.14, 0, np.zeros(3))

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
# robot.dq
dxh = X[6,:]
dyh = X[7,:]
dthh = X[8,:]
dtha1 = X[9,:]
dtha2 = X[10,:]
dtha3 = X[11,:]
dthw1 = X[12,:]
dthw2 = X[13,:]
dthw3 = X[14,:]
U = opti.variable(nu, N)   # control trajectory (throttle)
Ma1 = U[0,:]
Ma2 = U[1,:]
Ma3 = U[2,:]
Mw1 = U[3,:]
Mw2 = U[4,:]
Mw3 = U[5,:]
# T = opti.variable()      # final time
# dt = T/N  # length of a control interval
dt = 0.05

# ---- objective          ---------
# opti.minimize(T)  # race in minimal time
mterm = (
    (xh[-1] - 2.5)**2 + (yh[-1] - 0.14)**2 + thh[-1]**2 +
    (tha1[-1] + 60*deg)**2 + (tha2[-1] - 60*deg)**2 + (tha3[-1] - 60*deg)**2 + 
    dxh[-1]**2 + dyh[-1]**2 + dthh[-1]**2 +
    dtha1[-1]**2 + dtha2[-1]**2 + dtha3[-1]**2 + 
    dthw1[-1]**2 + dthw2[-1]**2 + dthw3[-1]**2
) / nx
lterm = sum1(sum2((U[:,0:N-1]-U[:,1:N])**2)) / (N*nu)
opti.minimize(mterm + 2 * lterm)


# ---- dynamic constraints --------
# dx/dt = f(x,u)
def f(x, u):
    thh = x[2]
    tha1 = x[3] + thh
    tha2 = x[4] + thh
    tha3 = x[5] + thh
    mr = robot.mh + 3*robot.mw
    Ia = robot.mw * robot.La**2
    Ir = robot.Ih + 3*Ia
    robot.q = x[:6]
    robot.dq = x[6:]
    Fx, Fy, Mz = robot.sum_wheel_forces(road)
    return vertcat(
        x[6], x[7], x[8], x[9], x[10], x[11],
        Fx / mr,
        Fy / mr - g,
        (Mz + robot.mw * g * (
            robot.Ls[0]*cos(thh) + robot.Hs*sin(thh) - robot.La*sin(thh + tha1) +
            robot.Ls[1]*cos(thh) + robot.Hs*sin(thh) - robot.La*sin(thh + tha2) +
            robot.Ls[2]*cos(thh) + robot.Hs*sin(thh) - robot.La*sin(thh + tha3)
            )) / Ir,
        (u[0] - robot.mw*g*robot.La*sin(thh + tha1)) / Ia,
        (u[1] - robot.mw*g*robot.La*sin(thh + tha2)) / Ia,
        (u[2] - robot.mw*g*robot.La*sin(thh + tha3)) / Ia,
        u[3] / robot.Iw,
        u[4] / robot.Iw,
        u[5] / robot.Iw,
    )


for k in range(N):  # loop over control intervals
    # Runge-Kutta 4 integration
    # k1 = f(X[:,k],         U[:,k])
    # k2 = f(X[:,k]+dt/2*k1, U[:,k])
    # k3 = f(X[:,k]+dt/2*k2, U[:,k])
    # k4 = f(X[:,k]+dt*k3,   U[:,k])
    # x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
    x_next = X[:,k] + f(X[:,k], U[:,k]) * dt
    opti.subject_to(X[:, k+1] == x_next)  # close the gaps

opti.subject_to(opti.bounded(-1, Ma1, 1))  # control is limited
opti.subject_to(opti.bounded(-1, Ma2, 1))  # control is limited
opti.subject_to(opti.bounded(-1, Ma3, 1))  # control is limited
opti.subject_to(opti.bounded(0, Mw1, 1))  # control is limited
opti.subject_to(opti.bounded(0, Mw2, 1))  # control is limited
opti.subject_to(opti.bounded(0, Mw3, 1))  # control is limited

# ---- boundary conditions --------
opti.subject_to(xh[0] == 0)
opti.subject_to(yh[0] == 0.14)
opti.subject_to(thh[0] == 0)
opti.subject_to(tha1[0] == -60*deg)
opti.subject_to(tha2[0] == 60*deg)
opti.subject_to(tha3[0] == 60*deg)
opti.subject_to(dxh[0] == 0)
opti.subject_to(dyh[0] == 0)
opti.subject_to(dthh[0] == 0)
opti.subject_to(dtha1[0] == 0)
opti.subject_to(dtha2[0] == 0)
opti.subject_to(dtha3[0] == 0)
opti.subject_to(dthw1[0] == 0)
opti.subject_to(dthw2[0] == 0)
opti.subject_to(dthw3[0] == 0)

#
opti.subject_to(opti.bounded(-90*deg, tha1,  0*deg))
opti.subject_to(opti.bounded(-90*deg, tha2, 90*deg))
opti.subject_to(opti.bounded(  0*deg, tha3, 90*deg))

# ---- misc. constraints  ----------
# opti.subject_to(T >= 0)  # Time must be positive

# ---- initial values for solver ---
# opti.set_initial(T, 10)
opti.set_initial(X, 0)
opti.set_initial(yh, 0.14)
opti.set_initial(tha1, -60*deg)
opti.set_initial(tha2, 60*deg)
opti.set_initial(tha3, 60*deg)
opti.set_initial(dthw1, 0*deg)
opti.set_initial(dthw2, 0*deg)
opti.set_initial(dthw3, 0*deg)
opti.set_initial(U, 0)
opti.set_initial(Mw1, 0)
opti.set_initial(Mw2, 0)
opti.set_initial(Mw3, 0)

# ---- solve NLP              ------
opti.solver("ipopt",{'expand':True},{'max_iter':100000})  # set numerical backend
sol = opti.solve()   # actual solve

# ---- post-processing        ------
t = np.arange(N+1)
tu = np.arange(N)

plt.figure(1)
plt.subplot(221)
plt.plot(t,sol.value(xh),t,sol.value(yh))
plt.legend(['xh','yh'])
plt.subplot(222)
plt.plot(t,sol.value(tha1)/deg,t,sol.value(tha2)/deg,t,sol.value(tha3)/deg)
plt.legend(['tha1','tha2','tha3'])
plt.subplot(223)
plt.plot(tu,sol.value(Ma1),tu,sol.value(Ma2),tu,sol.value(Ma3))
plt.legend(['Ma1','Ma2','Ma3'])
plt.subplot(224)
plt.plot(tu,sol.value(Mw1),tu,sol.value(Mw2),tu,sol.value(Mw3))
plt.legend(['Mw1','Mw2','Mw3'])

plt.figure(2)
plt.spy(sol.value(jacobian(opti.g, opti.x)))
plt.figure(3)
plt.spy(sol.value(hessian(opti.f+dot(opti.lam_g, opti.g), opti.x)[0]))

plt.show()

