# Car race along a track
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP

from pylab import plot, subplot, step, figure, legend, show, spy
import numpy as np
from casadi import *

from road import Road
from robot import Robot


N = 100  # number of control intervals
nx = 6
nu = 4

road = Road()
road.add_node(-10, 0)
road.add_node(1, 0)
road.add_node(1, 0.1)
road.add_node(2, 0.1)
road.add_node(2, 0)
road.add_node(10, 0)
road.auto_connect()

robot = Robot(0, 0.18, 0, np.zeros(3))

opti = Opti()  # Optimization problem

# ---- decision variables ---------
X = opti.variable(nx, N+1)  # state trajectory
xh = X[0, :]
yh = X[1, :]
thh = X[2, :]
tha1 = X[3, :]
tha2 = X[4, :]
tha3 = X[5, :]

U = opti.variable(nu, N)   # control trajectory (throttle)
vel = U[0, :]
dtha1 = U[1,:]
dtha2 = U[2,:]
dtha3 = U[3,:]

T = opti.variable()      # final time
dt = T/N  # length of a control interval

# ---- objective          ---------
opti.minimize(T)  # race in minimal time


# ---- dynamic constraints --------
# f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
def f(x, u):
    xh = x[0]
    yh = x[1]
    thh = x[2]
    tha = vertcat(x[3], x[4], x[5])
    vel = u[0]
    dtha = vertcat(u[1], u[2], u[3])
    xh1, yh1, thh1, tha1 = robot.step(xh, yh, thh, tha, vel, dtha, dt, road)
    dx1 = xh1
    dx2 = yh1
    dx3 = thh1
    dx4 = tha1[0]
    dx5 = tha1[1]
    dx6 = tha1[2]
    return vertcat(dx1, dx2, dx3, dx4, dx5, dx6)


for k in range(N):  # loop over control intervals
    # Runge-Kutta 4 integration
    # k1 = f(X[:, k],         U[:, k])
    # k2 = f(X[:, k]+dt/2*k1, U[:, k])
    # k3 = f(X[:, k]+dt/2*k2, U[:, k])
    # k4 = f(X[:, k]+dt*k3,   U[:, k])
    # x_next = X[:, k] + dt/6*(k1+2*k2+2*k3+k4)
    x_next = f(X[:,k], U[:,k])
    opti.subject_to(X[:, k+1] == x_next)  # close the gaps

# ---- path constraints -----------


# opti.subject_to(speed <= limit(pos))   # track speed limit

opti.subject_to(opti.bounded(-0.1, U, 0.1))  # control is limited

# ---- boundary conditions --------
opti.subject_to(xh[0] == 0)
opti.subject_to(xh[-1] == 1.5)
opti.subject_to(yh[0] == 0.18)
opti.subject_to(yh[-1] == 0.18)
opti.subject_to(thh[0] == 0)
opti.subject_to(thh[-1] == 0)

# ---- misc. constraints  ----------
opti.subject_to(T >= 0)  # Time must be positive

# ---- initial values for solver ---
opti.set_initial(T, 1)

# ---- solve NLP              ------
opti.solver("ipopt")  # set numerical backend
sol = opti.solve()   # actual solve

# ---- post-processing        ------
t = np.arange(N+1)

figure(1)
subplot(221)
plot(t,sol.value(xh),t,sol.value(yh))
legend('xh','yh')
subplot(222)
plot(t,sol.value(tha1),t,sol.value(tha2),t,sol.value(tha3))
legend('tha1','tha2','tha3')
subplot(223)
plot(t,sol.value(vel))
legend('vel')
subplot(224)
plot(t,sol.value(dtha1),t,sol.value(dtha2),t,sol.value(dtha3))
legend('dtha1','dtha2','dtha3')

# figure()
# spy(sol.value(jacobian(opti.g, opti.x)))
# figure()
# spy(sol.value(hessian(opti.f+dot(opti.lam_g, opti.g), opti.x)[0]))

show()
