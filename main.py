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
nx = 18
nu = 6
g = 9.80665

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
xh = X[0,:]
yh = X[1,:]
thh = X[2,:]
tha1 = X[3,:]
tha2 = X[4,:]
tha3 = X[5,:]
U = opti.variable(nu, N)   # control trajectory (throttle)
vel = U[0,:]
dtha1 = U[1,:]
dtha2 = U[2,:]
dtha3 = U[3,:]
T = opti.variable()      # final time
dt = T/N  # length of a control interval

# ---- objective          ---------
opti.minimize(T)  # race in minimal time


# ---- dynamic constraints --------
# dx/dt = f(x,u)
def f(x, u):
    thh = x[2]
    tha1 = x[3]
    tha2 = x[4]
    tha3 = x[5]
    mr = robot.mh + 3*robot.mw
    Ia = robot.mw * robot.La**2
    robot.q = x[:9]
    robot.dq = x[9:]
    Fx, Fy, Mz = robot.sum_wheel_forces(road)
    return vertcat(
        x[9], x[10], x[11], x[12], x[13], x[14], x[15], x[16], x[17],
        Fx / mr,
        Fy / mr - g,
        (Mz - robot.mw * g * (
            -robot.Ls[0]*cos(thh) - robot.Hs*sin(thh) + robot.La*sin(thh + tha1)
            -robot.Ls[1]*cos(thh) - robot.Hs*sin(thh) + robot.La*sin(thh + tha2)
            -robot.Ls[2]*cos(thh) - robot.Hs*sin(thh) + robot.La*sin(thh + tha3)
            )) / (robot.Ih + 3*Ia),
        (U[0] - robot.mw*g*robot.La*sin(thh + tha1)) / Ia,
        (U[1] - robot.mw*g*robot.La*sin(thh + tha2)) / Ia,
        (U[2] - robot.mw*g*robot.La*sin(thh + tha3)) / Ia,
        U[3] / robot.Iw,
        U[4] / robot.Iw,
        U[5] / robot.Iw,
    )


for k in range(N):  # loop over control intervals
    # Runge-Kutta 4 integration
    # k1 = f(X[:, k],         U[:, k])
    # k2 = f(X[:, k]+dt/2*k1, U[:, k])
    # k3 = f(X[:, k]+dt/2*k2, U[:, k])
    # k4 = f(X[:, k]+dt*k3,   U[:, k])
    # x_next = X[:, k] + dt/6*(k1+2*k2+2*k3+k4)
    x_next = X[:,k] + f(X[:,k], U[:,k]) * dt
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
opti.set_initial(T, 10)

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
