


Lh = robot.Lh
Hh = robot.Hh
La = robot.La
Rw = robot.Rw
Ls = robot.Ls
Hs = robot.Hs

def draw_circle(x,y,r):
    xs = []
    ys = []
    for th in np.linspace(0,2*np.pi,16):
        xs.append(x + r*cos(th))
        ys.append(y + r*sin(th))
    xs.append(x + r)
    ys.append(y)
    plt.plot(xs,ys)

# ---- animate ----
t = np.arange(N+1)

plt.figure(11)
fig = plt.figure(11, frameon=False)
# ax.set_xlim(-0.5, 3)
# ax.set_ylim(-0.5,0.5)
# ax.axis('off')
# plt.clf()
# ax.set_aspect(1)

# road
def draw_road():
    road_x = np.linspace(-0.5,3,100)
    # road_y = road_amp*np.sin(2*np.pi*road_freq*road_x)
    road_y = road_amp*np.tanh(100*(road_x - 1))
    plt.plot(road_x,road_y,'k')

# robot
def update(i):
    plt.clf()
    plt.axis('equal')
    draw_road()
    xhi = opti.debug.value(xh[i])
    yhi = opti.debug.value(yh[i])
    thhi = opti.debug.value(thh[i])
    tha1i = opti.debug.value(tha1[i])
    tha2i = opti.debug.value(tha2[i])
    tha3i = opti.debug.value(tha3[i])

    # hull
    hull_x_ur = xhi - Lh*cos(-thhi) - Hh*sin(-thhi)
    hull_x_uf = xhi + Lh*cos(-thhi) - Hh*sin(-thhi)
    hull_x_df = xhi + Lh*cos(-thhi) + Hh*sin(-thhi)
    hull_x_dr = xhi - Lh*cos(-thhi) + Hh*sin(-thhi)
    hull_y_ur = yhi - Lh*sin(-thhi) + Hh*cos(-thhi)
    hull_y_uf = yhi + Lh*sin(-thhi) + Hh*cos(-thhi)
    hull_y_df = yhi + Lh*sin(-thhi) - Hh*cos(-thhi)
    hull_y_dr = yhi - Lh*sin(-thhi) - Hh*cos(-thhi)
    hull_x = np.array([hull_x_ur, hull_x_uf, hull_x_df, hull_x_dr, hull_x_ur])
    hull_y = np.array([hull_y_ur, hull_y_uf, hull_y_df, hull_y_dr, hull_y_ur])
    plt.plot(hull_x, hull_y)

    # arm
    arm_1_x = xhi + Ls[0]*cos(-thhi) + Hs*sin(-thhi)
    arm_2_x = xhi + Ls[1]*cos(-thhi) + Hs*sin(-thhi)
    arm_3_x = xhi + Ls[2]*cos(-thhi) + Hs*sin(-thhi)
    arm_1_y = yhi + Ls[0]*sin(-thhi) - Hs*cos(-thhi)
    arm_2_y = yhi + Ls[1]*sin(-thhi) - Hs*cos(-thhi)
    arm_3_y = yhi + Ls[2]*sin(-thhi) - Hs*cos(-thhi)
    wheel_1_x = arm_1_x + La*sin(-thhi-tha1i)
    wheel_2_x = arm_2_x + La*sin(-thhi-tha2i)
    wheel_3_x = arm_3_x + La*sin(-thhi-tha3i)
    wheel_1_y = arm_1_y - La*cos(-thhi-tha1i)
    wheel_2_y = arm_2_y - La*cos(-thhi-tha2i)
    wheel_3_y = arm_3_y - La*cos(-thhi-tha3i)
    plt.plot([arm_1_x, wheel_1_x], [arm_1_y, wheel_1_y])
    plt.plot([arm_2_x, wheel_2_x], [arm_1_y, wheel_2_y])
    plt.plot([arm_3_x, wheel_3_x], [arm_1_y, wheel_3_y])
    draw_circle(wheel_1_x,wheel_1_y,Rw)
    draw_circle(wheel_2_x,wheel_2_y,Rw)
    draw_circle(wheel_3_x,wheel_3_y,Rw)


ani = FuncAnimation(fig, update, frames=np.arange(N+1), interval=100)
plt.show()
