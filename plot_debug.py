
t = np.arange(N)
t1 = np.arange(N-1)

plt.figure(1)
plt.clf()

plt.subplot(531)
plt.plot(t,opti.debug.value(xh))
plt.legend(['xh'])
plt.subplot(532)
plt.plot(t,opti.debug.value(yh))
plt.legend(['yh'])
plt.subplot(533)
plt.plot(t,opti.debug.value(thh)/deg)
plt.legend(['thh'])
plt.subplot(537)
plt.plot(t,opti.debug.value(tha3)/deg)
plt.legend(['tha3'])
plt.subplot(538)
plt.plot(t,opti.debug.value(tha2)/deg)
plt.legend(['tha2'])
plt.subplot(539)
plt.plot(t,opti.debug.value(tha1)/deg)
plt.legend(['tha1'])

plt.subplot(534)
plt.plot(t1,opti.debug.value(dxh))
plt.legend(['dxh'])

# k = opti.debug.value(K)
# d = opti.debug.value(D)
x = np.linspace(-0.5,2,100)
road_y = obs_height*sigmoid(100*(x-1))
# wheel_y = wheel_path(x, k, d)

plt.subplot(5,3,13)
plt.plot(x,road_y,'k',opti.debug.value(wheel3x),opti.debug.value(wheel3y))
plt.legend(['wheel3_path'])
plt.subplot(5,3,14)
plt.plot(x,road_y,'k',opti.debug.value(wheel2x),opti.debug.value(wheel2y))
plt.legend(['wheel2_path'])
plt.subplot(5,3,15)
plt.plot(x,road_y,'k',opti.debug.value(wheel1x),opti.debug.value(wheel1y))
plt.legend(['wheel1_path'])

# plt.subplot(5,3,13)
# plt.plot(x,road_y,'k',x,wheel_y,'r',opti.debug.value(wheel3x),opti.debug.value(wheel3y))
# plt.legend(['wheel3_path','wheel3y'])
# plt.subplot(5,3,14)
# plt.plot(x,road_y,'k',x,wheel_y,'r',opti.debug.value(wheel2x),opti.debug.value(wheel2y))
# plt.legend(['wheel2_path','wheel2y'])
# plt.subplot(5,3,15)
# plt.plot(x,road_y,'k',x,wheel_y,'r',opti.debug.value(wheel1x),opti.debug.value(wheel1y))
# plt.legend(['wheel1_path','wheel1y'])

plt.show()

# plt.subplot(5,3,10)
# plt.plot(t,opti.debug.value(hull_gap))
# plt.legend(['hull_gap'])
plt.subplot(5,3,11)
plt.plot(t,opti.debug.value(Fy))
plt.legend(['Fy'])
plt.subplot(5,3,12)
plt.plot(t,opti.debug.value(Mz))
plt.legend(['Mz'])




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
# road
def draw_road():
    # k = opti.debug.value(K)
    # d = opti.debug.value(D)
    x = np.linspace(-0.5,3,100)
    road_y = obs_height*sigmoid(100*(x-1))
    # wheel_y = wheel_path(x, k, d)
    # plt.plot(x,road_y,'k',x,wheel_y,'b')
    plt.plot(x,road_y,'k')

fig = plt.figure(11, frameon=False)

# robot
def update(i):
    plt.figure(11)
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
    plt.plot([arm_2_x, wheel_2_x], [arm_2_y, wheel_2_y])
    plt.plot([arm_3_x, wheel_3_x], [arm_3_y, wheel_3_y])
    draw_circle(wheel_1_x,wheel_1_y,Rw)
    draw_circle(wheel_2_x,wheel_2_y,Rw)
    draw_circle(wheel_3_x,wheel_3_y,Rw)


ani = FuncAnimation(fig, update, frames=np.arange(N), interval=100)

plt.show()

