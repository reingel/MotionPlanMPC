import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
# sys.path.append('../../')
import do_mpc

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle
from matplotlib import rcParams
from matplotlib.animation import FuncAnimation, FFMpegWriter, ImageMagickWriter
# Plot settings
rcParams['text.usetex'] = False
rcParams['axes.grid'] = True
rcParams['lines.linewidth'] = 2.0
rcParams['axes.labelsize'] = 'xx-large'
rcParams['xtick.labelsize'] = 'xx-large'
rcParams['ytick.labelsize'] = 'xx-large'


import time

from template_mpc import template_mpc
from template_simulator import template_simulator
from template_model import template_model

from phyunits import *
from road import Road, Wheel

""" User settings: """
show_animation = True
store_animation = False
store_results = False

# Define obstacles to avoid
road = Road()
road.add_node(-10, 0)
road.add_node(1, 0)
road.add_node(1, 0.1)
road.add_node(2, 0.1)
road.add_node(2, 0)
road.add_node(10, 0)
road.auto_connect()


# ------------------


scenario = 1  # 1 = down-down start, 2 = up-up start, both with setpoint change.

"""
Get configured do-mpc modules:
"""

model = template_model(road)
simulator = template_simulator(model)
mpc = template_mpc(model)
estimator = do_mpc.estimator.StateFeedback(model)

"""
Set initial state
"""

simulator.x0['xh'] = 0
simulator.x0['yh'] = 0.18
simulator.x0['thh'] = 0
simulator.x0['tha', 0] = 0
simulator.x0['tha', 1] = 0
simulator.x0['tha', 2] = 0

x0 = simulator.x0.cat.full()

mpc.x0 = x0
estimator.x0 = x0

mpc.set_initial_guess()

"""
Setup graphic:
"""


mpc_graphics = do_mpc.graphics.Graphics(mpc.data)

fig = plt.figure(figsize=(16,9))
plt.ion()

ax1 = plt.subplot2grid((4, 2), (0, 0), rowspan=4)
ax2 = plt.subplot2grid((4, 2), (0, 1))
ax3 = plt.subplot2grid((4, 2), (1, 1))
ax4 = plt.subplot2grid((4, 2), (2, 1))
ax5 = plt.subplot2grid((4, 2), (3, 1))

ax2.set_ylabel('hull center position')
ax3.set_ylabel('leg angles')
ax4.set_ylabel('velocity')
ax5.set_ylabel('leg angular velocities')

mpc_graphics.add_line(var_type='_x', var_name='xh', axis=ax2)
mpc_graphics.add_line(var_type='_x', var_name='yh', axis=ax2)
mpc_graphics.add_line(var_type='_x', var_name='tha', axis=ax3)
mpc_graphics.add_line(var_type='_u', var_name='vel', axis=ax4)
mpc_graphics.add_line(var_type='_u', var_name='dtha', axis=ax5)

ax1.axhline(0,color='black')

# Axis on the right.
for ax in [ax2, ax3, ax4, ax5]:
    ax.yaxis.set_label_position("right")
    ax.yaxis.tick_right()

    if ax != ax5:
        ax.xaxis.set_ticklabels([])

ax5.set_xlabel('time [s]')

bar1 = ax1.plot([],[], '-o', linewidth=5, markersize=10)
bar2 = ax1.plot([],[], '-o', linewidth=5, markersize=10)


# for obs in obstacles:
#     circle = Circle((obs['x'], obs['y']), obs['r'])
#     ax1.add_artist(circle)

# ax1.set_xlim(-1.8,1.8)
# ax1.set_ylim(-1.2,1.2)
# ax1.set_axis_off()

fig.align_ylabels()
fig.tight_layout()


"""
Run MPC main loop:
"""
time_list = []

n_steps = 250
for k in range(n_steps):
    tic = time.time()
    u0 = mpc.make_step(x0)
    toc = time.time()
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

    time_list.append(toc-tic)

    print(x0)

time_arr = np.array(time_list)
mean = np.round(np.mean(time_arr[1:])*1000)
var = np.round(np.std(time_arr[1:])*1000)
print('mean runtime:{}ms +- {}ms for MPC step'.format(mean, var))


# Store results:
if store_results:
    do_mpc.data.save_results([mpc, simulator], 'dip_mpc')

# input('Press any key to exit.')
