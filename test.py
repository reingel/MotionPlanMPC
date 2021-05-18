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

# from template_mpc import template_mpc
# from template_simulator import template_simulator
# from template_model import template_model

from road import *
from phyunits import *

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

pw = WheelPosition(0.99, 0.06, 0.06)
ph = HullPose(0, 0.1, 10*deg, 0.2, 0.06)

w_distances = road.distance_vectors_with_circle(pw)
w_d_min = road.repulse_vector_of_circle(pw)

# h_distances = road.get_hull_distances(ph)
# h_d_min = road.get_hull_min_distance(ph)

print(np.round(w_distances, 4))
print(np.round(w_d_min, 4))
# print(np.round(h_distances, 4))
# print(np.round(h_d_min, 4))
