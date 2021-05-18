import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
# sys.path.append('../../')
import do_mpc

from phyunits import *

def template_model(road):
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # parameters
    dt = 0.1*sec # delta time
    Lh = 40*cm # hull length
    Hh = 12*cm # hull height
    Ls = 16*cm # shoulder gap x from hull CG
    Hs =  4*cm # shoulder gap z from hull CG
    La =  8*cm # arm length
    Rw =  6*cm # wheel radius

    # States struct:
    xh = model.set_variable('_x',  'xh')
    yh = model.set_variable('_x',  'yh')
    thh = model.set_variable('_x',  'thh')
    tha = model.set_variable('_x',  'tha', (3,1))

    # Input struct (optimization variables):
    vel = model.set_variable('_u',  'vel')
    dtha = model.set_variable('_u',  'dtha', (3,1))

    # Find next states

    # Differential equations
    model.set_rhs('xh', vel*dt)
    model.set_rhs('yh', )
    model.set_rhs('thh', )
    model.set_rhs('tha', dtha*dt)

    # Calculations to avoid obstacles:

    # Coordinates of the nodes:
    node0_x = model.x['pos']
    node0_y = np.array([0])

    node1_x = node0_x+L1*sin(model.x['theta',0])
    node1_y = node0_y+L1*cos(model.x['theta',0])

    node2_x = node1_x+L2*sin(model.x['theta',1])
    node2_y = node1_y+L2*cos(model.x['theta',1])

    obstacle_distance = []

    for obs in obstacles:
        d0 = sqrt((node0_x-obs['x'])**2+(node0_y-obs['y'])**2)-obs['r']*1.05
        d1 = sqrt((node1_x-obs['x'])**2+(node1_y-obs['y'])**2)-obs['r']*1.05
        d2 = sqrt((node2_x-obs['x'])**2+(node2_y-obs['y'])**2)-obs['r']*1.05
        obstacle_distance.extend([d0, d1, d2])


    model.set_expression('obstacle_distance',vertcat(*obstacle_distance))
    model.set_expression('tvp', pos_set)


    # Build the model
    model.setup()

    return model
