import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
# sys.path.append('../../')
import do_mpc

from phyunits import *
from robot import Robot
from road import Road, Wheel

robot = Robot(0, 0.18, 0, np.zeros(3))

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

    # States struct:
    xh = model.set_variable('_x',  'xh')
    yh = model.set_variable('_x',  'yh')
    thh = model.set_variable('_x',  'thh')
    tha = model.set_variable('_x',  'tha', (3,1))

    # Input struct (optimization variables):
    vel = model.set_variable('_u',  'vel')
    dtha = model.set_variable('_u',  'dtha', (3,1))

    # Find next states
    xh1, yh1, thh1, tha1 = robot.step(xh, yh, thh, tha, vel, dtha, dt, road)

    # Differential equations
    model.set_rhs('xh', xh1)
    model.set_rhs('yh', yh1)
    model.set_rhs('thh', thh1)
    model.set_rhs('tha', tha1)

    # Calculations to avoid obstacles:

    # obstacle_distance = []

    # for obs in obstacles:
    #     d0 = sqrt((node0_x-obs['x'])**2+(node0_y-obs['y'])**2)-obs['r']*1.05
    #     d1 = sqrt((node1_x-obs['x'])**2+(node1_y-obs['y'])**2)-obs['r']*1.05
    #     d2 = sqrt((node2_x-obs['x'])**2+(node2_y-obs['y'])**2)-obs['r']*1.05
    #     obstacle_distance.extend([d0, d1, d2])


    # model.set_expression('obstacle_distance',vertcat(*obstacle_distance))
    # model.set_expression('tvp', pos_set)


    # Build the model
    model.setup()

    return model
