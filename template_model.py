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
    xh1, yh1, thh1, tha1 = robot.step(model.x['xh'], model.x['yh'], model.x['thh'], model.x['tha'], model.u['vel'], model.u['dtha'], dt, road)

    # Differential equations
    model.set_rhs('xh', xh1)
    model.set_rhs('yh', yh1)
    model.set_rhs('thh', thh1)
    model.set_rhs('tha', tha1)

    # Calculations to avoid obstacles:
    d = robot.distance_to_road(model.x['xh'], model.x['yh'], model.x['thh'], model.x['tha'], model.u['vel'], model.u['dtha'], dt, road)

    model.set_expression('distance_to_road', d)

    # Build the model
    model.setup()

    return model
