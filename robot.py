from Vector2D import Vector2D
from Geometry import Point2D
from copy import copy
import numpy as np
from math import sqrt, sin, cos, atan2, floor
from phyunits import *
from Geometry import Point2D, Line2D, Circle, Box
from road import Road, Wheel


class Robot(object):
    nLeg = 3 # no. of legs
    Lh = 40*cm # hull length
    Hh = 12*cm # hull height
    Ls = [16*cm, 0*cm, -16*cm] # shoulder gap x from hull CG
    Hs = -4*cm # shoulder gap z from hull CG
    La =  8*cm # arm length
    Rw =  6*cm # wheel radius

    def __init__(self, xh, yh, thh, tha):
        assert len(tha) == 3
        self.ph = Point2D(xh, yh)
        self.thh = thh
        self.tha = tha
    
    def hull(self):
        return Box(self.ph, Robot.Lh, Robot.Hh)

    def wheel(self, i):
        assert 0 <= i <= 2
        pw = self.ph + Point2D(Robot.Ls[i], Robot.Hs) + Robot.La*Point2D(-np.sin(self.tha[i]), -np.cos(self.tha[i]))
        return Wheel(pw, Robot.Rw)
    
    def repulse_vector_by_wheels_(self, road):
        ph0 = copy(self.ph)
        for i in range(self.nLeg):
            w = self.wheel(i)
            vr = road.repulse_vector_of_wheel(w)
            self.ph += vr
        return self.ph - ph0
        

    def step(self, xh, yh, thh, tha, vel, dtha, dt, road):
        self.ph = Point2D(
            xh + vel*dt,
            yh,
        )
        self.thh = thh
        self.tha = tha + dtha*dt
        self.repulse_vector_by_wheels_(road)
        return self.ph.x, self.ph.y, self.thh, self.tha


if __name__ == '__main__':
    road = Road()
    road.add_node(-10, 0)
    road.add_node(1, 0)
    road.add_node(1, 0.1)
    road.add_node(2, 0.1)
    road.add_node(2, 0)
    road.add_node(10, 0)
    road.auto_connect()

    robot = Robot(-0.16-0.06 + 0.01, 0.18, 0, np.zeros(3))
    print(robot.hull())
    for i in range(robot.nLeg):
        print(robot.wheel(i))
    print(robot.repulse_vector_by_wheels_(road))

    x0 = [0, 0.18, 0, np.zeros(3)]
    u = [0.1, np.array([0.1, 0, 0])]
    dt = 0.1
    print(x0)
    xh1, yh1, thh1, tha1 = robot.step(*x0, *u, dt, road)
    print(xh1, yh1, thh1, tha1)