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
    Lh = 20*cm # hull length
    Hh = 6*cm # hull height
    Ls = [16*cm, 0*cm, -16*cm] # shoulder gap x from hull CG
    Hs =  4*cm # shoulder gap z from hull CG
    La =  8*cm # arm length
    Rw =  6*cm # wheel radius
    mh = 1.5
    mw = 0.2
    Ih = 1
    Iw = 0.2

    def __init__(self, xh, yh, thh, tha):
        assert len(tha) == 3
        self.q = np.array([xh, yh, thh, tha[0], tha[1], tha[2], 0, 0, 0])
        self.dq = np.zeros(9)
    
    def hull(self):
        return Box(self.ph, Robot.Lh, Robot.Hh)

    def wheel(self, i):
        assert 0 <= i <= 2
        xh = self.q[0]
        yh = self.q[1]
        thh = self.q[2]
        thai = self.q[2] + self.q[i+3]
        ph = Point2D(xh, yh)
        phs = Point2D(Robot.Ls[i], Robot.Hs).rotate(-thh)
        psw = Point2D(0,-Robot.La).rotate(-thai)
        pw = ph + phs + psw

        dxh = self.dq[0]
        dyh = self.dq[1]
        dthh = self.dq[2]
        dthai = self.dq[2] + self.dq[i+3]
        vh = Point2D(dxh, dyh)
        vhs = phs.rotate90() * -dthh
        vsw = psw.rotate90() * -dthai
        vw = vh + vhs + vsw

        dthw = self.dq[i+6]

        return Wheel(pw, Robot.Rw), vw, -dthw
    
    def wheel_spring_force(self, distance):
        fN0 = 0.1
        fN_hat = 1000
        gN_hat = 0.01
        return fN0 * (fN_hat / fN0)**(-distance / gN_hat)
    
    def friction_coef(self, slip_rate):
        v_hat = 0.01
        return np.tanh(slip_rate / v_hat)

    def wheel_forces(self, road):
        forces = []
        for i in range(self.nLeg):
            wheel, vw, dthw = self.wheel(i)
            road_height = 0 #road.height(wheel.o.x)
            pc = wheel.o + Vector2D(0, -wheel.r)
            distance = pc.y - road_height
            slip_rate = vw.x + wheel.r * dthw
            fn = self.wheel_spring_force(distance)
            ft = -self.friction_coef(slip_rate) * fn
            fw = Vector2D(ft, fn)
            forces.extend([(pc,fw)])
        return forces
    
    def sum_wheel_forces(self, road):
        forces = self.wheel_forces(road)
        Fx = 0
        Fy = 0
        Mz = 0
        xh = self.q[0]
        yh = self.q[1]
        ph = Vector2D(xh, yh)
        for pc, fw in forces:
            Fx += fw.x
            Fy += fw.y
            r = pc - ph
            Mz += -r.cross(fw)
        return Fx, Fy, Mz
    
    def distance_to_road(self, xh, yh, thh, tha, vel, dtha, dt, road):
        self.ph = Point2D(
            xh + vel*dt,
            yh,
        )
        self.thh = thh
        self.tha = tha + dtha*dt
        dph = self.repulse_vector_by_wheels_(road)
        return abs(dph)


if __name__ == '__main__':
    road = Road()
    road.add_node(-10, 0)
    road.add_node(1, 0)
    road.add_node(1, 0.1)
    road.add_node(2, 0.1)
    road.add_node(2, 0)
    road.add_node(10, 0)
    road.auto_connect()

    robot = Robot(1-0.16-0.06+0.01, 0.17, 0, np.zeros(3))
    robot.dq[8] = 1
    print(robot.wheel_forces(road))
    print(robot.sum_wheel_forces(road))