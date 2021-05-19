import numpy as np
from math import sqrt, sin, cos, atan2, floor
from Vector2D import Vector2D

class Line2D(object):
    pass


class Point2D(Vector2D):
    def __add__(self, other):
        if not isinstance(other, Vector2D):
            raise TypeError('can only add Vector2D object')
        return Point2D(self.x + other.x, self.y + other.y)
    
    def location_wrt(self, l:Line2D):
        p = self - l.p1
        t = l.tangent_vector()
        angle = p.angle_wrt(t)
        lp = p.norm()
        ll = l.len()
        x = lp*np.cos(angle)
        y = lp*np.sin(angle)
        lmr = np.floor(x / ll)
        ud = np.sign(y)
        return x, y, lmr, ud
    
    def distance_to_line(self, l: Line2D):
        x, y, lmr, ud = self.location_wrt(l)
        return y
    
    def hcross(self, other): # homogeneous cross product
        p1 = Point2D(self.x, self.y, 1)
        p2 = Point2D(other.x, other.y, 1)
        c = p1.cross(p2)
        if c.z == 0:
            return None # point at infinity
        else:
            return Point2D(c.x, c.y) / c.z


class Box(object):
    def __init__(self, cg: Point2D, L: float, H: float): # L, H: half length/height
        self.cg = cg
        self.L = L
        self.H = H
    
    def __repr__(self):
        return f'{self.cg}:{self.L}:{self.H}'
    
    # def location_wrt(self, l: Line2D):
    #     return self.o.location_wrt(l)
    
    # box to point
    def direction_to_point(self, p: Point2D):
        return (p - self.o).unit()
    
    def distance_to_point(self, p: Point2D):
        return self.o.distance_to(p) - self.r
    
    def contact_vector_to_point(self, p: Point2D): # vector in order to make this circle contact with a point
        return self.distance_to_point(p) * self.direction_to_point(p)
    
    # box to line
    def direction_to_line(self, l: Line2D):
        return -l.normal_vector()

    def distance_to_line(self, l: Line2D):
        return self.o.distance_to_line(l) - self.r
    
    def contact_vector_to_line(self, l: Line2D): # vector in order to make this circle contact with a infinite line (not segment)
        return self.distance_to_line(l) * self.direction_to_line(l)


class Circle(object):
    def __init__(self, o: Point2D, r: float):
        self.o = o
        self.r = r
    
    def __repr__(self):
        return f'{self.o}:{self.r}'
    
    def location_wrt(self, l: Line2D):
        return self.o.location_wrt(l)
    
    # circle to point
    def direction_to_point(self, p: Point2D):
        return (p - self.o).unit()
    
    def distance_to_point(self, p: Point2D):
        return self.o.distance_to(p) - self.r
    
    def contact_vector_to_point(self, p: Point2D): # vector in order to make this circle contact with a point
        return self.distance_to_point(p) * self.direction_to_point(p)
    
    # circle to line
    def direction_to_line(self, l: Line2D):
        dir = -l.normal_vector()
        return dir

    def distance_to_line(self, l: Line2D):
        dist = self.o.distance_to_line(l) - self.r
        return dist
    
    def contact_vector_to_line(self, l: Line2D): # vector in order to make this circle contact with a infinite line (not segment)
        vec = self.distance_to_line(l) * self.direction_to_line(l)
        return vec


class Line2D(object):
    def __init__(self, p1: Point2D, p2: Point2D):
        if isinstance(p1, Point2D) and isinstance(p2, Point2D):
            self.p1 = p1
            self.p2 = p2
        else:
            raise TypeError('Two points are needed')
    
    def __repr__(self):
        return f'{self.p1}---{self.p2}'
    
    def len(self):
        return abs(self.p1 - self.p2)
    
    def tangent_vector(self):
        return (self.p2 - self.p1).unit()
    
    def normal_vector(self):
        return self.tangent_vector().rotate90()
    
    # line and line
    def intersect(self, other):
        l1 = self.p1.cross(self.p2)
        l2 = other.p1.cross(other.p2)
        w = (self.p1.y - self.p2.y)*(other.p2.x - other.p1.x) - (other.p1.y - other.p2.y)*(self.p2.x - self.p1.x)
        if w == 0:
            return None
        else:
            pi = Point2D(
                (self.p2.x - self.p1.x)*l2 - (other.p2.x - other.p1.x)*l1,
                (other.p1.y - other.p2.y)*l1 - (self.p1.y - self.p2.y)*l2
            ) / w
            L1 = abs(pi - self.p1)
            L2 = abs(pi - self.p2)
            L = abs(self.p1 - self.p2)
            if L1 + L2 > L:
                return None
            else:
                return pi

if __name__ == '__main__':
    p1 = Point2D(5,0)
    p2 = Point2D(1,-2)
    l1 = Line2D(p1, p2)
    p3 = Point2D(-2,-2)
    p4 = Point2D(4,-2)
    l2 = Line2D(p3,p4)
    # print(l1)
    # print(l1.len())
    # print(l1.tangent_vector())
    # print(l1.normal_vector())
    # print(p1.location_wrt(l1))
    # print(l1.intersect(l2))
    # print(p1.location_wrt(l2))

    c1 = Circle(Point2D(0,0), 3)
    # print(c1)
    # print(c1.location_wrt(l2))
    # print(c1.distance_to_point(p1))
    # print(c1.distance_to_line(l2))

    b1 = Box(Point2D(1, 3), 4, 2)
    print(b1)