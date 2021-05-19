from Vector2D import Vector2D
import numpy as np
from copy import copy, deepcopy
from math import sin, cos, asin, acos, sqrt
from phyunits import *
from Geometry import Point2D, Line2D, Circle
from casadi import *

Wheel = Circle

class HullSurface:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

class HullPose:
    def __init__(self, x, z, th, L, H):
        self.x = x
        self.z = z
        self.th = th # CW+
        self.L = L # half length
        self.H = H # half height

    def corners(self):
        x, z, th, L, H = self.x, self.z, self.th, self.L, self.H
        cth, sth = cos(th), sin(th)
        l2cth, l2sth = L*cth, L*sth
        h2cth, h2sth = H*cth, H*sth
        # rear-up, read-down, front-up, front-down
        ru = Point(x - l2cth + h2sth, z + l2sth + h2cth)
        rd = Point(x - l2cth - h2sth, z + l2sth - h2cth)
        fd = Point(x + l2cth - h2sth, z - l2sth - h2cth)
        fu = Point(x + l2cth + h2sth, z - l2sth + h2cth)
        return ru, rd, fd, fu
    
    def surfaces(self):
        ru, rd, fd, fu = self.corners()
        # surf_rear, surf_bottom, surf_front
        sr = HullSurface(ru, rd)
        sb = HullSurface(rd, fd)
        sf = HullSurface(fd, fu)
        su = HullSurface(fu, ru)
        return sr, sb, sf, su

class Road(object):
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.nNodes = 0
        self.nEdges = 0
    
    def add_node(self, x, z):
        self.nodes.append(Point2D(x, z))
        self.nNodes += 1
    
    def add_segment(self, i1: int, i2: int):
        p1 = self.nodes[i1]
        p2 = self.nodes[i2]
        self.edges.append(Line2D(p1, p2))
        self.nEdges += 1

    def auto_connect(self):
        n = self.nNodes
        assert n >= 2
        for i in range(n-1):
            self.add_segment(i, i+1)
    
    def is_empty(self):
        return True if self.nEdges == 0 else False
    
    def normal_vectors(self):
        if self.is_empty():
            return None
        normals = []
        for edge in self.edges:
            normals.append(edge.normal_vector())
        return normals

    def repulse_vector_of_wheel(self, c: Wheel):
        if not isinstance(c, Wheel):
            raise TypeError('can only take Wheel')
        if self.is_empty():
            return None
        c0 = copy(c)
        for edge in self.edges:
            x, y, lmr, up = c.location_wrt(edge)
            # if lmr == 0 and 0 < y < c.r: # middle, wheel center is in the ground (max = wheel radius)
                # c.o += c.contact_vector_to_line(edge)
            # is_wheel_contact = logic_and(y > 0, y < c.r)
            # is_wheel_in_range = logic_and(lmr == 0, is_wheel_contact)
            vc = c.contact_vector_to_line(edge)
            # c.o.x += if_else(is_wheel_in_range, vc.x, 0)
            # c.o.y += if_else(is_wheel_in_range, vc.y, 0)
            c.o += vc
        for i in range(self.nEdges - 1):
            e1 = self.edges[i]
            e2 = self.edges[i+1]
            _, _, lmr1, ud1 = c.location_wrt(e1)
            x, y, lmr2, ud2 = c.location_wrt(e2)
            d = abs(c.o - e2.p1)
            # if lmr1 == 1 and ud1 >= 0 and lmr2 == -1 and ud2 >= 0 and d < c.r: # wheel center is between and above two ground segment
            #     c.o += c.contact_vector_to_point(e2.p1)
            # is_wheel_in_right_of_seg1 = logic_and(lmr1 == 1, ud1 >= 0)
            # is_wheel_in_left_of_seg2 = logic_and(lmr2 == -1, ud2 >= 0)
            # is_wheel_in_side = logic_and(is_wheel_in_right_of_seg1, is_wheel_in_left_of_seg2)
            # is_wheel_in_range = logic_and(is_wheel_in_side, d < c.r)
            vc = c.contact_vector_to_point(e2.p1)
            # c.o.x += if_else(is_wheel_in_range, vc.x, 0)
            # c.o.y += if_else(is_wheel_in_range, vc.y, 0)
            c.o += vc

        return c.o - c0.o

    def get_hull_distances(self, ph):
        if self.is_empty():
            return None
        hull_surfaces = ph.surfaces()
        distances = []
        for edge in self.edges:
            node1 = self.nodes[edge.node_id1]
            node2 = self.nodes[edge.node_id2]
            vec_seg = np.array([node2.x - node1.x, node2.z - node1.z])
            lseg = two_points_distance(node1, node2)
            for surf in hull_surfaces:
                corner1 = surf.p1
                corner2 = surf.p2
                vec_surf = np.array([corner1.x - corner2.x, corner1.z - corner2.z])
                if np.dot(vec_seg, vec_surf) < 0:
                    x1, z1, th1 = three_points2pos_ang(corner1, node1, node2)
                    x2, z2, th2 = three_points2pos_ang(corner2, node1, node2)
                    if 0 < x1 < lseg and 0 < x2 < lseg:
                        d = min(z1, z2)
                        distances.append(d)
                    elif 0 < x2 < lseg:
                        _, d, _ = three_points2pos_ang(node1, corner2, corner1)
                        distances.append(d)
                    elif 0 < x1 < lseg:
                        _, d, _ = three_points2pos_ang(node2, corner2, corner1)
                        distances.append(d)
        return np.array(distances)
    
    def get_hull_min_distance(self, ph):
        distances = self.get_hull_distances(ph)
        d_min = 1e9
        for dist in distances:
            if dist > -ph.H:
                d_min = min(d_min, dist)
        return d_min

if __name__ == '__main__':
    from Geometry import Point2D, Line2D, Circle
    Wheel = Circle

    road = Road()
    road.add_node(-10, 0)
    road.add_node(1, 0)
    road.add_node(1, 0.1)
    road.add_node(2, 0.1)
    road.add_node(2, 0)
    road.add_node(10, 0)
    road.auto_connect()

    c = Wheel(Point2D(2.01, 0.101), 0.06)

    # print(road.normal_vectors())
    print(road.repulse_vector_of_wheel(c))
    

    # ph = HullPose(0, 0.1, 10*deg, 0.2, 0.06)

    # w_distances = road.distance_vectors_with_circle(pw)
    # w_d_min = road.repulse_vector_of_circle(pw)

    # h_distances = road.get_hull_distances(ph)
    # h_d_min = road.get_hull_min_distance(ph)

    # print(np.round(w_distances, 4))
    # print(np.round(w_d_min, 4))
    # print(np.round(h_distances, 4))
    # print(np.round(h_d_min, 4))
