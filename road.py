import numpy as np
from math import sin, cos, asin, acos, sqrt
from phyunits import *


class Point:
    def __init__(self, x, z):
        self.x = x
        self.z = z

class WheelPosition:
    def __init__(self, x, z, r):
        self.x = x
        self.z = z
        self.r = r

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

def two_points_distance(p1, p2):
    return sqrt((p2.x - p1.x)**2 + (p2.z - p1.z)**2)

def three_points2pos_ang_section(p1, p2, p3):
    x1, z1 = p1.x, p1.z
    x2, z2 = p2.x, p2.z
    x3, z3 = p3.x, p3.z
    v1 = np.array([x1 - x2, z1 - z2])
    v2 = np.array([x3 - x2, z3 - z2])
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    v1_hat = v1 / v1_norm
    v2_hat = v2 / v2_norm
    th_dot = acos(np.dot(v2_hat, v1_hat))
    th_cross = asin(np.cross(v2_hat, v1_hat))
    th = th_dot * np.sign(th_cross)
    x = v1_norm * cos(th)
    z = v1_norm * sin(th)
    section = np.floor(x / v1_norm) + (3 if z < 0 else 0)
    return x, z, th, section

class Node:
    def __init__(self, x, z):
        self.x = x
        self.z = z

class Segment:
    def __init__(self, node_id1, node_id2):
        self.node_id1 = node_id1
        self.node_id2 = node_id2

class Road:
    def __init__(self):
        self.nodes = []
        self.segs = []
    
    def add_node(self, x, z):
        self.nodes.append(Node(x, z))
    
    def add_segment(self, p1, p2):
        self.segs.append(Segment(p1, p2))

    def auto_connect(self):
        n = len(self.nodes)
        assert n >= 2
        for i in range(n-1):
            self.add_segment(i, i+1)
    
    def normal_vectors(self, i=None):
        nseg = len(self.segs)
        assert nseg > 0
        normals = []
        for seg in self.segs:
            node1 = self.nodes[seg.node_id1]
            node2 = self.nodes[seg.node_id2]
            v = np.array([node2.x - node1.x, node2.z - node1.z])
            v = v / np.linalg.norm(v)
            n = np.array([-v[1], v[0]])
            normals.append(n)
        return normals

    def distance_vectors_with_circle(self, pw):
        nseg = len(self.segs)
        assert nseg > 0
        distances = []
        for seg in self.segs:
            node1 = self.nodes[seg.node_id1]
            node2 = self.nodes[seg.node_id2]
            x, z, th, section = three_points2pos_ang_section(pw, node1, node2)
            d = z -pw.r
            distances.append((d,section))
        return np.array(distances)

    def repulse_vector_of_circle(self, pw):
        normals = self.normal_vectors()
        distances = self.distance_vectors_with_circle(pw)
        vector = np.zeros(2)
        for i, (dist, section) in enumerate(distances):
            if (section == 0 or section == 3) and (-pw.r < dist < 0):
                vector += -dist * normals[i]
        return vector

    def get_hull_distances(self, ph):
        nseg = len(self.segs)
        assert nseg > 0
        hull_surfaces = ph.surfaces()
        distances = []
        for seg in self.segs:
            node1 = self.nodes[seg.node_id1]
            node2 = self.nodes[seg.node_id2]
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