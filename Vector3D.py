import numpy as np
from math import sqrt, sin, cos, atan2


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self):
        return f'({self.x},{self.y},{self.z})'

    def __add__(self, other):
        if not isinstance(other, Vector3D):
            raise TypeError('can only add Vector3D object')
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        if not isinstance(other, Vector3D):
            raise TypeError('can only subtract Vector3D object')
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def dot(self, other):
        if not isinstance(other, Vector3D):
            raise TypeError('can only dot product Vector3D object')
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    __matmul__ = dot
    
    def cross(self, other):
        if not isinstance(other, Vector3D):
            raise TypeError('can only cross product Vector3D object')
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def norm(self):
        return sqrt(self.x**2 + self.y**2 + self.z**2)
    
    __abs__ = norm

    def unit(self):
        return self / self.norm()
    
    # def angle(self):
    #     return atan2(self.y, self.x)
    
    # def rotate90(self):
    #     return Vector3D(-self.y, self.x)
    
    # def rotate(self, angle):
    #     return Vector3D(
    #         cos(angle)*self.x - sin(angle)*self.y,
    #         sin(angle)*self.x + cos(angle)*self.y
    #     )
    
    def __neg__(self):
        return Vector3D(-self.x, -self.y, -self.z)
    
    def __mul__(self, scalar):
        if isinstance(scalar, int) or isinstance(scalar, float):
            return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)
        raise NotImplementedError('can only multiply with int or float')

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        if isinstance(scalar, int) or isinstance(scalar, float):
            return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)
        raise NotImplementedError('can only divide by int or float')
    
    def distance_to(self, other):
        return abs(self - other)
    
    # def to_polar(self):
    #     return self.__abs__(), self.angle()
    
    # def angle_wrt(self, other):
    #     return self.angle() - other.angle()


if __name__ == '__main__':
    deg = np.pi/180
    a = Vector3D(1,2,0)
    b = Vector3D(3,4,-1)
    print(-a + b)
    print(a.dot(b))
    print(a @ b)
    print(a.cross(b))
    print(a.norm())
    print(b.unit())
    print(a.distance_to(b))