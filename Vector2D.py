import numpy as np
from math import sqrt, sin, cos, atan2
np.set_printoptions(precision=4)


class Vector2D(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f'({round(self.x,4)},{round(self.y,4)})'

    def __add__(self, other):
        if not isinstance(other, Vector2D):
            raise TypeError('can only add Vector2D object')
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        if not isinstance(other, Vector2D):
            raise TypeError('can only subtract Vector2D object')
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def dot(self, other):
        if not isinstance(other, Vector2D):
            raise TypeError('can only dot product Vector2D object')
        return self.x * other.x + self.y * other.y
    
    __matmul__ = dot
    
    def cross(self, other):
        if not isinstance(other, Vector2D):
            raise TypeError('can only cross product Vector2D object')
        return self.x * other.y - self.y * other.x
    
    def norm(self):
        return np.sqrt(self.x**2 + self.y**2)
    
    __abs__ = norm

    def unit(self):
        return self / self.norm()
    
    def angle(self):
        return np.arctan2(self.y, self.x)
    
    def rotate90(self):
        return Vector2D(-self.y, self.x)
    
    def rotate(self, angle):
        return Vector2D(
            cos(angle)*self.x - sin(angle)*self.y,
            sin(angle)*self.x + cos(angle)*self.y
        )
    
    def __neg__(self):
        return Vector2D(-self.x, -self.y)
    
    def __mul__(self, scalar):
        # if isinstance(scalar, int) or isinstance(scalar, float):
        return Vector2D(self.x * scalar, self.y * scalar)
        # raise NotImplementedError('can only multiply with int or float')

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        # if isinstance(scalar, int) or isinstance(scalar, float):
        return Vector2D(self.x / scalar, self.y / scalar)
        # raise NotImplementedError('can only divide by int or float')
    
    def distance_to(self, other):
        return abs(self - other)
    
    def to_polar(self):
        return self.__abs__(), self.angle()
    
    def angle_wrt(self, other):
        return self.angle() - other.angle()


if __name__ == '__main__':
    deg = np.pi/180
    a = Vector2D(1,2)
    b = Vector2D(3,4)
    print(-a + b)
    print(a.dot(b))
    print(a @ b)
    print(a.cross(b))
    print(a.norm())
    print(a.angle())
    print(b.unit())
    print(a.distance_to(b))
    print(a.to_polar())
    print(a.angle_wrt(b)/deg)
    print(a.rotate90())
    print(a.rotate(45*deg))