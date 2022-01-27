from math import cos, sin


def tuple_add(a: tuple, b: tuple) -> tuple:
    return tuple(map(lambda x, y: x + y, a, b))


def rotate(point_2d, angle_radians):
    """
    Rotate point_2d around the origin (0, 0) by angle_radians and return it.
    https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
    """
    c = cos(angle_radians)
    s = sin(angle_radians)
    x = point_2d[0] * c - point_2d[1] * s
    y = point_2d[0] * s + point_2d[1] * c
    return x, y
