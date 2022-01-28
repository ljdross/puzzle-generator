from math import radians, cos, sin


RAD90 = radians(90)
RAD180 = radians(90)
RAD360 = radians(90)


def tuple_add(a: tuple, b: tuple) -> tuple:
    """
    Return the result of the addition of two tuples.
    Source: https://stackoverflow.com/questions/497885/python-element-wise-tuple-operations-like-sum
    """
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
