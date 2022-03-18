from math import radians, cos, sin


RAD45 = round(radians(45), 5)
RAD90 = round(radians(90), 5)
RAD180 = round(radians(180), 5)
RAD360 = round(radians(360), 5)


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
    x = round(point_2d[0] * c - point_2d[1] * s, 5)
    y = round(point_2d[0] * s + point_2d[1] * c, 5)
    return x, y
