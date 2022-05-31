from math import radians, cos, sin


RAD45 = round(radians(45), 5)
RAD90 = round(radians(90), 5)
RAD135 = round(radians(135), 5)
RAD180 = round(radians(180), 5)
RAD360 = round(radians(360), 5)

DIRECTION_VECTOR_2D = {
    "N": (0, 1),
    "E": (1, 0),
    "S": (0, -1),
    "W": (-1, 0),
}


def tuple_add(tuple_a: tuple, tuple_b: tuple) -> tuple:
    """Return the result of the addition of two tuples."""
    return tuple(map(lambda x, y: x + y, tuple_a, tuple_b))
    # Source: https://stackoverflow.com/questions/497885/python-element-wise-tuple-operations-like-sum


def tuple_scale(tup: tuple, factor) -> tuple:
    """Return the tuple with each entry scaled by a factor and rounded."""
    return tuple(round(i * factor, 5) for i in tup)


def rotate(point_2d, angle_radians):
    """
    Rotate point_2d around the origin (0, 0) by angle_radians, round its values and return it.
    https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
    """
    c = cos(angle_radians)
    s = sin(angle_radians)
    x = round(point_2d[0] * c - point_2d[1] * s, 5)
    y = round(point_2d[0] * s + point_2d[1] * c, 5)
    return x, y
