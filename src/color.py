import bpy


def new_color_material(rgba_tuple, name="rgba_color"):
    color = bpy.data.materials.new(name)
    color.diffuse_color = rgba_tuple
    return color


LAVENDER = new_color_material((0.8, 0.8, 1, 1), "lavender")
RED = new_color_material((1, 0, 0, 1), "red")
GREEN = new_color_material((0, 1, 0, 1), "green")
GREEN_TRANSLUCENT = new_color_material((0, 1, 0, 0.25), "green_translucent")
YELLOW = new_color_material((1, 1, 0, 1), "yellow")
WHITE = new_color_material((1, 1, 1, 1), "white")
GRAY = new_color_material((0.5, 0.5, 0.5, 1), "gray")
