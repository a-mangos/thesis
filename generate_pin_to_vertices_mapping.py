import open3d as o3d
import numpy as np
from collections import defaultdict
import pickle

# pin_vertex_ids = {
#     "L0": 65889,
#     "L1": 65873,
#     "L2": 69862,
#     "L3": 84126,
#     "L4": 60724,
#     "L5": 89062,
#     "L6": 63148,
#     "L7": 92559,
#     "L8": 64613,
#     "L9": 63489,
#     "L10": 60946,
#     "L11": 76331,
#     "L12": 67345,
#     "R0": 99599,
#     "R1": 65231,
#     "R2": 83072,
#     "R3": 108406,
#     "R4": 100310,
#     "R5": 83227,
#     "R6": 83841,
#     "R7": 107815,
#     "R8": 74954,
#     "R9": 93426,
#     "R10": 61239,
#     "R11": 112630,
#     "R12": 110331,
#     "F0": 94313,
#     "F1": 107637,
#     "F2": 92998,
#     "F3": 74702,
#     "F4": 64456,
#     "F5": 61439,
#     "F6": 84576,
#     "F7": 107112,
#     "F8": 66519,
#     "F9": 94072,
#     "F10": 87374,
#     "F11": 72249,
#     "F12": 91853,
#     "B0": 63142,
#     "B1": 109990,
#     "B2": 72823,
#     "B3": 62776,
#     "B4": 106141,
#     "B5": 83637,
#     "B6": 97770,
#     "B7": 69393,
#     "B8": 82246,
#     "B9": 73745,
#     "B10": 90485,
#     "B11": 76795,
#     "B12": 93939
# }



pin_vertex_ids = {
    "L0": 61439,
    "L1": 65889,
    "L2": 60946,
    "L3": 67345,
    "L4": 61239,
    "L5": 60724,
    "L6": 93939,
    "L7": 76795,
    "L8": 65231,
    "L9": 66519,
    "L10": 65873,
    "L11": 94072,
    "L12": 93426,
    "R0": 83841,
    "R1": 83637,
    "R2": 62776,
    "R3": 72823,
    "R4": 89062,
    "R5": 69393,
    "R6": 69862,
    "R7": 83227,
    "R8": 100310,
    "R9": 84576,
    "R10": 72249,
    "R11": 92998,
    "R12": 84126,
    "F0": 63142,
    "F1": 74954,
    "F2": 99599,
    "F3": 73745,
    "F4": 82246,
    "F5": 64456,
    "F6": 63489,
    "F7": 74702,
    "F8": 97770,
    "F9": 76331,
    "F10": 63148,
    "F11": 64613,
    "F12": 83072,
    "B0": 109990,
    "B1": 90485,
    "B2": 110331,
    "B3": 107112,
    "B4": 91853,
    "B5": 106141,
    "B6": 107637,
    "B7": 112630,
    "B8": 94313,
    "B9": 92559,
    "B10": 87374,
    "B11": 108406,
    "B12": 107815
}

fetal_head = o3d.io.read_triangle_mesh("whole_head_model.STL")

vertices = fetal_head.vertices
result = defaultdict(list)
for pin_name, vertex_id in pin_vertex_ids.items():
    point_coords = vertices[vertex_id]

    for index in range(len(vertices) - 1):
        if np.sqrt((vertices[index][0] - point_coords[0]) ** 2 +
                   (vertices[index][1] - point_coords[1]) ** 2 +
                   (vertices[index][2] - point_coords[2]) ** 2) <= 7:
            result[pin_name].append(index)
            # print(f"Adding {index} to {pin_name}")

with open('pin_to_vertices_mapping.pickle', 'wb') as handle:
    pickle.dump(result, handle, protocol=pickle.HIGHEST_PROTOCOL)
