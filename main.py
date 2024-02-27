import open3d as o3d
import numpy as np
import serial
import math
import copy
from struct import unpack
from statistics import mean
from collections import deque
from scipy.stats import circmean
from time import sleep
from dataclasses import dataclass


@dataclass
class AccelerationData:
    x: float
    y: float
    z: float


def open_serial(ser):
    ser.baudrate = 115200
    ser.port = 'COM3'
    ser.open()


def calculate_head_rotation(acceleration_data: AccelerationData) -> float:
    calibrated_x = (acceleration_data.x - 321) / 67
    calibrated_y = (acceleration_data.y - 323.5) / 66.5
    return math.atan2(calibrated_x, calibrated_y) * 180 / math.pi - 46


def calculate_head_flexion(acceleration_data: AccelerationData) -> float:
    calibrated_x = (acceleration_data.x - 321) / 67
    calibrated_z = (acceleration_data.z - 290) / 57
    return math.atan2(calibrated_x, calibrated_z) * 180 / math.pi - 90


def read_from_serial(ser):
    def align_to_header(ser):
        header = [0xDE, 0xAD, 0xBE, 0xEF, 0xC0, 0x01, 0xCA, 0xFE]
        max_attempts = 1000
        attempt_counter = 0
        while True:
            found = True
            for byte in header:
                if byte != int.from_bytes(ser.read(), 'big'):
                    if attempt_counter > max_attempts:
                        raise ValueError("Could not find the header")
                    attempt_counter += 1
                    found = False
                    break
            if found:
                return

    ser.flushInput()
    align_to_header(ser)
    NUM_READINGS = 64
    return unpack(f">{NUM_READINGS}h", ser.read(NUM_READINGS * 2))


def normalize_data(data):
    return (data - 0) / (4 - 0)


if __name__ == "__main__":
    ser = serial.Serial()
    open_serial(ser)

    # Read in file
    fetal_head = o3d.io.read_triangle_mesh("whole_head_model.STL")

    # Initial offset from original model to line up the head with axes of render
    # offsets: flexion -0.4 rotation -0.615 deflection -0.2
    initial_position = fetal_head.get_rotation_matrix_from_xyz((-0.4, -0.615, -0.2))
    fetal_head.rotate(initial_position, fetal_head.get_center())

    # Copy head for dynamic rotation
    fetal_head_r = copy.deepcopy(fetal_head)
    fetal_head_r.compute_vertex_normals()
    fetal_head_r.paint_uniform_color([0.7, 0.7, 0.7])

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="STL", left=1000, top=200, width=800, height=800)

    # MERGE_CLOSE_VERTICES - INTERESTING FOR MERGING COLORS
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=fetal_head_r.get_center())
    vis.add_geometry(coord_frame)
    vis.add_geometry(fetal_head_r)

    # vis.run()
    # vis.destroy_window()
    # vertices_list = vis.get_picked_points()
    #
    # with open(r"pin_locations.txt", "w") as fp:
    #     for item in vertices_list:
    #         fp.write("%s\n" % item.index)

    roll_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    flexion_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    data_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    points = []
    with open(r"pin_locations.txt", "r") as f:
        for line in f:
            points.append([int(e) for e in line.split()])

    print(len(points))

    vertices = np.asarray(fetal_head_r.vertices)
    spheres = []
    for point in points:
        point_coord = vertices[point]
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3, resolution=5)
        sphere.translate(np.transpose(point_coord))
        spheres.append(sphere)
        vis.add_geometry(sphere)

    view_control = vis.get_view_control()
    parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-02-22-08-39-51.json")
    view_control.convert_from_pinhole_camera_parameters(parameters, True)

    previous_position = np.identity(3)
    while True:
        data = read_from_serial(ser)
        acceleration_data = AccelerationData(x=data[13], y=data[14], z=data[15])

        roll = calculate_head_rotation(acceleration_data)
        roll_rolling.append(roll)
        roll_rolling.pop(0)

        flexion = calculate_head_flexion(acceleration_data)
        flexion_rolling.append(flexion)
        flexion_rolling.pop(0)

        fetal_head_r.rotate(np.transpose(previous_position), fetal_head_r.get_center())
        for sphere in spheres:
            sphere.rotate(np.transpose(previous_position), fetal_head_r.get_center())

        new_position = fetal_head_r.get_rotation_matrix_from_xyz((circmean(flexion_rolling, high=360) * np.pi/180,
                                                                  circmean(roll_rolling, high=360) * np.pi / 180,
                                                                  0))
        for sphere in spheres:
            sphere.rotate(new_position, fetal_head_r.get_center())
        fetal_head_r.rotate(new_position, fetal_head_r.get_center())
        previous_position = new_position

        data_rolling.append(normalize_data(data[41] * 14 / 512))
        data_rolling.pop(0)

        for sphere in spheres:
            sphere.paint_uniform_color([mean(data_rolling), 1 - mean(data_rolling), 0])

        vis.update_geometry(fetal_head_r)
        for sphere in spheres:
            vis.update_geometry(sphere)
        vis.update_renderer()
        vis.poll_events()
