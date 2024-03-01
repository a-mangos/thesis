import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import serial
import math
import copy
from struct import unpack
from statistics import mean
from scipy.stats import circmean
from dataclasses import dataclass
import vis_gui


@dataclass
class AccelerationData:
    x_raw: int
    y_raw: int
    z_raw: int

    x_cal: int
    y_cal: int
    z_cal: int

def open_serial(ser):
    ser.baudrate = 115200
    ser.port = ('COM4')
    ser.open()

# def calculate_head_rotation(acceleration_data: AccelerationData) -> float:
#     calibrated_x = (acceleration_data.x - 321) / 67
#     calibrated_y = (acceleration_data.y - 323.5) / 66.5
#     return math.atan2(calibrated_x, calibrated_y) * 180 / math.pi - 46
#
#
# def calculate_head_pitch(acceleration_data: AccelerationData) -> float:
#     calibrated_x = (acceleration_data.x - 321) / 67
#     calibrated_z = (acceleration_data.z - 290) / 57
#     return math.atan2(calibrated_x, calibrated_z) * 180 / math.pi - 90

# returns roll, pitch
def calculate_head_angles(acceleration_data: AccelerationData):
    # sign = 1 if acceleration_data.z_cal > 0 else -1
    # miu = 0.001
    sign = 1
    miu = 0
    return [math.atan2(acceleration_data.y_cal,
                       sign * np.sqrt(acceleration_data.z_cal**2 + miu * acceleration_data.x_cal**2)),
            math.atan2(-acceleration_data.x_cal,
                       np.sqrt(acceleration_data.y_cal**2 + acceleration_data.z_cal**2))]


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
    return unpack(f"<{NUM_READINGS}h", ser.read(NUM_READINGS * 2))


def normalize_data(data):
    return (data - 0) / (4 - 0)


def calibrate_and_assign_acceleration_data(data_x, data_y, data_z):
    x = data_x.to_bytes(2, 'big', signed=True)
    x_raw = (x[0] << 8 | x[1]) >> 4
    x_cal = (x_raw / (1 << 11))

    y = data_y.to_bytes(2, 'big', signed=True)
    y_raw = (y[0] << 8 | y[1]) >> 4
    y_cal = (y_raw / (1 << 11))

    z = data_z.to_bytes(2, 'big', signed=True)
    z_raw = (z[0] << 8 | z[1]) >> 4
    z_cal = (z_raw / (1 << 11))

    return AccelerationData(x_raw=x_raw, y_raw=y_raw, z_raw=z_raw, x_cal=x_cal, y_cal=y_cal, z_cal=z_cal)


if __name__ == "__main__":
    # Initialise serial
    # ser = serial.Serial()
    # open_serial(ser)

    gui.Application.instance.initialize()
    app = vis_gui.AppWindow(1024, 768)
    # window = gui.Application.instance.create_window("Birth Simulator", width=1000, height=1000)
    # widget = gui.SceneWidget()
    # widget.scene = rendering.Open3DScene(window.renderer)
    # window.add_child(widget)
    path = "whole_head_model.STL"

    app.load(path)
    # Read in file
    # fetal_head = o3d.io.read_triangle_mesh(path)
    #
    # # Initial offset from original model to line up the head with axes of render
    # # offsets: flexion -0.4 rotation -0.615 deflection -0.2
    # initial_position = fetal_head.get_rotation_matrix_from_xyz((-0.4, -0.615, -0.2))
    # fetal_head.rotate(initial_position, fetal_head.get_center())
    #
    # # Copy head for dynamic rotation
    # fetal_head_r = copy.deepcopy(fetal_head)
    # fetal_head_r.compute_vertex_normals()
    # fetal_head_r.paint_uniform_color([0.7, 0.7, 0.7])
    #
    # material = rendering.MaterialRecord()
    # material.shader = 'defaultLit'
    #
    # # widget.scene.camera.look_at(fetal_head.get_center().tolist(), [1,1,1], [0,0,0])
    #
    #
    # # vis = o3d.visualization.Visualizer()
    # # vis.create_window(window_name="STL", left=1000, top=200, width=800, height=800)
    #
    # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=fetal_head_r.get_center())
    # # vis.add_geometry(coord_frame)
    # # widget.scene.add_geometry(coord_frame)
    # widget.scene.add_geometry("head", fetal_head_r, material)
    # # vis.add_geometry(fetal_head_r)


    gui.Application.instance.run()

    # vis.run()
    # vis.destroy_window()
    # vertices_list = vis.get_picked_points()
    #
    # with open(r"pin_locations.txt", "w") as fp:
    #     for item in vertices_list:
    #         fp.write("%s\n" % item.index)

    # points = []
    # with open(r"pin_locations.txt", "r") as f:
    #     for line in f:
    #         points.append([int(e) for e in line.split()])
    #
    # vertices = np.asarray(fetal_head_r.vertices)
    # spheres = []
    # for point in points:
    #     point_coord = vertices[point]
    #     print(point_coord[0][0], point_coord[0][1], point_coord[0][2])
    #     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=6, resolution=5)
    #     sphere.translate(np.transpose(point_coord))
    #     spheres.append(sphere)
    #     vis.add_geometry(sphere)
    #
    # # code to paint the head instead of making spheres
    # # vertex_points_to_colour = []
    # # for point in points:
    # #     point_coord = vertices[point]
    # #     vertex_points_to_colour.append(point)
    # #     for index in range(len(vertices) - 1):
    # #         if np.sqrt((vertices[index][0] - point_coord[0][0])**2 +
    # #                    (vertices[index][1] - point_coord[0][1])**2 +
    # #                    (vertices[index][2] - point_coord[0][2])**2) <= 6:
    # #             vertex_points_to_colour.append(index)
    # # vertex_colors = np.asarray(fetal_head_r.vertex_colors)
    #
    # view_control = vis.get_view_control()
    # parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-02-22-08-39-51.json")
    # view_control.convert_from_pinhole_camera_parameters(parameters, True)
    #
    # previous_position = np.identity(3)
    #
    # roll_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # pitch_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # data_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #
    # while True:
    #     data = read_from_serial(ser)
    #     acceleration_data = calibrate_and_assign_acceleration_data(data[13], data[14], data[15])
    #     roll, pitch = calculate_head_angles(acceleration_data)
    #
    #     roll_rolling.append(roll)
    #     roll_rolling.pop(0)
    #
    #     pitch_rolling.append(pitch)
    #     pitch_rolling.pop(0)
    #
    #     # accel_aqua = AQUA(None, np.asarray((acceleration_data.x_cal, acceleration_data.y_cal, acceleration_data.z_cal)), None)
    #     # quaternion = accel_aqua.estimate(accel_aqua.acc, None)
    #
    #     # print(acceleration_data)
    #
    #     fetal_head_r.rotate(np.linalg.inv(previous_position), fetal_head_r.get_center())
    #     for sphere in spheres:
    #         sphere.rotate(np.linalg.inv(previous_position), fetal_head_r.get_center())
    #
    #     # new_position = fetal_head_r.get_rotation_matrix_from_xyz((0,
    #     #                                                           circmean(roll_rolling, high=360) * np.pi / 180,
    #     #                                                           0))
    #
    #     # new_position = fetal_head_r.get_rotation_matrix_from_xyz((mean(pitch_rolling),
    #     #                                                           mean(roll_rolling),
    #     #                                                           0))
    #     new_position = fetal_head_r.get_rotation_matrix_from_xyz((0,
    #                                                               mean(roll_rolling),
    #                                                               0))
    #     # new_position = fetal_head_r.get_rotation_matrix_from_quaternion(quaternion)
    #
    #     for sphere in spheres:
    #         sphere.rotate(new_position, fetal_head_r.get_center())
    #     fetal_head_r.rotate(new_position, fetal_head_r.get_center())
    #     previous_position = new_position
    #
    #     # data_rolling.append(normalize_data(data[41] * 14 / 512))
    #     # data_rolling.pop(0)
    #     # for sphere in spheres:
    #     #     sphere.paint_uniform_color([mean(data_rolling), 1 - mean(data_rolling), 0])
    #
    #     for sphere in spheres:
    #         sphere.paint_uniform_color([1, 0, 0])
    #
    #     # for vertex in vertex_points_to_colour:
    #     #     vertex_colors[vertex] = [1, 0, 0]
    #
    #     # fetal_head_r.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
    #
    #     vis.update_geometry(fetal_head_r)
    #     for sphere in spheres:
    #         vis.update_geometry(sphere)
    #     vis.update_renderer()
    #     vis.poll_events()