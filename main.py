import open3d as o3d
import numpy as np
import serial
import math
import copy
from struct import unpack
from statistics import mean
from collections import deque
from scipy.stats import circmean
from scipy.spatial.transform import Rotation as R
from ahrs.filters import AQUA

def open_serial(ser):
    ser.baudrate = 115200
    ser.port = 'COM4'
    ser.open()

def calculate_head_rotation(force_x: int, force_y: int) -> float:
    calibrated_x = (force_x - 321) / 67
    calibrated_y = (force_y - 323.5) / 66.5
    return math.atan2(calibrated_x, calibrated_y) * 180 / math.pi - 46

def calculate_head_flexion(force_x: int, force_z: int) -> float:
    calibrated_x = (force_x - 321) / 67
    calibrated_z = (force_z - 290) / 57
    return math.atan2(calibrated_x, calibrated_z) * 180 / math.pi - 90

# def calibrate_x_axis(data_x):
#     # 370 in OA, 271 in OP  -> 320.5 mean. Then solve for 9.8 m/s^2 in OA to get 5.05
#     return (data_x - 321) / 67
#
# def calibrate_y_axis(data_y):
#     # 359 in OA, 272 in OP -> 315.5 mean. Then solve for 9.8 m/s^2 to get 4.44
#     return (data_y - 323.5) / 66.5
#
# def calibrate_z_axis(data_z):
#     # 289 in OA, 296 in OP -> 292.5 mean. Then solve for 9.8 m/s^2 to get 0.51
#     return (data_z - 290) / 57
1
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
    fetal_head = o3d.io.read_triangle_mesh("../whole_head_model.STL")

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
    # vis.add_geometry(coord_frame)
    vis.add_geometry(fetal_head_r)

    view_control = vis.get_view_control()
    parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-02-22-08-39-51.json")
    view_control.convert_from_pinhole_camera_parameters(parameters, True)

    # vis.run()
    # vis.destroy_window()
    # vertices_list = vis.get_picked_points()
    #
    # with open(r"../picked_points.txt", "w") as fp:
    #     for item in vertices_list:
    #         fp.write("%s\n" % item.index)

    # previous_flexion = fetal_head_r.get_rotation_matrix_from_xyz((0, 0, 0))
    # previous_roll = fetal_head_r.get_rotation_matrix_from_xyz((0, 0, 0))

    roll_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    flexion_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    yaw_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    data_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    points = []
    with open(r"../picked_points.txt", "r") as f:
        for line in f:
            points.append([int(e) for e in line.split()])

    previous_position = np.identity(3)
    while True:
        data = read_from_serial(ser)

        # roll = calculate_head_rotation(data[13], data[14])
        # roll_rolling.append(roll)
        # roll_rolling.pop(0)
        #
        # flexion = calculate_head_flexion(data[13], data[15])
        # flexion_rolling.append(flexion)
        # flexion_rolling.pop(0)

        # roll = (data[13] - 321) / 67 * 9.8
        # roll_rolling.append(roll)
        # roll_rolling.pop(0)
        #
        # flexion = (data[14] - 323.5) / 66.5 * 9.8
        # flexion_rolling.append(flexion)
        # flexion_rolling.pop(0)
        #
        # yaw = (data[15] - 290) / 57 * 9.8
        # yaw_rolling.append(yaw)
        # yaw_rolling.pop(0)



        fetal_head_r.rotate(np.transpose(previous_position), fetal_head_r.get_center())

        # new_position = fetal_head_r.get_rotation_matrix_from_xyz((circmean(flexion_rolling, high=360) * np.pi / 180,
        #                                                           circmean(roll_rolling, high=360) * np.pi / 180,
        #                                                           0)
        # new_roll = fetal_head_r.get_rotation_matrix_from_xyz((0,
        #                                                       circmean(roll_rolling, high=360) * np.pi / 180,
        #                                                       0))
        # new_flexion = fetal_head_r.get_rotation_matrix_from_xyz((circmean(flexion_rolling, high=360) * np.pi / 180,
        #                                                         0,
        #                                                         0))

        data_as_aqua = AQUA()
        data_as_aqua.acc = np.asarray([mean(roll_rolling), mean(flexion_rolling), 0])
        data_quaternion = data_as_aqua.estimate(data_as_aqua.acc)

        new_position = fetal_head_r.get_rotation_matrix_from_quaternion(data_quaternion)

        # new_position = np.matmul(new_flexion, new_roll)

        fetal_head_r.rotate(new_position, fetal_head_r.get_center())
        previous_position = new_position

        # fetal_head_r.rotate(np.transpose(previous_flexion))
        # fetal_head_r.rotate(np.transpose(previous_roll))

        # fetal_head_r.rotate(np.transpose(previous_position))
        #
        # r1 = R.from_euler("xyz", [0, circmean(roll_rolling, high=360) * np.pi / 180, 0], degrees=False)
        #
        # r2 = R.from_euler("XYZ", [circmean(flexion_rolling, high=360) * np.pi / 180, 0, 0], degrees=False)
        #
        # r2.apply(r1)
        #
        # new_position = r2.as_matrix()
        #
        # # new_roll = fetal_head_r.get_rotation_matrix_from_zyx((0,
        # #                                                       circmean(roll_rolling, high=360) * np.pi / 180,
        # #                                                       0))
        # #
        # # new_flexion = fetal_head.get_rotation_matrix_from_xyz((circmean(flexion_rolling, high=360) * np.pi / 180,
        # #                                                        0,
        # #                                                        0))
        #
        # new_position = new_flexion * new_roll

        # fetal_head_r.rotate(new_position)
        #
        # # previous_roll = new_roll
        # # previous_flexion = new_flexion
        #
        # previous_position = new_position

        # print(data[41] * 14 / 512)
        # print(data[33] * 14/512, data[34] * 14/512)
        # print(data[14])
        # print(f"{data[13]}, {data[14]}, {data[15]}")

        # print(f"{calibrate_x_axis(data[13])}, {calibrate_y_axis(data[14])}, {calibrate_z_axis(data[15])}")

        get_x(data[13])




        data_rolling.append(normalize_data(data[41] * 14 / 512))
        data_rolling.pop(0)

        vertex_colours = np.asarray(fetal_head_r.vertex_colors)
        for point in points:
            vertex_colours[point] = [mean(data_rolling), 1 - mean(data_rolling), 0]

        fetal_head_r.vertex_colors = o3d.utility.Vector3dVector(vertex_colours)

        # fetal_head_r.vertex_colors = o3d.utility.Vector3dVector(
        #     np.random.uniform(0, 1, size=(len(fetal_head_r.vertices), 3)))

        # fetal_head_r.paint_uniform_color([1, 0, 0])

        vis.update_geometry(fetal_head_r)
        vis.update_renderer()
        vis.poll_events()
