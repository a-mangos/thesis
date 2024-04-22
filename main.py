import open3d as o3d
import numpy as np
import serial
import serial.tools.list_ports
import math
import copy
import pickle
from struct import unpack
from scipy.stats import circmean
from dataclasses import dataclass
import time




@dataclass
class AccelerationData:
    x: float
    y: float
    z: float


def open_serial(ser):
    ser.baudrate = 115200
    # ser.port = list(serial.tools.list_ports.comports())[0].device
    ser.port = 'COM4'
    ser.open()


def calculate_head_rotation(acceleration_data: AccelerationData) -> float:
    calibrated_x = (acceleration_data.x - 321) / 67
    calibrated_y = (acceleration_data.y - 323.5) / 66.5
    return -1 * (math.atan2(calibrated_x, calibrated_y) * 180 / math.pi - 46)


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

    with open('pin_to_vertices_mapping.pickle', 'rb') as handle:
        pin_to_vertices_mapping = pickle.load(handle)

    vertex_colors = np.asarray(fetal_head_r.vertex_colors)

    vis_one = o3d.visualization.Visualizer()
    vis_one.create_window(window_name="Left", left=1000, top=200, width=800, height=800)

    vis_two = o3d.visualization.Visualizer()
    vis_two.create_window(window_name="Right", left=1000, top=200, width=800, height=800)

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=fetal_head_r.get_center())
    vis_one.add_geometry(coord_frame)
    vis_one.add_geometry(fetal_head_r)

    vis_two.add_geometry(coord_frame)
    vis_two.add_geometry(fetal_head_r)

    # colors = np.asarray(fetal_head_r.vertex_colors)
    # for point in points:
    #     colors[point] = [1, 0, 0]
    # fetal_head_r.vertex_colors = o3d.utility.Vector3dVector(colors)
    #
    # vis_one.run()
    # vis_one.destroy_window()

    roll_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    flexion_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    data_to_pin = {
        'B0': 0,
        'B1': 1,
        'B2': 2,
        'B3': 3,
        'B4': 4,
        'B5': 5,
        'B6': 6,
        'B7': 7,
        'B8': 8,
        'B9': 9,
        'B10': 10,
        'B11': 11,
        'B12': 12,
        'R0': 16,
        'R1': 17,
        'R2': 18,
        'R3': 19,
        'R4': 20,
        'R5': 21,
        'R6': 22,
        'R7': 23,
        'R8': 24,
        'R9': 25,
        'R10': 26,
        'R11': 27,
        'R12': 28,
        'L0': 32,
        'L1': 33,
        'L2': 34,
        'L3': 35,
        'L4': 36,
        'L5': 37,
        'L6': 38,
        'L7': 39,
        'L8': 40,
        'L9': 41,
        'L10': 42,
        'L11': 43,
        'L12': 44,
        'F0': 48,
        'F1': 49,
        'F2': 50,
        'F3': 51,
        'F4': 52,
        'F5': 53,
        'F6': 54,
        'F7': 55,
        'F8': 56,
        'F9': 57,
        'F10': 58,
        'F11': 59,
        'F12': 60,
    }

    view_parameters_one = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-02-22-08-39-51.json")
    view_parameters_two = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-04-22-21-46-33.json")

    view_control_one = vis_one.get_view_control()
    view_control_one.convert_from_pinhole_camera_parameters(view_parameters_one, True)

    view_control_two = vis_two.get_view_control()
    view_control_two.convert_from_pinhole_camera_parameters(view_parameters_two, True)

    previous_position = np.identity(3)

    pressure_readings = {}
    last_pressure_decay_time = time.time()
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

        new_position = fetal_head_r.get_rotation_matrix_from_xyz((0,
                                                                  circmean(roll_rolling, high=360) * np.pi / 180,
                                                                  0))

        fetal_head_r.rotate(new_position, fetal_head_r.get_center())
        previous_position = new_position

        if (time.time() - last_pressure_decay_time) > 0.1:
            for pin in pressure_readings.keys():
                pressure_readings[pin] = max(pressure_readings[pin] * 0.7,  1)
                last_pressure_decay_time = time.time()

        for pin, vertices in pin_to_vertices_mapping.items():
            new_reading = data[data_to_pin.get(pin)]
            if new_reading < 50:
                new_reading = 1
            if new_reading > 300:
                new_reading = 300
            if pin not in pressure_readings or new_reading > pressure_readings[pin]:
                pressure_readings[pin] = max(pressure_readings.get(pin, 1), new_reading)
                # if pressure_readings.get(pin, 1) < new_reading:
                #     pressure_readings[pin] = new_reading if new_reading > 30 else 1

            G_B_channels = 0.7/pressure_readings.get(pin, 1)
            colour = [0.7, G_B_channels, G_B_channels]
            for vertex in vertices:
                vertex_colors[vertex] = colour

        vis_one.update_geometry(fetal_head_r)
        vis_two.update_geometry(fetal_head_r)

        vis_one.update_renderer()
        vis_one.poll_events()
        vis_two.update_renderer()
        vis_two.poll_events()
