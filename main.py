import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import serial
from dataclasses import dataclass, field
from scipy.stats import circmean
import serial.tools.list_ports
import math
import pickle
from struct import unpack
import time
import RPi.GPIO as GPIO
import os
import random
import threading

@dataclass
class AccelerationData:
    x: float
    y: float
    z: float


class AppWindow:
    def __init__(self):
        with open('pin_to_vertices_mapping.pickle', 'rb') as handle:
            self._pin_to_vertices_mapping = pickle.load(handle)

        self._data_to_pin = {
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

        self._app = o3d.visualization.gui.Application.instance
        self._app.initialize()

        self._window = self._app.create_window("Birth Simulator", width=1920, height=1080)

        self._widgetLeft = gui.SceneWidget()
        self._widgetLeft.scene = rendering.Open3DScene(self._window.renderer)
        self._window.add_child(self._widgetLeft)

        self._widgetRight = gui.SceneWidget()
        self._widgetRight.scene = rendering.Open3DScene(self._window.renderer)
        self._window.add_child(self._widgetRight)

        self._progressBar = gui.ProgressBar()
        self._progressBar.value = 0
        self._window.add_child(self._progressBar)

        self._coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
        self._fetal_head = self.load_fetal_head("whole_head_model.STL")

        self._previous_position = np.identity(3)
        self._roll_rolling = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._vertices = np.asarray(self._fetal_head.vertices)
        self._pressure_readings = {}
        self._vertex_colors = np.asarray(self._fetal_head.vertex_colors)

        self._last_pressure_decay_time = time.time()

        self._mat = rendering.MaterialRecord()
        self._mat.shader = 'defaultLit'

        self.set_window_layout()
        self.setup_scenes()

    def load_fetal_head(self, path):
        fetal_head = o3d.io.read_triangle_mesh(path)
        fetal_head.paint_uniform_color([0.7, 0.7, 0.7])
        fetal_head.compute_vertex_normals()

        return fetal_head

    def setup_scenes(self):
        self._widgetLeft.scene.clear_geometry()
        self._widgetRight.scene.clear_geometry()

        self.initial_transform()

        self._widgetLeft.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetLeft.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)
        self._widgetLeft.scene.camera.look_at(self._fetal_head.get_center(),
                                              np.asarray([75, 50, -25]), np.asarray([0, 0, -1]))

        self._widgetRight.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetRight.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)
        self._widgetRight.scene.camera.look_at(self._fetal_head.get_center(),
                                               np.asarray([-100, 0, 0]), np.asarray([0, 0, -1]))

    def set_window_layout(self):
        r = self._window.content_rect
        scale_factor = 15/16
        self._widgetLeft.frame = gui.Rect(r.x, r.y, r.width / 2, r.height * scale_factor)
        self._widgetRight.frame = gui.Rect(r.x + r.width / 2 + 1, r.y, r.width / 2, r.height * scale_factor)
        self._progressBar.frame = gui.Rect(r.x, r.y + r.height * scale_factor, r.width, r.height * (1 - scale_factor))

    def initial_transform(self):
        # Initial offset from original model to line up the head with axes of render
        # offsets: flexion -0.4 rotation -0.615 deflection -0.2
        initial_position = self._fetal_head.get_rotation_matrix_from_xyz((-0.4, -0.615, -0.2))
        self._fetal_head.rotate(initial_position, self._fetal_head.get_center())

        self._coord_frame.translate((0, 0, 0), relative=False)
        self._fetal_head.translate(self._coord_frame.get_center(), relative=False)

    def update_geometry(self, serial_data):
        self._widgetLeft.scene.clear_geometry()
        self._widgetRight.scene.clear_geometry()

        acceleration_data = AccelerationData(x=serial_data[13], y=serial_data[14], z=serial_data[15])
        self.update_pose(acceleration_data)
        self.update_colour(serial_data)
        self._progressBar.value = (serial_data[64] - 1000)/1000

        self._widgetLeft.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetLeft.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)

        self._widgetRight.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetRight.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)

        self._window.post_redraw()

    def update_pose(self, acceleration_data):

        self._roll_rolling.append(calculate_head_rotation(acceleration_data))
        self._roll_rolling.pop(0)

        self._fetal_head.rotate(np.transpose(self._previous_position), self._fetal_head.get_center())

        new_position = self._fetal_head.get_rotation_matrix_from_xyz(
            (0, circmean(self._roll_rolling, high=360) * np.pi / 180, 0))

        self._fetal_head.rotate(new_position, self._fetal_head.get_center())
        self._previous_position = new_position

    def update_colour(self, serial_data):
        if (time.time() - self._last_pressure_decay_time) > 0.1:
            for pin in self._pressure_readings.keys():
                self._pressure_readings[pin] = max(self._pressure_readings[pin] * 0.75, 1)
                self._last_pressure_decay_time = time.time()

        for pin, vertices in self._pin_to_vertices_mapping.items():
            new_reading = serial_data[self._data_to_pin.get(pin)]
            
            if new_reading < 8:
                new_reading = 1
            # 550 pressure sensor reading through the cap is around 15N force on the sensor.
            if new_reading > 550:
                new_reading = 550
            if pin not in self._pressure_readings or new_reading > self._pressure_readings[pin]:
                self._pressure_readings[pin] = max(self._pressure_readings.get(pin, 1), new_reading)

            # [0.7, 0.7, 0.7] is the grey colour of the head
            G_B_channels = 0.7 / self._pressure_readings.get(pin, 1)
            colour = [0.7 - G_B_channels, G_B_channels, G_B_channels]

            for vertex in vertices:
                self._vertex_colors[vertex] = colour


def calculate_head_rotation(acceleration_data: AccelerationData) -> float:
    calibrated_x = (acceleration_data.x - 325)/67
    calibrated_y = (acceleration_data.y - 325)/67
    return math.atan2(calibrated_y, calibrated_x) * 180 / math.pi - 46


class SerialData:
    def __init__(self):
        self._ser = serial.Serial()
        self._ser.baudrate = 115200
        self._ser.port = list(serial.tools.list_ports.comports())[0].device

        TIMEOUT_SECONDS = 60
        start_time = time.time()
        print("Waiting for Arduino to become ready on COM port...")
        while(True):
            try:
                self._ser.open()
                break
            except serial.serialutil.SerialException as e:
                if (time.time() - start_time) > TIMEOUT_SECONDS:
                    raise e
        print("Done")
        print("Waiting a few seconds to allow Arduino to bootstrap...")
        time.sleep(6)
        print("Done")

    def read_from_serial(self):
        def align_to_header(ser):
            header = [0xDE, 0xAD, 0xBE, 0xEF, 0xC0, 0x01, 0xCA, 0xFE]
            attempt_counter = 0
            max_num_attempts = 1000
            while True:
                found = True
                for byte in header:
                    if byte != int.from_bytes(ser.read(), 'big'):
                        if attempt_counter > max_num_attempts:
                            raise ValueError("Valid header could not be read from serial port")
                        attempt_counter += 1
                        found = False
                        break
                if found:
                    return
        self._ser.reset_input_buffer()
        align_to_header(self._ser)
        num_readings = 65
        return list(unpack(f">{num_readings}h", self._ser.read(num_readings * 2)))


def calibrate(serial_data):
    # Manually offset some values because of the cap pressure
    serial_data[0] = max(serial_data[0] - 9, 0)
    serial_data[4] = max(serial_data[4] - 1, 0)
    serial_data[6] = max(serial_data[6] - 31, 0)
    serial_data[7] = max(serial_data[7] - 1, 0)
    serial_data[9] = max(serial_data[9] - 1, 0)
    serial_data[10] = max(serial_data[10] - 14, 0)
    serial_data[11] = max(serial_data[11] - 2, 0)
    serial_data[27] = max(serial_data[27] - 8, 0)
    serial_data[37] = max(serial_data[37] - 10, 0)
    serial_data[39] = max(serial_data[39] - 6, 0)
    serial_data[50] = max(serial_data[50] - 2, 0)
    serial_data[53] = max(serial_data[53] - 6, 0)
    serial_data[54] = max(serial_data[54] - 36, 0)
    serial_data[57] = max(serial_data[57] - 13, 0)
    serial_data[58] = max(serial_data[58] - 54, 0)
    serial_data[59] = max(serial_data[59] - 195, 0)
    serial_data[60] = max(serial_data[60] - 21, 0)
    return serial_data

SHUTDOWN_PIN = 18

if __name__ == "__main__":
    # Initialise gui display and serial instance
    serial_instance = SerialData()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SHUTDOWN_PIN, GPIO.IN)

    while True:
        i = 0
        app_instance = AppWindow()
        
        while app_instance._app.run_one_tick():
            i += 1
            if i > 16000:
                app_instance._app.quit()
                del app_instance
                time.sleep(1)
                break
            raw = serial_instance.read_from_serial()
            calibrated = calibrate(raw)
            app_instance.update_geometry(calibrated)
        
            if GPIO.input(SHUTDOWN_PIN):
                GPIO.cleanup()
                os.system("shutdown -h now")