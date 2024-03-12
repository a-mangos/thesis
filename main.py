import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import serial
import math
import copy
from struct import unpack
from dataclasses import dataclass, field
from statistics import mean
from scipy.stats import circmean
from ahrs.filters import AQUA


@dataclass
class AccelerationData:
    x_raw: int = 0
    y_raw: int = 0
    z_raw: int = 0

    x_cal: int = 0
    y_cal: int = 0
    z_cal: int = 0

    def assign_variables(self, x_raw, y_raw, z_raw, x_cal, y_cal, z_cal):
        self.x_raw = x_raw
        self.y_raw = y_raw
        self.z_raw = z_raw
        self.x_cal = x_cal
        self.y_cal = y_cal
        self.z_cal = z_cal


@dataclass
class PressureData:
    back_sensors: tuple = field(default_factory=tuple)
    left_sensors: tuple = field(default_factory=tuple)
    right_sensors: tuple = field(default_factory=tuple)
    front_sensors: tuple = field(default_factory=tuple)


class AppWindow:
    def __init__(self):
        self._app = o3d.visualization.gui.Application.instance
        self._app.initialize()

        self._window = self._app.create_window("Birth Simulator", width=1280, height=720)

        self._widgetLeft = gui.SceneWidget()
        self._widgetLeft.scene = rendering.Open3DScene(self._window.renderer)
        self._window.add_child(self._widgetLeft)

        self._widgetRight = gui.SceneWidget()
        self._widgetRight.scene = rendering.Open3DScene(self._window.renderer)
        self._window.add_child(self._widgetRight)

        self._coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
        self._fetal_head = self.load_fetal_head("whole_head_model.stl")

        self._previous_transform = np.eye(4)
        self._roll_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._pitch_rolling = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self._head_pin_locations = read_head_pin_locations_from_file()
        vertices = np.asarray(self._fetal_head.vertices)

        self._mat = rendering.MaterialRecord()
        self._mat.shader = 'defaultLit'

        self.set_window_layout()
        self.setup_scenes()

        # map data to vertex at a given index. The index is in pin location
        # manual assignment prevents the use of two hash maps; firstly mapping pin locations to head sensor spots
        # then mapping head sensor spots to location on data array
        # either way would have to manually map the pin locations to head sensor spots
        # this problem arose because the pin locations given on the VisualiserWithVertexSelection picked_points()
        # function were random (I don't think I chose them randomly, if I did then manual mapping could be avoided?)
        # eg. data[0] = B0 = vertices[pin_location[27]]
        # data[1] =

        # what do i want to do
        # I have to go through the mesh and find the spheres as I've done before
        # but when I do it this time, assign each vertex value to the corresponding data point
        # because when I get the saved vertices, I won't know which one is which
        # but how do i save it each time

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
        bounds = self._widgetLeft.scene.bounding_box
        self._widgetLeft.setup_camera(60, bounds, bounds.get_center())

        self._widgetRight.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetRight.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)
        bounds = self._widgetRight.scene.bounding_box
        self._widgetRight.setup_camera(60, bounds, bounds.get_center())

    def set_window_layout(self):
        r = self._window.content_rect
        self._widgetLeft.frame = gui.Rect(r.x, r.y, r.width / 2, r.height)
        self._widgetRight.frame = gui.Rect(r.x + r.width / 2 + 1, r.y, r.width / 2, r.height)

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

        self.update_pose(serial_data[0])
        self.update_colour(serial_data[1])

        self._widgetLeft.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetLeft.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)

        self._widgetRight.scene.add_geometry("__fetal_head__", self._fetal_head, self._mat)
        self._widgetRight.scene.add_geometry("__coord_frame__", self._coord_frame, self._mat)

        self._window.post_redraw()

    def update_pose(self, acceleration_data):
        self._fetal_head.transform(np.linalg.inv(self._previous_transform))
        # roll, pitch = calculate_head_angles(acceleration_data)
        #
        # self._roll_rolling.append(roll)
        # self._roll_rolling.pop(0)
        #
        # self._pitch_rolling.append(pitch)
        # self._pitch_rolling.pop(0)
        #
        # transform = np.eye(4)
        # transform[:3, :3] = self._fetal_head.get_rotation_matrix_from_xyz((0,
        #                                                                    circmean(self._roll_rolling, high=2) * np.pi/180,
        #                                                                    0))
        accel_aqua = AQUA(None,
                          np.asarray((acceleration_data.x_cal, acceleration_data.y_cal, acceleration_data.z_cal)),
                          None)
        quaternion = accel_aqua.estimate(accel_aqua.acc, None)

        transform = np.eye(4)
        transform[:3, :3] = self._fetal_head.get_rotation_matrix_from_quaternion(quaternion)

        self._fetal_head.transform(transform)

        self._previous_transform = transform

    def update_colour(self, pressure_sensor_data):
        # vertices = np.asarray(fetal_head.vertices)
        # # code to paint the head instead of making spheres
        # vertex_points_to_colour = []
        # for pin_location in pin_locations:
        #     point_coord = vertices[pin_location]
        #     vertex_points_to_colour.append(pin_location)
        #     for index in range(len(vertices) - 1):
        #         if np.sqrt((vertices[index][0] - point_coord[0][0]) ** 2 +
        #                    (vertices[index][1] - point_coord[0][1]) ** 2 +
        #                    (vertices[index][2] - point_coord[0][2]) ** 2) <= 6:
        #             vertex_points_to_colour.append(index)
        #
        # vertex_colors = np.asarray(fetal_head.vertex_colors)
        # for vertex in vertex_points_to_colour:
        #     vertex_colors[vertex] = [1, 0, 0]

        vertex_colors = np.asarray(self._fetal_head.vertex_colors)
        for vertex in self._head_pin_locations:
            vertex_colors[vertex] = [1, 0, 0]

        self._fetal_head.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)


# returns roll, pitch
def calculate_head_angles(acceleration_data: AccelerationData):
    return [math.atan2(acceleration_data.y_cal, acceleration_data.z_cal),
            math.atan2(-acceleration_data.x_cal,
                       np.sqrt(acceleration_data.y_cal**2 + acceleration_data.z_cal**2))]


def read_head_pin_locations_from_file():
    pin_locations = []
    with open(r"pin_locations.txt", "r") as f:
        for line in f:
            pin_locations.append([int(e) for e in line.split()])
    return pin_locations


class SerialData:
    def __init__(self):
        self._ser = serial.Serial()
        self._ser.baudrate = 115200
        self._ser.port = 'COM3'
        self._ser.open()

        self._acceleration_data = AccelerationData()
        self._pressure_data = PressureData()

    def read_from_serial(self):
        def calibrate_and_assign_acceleration_data(data_x, data_y, data_z):
            # calibrated values need to be in m/s^2 for algorithm
            x = data_x.to_bytes(2, 'big', signed=True)
            x_raw = (x[0] << 8 | x[1]) >> 4
            x_cal = ((x_raw / (1 << 11)) - 1) * 9.81

            y = data_y.to_bytes(2, 'big', signed=True)
            y_raw = (y[0] << 8 | y[1]) >> 4
            y_cal = ((y_raw / (1 << 11)) - 1) * 9.81

            z = data_z.to_bytes(2, 'big', signed=True)
            z_raw = (z[0] << 8 | z[1]) >> 4
            z_cal = ((z_raw / (1 << 11)) - 1) * 9.81

            self._acceleration_data.assign_variables(x_raw, y_raw, z_raw, x_cal, y_cal, z_cal)

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

        self._ser.flushInput()
        align_to_header(self._ser)
        num_readings = 64
        raw_data_array = unpack(f"<{num_readings}h", self._ser.read(num_readings * 2))

        calibrate_and_assign_acceleration_data(raw_data_array[13], raw_data_array[14], raw_data_array[15])
        self._pressure_data.back_sensors = raw_data_array[0:12]
        self._pressure_data.right_sensors = raw_data_array[16:28]
        self._pressure_data.left_sensors = raw_data_array[32:44]
        self._pressure_data.front_sensors = raw_data_array[48:60]

        return self._acceleration_data, self._pressure_data


def main():
    # Initialise gui display and serial instance
    serial_instance = SerialData()
    app_instance = AppWindow()

    while app_instance._app.run_one_tick():
        app_instance.update_geometry(serial_instance.read_from_serial())

if __name__ == "__main__":
    main()
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

# another option (slower), that adds spheres where the points are
# def add_spheres_to_head():
    # vertices = np.asarray(fetal_head_r.vertices)
    # spheres = []
    # for point in points:
    #     point_coord = vertices[point]
    #     print(point_coord[0][0], point_coord[0][1], point_coord[0][2])
    #     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=6, resolution=5)
    #     sphere.translate(np.transpose(point_coord))
    #     spheres.append(sphere)
    #     vis.add_geometry(sphere)

# don't need once already picked points
# def get_picked_points_from_visualiser(vis):
#     vertices_list = vis.get_picked_points()
#
#     with open(r"pin_locations.txt", "w") as fp:
#         for item in vertices_list:
#             fp.write("%s\n" % item.index)

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

# This works but have to resort to removing/readding geometry because there is no method to update the colors
# of the mesh in real time for the gui :(
# However it does work when using the visualiser library
# def update_geometry(self, fetal_head, i):
#     transform = np.eye(4)
#     transform[:3, :3] = fetal_head.get_rotation_matrix_from_xyz((i * np.pi/180, 0, 0))
#
#     self._widgetLeft.scene.set_geometry_transform("__fetal_head__", transform)
#     self._widgetRight.scene.set_geometry_transform("__fetal_head__", transform)
#
#     self._window.post_redraw()