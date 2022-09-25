import math

import carla
import numpy as np
from transforms3d.euler import euler2mat


class Sensor:
    def __init__(self, parent, sensor_type):
        self.parent = parent
        self.sensor = None
        self.sensor_type = sensor_type
        self.world = self.parent.get_world()

        self.listen_callbacks = []

    def listen(self, callback):
        self.listen_callbacks.append(callback)

    def tick(self):
        for callback in self.listen_callbacks:
            callback(self.fetch())

    def fetch(self):
        raise NotImplementedError()

    def destroy(self):
        if self.sensor is not None and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()
            self.sensor = None

        self.listen_callbacks = []


class RGBCamera(Sensor):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.rgb')

        self.transform = transform
        self.sensor = self.init_sensor(self.sensor_type, transform, parent, sensor_options)
        self.sensor_options = sensor_options

        self.camera_bp = None
        self.data = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        self.camera_bp = self.world.get_blueprint_library().find(sensor_type)

        for key, value in sensor_options.items():
            self.camera_bp.set_attribute(key, str(value))

        camera = self.world.spawn_actor(self.camera_bp, transform, attach_to=attached)

        calibration = np.identity(3)
        calibration[0, 2] = sensor_options['image_size_x'] / 2.0
        calibration[1, 2] = sensor_options['image_size_y'] / 2.0
        calibration[0, 0] = calibration[1, 1] = sensor_options['image_size_x'] / (
                    2.0 * np.tan(sensor_options['fov'] * np.pi / 360.0))
        camera.calibration = calibration
        camera.listen(self.save_image)

        return camera

    def get_camera_info(self):
        translation = [self.transform.location.x, self.transform.location.y, self.transform.location.z]

        roll = math.radians(self.transform.rotation.roll-90)
        pitch = -math.radians(self.transform.rotation.pitch)
        yaw = -math.radians(self.transform.rotation.yaw)
        rotation_matrix = euler2mat(roll, pitch, yaw)
        intrinsics = self.sensor.calibration

        return rotation_matrix, translation, intrinsics

    def save_image(self, image):
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        img = array.copy()

        self.data = img

    def fetch(self):
        return self.data


class SemanticSegmentationCamera(Sensor):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.semantic_segmentation')
        self.transform = transform

        self.sensor = self.init_sensor(self.sensor_type, transform, parent, sensor_options)
        self.sensor_options = sensor_options

        self.rgb_image = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))
        self.data = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        camera_bp = self.world.get_blueprint_library().find(sensor_type)

        for key, value in sensor_options.items():
            camera_bp.set_attribute(key, str(value))

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)

        calibration = np.identity(3)
        calibration[0, 2] = sensor_options['image_size_x'] / 2.0
        calibration[1, 2] = sensor_options['image_size_y'] / 2.0
        calibration[0, 0] = calibration[1, 1] = sensor_options['image_size_x'] / (
                    2.0 * np.tan(sensor_options['fov'] * np.pi / 360.0))
        camera.calibration = calibration

        camera.listen(self.save_data)

        return camera

    def save_image(self, image):
        if image is None:
            return

        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        img = array.copy()

        self.rgb_image = img

    def save_data(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.save_image(image)

        self.data = array.copy()

    def fetch(self):
        return self.rgb_image


class DepthCamera(Sensor):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.depth')

        self.transform = transform
        self.sensor = self.init_sensor(self.sensor_type, transform, parent, sensor_options)
        self.sensor_options = sensor_options

        self.rgb_image = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))
        self.data = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        camera_bp = self.world.get_blueprint_library().find(sensor_type)

        for key, value in sensor_options.items():
            camera_bp.set_attribute(key, str(value))

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)

        calibration = np.identity(3)
        calibration[0, 2] = sensor_options['image_size_x'] / 2.0
        calibration[1, 2] = sensor_options['image_size_y'] / 2.0
        calibration[0, 0] = calibration[1, 1] = sensor_options['image_size_x'] / (
                    2.0 * np.tan(sensor_options['fov'] * np.pi / 360.0))
        camera.calibration = calibration

        camera.listen(self.save_data)

        return camera

    def save_data(self, image):
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.data = array.copy()

        self.save_image(image)

    def save_image(self, image):
        image.convert(carla.ColorConverter.LogarithmicDepth)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.rgb_image = array.copy()

    def fetch(self):
        return self.data

    def fetch_image(self):
        return self.rgb_image

