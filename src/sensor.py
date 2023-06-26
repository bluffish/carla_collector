import math

import carla
import numpy as np
from src.utils import get_intrinsics


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


class Camera(Sensor):
    def __init__(self, parent, sensor_type, transform, sensor_options):
        super().__init__(parent, sensor_type)

        self.transform = transform

        self.sensor = self.init_sensor(self.sensor_type, transform, parent, sensor_options)
        self.sensor_options = sensor_options

        self.data = np.zeros(shape=(sensor_options['image_size_y'], sensor_options['image_size_x']))
        self.converter = None

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        camera_bp = self.world.get_blueprint_library().find(sensor_type)

        for key, value in sensor_options.items():
            camera_bp.set_attribute(key, str(value))

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
        camera.calibration = get_intrinsics(sensor_options)
        camera.listen(self.save)

        return camera

    def save(self, image):
        image.convert(self.converter)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        self.data = array[:, :, ::-1]

    def fetch(self):
        return self.data


class RGBCamera(Camera):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.rgb', transform, sensor_options)
        self.converter = carla.ColorConverter.Raw


class SemanticSegmentationCamera(Camera):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.semantic_segmentation', transform, sensor_options)
        self.converter = carla.ColorConverter.CityScapesPalette


class DepthCamera(Camera):
    def __init__(self, parent, transform, sensor_options):
        super().__init__(parent, 'sensor.camera.depth', transform, sensor_options)
        self.converter = carla.ColorConverter.LogarithmicDepth
