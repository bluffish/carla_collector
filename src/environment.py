import json
import os
import random
import time

import carla
import numpy as np

from tqdm import tqdm
from src.utils import find_free_port
from src.vehicle import Vehicle
from PIL import Image


def get_intrinsics(sensor_options):
    intrinsics = np.identity(3)
    intrinsics[0, 2] = sensor_options['image_size_x'] / 2.0
    intrinsics[1, 2] = sensor_options['image_size_y'] / 2.0
    intrinsics[0, 0] = intrinsics[1, 1] = sensor_options['image_size_x'] / (
            2.0 * np.tan(sensor_options['fov'] * np.pi / 360.0))

    return intrinsics


def get_extrinsics(translation, rotation):
    pitch, yaw, roll = np.radians([rotation.pitch, rotation.yaw, -rotation.roll])

    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    R = np.array([
        [cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr, sy * cp],
        [cp * sr, cp * cr, -sp],
        [-sy * cr + cy * sp * sr, sy * sr + cy * sp * cr, cy * cp]
    ])

    t = np.array([translation.x, translation.y, translation.z]).reshape(3, 1)

    H = np.zeros((4, 4))
    H[0:3, 0:3] = R
    H[0:3, 3] = t[:, 0]
    H[3, 3] = 1

    return H


class Environment:
    def __init__(
            self,
            carla_host='127.0.0.1',
            carla_port='6000',
            carla_timeout=5.0,
            sync=True,
            tick_interval=0.05):

        self.carla_host = carla_host
        self.carla_port = carla_port
        self.carla_timeout = carla_timeout
        self.tick_interval = tick_interval
        self.sync = sync

        self.carla = carla.Client(self.carla_host, int(self.carla_port))
        self.carla.set_timeout(self.carla_timeout)
        self.traffic_manager_port = find_free_port()
        self.traffic_manager = self.carla.get_trafficmanager(self.traffic_manager_port)

        # self.load_world("Town01_Opt", tick_interval, sync=sync)

        self.frame = 0
        self.timestamp = 0.0
        self.count = 0

        self.vehicles = []

        self.weather = [
            "Default",
            "ClearNoon",
            "CloudyNoon",
            "WetNoon",
            "WetCloudyNoon",
            "MidRainyNoon",
            "HardRainNoon",
            "SoftRainNoon",
            "ClearSunset",
            "CloudySunset",
            "WetSunset",
            "WetCloudySunset",
            "MidRainSunset",
            "HardRainSunset",
            "SoftRainSunset",
        ]

        self.towns = [
            "Town01_Opt",
            "Town02_Opt",
            "Town04_Opt",
            "Town05_Opt",
            "Town06_Opt",
            "Town07_Opt",
            "Town10HD_Opt",
        ]

        self.current_town = -1

    def load_world(self, town, tick_interval, sync=True):
        self.world = self.carla.load_world(town)

        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.Foliage)

        if self.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = tick_interval
            self.world.apply_settings(settings)

        self.world.unload_map_layer(carla.MapLayer.Foliage)

    def add_vehicle(self, ego=False, ood=False):
        vehicle = Vehicle(self, ego=ego, ood=ood)
        self.vehicles.append(vehicle)
        return vehicle

    def run_episode(self, save_path, num_ego=5, num_traffic=50, episode_length=50):
        self.current_town += 1
        self.current_town = self.current_town % len(self.towns)

        print(f"Starting new episode on {self.towns[self.current_town]}")
        self.load_world(self.towns[self.current_town], self.tick_interval, sync=self.sync)

        ego_vehicles = []

        for i in tqdm(range(0, num_ego), desc="Spawning ego vehicles..."):
            ego_vehicles.append(self.add_vehicle(ego=True))

        for i in tqdm(range(0, num_traffic), desc="Spawning traffic..."):
            self.add_vehicle(ood=False)

        self.world.set_weather(getattr(carla.WeatherParameters, random.choice(self.weather)))

        if os.path.exists(os.path.join("./", save_path, "agents", "0", "back_camera")) and self.count == -1:
            self.count = len(os.listdir(os.path.join("./", save_path, "agents", "0", "back_camera")))

        for i in range(5):
            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        for tick in tqdm(range(0, episode_length*5), desc="Gathering data..."):
            if tick % 5 == 0:
                for vehicle_id, vehicle in enumerate(ego_vehicles):
                    vehicle.tick()

                    agent_path = os.path.join("./", save_path, "agents", str(vehicle_id))

                    if not os.path.exists(agent_path):
                        os.makedirs(agent_path, exist_ok=True)

                        info_data = {
                            'sensors': {},
                        }

                        for sensor_name, sensor in vehicle.sensors.items():
                            info_data['sensors'][sensor_name] = {}
                            info_data['sensors'][sensor_name]['sensor_type'] = sensor.sensor_type
                            if hasattr(sensor, 'transform'):
                                info_data['sensors'][sensor_name]['transform'] = {
                                    'location': [
                                        sensor.transform.location.x,
                                        sensor.transform.location.y,
                                        sensor.transform.location.z,
                                    ],
                                    'rotation': [
                                        sensor.transform.rotation.yaw,
                                        sensor.transform.rotation.pitch,
                                        sensor.transform.rotation.roll,
                                    ],
                                }

                                info_data['sensors'][sensor_name]['extrinsic'] = get_extrinsics(sensor.transform.location, sensor.transform.rotation).tolist()

                            if hasattr(sensor, 'sensor_options'):
                                info_data['sensors'][sensor_name]['sensor_options'] = sensor.sensor_options
                                info_data['sensors'][sensor_name]['intrinsic'] = get_intrinsics(sensor.sensor_options).tolist()

                        with open(os.path.join(agent_path, 'sensors.json'), 'w') as f:
                            json.dump(info_data, f)

                    for sensor_name, sensor in vehicle.sensors.items():
                        sensor_path = os.path.join(agent_path, sensor_name)
                        os.makedirs(sensor_path, exist_ok=True)
                        data = sensor.fetch()

                        if sensor.sensor_type == 'sensor.camera.rgb':
                            im = Image.fromarray(data, mode='RGB')

                            with open(os.path.join(sensor_path, str(self.count) + '.png'), 'wb') as f:
                                im.save(f, format='PNG')

                        elif sensor.sensor_type == 'sensor.camera.depth':
                            im = Image.fromarray(data, mode='RGB')

                            with open(os.path.join(sensor_path, str(self.count) + ".png"), 'wb') as f:
                                im.save(f, format='PNG')

                        elif sensor.sensor_type == 'sensor.camera.semantic_segmentation':
                            im = Image.fromarray(data, mode='RGB')

                            with open(os.path.join(sensor_path, str(self.count) + '.png'), 'wb') as f:
                                im.save(f, format='PNG')

                self.count += 1

            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        for vehicle in self.vehicles:
            vehicle.destroy()
        self.vehicles = []

        time.sleep(2)

        print("Done\n")

