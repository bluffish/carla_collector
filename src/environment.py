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

        self.world = self.carla.get_world()
        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.Foliage)

        if self.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = tick_interval
            self.world.apply_settings(settings)

        self.frame = 0
        self.timestamp = 0.0
        self.count = 0

        self.vehicles = []

    def add_vehicle(self, ego=False):
        vehicle = Vehicle(self, ego=ego)
        self.vehicles.append(vehicle)
        return vehicle

    def run_episode(self, name, num_ego=5, num_traffic=50, episode_length=50):
        print("Starting new episode...")

        ego_vehicles = []

        for i in tqdm(range(0, num_ego), desc="Spawning ego vehicles..."):
            ego_vehicles.append(self.add_vehicle(ego=True))

        for i in tqdm(range(0, num_traffic), desc="Spawning traffic..."):
            self.add_vehicle()

        self.world.set_weather(getattr(carla.WeatherParameters, random.choice([
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
        ])))

        if os.path.exists(os.path.join("./"+name, "agents", "0", "back_camera")) and self.count == -1:
            self.count = len(os.listdir(os.path.join("./"+name, "agents", "0", "back_camera")))

        for tick in tqdm(range(0, episode_length*5), desc="Gathering data..."):
            if tick % 5 == 0:
                for vehicle_id, vehicle in enumerate(ego_vehicles):
                    vehicle.tick()

                    agent_path = os.path.join("./"+name, "agents", str(vehicle_id))

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

                            if hasattr(sensor, 'sensor_options'):
                                info_data['sensors'][sensor_name]['sensor_options'] = sensor.sensor_options

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

        time.sleep(0.5)

        print("Done\n")

