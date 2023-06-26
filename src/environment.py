import json
import os
import random
import time

import carla
import numpy as np

from tqdm import tqdm
from src.utils import find_free_port
from src.vehicle import Vehicle
from src.utils import get_intrinsics
from PIL import Image


class Environment:
    def __init__(
        self,
        carla_host='127.0.0.1',
        carla_port='6000',
        carla_timeout=5.0,
        sync=True,
        tick_interval=0.05
    ):

        self.world = None
        self.original_settings = None
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.carla_timeout = carla_timeout
        self.tick_interval = tick_interval
        self.sync = sync

        self.carla = carla.Client(self.carla_host, int(self.carla_port))
        self.carla.set_timeout(self.carla_timeout)

        self.traffic_manager_port = find_free_port()
        self.traffic_manager = self.carla.get_trafficmanager(self.traffic_manager_port)

        self.frame, self.count = 0, 0
        self.timestamp = 0.0

        self.vehicles = []

        self.weather = [
            "ClearNoon",
            "CloudyNoon",
            "WetNoon",
            "MidRainyNoon",
            "SoftRainNoon",
            "ClearSunset",
            "CloudySunset",
            "WetSunset",
            "WetCloudySunset",
            "SoftRainSunset",
        ]

        self.town_index = 0
        self.towns = [
            "Town10HD_Opt",
            "Town03_Opt",
            "Town05_Opt",
            "Town02_Opt"
        ]

    def load_world(self, town):
        self.world = self.carla.load_world(town)

        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.Foliage)
        self.world.unload_map_layer(carla.MapLayer.Props)
        self.world.unload_map_layer(carla.MapLayer.StreetLights)

        if self.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.tick_interval
            self.world.apply_settings(settings)

    def add_vehicle(self, ego=False, ood=False):
        vehicle = Vehicle(self, ego=ego, ood=ood)
        self.vehicles.append(vehicle)
        return vehicle

    def run_episode(self, save_path, num_ego=5, num_traffic=50, episode_length=50):
        ego_vehicles = []

        print(f"Using town: {self.towns[self.town_index]}")
        self.load_world(self.towns[self.town_index])
        self.town_index += 1
        self.town_index %= len(self.towns)

        weather_index = 0

        for _ in tqdm(range(num_ego), desc="Spawning ego vehicles..."):
            ego_vehicles.append(self.add_vehicle(ego=True))
        for _ in tqdm(range(num_traffic), desc="Spawning traffic..."):
            self.add_vehicle(ood=False)

        if os.path.exists(os.path.join("./", save_path, "agents", "0", "back_camera")) and self.count == -1:
            self.count = len(os.listdir(os.path.join("./", save_path, "agents", "0", "back_camera")))

        for _ in range(5):
            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        for vehicle_id, vehicle in enumerate(ego_vehicles):
            info_data = {'sensors': {}}
            agent_path = os.path.join("./", save_path, "agents", str(vehicle_id))
            os.makedirs(agent_path, exist_ok=True)

            for sensor_name, sensor in vehicle.sensors.items():
                info_data['sensors'][sensor_name] = {
                    'sensor_options': sensor.sensor_options,
                    'intrinsic': get_intrinsics(sensor.sensor_options).tolist(),
                    'sensor_type': sensor.sensor_type,
                    'transform': {
                        'location': [sensor.transform.location.x, sensor.transform.location.y,
                                     sensor.transform.location.z],
                        'rotation': [sensor.transform.rotation.yaw, sensor.transform.rotation.pitch,
                                     sensor.transform.rotation.roll],
                    },
                }

            with open(os.path.join(agent_path, 'sensors.json'), 'w') as f:
                json.dump(info_data, f)

        bar = tqdm(range(episode_length * 5))
        for tick in bar:
            if tick % (episode_length / 2) == 0:
                self.world.set_weather(getattr(carla.WeatherParameters, self.weather[weather_index]))
                bar.set_description(f"Current weather: {self.weather[weather_index]}")
                weather_index += 1

            if tick % 5 == 0:
                for vehicle_id, vehicle in enumerate(ego_vehicles):
                    agent_path = os.path.join("./", save_path, "agents", str(vehicle_id))
                    vehicle.tick()

                    for sensor_name, sensor in vehicle.sensors.items():
                        sensor_path = os.path.join(agent_path, sensor_name)
                        os.makedirs(sensor_path, exist_ok=True)
                        data = sensor.fetch()

                        if sensor.sensor_type == 'sensor.camera.rgb' \
                                or sensor.sensor_type == 'sensor.camera.depth' \
                                or sensor.sensor_type == 'sensor.camera.semantic_segmentation':
                            im = Image.fromarray(data, mode='RGB')

                            with open(os.path.join(sensor_path, str(self.count) + '.png'), 'wb') as f:
                                im.save(f, format='PNG')

                self.count += 1

            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        print("Destroying")
        time.sleep(2)

        for vehicle in self.vehicles:
            vehicle.destroy()
        self.vehicles = []
        time.sleep(2)

        print("Done\n")
