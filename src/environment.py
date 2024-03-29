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
        tick_interval=0.05,
        ood=None
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
        self.animal_locations = []
        self.ego_vehicles = []
        self.animals = []

        self.ood = ood

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
            "Town07_Opt",
            "Town02_Opt"
        ]

    def load_world(self, town):
        self.world = self.carla.load_world(town)

        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.Foliage)
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)

        if self.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.tick_interval
            self.world.apply_settings(settings)

    def add_vehicle(self, ego=False, ood=False):
        vehicle = Vehicle(self, ego=ego, ood=ood)
        self.vehicles.append(vehicle)
        return vehicle

    def spawn_animal(self):
        blueprint = self.world.get_blueprint_library().find(random.choice(self.ood))
        animal = None

        k = 0

        while animal is None:
            if k > 30:
                break
            k += 1

            transform = random.choice(self.world.get_map().get_spawn_points())
            transform.location.z = 0.
            transform.rotation.yaw = random.randint(0, 360)

            good = True

            for location in self.animal_locations:
                d = transform.location.distance(location)

                if d < 5.:
                    good = False
                    break

            for actor in self.vehicles:
                d = transform.location.distance(actor.vehicle.get_location())

                if d < 5.:
                    good = False
                    break

            if good:
                animal = self.world.spawn_actor(blueprint, transform)
                self.animal_locations.append(transform.location)
                self.animals.append(animal)

    def run_episode(self, save_path, num_ego=5, num_traffic=50, episode_length=50):
        self.ego_vehicles = []
        self.animal_locations = []
        self.animals = []

        print(f"Using town: {self.towns[self.town_index]}")
        self.load_world(self.towns[self.town_index])
        self.town_index += 1
        self.town_index %= len(self.towns)

        weather_index = 0
        for _ in tqdm(range(num_ego), desc="Spawning ego vehicles..."):
            self.ego_vehicles.append(self.add_vehicle(ego=True))

        for _ in tqdm(range(num_traffic), desc="Spawning traffic..."):
            self.add_vehicle(ood=False)

        for _ in range(5):
            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        if self.ood is not None:
            for _ in tqdm(range(40), "Spawning OOD"):
                self.spawn_animal()

        for _ in range(5):
            if self.sync:
                self.world.tick()
            else:
                self.world.wait_for_tick()

        if os.path.exists(os.path.join("./", save_path, "agents", "0", "back_camera")) and self.count == -1:
            self.count = len(os.listdir(os.path.join("./", save_path, "agents", "0", "back_camera")))
            print(f"Setting count to {self.count}")
        elif self.count == -1
            self.count = 0

        for vehicle_id, vehicle in enumerate(self.ego_vehicles):
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

            for animal in self.animals:
                for actor in self.vehicles:
                    d = animal.get_transform().location.distance(actor.vehicle.get_location())

                    if d < 3.:
                        animal.destroy()
                        self.animals.remove(animal)
                        self.spawn_animal()
                        print(len(self.animals))
                        break

            if tick % 5 == 0:
                for vehicle_id, vehicle in enumerate(self.ego_vehicles):
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
        time.sleep(4)

        for vehicle in self.vehicles:
            vehicle.destroy()
        self.vehicles = []

        time.sleep(4)

        print("Done\n")
