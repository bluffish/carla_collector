import time

import carla
import random

from src.sensor import *


class Vehicle:
    def __init__(self, env, ego=False, ood=False):
        self.env = env
        self.ego = ego

        bp = None

        if ego:
            bp = self.env.world.get_blueprint_library().filter('charger_2020')[0]
        else:
            blueprints = self.env.world.get_blueprint_library().filter("vehicle.*")

            if ood:
                blueprints = [x for x in blueprints if x.id.endswith('microlino') or
                              x.id.endswith('carlacola') or
                              x.id.endswith('cybertruck') or
                              x.id.endswith('t2') or
                              x.id.endswith('sprinter') or
                              x.id.endswith('firetruck') or
                              x.id.endswith('ambulance')]
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) != 4]
            else:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]
                blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
                blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

            bp = random.choice(blueprints)

        self.control = carla.VehicleControl()

        self.vehicle = None

        while self.vehicle is None:
            try:
                self.vehicle = self.env.world.spawn_actor(bp,
                                                          random.choice(self.env.world.get_map().get_spawn_points()))
            except RuntimeError:
                pass

        self.vehicle.apply_control(self.control)
        self.vehicle.set_autopilot(True, self.env.traffic_manager_port)
        self.sensors = {}

        if ego:
            options = {
                'image_size_x': 480,
                'image_size_y': 224,
                'fov': 90
            }

            translations = {
                'left_front': (0, 0, 2.4, -60),
                'front': (0, 0, 2.4, 0),
                'right_front': (0, 0, 2.4, 60),
                'left_back': (0, 0, 2.4, -120),
                'back_camera': (0, 0, 2.4, 180),
                'right_back': (0, 0, 2.4, 120),
            }

            for sensor, translation in translations.items():
                transform = carla.Transform(carla.Location(x=translation[0], y=translation[1], z=translation[2]), carla.Rotation(yaw=translation[3]))

                self.sensors[f"{sensor}_camera"] = RGBCamera(self.vehicle, transform, options)
                # self.sensors[f"{sensor}_semantic"] = SemanticSegmentationCamera(self.vehicle, transform, options)

            fov_degrees = 90
            bev_height = (100 / 2) / math.tan(math.radians(fov_degrees) / 2)

            self.sensors['bev_semantic'] = SemanticSegmentationCamera(self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=bev_height), carla.Rotation(pitch=-90)), {
                    'image_size_x': 200,
                    'image_size_y': 200,
                    'fov': fov_degrees,
                }
            )

    def tick(self):
        if self.vehicle is None:
            return

        for name, sensor in self.sensors.items():
            sensor.tick()

    def destroy(self):
        if self.vehicle is None:
            return

        for sensor in self.sensors.values():
            sensor.destroy()

        self.sensors.clear()

        if self.vehicle is not None and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.vehicle = None
