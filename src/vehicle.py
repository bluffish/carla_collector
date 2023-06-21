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
                # blueprints = [x for x in blueprints if x.id.endswith('microlino') or
                #               x.id.endswith('carlacola') or
                #               x.id.endswith('cybertruck') or
                #               x.id.endswith('t2') or
                #               x.id.endswith('sprinter') or
                #               x.id.endswith('firetruck') or
                #               x.id.endswith('ambulance')]
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) != 4]
                # print(blueprints)
            else:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]
                blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
                blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

            # print(len(blueprints))
            bp = random.choice(blueprints)

        self.control = carla.VehicleControl()

        self.vehicle = None

        while self.vehicle is None:
            try:
                self.vehicle = self.env.world.spawn_actor(bp, random.choice(self.env.world.get_map().get_spawn_points()))
            except RuntimeError:
                pass

        self.vehicle.apply_control(self.control)
        self.vehicle.set_autopilot(True, self.env.traffic_manager_port)
        self.sensors = {}

        w, h = 480, 224

        if ego:
            self.sensors['left_front_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['left_front_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['left_front_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            self.sensors['front_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=0)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['front_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=0)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['front_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=0)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            self.sensors['right_front_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['right_front_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['right_front_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=60)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            self.sensors['left_back_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['left_back_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['left_back_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=-120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            self.sensors['back_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=180)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['back_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=180)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['back_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=180)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            self.sensors['right_back_camera'] = RGBCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['right_back_camera_semantic'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )
            self.sensors['right_back_camera_depth'] = DepthCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=2.4), carla.Rotation(yaw=120)),
                {
                    'image_size_x': w,
                    'image_size_y': h,
                    'fov': 90,
                }
            )

            fov_degrees = 90
            fov_radians = math.radians(fov_degrees)
            height = (100 / 2) / math.tan(fov_radians / 2)

            self.sensors['birds_view_semantic_camera'] = SemanticSegmentationCamera(
                self.vehicle,
                carla.Transform(carla.Location(x=0, y=0, z=height), carla.Rotation(pitch=-90)),
                {
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
        for sensor in self.sensors.values():
            sensor.destroy()

        self.sensors.clear()

        if self.vehicle is not None and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.vehicle = None
