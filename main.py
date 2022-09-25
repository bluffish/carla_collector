import traceback

from src.environment import Environment
from contextlib import closing

import subprocess
import argparse
import signal
import socket
import json
import time
import os


def find_free_port():
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return s.getsockname()[1]


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    args = parser.parse_args()

    print(f"Using config {args.config}");
    with open(args.config) as f:
        conf = json.load(f)

    name = conf['name']
    episode_count = conf['episode_count']
    carla_port = conf['carla_port']

    if carla_port == -1:
        carla_port = find_free_port()
        print(f"Using port {carla_port}")

    carla_host = conf['carla_host']
    carla_timeout = conf['carla_timeout']
    tick_interval = conf['tick_interval']
    num_ego = conf['num_ego']
    num_traffic = conf['number-of-vehicles']
    episode_length = conf['episode_length']
    start_tick = conf['start_tick']

    carla = None

    try:
        print("Starting CARLA...")
        carla = subprocess.Popen(["../carla/CarlaUE4.sh", "-RenderOffScreen", f"-world-port={carla_port}", "-quality", "-level=Epic"])
        time.sleep(20)
        print("Done...")

        env = Environment(carla_host=carla_host, carla_port=carla_port, carla_timeout=carla_timeout,
                          tick_interval=tick_interval)

        env.count = start_tick

        for i in range(0, episode_count):
            env.run_episode(name, num_ego=num_ego, num_traffic=num_traffic,
                            episode_length=episode_length)

        print(f"Done gathering {episode_count} episodes.")
        print("Exiting...")
        os.killpg(os.getpgid(carla.pid), signal.SIGTERM)
    except Exception as e:
        print(traceback.format_exc())
        print("Exiting...")
        os.killpg(os.getpgid(carla.pid), signal.SIGTERM)


if __name__ == '__main__':
    run()
