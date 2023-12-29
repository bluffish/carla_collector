import argparse
import subprocess
import signal
import time
import yaml

from src.environment import Environment
from src.utils import find_free_port


def run():
    print(f"Using config {args.config}")

    with open(args.config, 'r') as file:
        config = yaml.safe_load(file)

    if config['carla_port'] == -1:
        config['carla_port'] = find_free_port()
        print(f"Using port {config['carla_port']}")
        print("Starting CARLA...")

        carla = subprocess.Popen([
            config['carla_path'],
            "-RenderOffScreen",
            f"-world-port={config['carla_port']}",
            "-quality-level=Epic",
            f"-graphicsadapter={gpu}"
        ])

        time.sleep(10)
        print("Done...")

    if 'ood' in config:
        ood = config['ood']
    else:
        ood = None

    env = Environment(
        carla_host=config['carla_host'],
        carla_port=config['carla_port'],
        carla_timeout=config['carla_timeout'],
        tick_interval=config['tick_interval'],
        ood=ood
    )

    env.count = config['start_tick']

    for i in range(0, config['episode_count']):
        env.run_episode(
            config['save_path'],
            num_ego=config['num_ego'],
            num_traffic=config['num_vehicles'],
            episode_length=config['episode_length']
        )

    print(f"Done gathering {config['episode_count']} episodes.\nExiting...")


if __name__ == '__main__':
    import os
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument('-g', '--gpu', required=False, default=0, type=int)
    args = parser.parse_args()

    os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    config = args.config
    gpu = args.gpu + 1

    if gpu == 1:
        print("GPU 0 is broken... Only allah knows why. Every other gpu works.")

    run()