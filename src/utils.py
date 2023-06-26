import socket
from contextlib import closing
import numpy as np


def find_free_port():
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return s.getsockname()[1]


def get_intrinsics(sensor_options):
    intrinsics = np.identity(3)
    intrinsics[0, 2] = sensor_options['image_size_x'] / 2.0
    intrinsics[1, 2] = sensor_options['image_size_y'] / 2.0
    intrinsics[0, 0] = intrinsics[1, 1] = sensor_options['image_size_x'] / (
            2.0 * np.tan(sensor_options['fov'] * np.pi / 360.0))

    return intrinsics