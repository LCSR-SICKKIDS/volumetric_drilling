import time

import numpy as np

from utils import init_ambf


def move_cam_sin(objects, x=True, y=False, z=False, offset=10):
    # Moves the camera in sinusoidally
    t = np.linspace(0.0, 6.28, num=100, endpoint=True)

    if x:
        path_x = -2.5 * np.sin(t) + offset
        path_y = np.zeros_like(t)
        path_z = np.zeros_like(t)
    elif y:
        path_x = np.zeros_like(t) + offset
        path_y = -2 * np.sin(t)
        path_z = np.zeros_like(t)
    elif z:
        path_x = np.zeros_like(t) + offset
        path_y = np.zeros_like(t)
        path_z = -1.5 * np.sin(t)
    else:
        path_x = -2.5 * np.sin(t) + offset
        path_y = -0.5 * np.sin(t * 2)
        path_z = -0.5 * np.sin(t / 2)

    for i in range(len(path_x)):
        temp_pos = [path_x[i], path_y[i], path_z[i]]
        print(temp_pos)
        objects['main_camera'].set_pos(temp_pos[0], temp_pos[1], temp_pos[2])
        time.sleep(0.2)


if __name__ == '__main__':
    _client, objects = init_ambf('move')
    move_cam_sin(objects, x=True, offset=0)
    _client.clean_up()
