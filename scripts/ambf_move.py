import time

import numpy as np
from ambf_client import Client


def move(object_name='main_camera'):
    # Moves the object in sinusoidally
    t = np.linspace(0.0, 6.28, num=100, endpoint=True)

    path_x = -np.sin(t)
    path_y = -0.5 * np.sin(t * 2)
    path_z = -0.5 * np.sin(t / 2)

    obj = _client.get_obj_handle(object_name)
    time.sleep(0.2)  # Always good to put a small wait after getting an obj handle for initialization
    rpy = obj.get_rpy()  # Get the init RPY
    init_pos = obj.get_pos()  # Get the init pose
    print(init_pos)
    obj.set_rpy(rpy[0], rpy[1], rpy[2])  # Set the RPY, can also directly set the Quaternion (otherwise rpy is NaN)

    for i in range(len(path_x)):
        temp_pos = [path_x[i] + init_pos.x, path_y[i] + init_pos.y, path_z[i] + init_pos.z]
        print(temp_pos)
        obj.set_pos(temp_pos[0], temp_pos[1], temp_pos[2])
        time.sleep(0.2)


if __name__ == '__main__':
    _client = Client()
    _client.connect()

    move(object_name='mastoidectomy_drill')
    _client.clean_up()
