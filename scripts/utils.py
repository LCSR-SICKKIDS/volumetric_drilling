import os

from ambf_client import Client


def init_ambf(node_name='default'):
    # Generate all objects in scene, even ones that may not be needed
    _client = Client(node_name)
    _client.connect()
    objects_dict = {}

    # TODO: maybe this can be an argument?
    ignore_list = ['World', 'light']  # remove world and light as we don't need them

    # Create python instances of AMBF objects
    obj_names = _client.get_obj_names()
    for obj_name in obj_names:
        obj_name = os.path.basename(os.path.normpath(obj_name))  # Get the last part of file path
        for ignore in ignore_list:
            if ignore in obj_name:
                break
        else:
            objects_dict[obj_name] = _client.get_obj_handle(obj_name)

    return _client, objects_dict
