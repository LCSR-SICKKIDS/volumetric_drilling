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

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def toStr(f):
    return f"{f:.3f}"


def WARN_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.WARNING + val + bcolors.ENDC
    return valStr


def WARN2_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKCYAN + val + bcolors.ENDC
    return valStr


def OK_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKGREEN + val + bcolors.ENDC
    return valStr


def INFO_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKBLUE + val + bcolors.ENDC
    return valStr


def FAIL_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.FAIL + val + bcolors.ENDC
    return valStr