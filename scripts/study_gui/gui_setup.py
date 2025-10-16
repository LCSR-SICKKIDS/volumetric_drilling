import yaml
import pathlib
from enum import Enum


class VolumeInfo:
    def __init__(self, name, adf_path, icon_path):
        self.name = name
        self.adf_path = adf_path
        self.icon_path = icon_path

    def print_info(self):
        print('ADF Path', self.adf_path)
        print('Icon Path', self.icon_path)

class ParamType(Enum):
    FILE=1
    FOLDER=2

class GUIParam:
    def __init__(self, id, val, ptype: ParamType):
        self._id = id
        self._param = val
        self._ptype = ptype

    def get_id(self):
        return self._id

    def get(self):
        return self._param

    def get_as_str(self):
        return str(self._param)

    def set(self, val):
        self._param = val

    def get_type(self):
        return self.get_type
    
    def set_type(self, ptype):
        self._ptype = ptype


class GUIConfiguration:
    def __init__(self, setup_filename):
        self.setup_file_name = setup_filename
        self.yaml_file_path = pathlib.Path(self.setup_file_name)
        self.yaml_file = open(str(self.yaml_file_path), 'r')

        self._ambf_executable = None
        self._pupil_capture_executable = None
        self._recording_script = None
        self._recording_base_path = None
        self._launch_file = None
        self._footpedal_device = None

        self.param_keys = {"ambf_executable": ParamType.FILE,
                           "pupil_capture_executable": ParamType.FILE,
                           "recording_script_executable": ParamType.FILE,
                           "recording_base_path": ParamType.FOLDER,
                           "launch_file": ParamType.FILE,
                           "footpedal_device": ParamType.FILE}
        self.params = {}

        self.yaml_data = yaml.safe_load(self.yaml_file)

        for k, v in self.param_keys.items():
            self.params[k] = GUIParam(k, pathlib.Path(self.yaml_data[k]).resolve(), v)

        self._update_param_varialbes()

        volumes_data = self.yaml_data['volumes']
        self.volumes_info = []
        for v in volumes_data:
            adf_path = pathlib.Path(self.yaml_data[v]['adf_path']).resolve()
            icon_path = pathlib.Path(self.yaml_data[v]['icon_path']).resolve()
            name = self.yaml_data[v]['name']
            # print('Parsing: ', v, ' ADF Path: ', adf_path, ' Icon Path: ', icon_path)
            new_volume_info = VolumeInfo(name, adf_path, icon_path)
            self.volumes_info.append(new_volume_info)

    def _update_param_varialbes(self):
        self._ambf_executable = self.params["ambf_executable"]
        self._pupil_capture_executable = self.params["pupil_capture_executable"]
        self._recording_script = self.params["recording_script_executable"]
        self._recording_base_path = self.params['recording_base_path']
        self._launch_file = self.params["launch_file"]
        self._footpedal_device = self.params["footpedal_device"]

    def print_volumes_info(self):
        for v in self.volumes_info:
            print('-------')
            v.print_info()

    def print_gui_params(self):
        print('Printing GUI Params:')
        for k,v in self.params.items():
            print('\t', k, v.get_as_str())

    def save(self):
        for k, v in self.params.items():
            self.yaml_data[k] = v.get_as_str()
            print(k, v.get_as_str())
        self.yaml_file = open(str(self.yaml_file_path), 'w')
        yaml.dump(self.yaml_data, self.yaml_file)
        self.yaml_file.close()

    def reload(self):
        self.yaml_file = open(str(self.yaml_file_path), 'r')
        for k, v in self.param_keys.items():
            self.params[k] = GUIParam(k, pathlib.Path(self.yaml_data[k]).resolve(), v)
        self._update_param_varialbes()
        self.yaml_file.close()
        # self.print()


def main():
    gs = GUIConfiguration('gui_setup.yaml')
    gs.print_volumes_info()
    gs.print_gui_params()


if __name__ == "__main__":
    main()


