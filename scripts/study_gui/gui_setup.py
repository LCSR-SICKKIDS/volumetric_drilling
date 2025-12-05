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
    def __init__(self, id, val: str, ptype: ParamType):
        self._id = id
        self._param = val
        self._ptype = ptype

    def get_id(self):
        return self._id

    def get(self):
        return self._param

    def set(self, val: str):
        self._param = val

    def get_type(self):
        return self._ptype
    
    def set_type(self, ptype):
        self._ptype = ptype


class GUIConfiguration:
    def __init__(self, setup_filename):
        self.setup_file_name = setup_filename
        self.configuration_filepath = pathlib.Path(self.setup_file_name)

        self.param_keys = {"ambf_executable": ParamType.FILE,
                           "pupil_capture_executable": ParamType.FILE,
                           "recording_script_executable": ParamType.FILE,
                           "recording_base_path": ParamType.FOLDER,
                           "launch_file": ParamType.FILE,
                           "footpedal_device": ParamType.FILE}
        self.params = {}
        self.yaml_data = None
        self.volumes_info = []

        self.load_params_from_file()
        self.load_volumes_info_from_file()


    def print_volumes_info(self):
        for v in self.volumes_info:
            print('-------')
            v.print_info()

    def print_params(self):
        print('Printing GUI Params:')
        for k,v in self.params.items():
            print('\t', k, v.get())

    def _load_yaml_data(self, yaml_filepath=None):
        if yaml_filepath is None:
            yaml_filepath = self.configuration_filepath
        yaml_file = open(str(yaml_filepath), 'r')
        yaml_data = yaml.safe_load(yaml_file)
        yaml_file.close()
        return yaml_data

    def save_params_to_file(self, configuration_filepath=None):
        if configuration_filepath is None:
            configuration_filepath = self.configuration_filepath
        for k, v in self.params.items():
            self.yaml_data[k] = v.get()
            print("\tGUI Param: ", k, " | Raw Value: ", v.get())
        yaml_file = open(str(configuration_filepath), 'w')
        yaml.dump(self.yaml_data, yaml_file)
        yaml_file.close()

    def load_params_from_file(self, yaml_filepath=None):
        self.yaml_data = self._load_yaml_data(yaml_filepath)
        for k, v in self.param_keys.items():
            if self.params.get(k) is None:
                self.params[k] = GUIParam(k, self.yaml_data[k], v)
            else:
                self.params[k].set(self.yaml_data[k])
                self.params[k].set_type(v)
        # self.print()

    def load_volumes_info_from_file(self, yaml_filepath=None):
        self.yaml_data = self._load_yaml_data(yaml_filepath)
        volumes_data = self.yaml_data['volumes']
        self.volumes_info.clear()
        for v in volumes_data:
            adf_path = self.yaml_data[v]['adf_path']
            icon_path = self.yaml_data[v]['icon_path']
            name = self.yaml_data[v]['name']
            # print('Parsing: ', v, ' ADF Path: ', adf_path, ' Icon Path: ', icon_path)
            new_volume_info = VolumeInfo(name, adf_path, icon_path)
            self.volumes_info.append(new_volume_info)
        


def main():
    gs = GUIConfiguration('gui_setup.yaml')
    gs.print_volumes_info()
    gs.print_params()


if __name__ == "__main__":
    main()


