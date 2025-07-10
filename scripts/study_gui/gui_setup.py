import yaml
import pathlib


class VolumeInfo:
    def __init__(self, name, adf_path, icon_path):
        self.name = name
        self.adf_path = adf_path
        self.icon_path = icon_path

    def print_info(self):
        print('ADF Path', self.adf_path)
        print('Icon Path', self.icon_path)


class GUIParam:
    def __init__(self, id, val):
        self._id = id
        self._param = val

    def get_id(self):
        return self._id

    def get(self):
        return self._param

    def get_as_str(self):
        return str(self._param)

    def set(self, val):
        self._param = val


class GUIConfiguration:
    def __init__(self, setup_filename):
        self.setup_file_name = setup_filename
        self.yaml_file_path = pathlib.Path(self.setup_file_name)
        self.yaml_file = open(str(self.yaml_file_path), 'r')

        self.param_keys = ["ambf_executable",
                           "pupil_executable",
                           "recording_script_executable",
                           "recording_base_path",
                           "launch_file",
                           "footpedal_device"]
        self.params = {}

        self.yaml_data = yaml.safe_load(self.yaml_file)

        for l in self.param_keys:
            self.params[l] = GUIParam(l, pathlib.Path(self.yaml_data[l]).resolve())

        self.ambf_executable = self.params["ambf_executable"]
        self.pupil_executable = self.params["pupil_executable"]
        self.recording_script = self.params["recording_script_executable"]
        self.recording_base_path = self.params['recording_base_path']
        self.launch_file = self.params["launch_file"]
        self.footpedal_device = self.params["footpedal_device"]

        volumes_data = self.yaml_data['volumes']
        self.volumes_info = []
        for v in volumes_data:
            adf_path = pathlib.Path(self.yaml_data[v]['adf_path']).resolve()
            icon_path = pathlib.Path(self.yaml_data[v]['icon_path']).resolve()
            name = self.yaml_data[v]['name']
            # print('Parsing: ', v, ' ADF Path: ', adf_path, ' Icon Path: ', icon_path)
            new_volume_info = VolumeInfo(name, adf_path, icon_path)
            self.volumes_info.append(new_volume_info)

    def print_volumes_info(self):
        for v in self.volumes_info:
            print('-------')
            v.print_info()

    def print(self):
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
        for l in self.param_keys:
            self.params[l] = GUIParam(l, pathlib.Path(self.yaml_data[l]).resolve())

        self.ambf_executable = self.params["ambf_executable"]
        self.pupil_executable = self.params["pupil_executable"]
        self.recording_script = self.params["recording_script_executable"]
        self.recording_base_path = self.params['recording_base_path']
        self.launch_file = self.params["launch_file"]
        self.footpedal_device = self.params["footpedal_device"]
        self.yaml_file.close()
        # self.print()


def main():
    gs = GUIConfiguration('gui_setup.yaml')
    gs.print_volumes_info()


if __name__ == "__main__()":
    main()


