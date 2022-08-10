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


class SetupGUI:
    def __init__(self, setup_filename):
        self.setup_file_name = setup_filename
        self.file_path = pathlib.Path(self.setup_file_name).parent.resolve()
        self.yaml_file = open('gui_setup.yaml', 'r')

        self.yaml_data = yaml.safe_load(self.yaml_file)
        self.ambf_executable_path = pathlib.Path(self.yaml_data["ambf_executable_path"]).resolve()
        self.pupil_executable_path = pathlib.Path(self.yaml_data["pupil_executable_path"])
        self.recording_base_path = pathlib.Path(self.yaml_data['recording_base_path']).resolve()
        self.launch_file = pathlib.Path(self.yaml_data["launch_file_path"]).resolve()
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


def main():
    gs = SetupGUI('gui_setup.yaml')
    gs.print_volumes_info()


if __name__ == "__main__()":
    main()


