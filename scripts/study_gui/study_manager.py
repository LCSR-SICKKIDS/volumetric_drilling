import os
import subprocess
from pupil_manager import *

class StudyManager:
    def __init__(self, ambf_executable_path, pupil_executable_path):
        self.ambf_executable_path = str(ambf_executable_path)
        self.pupil_executable_path = str(pupil_executable_path)
        self.pupil_manager = PupilManager()
        self.ambf_handle = None
        self.pupil_service_handle = None
        self.xdotool_handle = None
        self._xdtool_window_key_cmd_prefix = 'xdotool key --window '

    def start_simulation(self, args):
        if not self.ambf_handle:
            self._launch_simulator(args)
        else:
            poll = self.ambf_handle.poll()
            if poll is None:
                print('INFO! AMBF Simulator already running. Close it to reopen again')
            else:
                self._launch_simulator(args)

    def _launch_simulator(self, args):
        print('Launch args: ', args)
        args_list = []
        args_list.append(self.ambf_executable_path)
        for a in args:
            args_list.append(a)
        self.ambf_handle = None
        self.ambf_handle = subprocess.Popen(args_list)

    def _get_ambf_main_window_handle(self):
        xdtool_str = 'xdotool search --class AMBF\ Simulator\ Window\ 1'
        xdtool_proc = subprocess.Popen(xdtool_str, shell=True, stdout=subprocess.PIPE)
        window_id = xdtool_proc.communicate()[0]
        window_id_str = window_id.decode().replace('\n', '')
        # print("AMBF Main Window ID: ", window_id_str)
        return window_id_str

    def close_simulation(self):
        if self.ambf_handle:
            self.ambf_handle.terminate()
            self.ambf_handle = None

    def start_pupil_service(self):
        if not self.pupil_service_handle:
            self._launch_pupil_service()
        else:
            poll = self.pupil_service_handle.poll()
            if poll is None:
                print('INFO! Pupil service already running. Close it to reopen again')
            else:
                self._launch_pupil_service()

    def toggle_volume_smoothening(self):
        self.send_xdotool_keycmd(self._get_ambf_main_window_handle(), 'alt+s')

    def toggle_shadows(self):
        self.send_xdotool_keycmd(self._get_ambf_main_window_handle(), 'ctrl+e')

    def reset_drill(self):
        self.send_xdotool_keycmd(self._get_ambf_main_window_handle(), 'ctrl+r')

    def reset_volume(self):
        self.send_xdotool_keycmd(self._get_ambf_main_window_handle(), 'alt+r')

    def send_xdotool_keycmd(self, window, key_str):
        if window is None:
            print("ERROR! AMBF Window Not Launched")
        else:
            cmd_str = self._xdtool_window_key_cmd_prefix + window + ' ' + key_str
            proc = subprocess.Popen(cmd_str, shell=True)
            print("Running Command ", cmd_str)

    def _launch_pupil_service(self):
        self.pupil_service_handle = None
        self.pupil_service_handle = subprocess.Popen(self.pupil_executable_path)

    def close_pupil_service(self):
        if self.pupil_service_handle:
            self.pupil_service_handle.terminate()
            self.pupil_service_handle = None

    def start_recording(self, path):
        os.makedirs(path)
        self.pupil_manager.sync_time()
        self.pupil_manager.start_recording(path)

    def stop_recording(self):
        self.pupil_manager.stop_recoding()

    def close(self):
        self.close_simulation()
        self.close_pupil_service()







