import subprocess


class StudyManager:
    def __init__(self, ambf_executable_path, pupil_executable_path):
        self.ambf_executable_path = ambf_executable_path
        self.pupil_executable_path = pupil_executable_path
        self.ambf_handle = None
        self.pupil_service_handle = None

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

    def _launch_pupil_service(self):
        self.pupil_service_handle = None
        self.pupil_service_handle = subprocess.Popen(self.pupil_executable_path)

    def close_pupil_service(self):
        if self.pupil_service_handle:
            self.pupil_service_handle.terminate()
            self.pupil_service_handle = None

    def close(self):
        self.close_simulation()
        self.close_pupil_service()







