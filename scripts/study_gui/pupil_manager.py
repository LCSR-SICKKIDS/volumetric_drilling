import zmq
import time


class PupilManager:
    def __init__(self):
        self.ctx = zmq.Context()
        self.pr = zmq.Socket(self.ctx, zmq.REQ)
        self.pr.connect('tcp://127.0.0.1:50020')
        self._recording = False

    def sync_time(self):
        self.pr.send_string('T ' + str(time.time()))
        self.pr.recv_string()

    def start_recording(self, dir_path):
        self.sync_time()
        self.pr.send_string('R ' + dir_path)
        self.pr.recv_string()
        self._recording = True

    def stop_recoding(self):
        if self._recording:
            self.pr.send_string('r')
            self.pr.recv_string()
        self._recording = False
