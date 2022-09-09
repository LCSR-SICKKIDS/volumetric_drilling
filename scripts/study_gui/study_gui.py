from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QProcess
import sys
from study_manager import StudyManager
from gui_setup import SetupGUI
import datetime


class Ui(QtWidgets.QWidget):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout.ui', self)

        self.gui_params = SetupGUI('volumes_info.yaml')
        self.study_manager = StudyManager(self.gui_params.ambf_executable_path, self.gui_params.pupil_executable_path)
        self.active_volume_adf = ''

        # Setup the grid layout for different volumes
        self.volumes_grid = self.findChild(QtWidgets.QGridLayout, 'gridLayout')

        for i in range(len(self.gui_params.volumes_info)):
            vinfo = self.gui_params.volumes_info[i]
            radio_button = QtWidgets.QRadioButton(vinfo.name)
            min_height = 400
            # radio_button.setMinimumHeight(min_height)
            # radio_button.setGeometry(200, 150, 100, 40)
            radio_button.volume_name = vinfo.name
            radio_button.volume_adf = str(vinfo.adf_path)
            radio_button.toggled.connect(self.radio_button_volume_selection)
            icon_path_str = str(vinfo.icon_path)
            print(icon_path_str)
            pixmap = QPixmap(icon_path_str)
            label = QtWidgets.QLabel(self)
            label.resize(150, 350)
            # label.setMinimumHeight(min_height)
            label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio))
            self.volumes_grid.addWidget(radio_button, i, 0)
            self.volumes_grid.addWidget(label, i, 1)
            if i == 0:
                radio_button.setChecked(True)

        self.button_start_simulation = self.findChild(QtWidgets.QPushButton, 'button_start_simulation')
        self.button_start_simulation.clicked.connect(self.pressed_start_simulation)

        self.button_pupil_service = self.findChild(QtWidgets.QPushButton, 'button_pupil_service')
        self.button_pupil_service.clicked.connect(self.pressed_pupil_service)

        self.button_reset_drill = self.findChild(QtWidgets.QPushButton, 'button_reset_drill')
        self.button_reset_drill.clicked.connect(self.study_manager.reset_drill)

        self.button_reset_drill = self.findChild(QtWidgets.QPushButton, 'button_reset_volume')
        self.button_reset_drill.clicked.connect(self.study_manager.reset_volume)

        self.button_reset_drill = self.findChild(QtWidgets.QPushButton, 'button_toggle_shadows')
        self.button_reset_drill.clicked.connect(self.study_manager.toggle_shadows)

        self.button_reset_drill = self.findChild(QtWidgets.QPushButton, 'button_toggle_vol_smooth')
        self.button_reset_drill.clicked.connect(self.study_manager.toggle_volume_smoothening)

        self.button_record_study = self.findChild(QtWidgets.QPushButton, 'button_record_study')
        self.button_record_study.clicked.connect(self.pressed_record_study)
        self.button_record_study.setStyleSheet("background-color: GREEN")

        self.text_participant_name = self.findChild(QtWidgets.QTextEdit, 'textEdit_participant_name')
        self.recording_button = self.findChild(QtWidgets.QPushButton, 'button_record_study')

        self.textEdit_info = self.findChild(QtWidgets.QPlainTextEdit, 'textEdit_info')

        self.textEdit_debug = self.findChild(QtWidgets.QPlainTextEdit, 'textEdit_debug')

        self._recording_study = False

        self._ambf_process = QProcess()
        self._ambf_process.readyReadStandardOutput.connect(self.handle_stdout)
        self._ambf_process.readyReadStandardError.connect(self.handle_stderr)

        self._pupil_process = QProcess()

        self.show()

    def pressed_start_simulation(self):
        args = ['--launch_file', str(self.gui_params.launch_file), '-l', '0,7', '-a', self.active_volume_adf]
        # self.study_manager.start_simulation(args)
        try:
            # self._ambf_process.start('ambf_simulator', args)
            self._ambf_process.start(str(self.gui_params.ambf_executable_path), args)
        except Exception as e:
            self.print('ERROR! Cant launch AMBF')
            print(e)

    def pressed_pupil_service(self):
        try:
            self._pupil_process.start(str(self.gui_params.pupil_executable_path))
        except Exception as e:
            self.print_info('ERROR! Cant launch Pupil Capture')
            print(e)

    def pressed_record_study(self):
        name = self.text_participant_name.toPlainText()
        self.print_info('Recording for : ' + name)

    def radio_button_volume_selection(self):
        button = self.sender()
        if button.isChecked():
            self.print_info('Active Volume is ' + button.volume_name)
            self.active_volume_adf = button.volume_adf

    def is_ready_to_record(self):
        record = True
        if self.text_participant_name.toPlainText() == '':
            self.print_info('INFO! Please enter a participant name')
            record = False
        if self._ambf_process.state() != QProcess.Running:
            self.print_info('INFO! Please run simulation before recording. Ignoring Record Request!')
            record = False
        if self._pupil_process.state() != QProcess.Running:
            self.print_info('INFO! Please launch pupil capture before recording. Ignoring Record Request!')
            record = False
        return record

    def pressed_record_study(self):
        if self._recording_study:
            self.study_manager.stop_recording()
            self._recording_study = False
            self.button_record_study.setText("Record Study")
            self.button_record_study.setStyleSheet("background-color: GREEN")
        else:
            if not self.is_ready_to_record():
                return -1
            base_path = str(self.gui_params.recording_base_path)
            participant_name = '/' + self.text_participant_name.toPlainText()
            date_time = '/' + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.study_manager.start_recording(base_path + participant_name + date_time)
            self._recording_study = True
            self.button_record_study.setText("STOP RECORDING")
            self.button_record_study.setStyleSheet("background-color: RED")

    def closeEvent(self, event):
        self.print_info('Terminate Called')
        self._ambf_process.close()
        self._pupil_process.close()
        self.study_manager.close()
        print('GOOD BYE')

    def get_time_as_str(self):
        return '[' + datetime.datetime.now().strftime("%H:%M:%S") + '] - '

    def print_info(self, msg):
        self.textEdit_info.insertPlainText(self.get_time_as_str() + msg + '\n')

    def print_debug(self, msg):
        self.textEdit_debug.insertPlainText(self.get_time_as_str() + msg)

    def handle_stderr(self):
        msg = bytes(self._ambf_process.readAllStandardError()).decode('utf-8')
        self.print_debug(msg)

    def handle_stdout(self):
        msg = bytes(self._ambf_process.readAllStandardOutput()).decode('utf-8')
        self.print_debug(msg)



app = QtWidgets.QApplication(sys.argv)
window = Ui()
app.exec_()
