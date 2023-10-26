from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtWidgets import QFormLayout, QGroupBox, QPushButton, QFileDialog
import sys
from study_manager import StudyManager, RecordOptions
from gui_setup import GUIConfiguration
import datetime
from enum import Enum
import json


class DialogType(Enum):
    FILE=0
    FOLDER=1


class Ui(QtWidgets.QWidget):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout.ui', self)

        self.gui_configuration = GUIConfiguration('gui_setup.yaml')
        self.study_manager = StudyManager(self.gui_configuration.ambf_executable.get(),
                                          self.gui_configuration.pupil_executable.get(),
                                          self.gui_configuration.recording_script.get())
        self.active_volume_adf = ''
        self.active_volume_name = ''
        self.record_options = RecordOptions()

        # Setup the grid layout for different volumes
        self.scroll_area = self.findChild(QtWidgets.QScrollArea, 'scrollArea')
        formLayout = QFormLayout()
        groupBox = QGroupBox('Anatomies')
        for i in range(len(self.gui_configuration.volumes_info)):
            vinfo = self.gui_configuration.volumes_info[i]
            radio_button = QtWidgets.QRadioButton(vinfo.name)
            min_height = 300
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
            formLayout.addRow(radio_button, label)
            if i == 0:
                radio_button.setChecked(True)
        groupBox.setLayout(formLayout)
        self.scroll_area.setWidget(groupBox)
        self.scroll_area.setWidgetResizable(True)

        self.button_start_simulation = self.findChild(QtWidgets.QPushButton, 'button_start_simulation')
        self.button_start_simulation.setStyleSheet("background-color: GREEN")
        self.button_start_simulation.clicked.connect(self.pressed_start_simulation)

        self.button_launch_vr = self.findChild(QtWidgets.QPushButton, 'button_launch_vr')

        self.button_stream_stereo = self.findChild(QtWidgets.QPushButton, 'button_stream_stereo')

        self.button_stream_depth = self.findChild(QtWidgets.QPushButton, 'button_stream_depth')

        self.button_pupil_service = self.findChild(QtWidgets.QPushButton, 'button_pupil_capture')
        self.button_pupil_service.clicked.connect(self.pressed_pupil_service)

        self.button_reset_drill = self.findChild(QtWidgets.QPushButton, 'button_reset_drill')
        self.button_reset_drill.clicked.connect(self.study_manager.reset_drill)

        self.button_reset_volume = self.findChild(QtWidgets.QPushButton, 'button_reset_volume')
        self.button_reset_volume.clicked.connect(self.study_manager.reset_volume)

        self.button_toggle_shadows = self.findChild(QtWidgets.QPushButton, 'button_toggle_shadows')
        self.button_toggle_shadows.clicked.connect(self.study_manager.toggle_shadows)

        self.button_toggle_vol_smooth = self.findChild(QtWidgets.QPushButton, 'button_toggle_vol_smooth')
        self.button_toggle_vol_smooth.clicked.connect(self.study_manager.toggle_volume_smoothening)

        self.button_record_study = self.findChild(QtWidgets.QPushButton, 'button_record_study')
        self.button_record_study.clicked.connect(self.pressed_record_study)
        self.button_record_study.setStyleSheet("background-color: GREEN")

        self.text_participant_name = self.findChild(QtWidgets.QTextEdit, 'textEdit_participant_name')
        self.recording_button = self.findChild(QtWidgets.QPushButton, 'button_record_study')

        self.textEdit_info = self.findChild(QtWidgets.QPlainTextEdit, 'textEdit_info')

        self.textEdit_debug = self.findChild(QtWidgets.QPlainTextEdit, 'textEdit_debug')

        self.gui_param_dialogs = {}
        self.connect_gui_param_to_dialog('ambf_executable', self.gui_configuration.ambf_executable.get_id(), DialogType.FILE)
        self.connect_gui_param_to_dialog('pupil_capture_executable', self.gui_configuration.pupil_executable.get_id(), DialogType.FILE)
        self.connect_gui_param_to_dialog('launch_file', self.gui_configuration.launch_file.get_id(), DialogType.FILE)
        self.connect_gui_param_to_dialog('recording_script_executable', self.gui_configuration.recording_script.get_id(), DialogType.FILE)
        self.connect_gui_param_to_dialog('recording_base_path', self.gui_configuration.recording_base_path.get_id(), DialogType.FOLDER)

        self.button_save_configuration = self.findChild(QtWidgets.QPushButton, 'pushButton_save_configuration')
        self.button_save_configuration.clicked.connect(self.gui_configuration.save)

        self.button_reload_configuration = self.findChild(QtWidgets.QPushButton, 'pushButton_reload_configuration')
        self.button_reload_configuration.clicked.connect(self.reload_configuration)

        self._recording_study = False

        self._ambf_process = QProcess()
        self._ambf_process.readyReadStandardOutput.connect(self.handle_stdout)
        self._ambf_process.readyReadStandardError.connect(self.handle_stderr)
        self._ambf_process.finished.connect(self._simulation_closed)
        self._pupil_process = QProcess()
        self._recording_process = QProcess()

        self.show()

    def connect_gui_param_to_dialog(self, name, param_id, dialog_type: DialogType):
        exec_file_text_edit = self.findChild(QtWidgets.QTextEdit, 'textEdit_' + name)
        exec_file_btn = self.findChild(QtWidgets.QPushButton, 'pushButton_' + name)
        if dialog_type == DialogType.FILE:
            exec_file_btn.clicked.connect(
                lambda: self.file_dialog(exec_file_text_edit, param_id))
        elif dialog_type == DialogType.FOLDER:
            exec_file_btn.clicked.connect(
                lambda: self.folder_dialog(exec_file_text_edit, param_id))

        exec_file_text_edit.setText(self.gui_configuration.params[param_id].get_as_str())
        self.gui_param_dialogs[param_id] = exec_file_text_edit
        return exec_file_text_edit

    def file_dialog(self, text_edit, param_id):
        file , check = QFileDialog.getOpenFileName(None, "QFileDialog.getOpenFileName()",
            "", "All Files (*);;Python Files (*.py);;Text Files (*.txt)")
        if check:
            print(file)
            self.gui_configuration.params[param_id].set(file)
            text_edit.setText(file)

    def folder_dialog(self, text_edit, param_id):
        folder = QFileDialog.getExistingDirectory(None, "QFileDialog.getExistingDirectory()",
            "",)
        print(folder)
        self.gui_configuration.params[param_id].set(folder)
        text_edit.setText(folder)

    def reload_configuration(self):
        print('Reloading Configuration')
        self.gui_configuration.reload()
        for k, v in self.gui_param_dialogs.items():
            v.setText(self.gui_configuration.params[k].get_as_str())

    def pressed_start_simulation(self):
        launch_file_adf_indices = '0,7'
        if self.button_stream_depth.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',4'
        if self.button_stream_stereo.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',5'
        if self.button_launch_vr.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',6'
        args = ['--launch_file', str(self.gui_configuration.launch_file.get()), '-l', launch_file_adf_indices, '-a', self.active_volume_adf]
        # self.study_manager.start_simulation(args)
        if self._ambf_process.state() != QProcess.Running:
            self._ambf_process.start(str(self.gui_configuration.ambf_executable.get()), args)
            self.button_start_simulation.setText('Close Simulation')
            self.button_start_simulation.setStyleSheet("background-color: RED")
        else:
            self._ambf_process.close()

    def _simulation_closed(self):
        self.button_start_simulation.setText('Start Simulation')
        self.button_start_simulation.setStyleSheet("background-color: GREEN")
        pass

    def pressed_pupil_service(self):
        try:
            self._pupil_process.start(str(self.gui_configuration.pupil_executable.get()))
        except Exception as e:
            self.print_info('ERROR! Cant launch Pupil Capture')
            print(e)

    def radio_button_volume_selection(self):
        button = self.sender()
        if button.isChecked():
            self.print_info('Active Volume is ' + button.volume_name)
            self.active_volume_adf = button.volume_adf
            self.active_volume_name = button.volume_name

    def is_ready_to_record(self):
        ready = True
        if self.text_participant_name.toPlainText() == '':
            self.print_info('ERROR! Please enter a participant name. Ignoring Record Request')
            ready = ready & False
        if self._ambf_process.state() != QProcess.Running:
            self.print_info('ERROR! Please run simulation before recording. Ignoring Record Request!')
            ready = ready & False

        if ready and self._pupil_process.state() != QProcess.Running:
            self.print_info('WARNING! Pupil capture not initialized!')
            dlg = QtWidgets.QMessageBox(self)
            dlg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            dlg.setText('Pupil capture not running. No pupil data will be recorded. OK?')
            button = dlg.exec()
            if button == QtWidgets.QMessageBox.Ok:
                ready = ready & True
            elif button == QtWidgets.QMessageBox.Cancel:
                ready = ready & False

        self.print_info('INFO! Ready to record ? ' + str(ready))
        return ready

    def get_record_options(self):
        record_options = RecordOptions()
        base_path = str(self.gui_configuration.recording_base_path.get())
        participant_name = '/' + self.text_participant_name.toPlainText().strip()
        date_time = '/' + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        record_options.path = base_path + participant_name + date_time
        record_options.simulator_data = True

        if self._pupil_process.state() == QProcess.Running:
            record_options.pupil_data = True
        else:
            record_options.pupil_data = False

        return record_options

    def save_metadata(self):
        notes, ok = QtWidgets.QInputDialog.getMultiLineText(
            self, 'Input Dialog', 'Enter notes:')

        metadata = {
                "participant_name": self.text_participant_name.toPlainText().strip(),
                "volume_adf": self.active_volume_adf,
                "volume_name": self.active_volume_name,
                "notes": notes
            }
        
        metadata_path = self.record_options.path + "/metadata.json"

        with open(metadata_path, "w") as outfile:
            json.dump(metadata, outfile, indent = 4)
        

    def pressed_record_study(self):
        if self._recording_study:
            # self._recording_process.close()
            self.study_manager.stop_recording()
            self._recording_study = False
            self.save_metadata()
            self.button_record_study.setText("Record Study")
            self.button_record_study.setStyleSheet("background-color: GREEN")
        else:
            if not self.is_ready_to_record():
                return -1
            self.record_options = self.get_record_options()
            self.study_manager.start_recording(self.record_options)
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
