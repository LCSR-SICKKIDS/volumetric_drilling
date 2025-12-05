from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtWidgets import QFormLayout, QGroupBox, QPushButton, QFileDialog
from PyQt5.QtWidgets import QTextEdit, QWidget, QScrollArea, QRadioButton, QLabel, QPlainTextEdit
from PyQt5.QtWidgets import QMessageBox, QInputDialog
import sys
from study_manager import StudyManager, RecordOptions
from gui_setup import GUIConfiguration, GUIParam, ParamType
import datetime
from pathlib import Path
import json


def get_resolved_path_as_str(path_str: str):
    raw_path = Path(path_str)
    resolved_path = None
    if (raw_path.exists()):
        resolved_path = raw_path.expanduser().resolve()
    else:
        resolved_path = raw_path
    str_path = str(resolved_path)
    return str_path


class Ui(QWidget):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout.ui', self)

        self.gui_conf = GUIConfiguration('gui_setup.yaml')

        self._ambf_executable = self.gui_conf.params["ambf_executable"]
        self._pupil_capture_executable = self.gui_conf.params["pupil_capture_executable"]
        self._recording_script = self.gui_conf.params["recording_script_executable"]
        self._recording_base_path = self.gui_conf.params['recording_base_path']
        self._launch_file = self.gui_conf.params["launch_file"]
        self._footpedal_device = self.gui_conf.params["footpedal_device"]

        self.study_manager = StudyManager(get_resolved_path_as_str(self._recording_script.get()))
        self.active_volume_adf = ''
        self.active_volume_name = ''
        self.record_options = RecordOptions()
        self.cnt = 0

        # Setup the grid layout for different volumes
        self.scroll_area = self.findChild(QScrollArea, 'scrollArea')
        formLayout = QFormLayout()
        groupBox = QGroupBox('Anatomies')
        for i in range(len(self.gui_conf.volumes_info)):
            vinfo = self.gui_conf.volumes_info[i]
            radio_button = QRadioButton(vinfo.name)
            # radio_button.setMinimumHeight(min_height)
            # radio_button.setGeometry(200, 150, 100, 40)
            radio_button.volume_name = vinfo.name
            radio_button.volume_adf = str(vinfo.adf_path)
            radio_button.toggled.connect(self.radio_button_volume_selection)
            icon_path_str = str(vinfo.icon_path)
            print(icon_path_str)
            pixmap = QPixmap(icon_path_str)
            label = QLabel(self)
            label.resize(150, 350)
            # label.setMinimumHeight(min_height)
            label.setPixmap(pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio))
            formLayout.addRow(radio_button, label)
            if i == 0:
                radio_button.setChecked(True)
        groupBox.setLayout(formLayout)
        self.scroll_area.setWidget(groupBox)
        self.scroll_area.setWidgetResizable(True)

        self.button_start_simulation = self.findChild(QPushButton, 'button_start_simulation')
        self.button_start_simulation.setStyleSheet("background-color: GREEN")
        self.button_start_simulation.clicked.connect(self.pressed_start_simulation)

        self.button_launch_vr = self.findChild(QPushButton, 'button_launch_vr')

        self.button_stream_stereo = self.findChild(QPushButton, 'button_stream_stereo')

        self.button_stream_depth = self.findChild(QPushButton, 'button_stream_depth')

        self.button_pupil_service = self.findChild(QPushButton, 'button_pupil_capture')
        self.button_pupil_service.clicked.connect(self.pressed_pupil_service)

        self.button_reset_drill = self.findChild(QPushButton, 'button_reset_drill')
        self.button_reset_drill.clicked.connect(self.study_manager.reset_drill)

        self.button_reset_volume = self.findChild(QPushButton, 'button_reset_volume')
        self.button_reset_volume.clicked.connect(self.study_manager.reset_volume)

        self.button_toggle_shadows = self.findChild(QPushButton, 'button_toggle_shadows')
        self.button_toggle_shadows.clicked.connect(self.study_manager.toggle_shadows)

        self.button_toggle_vol_smooth = self.findChild(QPushButton, 'button_toggle_vol_smooth')
        self.button_toggle_vol_smooth.clicked.connect(self.study_manager.toggle_volume_smoothening)

        self.button_record_study = self.findChild(QPushButton, 'button_record_study')
        self.button_record_study.clicked.connect(self.pressed_record_study)
        self.button_record_study.setStyleSheet("background-color: GREEN")

        self.text_participant_name = self.findChild(QTextEdit, 'textEdit_participant_name')
        self.recording_button = self.findChild(QPushButton, 'button_record_study')

        self.textEdit_info = self.findChild(QPlainTextEdit, 'textEdit_info')

        self.textEdit_debug = self.findChild(QPlainTextEdit, 'textEdit_debug')

        self.gui_param_objects = {}
        for k, v in self.gui_conf.params.items():
            self.gui_param_objects[k] = self.connect_gui_param_to_dialog(v)

        self.button_save_configuration = self.findChild(QPushButton, 'pushButton_save_configuration')
        self.button_save_configuration.clicked.connect(self.save_configuration)

        self.button_reload_configuration = self.findChild(QPushButton, 'pushButton_reload_configuration')
        self.button_reload_configuration.clicked.connect(self.reload_configuration)

        self._recording_study = False

        self._ambf_process = QProcess()
        self._ambf_process.readyReadStandardOutput.connect(self.handle_stdout)
        self._ambf_process.readyReadStandardError.connect(self.handle_stderr)
        self._ambf_process.finished.connect(self._simulation_closed)
        self._pupil_process = QProcess()
        self._recording_process = QProcess()

        self.show()

    def connect_gui_param_to_dialog(self, gui_param: GUIParam):
        param_text_edit_widget = self.findChild(QTextEdit, 'textEdit_' + gui_param.get_id())
        
        param_select_btn = self.findChild(QPushButton, 'pushButton_' + gui_param.get_id())

        param_select_btn.clicked.connect(
            lambda: self.param_button_clicked(param_text_edit_widget, gui_param))
        
        param_text_edit_widget.setText(gui_param.get())

        param_text_edit_widget.textChanged.connect(
            lambda: self.param_text_changed(param_text_edit_widget, gui_param))

        return param_text_edit_widget
    
    def param_text_changed(self, text_edit: QTextEdit, gui_param: GUIParam):
        gui_param.set(text_edit.toPlainText())
        self.cnt = self.cnt + 1

    def param_button_clicked(self, text_edit: QTextEdit, gui_param: GUIParam):
        valid = False
        file_or_folder = None
        if gui_param.get_type() == ParamType.FILE:
            file_or_folder, valid = QFileDialog.getOpenFileName(None, gui_param.get_id(),
                "", "All Files (*);;Python Files (*.py);;Text Files (*.txt)")
        elif gui_param.get_type() == ParamType.FOLDER:
            valid = True
            file_or_folder = QFileDialog.getExistingDirectory(None, gui_param.get_id(),
                "",)

        if valid:
            # gui_param.set(file_or_folder) # GUI Param should be automatically set with text changed event
            text_edit.setText(file_or_folder) 

    def save_configuration(self):
        self.gui_conf.save_params_to_file()

    def reload_configuration(self):
        print('Reloading Configuration')
        self.gui_conf.load_params_from_file()
        for k, v in self.gui_param_objects.items():
            v.setText(self.gui_conf.params[k].get())

    def pressed_start_simulation(self):
        launch_file_adf_indices = '0,7'
        if self.button_stream_depth.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',4'
        if self.button_stream_stereo.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',5'
        if self.button_launch_vr.isChecked():
            launch_file_adf_indices = launch_file_adf_indices + ',6'

        launch_file_resolved = get_resolved_path_as_str(self._launch_file.get())
        footpedal_device_resolved = get_resolved_path_as_str(self._footpedal_device.get())
        active_volume_adf_resolved = get_resolved_path_as_str(self.active_volume_adf)
        ambf_executable_resolved_path = get_resolved_path_as_str(self._ambf_executable.get())

        args = ['--launch_file', launch_file_resolved, '-l', launch_file_adf_indices, '-a', active_volume_adf_resolved,
                "--fp", footpedal_device_resolved]
        # self.study_manager.start_simulation(args)
        print("LAUNCHING AMBF WITH ARGS: ", args)
        if self._ambf_process.state() != QProcess.Running:
            self._ambf_process.start(ambf_executable_resolved_path, args)
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
            pupil_capture_executable_resolved = get_resolved_path_as_str(self._pupil_capture_executable.get())
            print("Launching ", pupil_capture_executable_resolved)
            self._pupil_process.start(pupil_capture_executable_resolved)
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
            dlg = QMessageBox(self)
            dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            dlg.setText('Pupil capture not running. No pupil data will be recorded. OK?')
            button = dlg.exec()
            if button == QMessageBox.Ok:
                ready = ready & True
            elif button == QMessageBox.Cancel:
                ready = ready & False

        self.print_info('INFO! Ready to record ? ' + str(ready))
        return ready

    def get_record_options(self):
        record_options = RecordOptions()
        base_path = get_resolved_path_as_str(self._recording_base_path.get())
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
        notes, ok = QInputDialog.getMultiLineText(
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
    
    def move_text_cursor_to_start(self, textEdit):
        cursor = textEdit.textCursor()
        cursor.setPosition(0)
        textEdit.setTextCursor(cursor)

    def scroll_textedit_to_end(self, textEdit):
        textEdit.verticalScrollBar().setValue(textEdit.verticalScrollBar().maximum())

    def print_info(self, msg):
        self.textEdit_info.insertPlainText(self.get_time_as_str() + msg + '\n')
        self.scroll_textedit_to_end(self.textEdit_info)

    def print_debug(self, msg):
        self.textEdit_debug.insertPlainText(self.get_time_as_str() + msg)
        self.scroll_textedit_to_end(self.textEdit_debug)

    def handle_stderr(self):
        msg = bytes(self._ambf_process.readAllStandardError()).decode('utf-8')
        self.print_debug(msg)

    def handle_stdout(self):
        msg = bytes(self._ambf_process.readAllStandardOutput()).decode('utf-8')
        self.print_debug(msg)



app = QtWidgets.QApplication(sys.argv)
window = Ui()
app.exec_()
