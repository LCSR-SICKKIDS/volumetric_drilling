from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import sys
from study_manager import StudyManager


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout.ui', self)

        self.study_manager = StudyManager()

        self.pixmap = QPixmap('plane0069.png')
        self.label = self.findChild(QtWidgets.QLabel, 'label_1')
        # self.label.setPixmap(self.pixmap)
        # self.label.setScaledContents(True)
        self.label.setPixmap(self.pixmap.scaled(self.label.width(), self.label.height(), Qt.KeepAspectRatio))

        self.button_start_simulation = self.findChild(QtWidgets.QPushButton, 'button_start_simulation')
        self.button_start_simulation.clicked.connect(self.pressed_start_simulation)

        self.button_pupil_service = self.findChild(QtWidgets.QPushButton, 'button_pupil_service')
        self.button_pupil_service.clicked.connect(self.pressed_pupil_service)

        self.button_record_study = self.findChild(QtWidgets.QPushButton, 'button_record_study')
        self.button_record_study.clicked.connect(self.pressed_record_study)

        self.text_participant_name = self.findChild(QtWidgets.QTextEdit, 'textEdit_participant_name')

        self.show()

    def pressed_start_simulation(self):
        self.study_manager.start_simulation()

    def pressed_pupil_service(self):
        self.study_manager.start_pupil_service()

    def pressed_record_study(self):
        name = self.text_participant_name.toPlainText()
        print('Will Start Recording Participant: ', name)

    def closeEvent(self, event):
        print('Terminate Called')
        self.study_manager.close()



app = QtWidgets.QApplication(sys.argv)
window = Ui()
app.exec_()
