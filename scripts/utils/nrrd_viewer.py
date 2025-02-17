#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019-2025

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <amunawa2@jh.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
import sys
import os
import nrrd
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QFormLayout, QFileDialog, QLabel, QLineEdit, QCheckBox
from PyQt5.QtCore import Qt
from seg_nrrd_to_pngs import SegmentInfo, SegNrrdCoalescer
from nrrd_to_adf import NrrdKinematicsData, nrrd_to_adf, ADFData
from volume_data_to_slices import save_volume_data_as_slices


class NRRDViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.nrrd_data = None
        self.nrrd_header = None
        self.nrrd_filepath = ""
        self.current_slice = [0, 0, 0]
        self.segment_infos = SegmentInfo()
        self.adf_handler = ADFData()
        self.nrrd_kinematics_data = NrrdKinematicsData()
        self.kinematics_widgets_list = []
        self.sub_layout_vspace = 30

        self.initUI()

    def initUI(self):
        layout = QGridLayout()

        # Load NRRD / SEG NRRD Layout
        load_nrrd_layout = QGridLayout()

        self.group_label = QLabel("STEP 1 - LOAD VOLUME")
        self.group_label.setAlignment(Qt.AlignCenter)
        load_nrrd_layout.addWidget(self.group_label, 0, 0, 1, 4)

        self.load_nrrd_label = QLabel("Load NRRD or Segmentation NRRD", self)
        load_nrrd_layout.addWidget(self.load_nrrd_label, 1, 0)
        
        self.nrrd_filepath = QLineEdit("", self)
        load_nrrd_layout.addWidget(self.nrrd_filepath, 1, 1)
        
        self.select_nrrd_filepath = QPushButton("...", self)
        self.select_nrrd_filepath.clicked.connect(self.select_nrrd_cb)
        load_nrrd_layout.addWidget(self.select_nrrd_filepath, 1, 2)
        
        self.load_nrrd_button = QPushButton("Load", self)
        self.load_nrrd_button.clicked.connect(self.load_nrrd_cb)
        self.load_nrrd_button.setEnabled(False)
        load_nrrd_layout.addWidget(self.load_nrrd_button, 2, 0, 1, 4)
        
        self.show_slices_button = QPushButton("Show Slices", self)
        self.show_slices_button.clicked.connect(self.show_slices_cb)
        self.show_slices_button.setEnabled(False)
        load_nrrd_layout.addWidget(self.show_slices_button, 3, 0, 1, 4)

        layout.addLayout(load_nrrd_layout, 0, 0)

        # Kinematics Layout
        layout.setVerticalSpacing(self.sub_layout_vspace)
        kinematics_layout = QGridLayout()

        self.group_label = QLabel("META DATA")
        self.group_label.setAlignment(Qt.AlignCenter)
        kinematics_layout.addWidget(self.group_label, 0, 0, 1, 4)

        self.kinematic_widget_parent = QWidget()
        kinematics_layout.addWidget(self.kinematic_widget_parent)

        self.origin_label = QLabel("Origin:")
        kinematics_layout.addWidget(self.origin_label, 1, 0, 1, 1)

        self.origin_x = QLineEdit("0.")
        kinematics_layout.addWidget(self.origin_x, 1, 1, 1, 1)

        self.origin_y = QLineEdit("0.")
        kinematics_layout.addWidget(self.origin_y, 1, 2, 1, 1)

        self.origin_z = QLineEdit("0.")
        kinematics_layout.addWidget(self.origin_z, 1, 3, 1, 1)

        self.orientation_label = QLabel("Orientation (RPY):")
        kinematics_layout.addWidget(self.orientation_label, 2, 0, 1, 1)

        self.orientation_roll = QLineEdit("0.")
        kinematics_layout.addWidget(self.orientation_roll, 2, 1, 1, 1)

        self.orientation_pitch = QLineEdit("0.")
        kinematics_layout.addWidget(self.orientation_pitch, 2, 2, 1, 1)

        self.orientation_yaw = QLineEdit("0.")
        kinematics_layout.addWidget(self.orientation_yaw, 2, 3, 1, 1)

        self.dimensions_label = QLabel("Dimensions:")
        kinematics_layout.addWidget(self.dimensions_label, 3, 0, 1, 1)

        self.dim_x = QLineEdit("0.")
        kinematics_layout.addWidget(self.dim_x, 3, 1, 1, 1)

        self.dim_y = QLineEdit("0.")
        kinematics_layout.addWidget(self.dim_y, 3, 2, 1, 1)

        self.dim_z = QLineEdit("0.")
        kinematics_layout.addWidget(self.dim_z, 3, 3, 1, 1)

        self.sizes_label = QLabel("Sizes:")
        kinematics_layout.addWidget(self.sizes_label, 4, 0, 1, 1)

        self.sizes_x = QLineEdit("0")
        self.sizes_x.setEnabled(False)
        kinematics_layout.addWidget(self.sizes_x, 4, 1, 1, 1)

        self.sizes_y = QLineEdit("0")
        self.sizes_y.setEnabled(False)
        kinematics_layout.addWidget(self.sizes_y, 4, 2, 1, 1)

        self.sizes_z = QLineEdit("0")
        self.sizes_z.setEnabled(False)
        kinematics_layout.addWidget(self.sizes_z, 4, 3, 1, 1)

        self.units_scale_label = QLabel("Units Scale (SI)")
        kinematics_layout.addWidget(self.units_scale_label, 5, 0, 1, 1)

        self.units_scale = QLineEdit("0.001")
        kinematics_layout.addWidget(self.units_scale, 5, 1, 1, 3)

        self.override_kinematics = QCheckBox("Override Meta Data")
        self.override_kinematics.checkState = False
        self.override_kinematics.clicked.connect(self.override_kinematics_cb)
        kinematics_layout.addWidget(self.override_kinematics, 6, 0)

        self.kinematics_widgets_list.append(self.origin_x)
        self.kinematics_widgets_list.append(self.origin_y)
        self.kinematics_widgets_list.append(self.origin_z)
        self.kinematics_widgets_list.append(self.orientation_roll)
        self.kinematics_widgets_list.append(self.orientation_pitch)
        self.kinematics_widgets_list.append(self.orientation_yaw)
        self.kinematics_widgets_list.append(self.dim_x)
        self.kinematics_widgets_list.append(self.dim_y)
        self.kinematics_widgets_list.append(self.dim_z)

        layout.addLayout(kinematics_layout, 1, 0)

        # Slices Layout
        layout.setVerticalSpacing(self.sub_layout_vspace)
        slices_layout = QGridLayout()

        self.group_label = QLabel("STEP 2 - SAVE SLICES")
        self.group_label.setAlignment(Qt.AlignCenter)
        slices_layout.addWidget(self.group_label, 0, 0, 1, 4)
        
        self.prefix_label = QLabel("Slices Prefix:", self)
        slices_layout.addWidget(self.prefix_label, 1, 0)

        self.slices_prefix = QLineEdit(self)
        self.slices_prefix.setText("slice0")
        slices_layout.addWidget(self.slices_prefix, 1, 1)
        
        self.slices_path_label = QLabel("Output Slices Dir:", self)
        slices_layout.addWidget(self.slices_path_label, 2, 0)

        self.slices_path = QLineEdit(self)
        slices_layout.addWidget(self.slices_path, 2, 1)

        self.select_folder_button = QPushButton("...", self)
        self.select_folder_button.clicked.connect(self.select_folder_cb)
        slices_layout.addWidget(self.select_folder_button, 2, 2)

        self.save_slices_button = QPushButton("Save slices as PNGs", self)
        self.save_slices_button.clicked.connect(self.save_slices_as_pngs_cb)
        self.save_slices_button.setEnabled(False)
        slices_layout.addWidget(self.save_slices_button, 3, 0, 1, 3)

        layout.addLayout(slices_layout, 2, 0)

        # ADF Layout
        layout.setVerticalSpacing(self.sub_layout_vspace)
        adf_layout = QGridLayout()

        self.group_label = QLabel("STEP 3 - SAVE ADF")
        self.group_label.setAlignment(Qt.AlignCenter)
        adf_layout.addWidget(self.group_label, 0, 0, 1, 4)

        self.adf_filepath_label = QLabel("ADF:", self)
        adf_layout.addWidget(self.adf_filepath_label, 1, 0)

        self.adf_filepath = QLineEdit(self)
        adf_layout.addWidget(self.adf_filepath, 1, 1)

        self.select_adf_filepath_button = QPushButton("...", self)
        self.select_adf_filepath_button.clicked.connect(self.select_adf_filepath_cb)
        adf_layout.addWidget(self.select_adf_filepath_button, 1, 2)

        self.save_adf_button = QPushButton("Save ADF file", self)
        self.save_adf_button.clicked.connect(self.save_adf_cb)
        self.save_adf_button.setEnabled(False)
        adf_layout.addWidget(self.save_adf_button, 2, 0, 1, 4)

        layout.addLayout(adf_layout, 3, 0)
        
        self.setLayout(layout)
        self.setWindowTitle("NRRD and SEG NRRD")
        self.setGeometry(100, 100, 800, 600)

        self.override_kinematics_cb()

    def _set_layout_from_nrrd_kinematics_data(self, nrrd_kin: NrrdKinematicsData):
        self.origin_x.setText(str(nrrd_kin.origin[0]))
        self.origin_y.setText(str(nrrd_kin.origin[1]))
        self.origin_z.setText(str(nrrd_kin.origin[2]))

        self.orientation_roll.setText(str(nrrd_kin.orientation_rpy[0]))
        self.orientation_pitch.setText(str(nrrd_kin.orientation_rpy[1]))
        self.orientation_yaw.setText(str(nrrd_kin.orientation_rpy[2]))

        self.dim_x.setText(str(nrrd_kin.dimensions[0]))
        self.dim_y.setText(str(nrrd_kin.dimensions[1]))
        self.dim_z.setText(str(nrrd_kin.dimensions[2]))

        self.sizes_x.setText(str(nrrd_kin.sizes[0]))
        self.sizes_y.setText(str(nrrd_kin.sizes[1]))
        self.sizes_z.setText(str(nrrd_kin.sizes[2]))

        self.units_scale.setText(str(nrrd_kin.units_scale))

    def _get_nrrd_kinematics_data_from_layout(self):
        nrrd_kin = NrrdKinematicsData()
        nrrd_kin.origin = np.array([0, 0, 0])
        nrrd_kin.origin[0] = float(self.origin_x.text())
        nrrd_kin.origin[1] = float(self.origin_y.text())
        nrrd_kin.origin[2] = float(self.origin_z.text())

        nrrd_kin.orientation_rpy = np.array([0, 0, 0])
        nrrd_kin.orientation_rpy[0] = float(self.orientation_roll.text())
        nrrd_kin.orientation_rpy[1] = float(self.orientation_pitch.text())
        nrrd_kin.orientation_rpy[2] = float(self.orientation_yaw.text())

        nrrd_kin.dimensions = np.array([0, 0, 0])
        nrrd_kin.dimensions[0] = float(self.dim_x.text())
        nrrd_kin.dimensions[1] = float(self.dim_y.text())
        nrrd_kin.dimensions[2] = float(self.dim_z.text())

        nrrd_kin.sizes = np.array([0, 0, 0])
        nrrd_kin.sizes[0] = int(self.sizes_x.text())
        nrrd_kin.sizes[1] = int(self.sizes_y.text())
        nrrd_kin.sizes[2] = int(self.sizes_z.text())

        nrrd_kin.units_scale = float(self.units_scale.text())

        return nrrd_kin

    def binary_to_rgba(self, binary_array, rgba, threshold=1):
        """
        Convert a 2D binary array (0s and 1s) to an RGBA image array.
        """
        height, width = binary_array.shape
        rgba_array = np.zeros((height, width, 4), dtype=np.uint8)

        # Set white (255,255,255,255) for 1s and black (0,0,0,255) for 0s
        rgba_array[binary_array == threshold] = rgba  # White with full opacity
        rgba_array[binary_array == 0] = [0, 0, 0, 0]        # Black with full opacity

        return rgba_array

    def select_nrrd_cb(self):
        options = QFileDialog.Options()
        nrrd_filepath, _ = QFileDialog.getOpenFileName(self, "Open NRRD File", "", "NRRD Files (*.nrrd *.seg.nrrd)", options=options)
        
        if nrrd_filepath:
            self.nrrd_filepath.setText(nrrd_filepath)
            self.load_nrrd_button.setEnabled(True)


    def load_nrrd_cb(self):
        if self.nrrd_filepath.text():
            self.nrrd_data, self.nrrd_header = nrrd.read(self.nrrd_filepath.text())
            self.nrrd_kinematics_data.load(self.nrrd_header)
            self._set_layout_from_nrrd_kinematics_data(self.nrrd_kinematics_data)

            if len(self.nrrd_data.shape) == 4:  # Handle 4D segmentation data
                nrrd_coalescer = SegNrrdCoalescer()
                nrrd_coalescer.set_nrrd(self.nrrd_header, self.nrrd_data)
                self.nrrd_data = nrrd_coalescer.get_coalesced_data()

            
            self.load_nrrd_label.setText(f"Loaded: {self.nrrd_filepath.text().split('/')[-1]}")
            self.show_slices_button.setEnabled(True)
            self.save_slices_button.setEnabled(True)
            self.current_slice = [self.nrrd_data.shape[0] // 2, self.nrrd_data.shape[1] // 2, self.nrrd_data.shape[2] // 2]
            # print("NRRD Metadata:", self.nrrd_header)
            # self.show_slices()

    def show_slices_cb(self):
        if self.nrrd_data is not None:
            self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
            
            self.update_slices()
            
            self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
            plt.show()

    def select_folder_cb(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Directory")
        if folder:
            self.slices_path.setText(folder)

    def save_slices_as_pngs_cb(self):
        color_map = 'gray' if 'segmentation' not in self.nrrd_header.get('type', '').lower() else 'jet'
        save_volume_data_as_slices(self.nrrd_data, self.slices_path.text(), self.slices_prefix.text(), color_map)

    def save_adf_cb(self):
        if self.override_kinematics.isChecked():
            nrrd_kinematics_data = self._get_nrrd_kinematics_data_from_layout()
        else:
            nrrd_kinematics_data = self.nrrd_kinematics_data

        nrrd_to_adf(nrrd_kinematics_data,
                    self.nrrd_data,
                    self.nrrd_filepath.text(),
                    self.adf_filepath.text(),
                    self.slices_path.text(),
                    self.slices_prefix.text())


    def select_adf_filepath_cb(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getSaveFileName(self, "Select or Create ADF File", "", "YAML Files (*.yaml)", options=options)
        if file_path:
            self.adf_filepath.setText(file_path)
            self.save_adf_button.setEnabled(True)

    def override_kinematics_cb(self):
        if self.override_kinematics.isChecked():
            for w in self.kinematics_widgets_list:
                w.setEnabled(True)

        else:
            for w in self.kinematics_widgets_list:
                w.setEnabled(False)

    
    def update_slices(self):
        cmap = 'gray' if 'segmentation' not in self.nrrd_header.get('type', '').lower() else 'jet'
        
        self.axes[0].cla()
        self.axes[0].imshow(self.nrrd_data[self.current_slice[0], :, :], cmap=cmap)
        self.axes[0].set_title(f"Axial Slice {self.current_slice[0]}")
        
        self.axes[1].cla()
        self.axes[1].imshow(self.nrrd_data[:, self.current_slice[1], :], cmap=cmap)
        self.axes[1].set_title(f"Coronal Slice {self.current_slice[1]}")
        
        self.axes[2].cla()
        self.axes[2].imshow(self.nrrd_data[:, :, self.current_slice[2]], cmap=cmap)
        self.axes[2].set_title(f"Sagittal Slice {self.current_slice[2]}")
        
        self.fig.canvas.draw()
    
    def on_scroll(self, event):
        if event.inaxes == self.axes[0]:
            self.current_slice[0] = min(max(self.current_slice[0] + (1 if event.step > 0 else -1), 0), self.nrrd_data.shape[0] - 1)
        elif event.inaxes == self.axes[1]:
            self.current_slice[1] = min(max(self.current_slice[1] + (1 if event.step > 0 else -1), 0), self.nrrd_data.shape[1] - 1)
        elif event.inaxes == self.axes[2]:
            self.current_slice[2] = min(max(self.current_slice[2] + (1 if event.step > 0 else -1), 0), self.nrrd_data.shape[2] - 1)
        
        self.update_slices()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = NRRDViewer()
    viewer.show()
    sys.exit(app.exec_())
