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
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QFileDialog, QLabel, QLineEdit
from PyQt5.QtCore import Qt
from seg_nrrd_to_pngs import SegmentInfo, SegNrrdCoalescer
from collections import OrderedDict
import yaml
from scipy.spatial.transform import Rotation

def represent_dictionary_order(self, dict_data):
    return self.represent_mapping('tag:yaml.org,2002:map', dict_data.items())

def setup_yaml():
    yaml.add_representer(OrderedDict, represent_dictionary_order)

def set_location_attributes(yaml_data, position, orientation):
        yaml_data["location"]["position"]["x"] = float(position[0])
        yaml_data["location"]["position"]["y"] = float(position[1])
        yaml_data["location"]["position"]["z"] = float(position[2])

        yaml_data["location"]["orientation"]["r"] = float(orientation[0])
        yaml_data["location"]["orientation"]["p"] = float(orientation[1])
        yaml_data["location"]["orientation"]["y"] = float(orientation[2])

class ADFData:
    def __init__(self):
        self.meta_data = OrderedDict()
        self.meta_data["ADF Version"] = 1.0
        self.meta_data["volumes"] = []
        self.meta_data["bodies"] = []

        self.volume_data = OrderedDict()
        self.volume_data["name"] = ""
        self.volume_data["location"] = OrderedDict()
        self.volume_data["location"]["position"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.volume_data["location"]["orientation"] = {"r": 0.0, "p": 0.0, "y": 0.0}
        self.volume_data["scale"] = 1.0
        self.volume_data["dimensions"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.volume_data["images"] = {"path": "", "prefix": "", "count": 0, "format": "png"}

        self.parent_body_data = OrderedDict()
        self.parent_body_data["name"] = ""
        self.parent_body_data["location"] = OrderedDict()
        self.parent_body_data["location"]["position"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.parent_body_data["location"]["orientation"] = {"r": 0.0, "p": 0.0, "y": 0.0}
        self.parent_body_data["mass"] = 0.0

    def set_volume_name_attirubte(self, name):
        self.volume_data["name"] = name

    def set_volume_kinematics_attributes(self, nrrd_hdr):
        space_origin = nrrd_hdr['space origin']
        space_directions = nrrd_hdr['space directions']
        if space_directions.shape[0] == 4: # Segmented NRRD, take the last three rows
            space_directions = space_directions[1:4, :]
        space_resolution = np.linalg.norm(space_directions, axis=1)
        
        sizes = nrrd_hdr['sizes']
        if sizes.shape[0] == 4: # Seg NRRD, take the last three values
            sizes = sizes[1:4]

        dimensions = space_resolution * sizes
        coordinate_representation = nrrd_hdr['space'] # Usually LPS or RAS

        if coordinate_representation.lower() != 'left-posterior-superior':
            print("INFO! NRRD NOT USING LPS CONVENTION")
        
        rotation_offset = Rotation.from_euler('ZYX', [0., 0., 0.], degrees=True)
        if coordinate_representation.lower() == 'right-anterior-superior':
            # Perform 180 degree rotation
            rotation_offset = Rotation.from_euler('ZYX', [180., 0., 0.], degrees=True)
        pass
        
        U, _, Vt = np.linalg.svd(space_directions)
        orientation_mat = U @ Vt @ rotation_offset.as_matrix()
        space_orientaiton_rpy = Rotation.from_matrix(orientation_mat).as_euler('ZYX', degrees=False) #intrinsic ZYX == extrinsic XYZ
        self._set_volume_kinematics_attributes(space_origin, space_orientaiton_rpy, dimensions)

    def _set_volume_kinematics_attributes(self, position, orientation, dimensions):
        set_location_attributes(self.volume_data, position, orientation)

        self.volume_data["dimensions"]["x"] = float(dimensions[0])
        self.volume_data["dimensions"]["y"] = float(dimensions[1])
        self.volume_data["dimensions"]["z"] = float(dimensions[2])

    def set_volume_data_attributes(self, image_path, image_prefix, image_count, image_format):
        self.volume_data["images"]["path"] = image_path
        self.volume_data["images"]["prefix"] = image_prefix
        self.volume_data["images"]["count"] = int(image_count)
        self.volume_data["images"]["format"] = image_format

    def set_parent_body_name_attribute(self, name):
        self.parent_body_data["name"] = name
        self.volume_data["parent"] = "BODY " + self.parent_body_data["name"]
    
    def set_parent_body_kinematics_attributes(self, position, orientation):
        set_location_attributes(self.parent_body_data, position, orientation)

    def _coalesce_adf_data(self):
        coalesced_data = OrderedDict()
        coalesced_data = self.meta_data
        if self.volume_data["name"]:
            volume_identifier = "VOLUME " + self.volume_data["name"]
            coalesced_data["volumes"].append(volume_identifier)
            coalesced_data[volume_identifier] = self.volume_data
        if self.parent_body_data["name"]:
            body_identifier = "BODY " + self.parent_body_data["name"]
            coalesced_data["bodies"].append(body_identifier)
            coalesced_data[body_identifier] = self.parent_body_data

        return coalesced_data

    def save(self, filepath):
        adf_data = self._coalesce_adf_data()
        print("ADF Data\n", adf_data)
        setup_yaml()
        with open(filepath, 'w') as adffile:
            yaml.dump(adf_data, adffile, default_flow_style=False)
            print("Saving ADF", filepath)
            adffile.close()


class NRRDViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.nrrd_data = None
        self.nrrd_header = None
        self.nrrd_filepath = ""
        self.current_slice = [0, 0, 0]
        self.segment_infos = SegmentInfo()
        self.adf_handler = ADFData()

    def initUI(self):
        layout = QGridLayout()
        
        self.label = QLabel("No file loaded", self)
        layout.addWidget(self.label, 0, 0)
        
        self.load_volume_button = QPushButton("Load File (NRRD or SEG NRRD)", self)
        self.load_volume_button.clicked.connect(self.load_nrrd)
        layout.addWidget(self.load_volume_button, 1, 0)
        
        self.show_slices_button = QPushButton("Show Slices", self)
        self.show_slices_button.clicked.connect(self.show_slices)
        self.show_slices_button.setEnabled(False)
        layout.addWidget(self.show_slices_button, 1, 1)

        self.pngs_prefix = QLineEdit(self)
        self.pngs_prefix.setText("slice0")
        layout.addWidget(self.pngs_prefix, 2, 0)
        
        self.pngs_folder_path = QLineEdit(self)
        layout.addWidget(self.pngs_folder_path, 2, 1)

        self.select_folder_button = QPushButton("Slices Directory", self)
        self.select_folder_button.clicked.connect(self.select_folder)
        layout.addWidget(self.select_folder_button, 2, 2)

        self.save_slices_button = QPushButton("Save slices as PNGs", self)
        self.save_slices_button.clicked.connect(self.save_slices_as_pngs)
        self.save_slices_button.setEnabled(False)
        layout.addWidget(self.save_slices_button, 3, 0)

        self.adf_filepath = QLineEdit(self)
        layout.addWidget(self.adf_filepath, 4, 0)

        self.select_adf_filepath_button = QPushButton("ADF filepath", self)
        self.select_adf_filepath_button.clicked.connect(self.select_adf_filepath)
        layout.addWidget(self.select_adf_filepath_button, 4, 1)

        self.save_adf_button = QPushButton("Save ADF file", self)
        self.save_adf_button.clicked.connect(self.save_adf)
        self.save_adf_button.setEnabled(False)
        layout.addWidget(self.save_adf_button, 5, 0)
        
        self.setLayout(layout)
        self.setWindowTitle("NRRD and SEG NRRD")
        self.setGeometry(100, 100, 300, 150)

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


    def load_nrrd(self):
        options = QFileDialog.Options()
        self.nrrd_filepath, _ = QFileDialog.getOpenFileName(self, "Open NRRD File", "", "NRRD Files (*.nrrd *.seg.nrrd)", options=options)
        
        if self.nrrd_filepath:
            self.nrrd_data, self.nrrd_header = nrrd.read(self.nrrd_filepath)
            if len(self.nrrd_data.shape) == 4:  # Handle 4D segmentation data
                # self.nrrd_data = np.sum(self.nrrd_data, axis=-1)  # Coalesce along the last dimension
                nrrd_coalescer = SegNrrdCoalescer()
                nrrd_coalescer.read_nrrd(self.nrrd_filepath)
                nrrd_coalescer.initialize_image_matrix()
                nrrd_coalescer.initialize_segments_infos(self.nrrd_header)
                nrrd_coalescer.coalesce_segments_into_3D_data()
                self.nrrd_data = nrrd_coalescer._images_matrix

            
            self.label.setText(f"Loaded: {self.nrrd_filepath.split('/')[-1]}")
            self.show_slices_button.setEnabled(True)
            self.save_slices_button.setEnabled(True)
            self.current_slice = [self.nrrd_data.shape[0] // 2, self.nrrd_data.shape[1] // 2, self.nrrd_data.shape[2] // 2]
            # print("NRRD Metadata:", self.nrrd_header)
            # self.show_slices()

    def show_slices(self):
        if self.nrrd_data is not None:
            self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
            
            self.update_slices()
            
            self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
            plt.show()

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Directory")
        if folder:
            self.pngs_folder_path.setText(folder)

    def save_slices_as_pngs(self):
        folder = self.pngs_folder_path.text()
        if folder:
            print("INFO! Path selected", folder) 
            if self.nrrd_data is not None:
                cmap = 'gray' if 'segmentation' not in self.nrrd_header.get('type', '').lower() else 'jet'
                for i in range(self.nrrd_data.shape[2]):
                    im_name = folder + '/' + self.pngs_prefix.text() + str(i) + '.png'
                    plt.imsave(im_name, self.nrrd_data[:, :, i], cmap=cmap)

    def save_adf(self):
        images_path = self.pngs_folder_path.text()
        adf_filepath = self.adf_filepath.text()
        rel_images_path = os.path.relpath(images_path, adf_filepath)
        adf_data = ADFData()
        adf_data.set_volume_kinematics_attributes(self.nrrd_header)
        adf_data.set_volume_data_attributes(rel_images_path, self.pngs_prefix.text(), self.nrrd_header["sizes"][-1], "png")
        volume_name = os.path.basename(self.nrrd_filepath).split('.')[0]
        adf_data.set_volume_name_attirubte(volume_name)

        adf_data.set_parent_body_name_attribute(volume_name + 'Anatomical Origin')
        adf_data.save(adf_filepath)


    def select_adf_filepath(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getSaveFileName(self, "Select or Create ADF File", "", "YAML Files (*.yaml)", options=options)
        if file_path:
            self.adf_filepath.setText(file_path)
            self.save_adf_button.setEnabled(True)

    
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
