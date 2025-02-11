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
import nrrd
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QFileDialog, QLabel
from PyQt5.QtCore import Qt
from seg_nrrd_to_pngs import SegmentInfo, SegNrrdCoalescer
import PIL.Image


class NRRDViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.nrrd_data = None
        self.nrrd_header = None
        self.current_slice = [0, 0, 0]
        self.segment_infos = SegmentInfo()

    def initUI(self):
        layout = QVBoxLayout()
        
        self.label = QLabel("No file loaded", self)
        layout.addWidget(self.label)
        
        self.loadButton = QPushButton("Load File (NRRD or SEG NRRD)", self)
        self.loadButton.clicked.connect(self.load_nrrd)
        layout.addWidget(self.loadButton)
        
        self.showButton = QPushButton("Show Slices", self)
        self.showButton.clicked.connect(self.show_slices)
        self.showButton.setEnabled(False)
        layout.addWidget(self.showButton)

        self.saveButton = QPushButton("Save slices as PNGs", self)
        self.saveButton.clicked.connect(self.save_slices_as_pngs)
        self.saveButton.setEnabled(False)
        layout.addWidget(self.saveButton)
        
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
        filePath, _ = QFileDialog.getOpenFileName(self, "Open NRRD File", "", "NRRD Files (*.nrrd *.seg.nrrd)", options=options)
        
        if filePath:
            self.nrrd_data, self.nrrd_header = nrrd.read(filePath)
            if len(self.nrrd_data.shape) == 4:  # Handle 4D segmentation data
                # self.nrrd_data = np.sum(self.nrrd_data, axis=-1)  # Coalesce along the last dimension
                nrrd_coalescer = SegNrrdCoalescer()
                nrrd_coalescer.read_nrrd(filePath)
                nrrd_coalescer.initialize_image_matrix()
                nrrd_coalescer.initialize_segments_infos(self.nrrd_header)
                nrrd_coalescer.coalesce_segments_into_3D_data()
                self.nrrd_data = nrrd_coalescer._images_matrix

            
            self.label.setText(f"Loaded: {filePath.split('/')[-1]}")
            self.showButton.setEnabled(True)
            self.saveButton.setEnabled(True)
            self.current_slice = [self.nrrd_data.shape[0] // 2, self.nrrd_data.shape[1] // 2, self.nrrd_data.shape[2] // 2]
            # print("NRRD Metadata:", self.nrrd_header)
            self.show_slices()

    def show_slices(self):
        if self.nrrd_data is not None:
            self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
            
            self.update_slices()
            
            self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
            plt.show()

    def save_slices_as_pngs(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Directory")

        if folder:
            print("INFO! Path selected", folder) 
            if self.nrrd_data is not None:
                cmap = 'gray' if 'segmentation' not in self.nrrd_header.get('type', '').lower() else 'jet'
                for i in range(self.nrrd_data.shape[2]):
                    im_name = folder + '/slice0' + str(i) + '.png'
                    plt.imsave(im_name, self.nrrd_data[:, :, i], cmap=cmap)

    
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
