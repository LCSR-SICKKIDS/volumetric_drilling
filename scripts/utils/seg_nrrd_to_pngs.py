#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2021

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

#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
import numpy as np
import nrrd
from argparse import ArgumentParser
import re
from pathlib import Path
from shutil import rmtree
import time
from volume_data_to_slices import *


class RGBA:
    def __init__(self, rgba):
        self.R = rgba[0]
        self.G = rgba[1]
        self.B = rgba[2]
        try:
            self.A = rgba[3]
        except:
            # only three values provided, just set alpha to 1.0
            self.A = 1.0

    def print_color(self):
        print('RGBA: ', self.R, self.G, self.B, self.A)

    def to_str(self):
        string = str(self.R) + ', ' + str(self.G) + ', ' + str(self.B) + ', ' + str(self.A)
        return string
    
    def as_list(self):
        return [self.R, self.G, self.B, self.A]
    


class SegmentInfo:
    def __init__(self):
        self.index = None
        self.name = ''
        self.layer = None
        self.label = None
        self.color = None

    def fill(self, index, name, layer, label, color):
        self.index = index
        self.name = name
        self.layer = layer
        self.label = label
        self.color = RGBA(color)

    def print_info(self):
        print('Layer No.: ', self.layer)
        print('Segmentation Idx: ', self.index)
        print('Segmentation Name: ', self.name)
        print('Segmentation Label: ', self.label)
        print('Segmentation Color: ', self.color.to_str())


class SegNrrdCoalescer:
    def __init__(self, x_ratio=1, y_ratio=1, z_ratio=1):
        self._coalesced_data = None
        self.num_layers = 1 # Number of layers, could be 1
        self.num_segments = 0 # Number of segmentations, could be 1
        self.num_channels = 1 # Number of color channels, 1 for Grayscale and 4 for RGBA
        self.x_dim = 0
        self.y_dim = 0
        self.z_dim = 0
        self._segments_colors = None # Color of each segment that can be extracted from NRRD header
        self._segments_infos = []
        self.nrrd_data = None
        self.nrrd_hdr = None
        self.x_dim_ratio = x_ratio
        self.y_dim_ratio = y_ratio
        self.z_dim_ratio = z_ratio

    def set_nrrd(self, hdr, data):
        self.nrrd_hdr = hdr
        self.nrrd_data = data[:, ::self.x_dim_ratio, ::self.y_dim_ratio, ::self.z_dim_ratio]

    def _initialize_coalesced_data(self):
        # Create a 3D block where the first index refers to individual images,
        # The second index refers to x_dim, the third to y_dim and the fourth to z_dim(e.g. Z)

        self.num_layers = self.nrrd_data.shape[0]
        self.x_dim = self.nrrd_data.shape[1]
        self.y_dim = self.nrrd_data.shape[2]
        self.z_dim = self.nrrd_data.shape[3]

        if self.num_layers > 0:
            self.num_channels = 4
        else:
            self.num_channels = 1

        if self.num_channels == 1:
            self._coalesced_data = np.zeros([self.x_dim, self.y_dim,  self.z_dim])
        if self.num_channels == 4:
            self._coalesced_data = np.zeros([self.x_dim, self.y_dim, self.z_dim, self.num_channels])
        else:
            # Throw some error or warning
            pass

    def _initialize_segments_infos(self, nrrd_hdr):
        self.num_segments = self.find_number_of_segments(nrrd_hdr)
        for i in range(self.num_segments):
            self._segments_infos.append(SegmentInfo())

        for k, i in nrrd_hdr.items():
            key_str = '_ID'
            if k.find(key_str) != -1:
                prefix = k.split(key_str)[0]
                name_str = prefix + '_Name'
                layer_str = prefix + '_Layer'
                color_str = prefix + '_Color'
                label_str = prefix + '_LabelValue'

                idx = int(re.findall(r'\d+', k)[0])
                name = nrrd_hdr[name_str]
                try:
                    label = int(nrrd_hdr[label_str])
                except:
                    print('WARN! For segmentation ', name, ' no label data specified, setting to 1')
                    label = 1

                try:
                    layer = int(nrrd_hdr[layer_str])
                except:
                    print('WARN! For segmentation ', name, ' no layer data specified, setting to ', idx)
                    layer = idx

                color_data_str = nrrd_hdr[color_str]
                color_data = np.fromstring(color_data_str, dtype=float, sep=' ')

                self._segments_infos[idx].fill(idx, name, layer, label, color_data)

    def print_segments_infos(self):
        for i in range(self.num_segments):
            print('No. ', i)
            self._segments_infos[i].print_info()
            print('-------------------')

    def _coalesce_segments_into_3D_data(self):
        self._initialize_coalesced_data()
        self._initialize_segments_infos(self.nrrd_hdr)
        if self.num_channels == 4:
            start_time = time.time()
            for seg_info in self._segments_infos:
                # The segments can be separated into layers or collapsed. This implementation handles both
                print('\t INFO! Processing Segment', seg_info.index, ": ", seg_info.name)
                seg_data = self.nrrd_data[seg_info.layer, :, :, :]
                rgba_data = self.binary_to_rgba(seg_data, seg_info.label, seg_info.color.as_list())
                self._coalesced_data += rgba_data

            self._coalesced_data = np.clip(self._coalesced_data, 0., 1.)
            print("INFO! Coalescing segments took", time.time() - start_time, "seconds")
        else:
            raise Exception("ERROR! Expecting 4D data while provided data dimensions are", self.num_channels)
        
    def get_coalesced_data(self):
        self._coalesce_segments_into_3D_data()
        return self._coalesced_data


    @staticmethod
    def are_labelmaps_collapsed(h):
        if h['Segmentation_ConversionParameters'].find('Collapse labelmaps|1|') != -1:
            return True
        else:
            return False

    @staticmethod
    def find_number_of_segments(h):
        num_segs = 0
        for k,i in h.items():
            if re.findall(r'Segment\d+_Color', k):
                segment_idx = int(re.findall(r'\d+', k)[0])
                if segment_idx > num_segs:
                    num_segs = segment_idx
        return num_segs + 1
    
    @staticmethod
    def binary_to_rgba(binary_array, label, rgba):
        """
        Convert a 3D binary array (0s and 1s) to an RGBA array.
        """
        x, y, z = binary_array.shape
        rgba_array = np.zeros((x, y, z, 4))

        # Set white (255,255,255,255) for 1s and black (0,0,0,255) for 0s
        rgba_array[binary_array == label] = rgba

        return rgba_array



def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-n', action='store', dest='nrrd_file', help='Specify Nrrd File', required = True)
    parser.add_argument('-p', action='store', dest='slices_prefix', help='Specify Image Prefix', default='slice0')
    parser.add_argument('-s', action='store', dest='slices_path', help='Specify folder to store slices')
    parser.add_argument('--rx', action='store', dest='x_skip', help='X axis order [1-100]. Higher value indicates greater reduction', default=1)
    parser.add_argument('--ry', action='store', dest='y_skip', help='Y axis order [1-100]. Higher value indicates greater reduction', default=1)
    parser.add_argument('--rz', action='store', dest='z_skip', help='Z axis order [1-100]. Higher value indicates greater reduction', default=1)
    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    nrrd_converter = SegNrrdCoalescer(int(parsed_args.x_skip), int(parsed_args.y_skip), int(parsed_args.z_skip))
    data, hdr = nrrd.read(parsed_args.nrrd_file)
    nrrd_converter.set_nrrd(hdr, data)
    nrrd_converter.print_segments_infos()

    data = nrrd_converter.get_coalesced_data()
    
    save_volume_data_as_slices(data, parsed_args.slices_path, parsed_args.slices_prefix, "jet")

if __name__ == '__main__':
    main()
