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
from PIL import Image
import numpy as np
import nrrd
from argparse import ArgumentParser
import re
from pathlib import Path
from shutil import rmtree


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


class NrrdConverter:
    def __init__(self, x_ratio, y_ratio, z_ratio):
        self._images_matrix = None
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

    def read_nrrd(self, filename):
        data, self.nrrd_hdr = nrrd.read(filename)
        z_step = self.z_dim_ratio
        y_step = self.y_dim_ratio
        x_step = self.x_dim_ratio

        self.nrrd_data = data[:, ::x_step, ::y_step, ::z_step]

    def initialize_image_matrix(self):
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
            self._images_matrix = np.zeros([self.x_dim, self.y_dim,  self.z_dim])
        if self.num_channels == 4:
            self._images_matrix = np.zeros([self.x_dim, self.y_dim, self.z_dim, self.num_channels])
        else:
            # Throw some error or warning
            pass

    def initialize_segments_infos(self):
        self.num_segments = self.find_number_of_segments(self.nrrd_hdr)
        for i in range(self.num_segments):
            self._segments_infos.append(SegmentInfo())

        for k, i in self.nrrd_hdr.items():
            key_str = '_ID'
            if k.find(key_str) != -1:
                prefix = k.split(key_str)[0]
                name_str = prefix + '_Name'
                layer_str = prefix + '_Layer'
                color_str = prefix + '_Color'
                label_str = prefix + '_LabelValue'

                idx = int(re.findall(r'\d+', k)[0])
                name = self.nrrd_hdr[name_str]
                try:
                    label = int(self.nrrd_hdr[label_str])
                except:
                    print('WARN! For segmentation ', name, ' no label data specified, setting to 1')
                    label = 1

                try:
                    layer = int(self.nrrd_hdr[layer_str])
                except:
                    print('WARN! For segmentation ', name, ' no layer data specified, setting to ', idx)
                    layer = idx

                color_data_str = self.nrrd_hdr[color_str]
                color_data = np.fromstring(color_data_str, dtype=float, sep=' ')

                self._segments_infos[idx].fill(idx, name, layer, label, color_data)

    def print_segments_infos(self):
        for i in range(self.num_segments):
            print('No. ', i)
            self._segments_infos[i].print_info()
            print('-------------------')

    def copy_volume_to_image_matrix(self):
        are_label_maps_collapsed = self.are_labelmaps_collapsed(self.nrrd_hdr)
        if self.num_channels == 4:
            # for nl in range(self.num_layers):
            #     seg_data = self.nrrd_data[nl, :, :, :]
            #     print('Layer Number: ', nl)
            #     R = seg_data * self._segments_colors[nl, 0]
            #     print('\tStep: ', 1)
            #     G = seg_data * self._segments_colors[nl, 1]
            #     print ('\tStep: ', 2)
            #     B = seg_data * self._segments_colors[nl, 2]
            #     print('\tStep: ', 3)
            #     A = seg_data * self._segments_colors[nl, 3]
            #     print('\tStep: ', 4)
            #     RGBA = np.stack((R, G, B, A), axis=-1)
            #     print('\tStep: ', 5)
            #     self._images_matrix += RGBA
            #     print('\tStep: ', 6)

            for seg_info in self._segments_infos:
                seg_data = self.nrrd_data[seg_info.layer, :, :, :]
                if are_label_maps_collapsed:
                    label = seg_info.label
                    print('Processing Segmentation')
                    seg_info.print_info()
                    data = (seg_data == label).astype(int)
                else:
                    data = seg_data
                print('Layer Number: ', seg_info.layer)
                R = data * seg_info.color.R
                print('\tStep: ', 1)
                G = data * seg_info.color.G
                print('\tStep: ', 2)
                B = data * seg_info.color.B
                print('\tStep: ', 3)
                A = data * seg_info.color.A
                print('\tStep: ', 4)
                RGBA = np.stack((R, G, B, A), axis=-1)
                print('\tStep: ', 5)
                self._images_matrix += RGBA
                print('\tStep: ', 6)

    def normalize_image_matrix_data(self):
        max = self._images_matrix.max()
        min = self._images_matrix.min()
        self._images_matrix = (self._images_matrix - min) / float(max - min)

    def scale_image_matrix_data(self, scale):
        self._images_matrix = self._images_matrix * scale

    def normalize_data(self, data):
        max = data.max()
        min = data.min()
        normalized_data = (data - min) / float(max - min)
        return normalized_data

    def scale_data(self, data, scale):
        scaled_data = data * scale
        return scaled_data

    def save_image(self, array, im_name):
        im = Image.fromarray(array.astype(np.uint8))
        im.save(im_name)

    def save_image_matrix_as_images(self, dst_path: Path, im_prefix):
        print("Saving volume to png images at " + str(dst_path) + "...")
        for nz in range(self.z_dim):
            im_name = im_prefix + f"{nz}" + ".png"
            im_name = str(dst_path / im_name)
            self.save_image(self._images_matrix[:, :, nz, :], im_name)

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


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-n', action='store', dest='nrrd_file', help='Specify Nrrd File', required = True)
    parser.add_argument('-p', action='store', dest='image_prefix', help='Specify Image Prefix', default='plane00')
    parser.add_argument("-dst_p", action='store', help="Directory to save all images (Required)", required=True)
    parser.add_argument('--rx', action='store', dest='x_skip', help='X axis order [1-100]. Higher value indicates greater reduction', default=2)
    parser.add_argument('--ry', action='store', dest='y_skip', help='Y axis order [1-100]. Higher value indicates greater reduction', default=2)
    parser.add_argument('--rz', action='store', dest='z_skip', help='Z axis order [1-100]. Higher value indicates greater reduction', default=2)
    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    nrrd_converter = NrrdConverter(int(parsed_args.x_skip), int(parsed_args.y_skip), int(parsed_args.z_skip))
    nrrd_converter.read_nrrd(parsed_args.nrrd_file)
    nrrd_converter.initialize_image_matrix()
    nrrd_converter.initialize_segments_infos()
    nrrd_converter.print_segments_infos()

    nrrd_converter.copy_volume_to_image_matrix()
    # nrrd_converter.normalize_image_matrix_data()
    nrrd_converter.scale_image_matrix_data(255)
    
    dst_path = Path(parsed_args.dst_p)
    if dst_path.exists():
        rmtree(dst_path)
    dst_path.mkdir()
    nrrd_converter.save_image_matrix_as_images(dst_path, parsed_args.image_prefix)


if __name__ == '__main__':
    main()
