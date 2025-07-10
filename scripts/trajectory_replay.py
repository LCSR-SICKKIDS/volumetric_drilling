import h5py
import numpy as np
from argparse import ArgumentParser
from pathlib import Path
from data_merger import DataMeger
from PyKDL import Frame, Rotation, Vector

def pose_list_to_pose_matrix(data):
    p = Vector([data[0], data[1], data[2]])
    r = Rotation.Quaternion([data[3], data[4], data[5], data[6]])
    return Frame(r, p)

def main(args):
    resolved_path = Path(args.path)
    data_meger = DataMeger()
    data = data_meger.get_merged_data(resolved_path)

    pose_list = data['data']['poses_mastoidectomy_drill']
    timestamp_list = data['data']['time']

    print("Size of timestamps list ", len(pose_list))
    print("Size of poses list ", len(timestamp_list))


if __name__ == __main__():
    parser = ArgumentParser()

    # path to adfs
    parser.add_argument('--path', type=str,
                        help='Path to recorded study with HDF5 files')
    args = parser.parse_args()
    main(args)