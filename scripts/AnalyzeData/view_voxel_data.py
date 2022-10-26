from re import I
import h5py
import numpy as np
from pathlib import Path
import pandas as pd

np.set_printoptions(precision=3)


def get_all(name):
    if "voxels" in name:
        print(name)


def main(f):
    f.visit(get_all)
    print()

    ts_dataset = f["voxels_removed/voxel_time_stamp"]
    voxel_color = f["voxels_removed/voxel_color"][()]
    voxel_index = f["voxels_removed/voxel_removed"][()]

    # Load hdf5 into dataframe
    voxel_color = pd.DataFrame(voxel_color, columns=["ts", "r", "g", "b", "a"])
    voxel_index = pd.DataFrame(voxel_index, columns=["ts", "x", "y", "z"])

    print("timestamps", ts_dataset.shape)
    print("voxel color", voxel_color.shape)
    print("voxel index", voxel_index.shape)

    print(f"voxel df\n{voxel_color.head()}\n")

    print(f"voxels removed at step 5000 {voxel_color.loc[voxel_color['ts']==5000].shape}")
    print(f"voxels removed at step 8000 {voxel_color.loc[voxel_color['ts']==8000].shape}")


if __name__ == "__main__":

    path = Path("scripts/data-sample/new_rec_format.hdf5")
    if not path.exists():
        print("path does not exists")

    with h5py.File(path, "r") as f:
        main(f)
