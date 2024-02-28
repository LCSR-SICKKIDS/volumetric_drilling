import h5py

def print_hdf5_hierarchy(group, prefix=""):
    for key in group.keys():
        item = group[key]
        if isinstance(item, h5py.Group):
            print(prefix + f"Group: {key}")
            print_hdf5_hierarchy(item, prefix + "  ")
        elif isinstance(item, h5py.Dataset):
            print(prefix + f"Dataset: {key} (Shape: {item.shape}, Data Type: {item.dtype})")
            # if item.size <= 10:
                # Print the data if the dataset size is small
                # print(prefix + "Data:")
                # print(item[:])

def explore_hdf5_file(file_path):
    try:
        with h5py.File(file_path, 'r') as file:
            print("HDF5 file hierarchy for", file_path)
            print_hdf5_hierarchy(file)
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    hdf5_file_path = '/home/nimesh/Downloads/test2/2023-11-08 16:56:48/20231108_165648.hdf5'
    explore_hdf5_file(hdf5_file_path)






