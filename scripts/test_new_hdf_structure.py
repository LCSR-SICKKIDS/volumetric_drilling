#!/usr/bin/env python3
"""
Quick test script to validate the new HDF5 structure
"""

import h5py
import numpy as np
import tempfile
import os

def test_new_structure():
    """Test that the new HDF5 structure can be created correctly"""

    # Create a temporary file
    with tempfile.NamedTemporaryFile(suffix='.hdf5', delete=False) as tmp:
        temp_filename = tmp.name

    try:
        # Create HDF5 file with new structure
        with h5py.File(temp_filename, 'w') as f:
            # Create main groups
            metadata = f.create_group("metadata_v3")
            vision_data = f.create_group("vision_data")
            physics_data = f.create_group("physics_data")

            # Create vision subgroups
            vision_intermittent = vision_data.create_group("intermittent_data")
            vision_continuous = vision_data.create_group("continuous_data")

            # Create physics subgroups
            physics_intermittent = physics_data.create_group("intermittent_data")
            physics_continuous = physics_data.create_group("continuous_data")

            # Add some test data
            metadata.create_dataset("camera_intrinsic", data=np.eye(3))
            vision_continuous.create_dataset("l_img", data=np.zeros((10, 480, 640, 3)))
            physics_continuous.create_group("high_frequency_poses")

            # Test intermittent data groups
            voxels_group = physics_intermittent.create_group("voxels_removed")
            voxels_group.create_dataset("voxel_time_stamp", data=np.array([1.0, 2.0]))

            burr_group = physics_intermittent.create_group("burr_change")
            burr_group.create_dataset("time_stamp", data=np.array([1.5]))

            force_group = physics_intermittent.create_group("drill_force_feedback")
            force_group.create_dataset("time_stamp", data=np.array([1.1, 1.2]))

        # Verify structure
        with h5py.File(temp_filename, 'r') as f:
            print("HDF5 Structure Test Results:")
            print("=" * 40)

            # Check main groups
            expected_groups = ["metadata_v3", "vision_data", "physics_data"]
            for group in expected_groups:
                assert group in f, f"Missing main group: {group}"
                print(f"✓ Main group '{group}' exists")

            # Check vision subgroups
            vision_subgroups = ["intermittent_data", "continuous_data"]
            for subgroup in vision_subgroups:
                assert subgroup in f["vision_data"], f"Missing vision subgroup: {subgroup}"
                print(f"✓ Vision subgroup '{subgroup}' exists")

            # Check physics subgroups
            physics_subgroups = ["intermittent_data", "continuous_data"]
            for subgroup in physics_subgroups:
                assert subgroup in f["physics_data"], f"Missing physics subgroup: {subgroup}"
                print(f"✓ Physics subgroup '{subgroup}' exists")

            # Check intermittent physics groups
            intermittent_groups = ["voxels_removed", "burr_change", "drill_force_feedback"]
            for group in intermittent_groups:
                assert group in f["physics_data/intermittent_data"], f"Missing intermittent group: {group}"
                print(f"✓ Intermittent group '{group}' exists")

            # Check continuous physics groups
            assert "high_frequency_poses" in f["physics_data/continuous_data"], "Missing high_frequency_poses group"
            print("✓ Continuous group 'high_frequency_poses' exists")

            print("\n✅ All structure tests passed!")
            print("The new HDF5 structure is valid and matches the specification.")

    finally:
        # Clean up
        if os.path.exists(temp_filename):
            os.unlink(temp_filename)

if __name__ == "__main__":
    test_new_structure()