# Old Data Structure Format

## Overview
This document describes the data structure and recording format used by `data_record_v3.py` for the FIVRS surgical simulator system. The script records data from both ROS and AMBF frameworks during virtual reality surgical simulations.

## Data Types Recorded

### 1. Main Data Group (`/data/`)
- **Left/Right Stereo Images** (`l_img`, `r_img`) - RGB camera images from stereo cameras
- **Depth Data** (`depth`) - 3D point cloud data converted to depth maps
- **Segmentation Images** (`segm`) - Segmented image data for object identification
- **Object Poses** (`pose_<object_name>`) - Position and orientation of tracked objects (drill, camera, anatomy)
- **Volume Pose** (`pose_mastoidectomy_volume`) - Static volume position/orientation in world coordinates

### 2. High-Frequency Poses (`/high_frequency_poses/`)
- **Drill Pose** (`drill_pose/`) - High-frequency drill position/orientation data
  - `time_stamp` - Timestamps for each pose sample
  - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]
- **Camera Pose** (`camera_pose/`) - High-frequency camera position/orientation data
  - `time_stamp` - Timestamps for each pose sample
  - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]
- **Anatomy Pose** (`anatomy_pose/`) - High-frequency anatomy object position/orientation data
  - `time_stamp` - Timestamps for each pose sample
  - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]

### 3. Voxel Data (`/voxels_removed/`)
- **Voxel Indices** (`voxel_removed`) - 3D coordinates of removed voxels during drilling
- **Voxel Colors** (`voxel_color`) - RGBA color values of removed voxels
- **Timestamps** (`voxel_time_stamp`) - When each voxel group was removed
- **README** - Documentation explaining voxel data structure

### 4. Force Feedback (`/drill_force_feedback/`)
- **Force/Torque Data** (`wrench`) - 6DOF force and torque values from haptic device
  - Format: [force_x, force_y, force_z, torque_x, torque_y, torque_z]
- **Timestamps** (`time_stamp`) - When each force measurement was taken

### 5. Burr Changes (`/burr_change/`)
- **Burr Size** (`burr_size`) - Drill burr size changes during simulation
- **Timestamps** (`time_stamp`) - When burr size changed

### 6. Metadata (`/metadata/`)
- **Camera Intrinsics** (`camera_intrinsic`) - Camera calibration matrix (3x3)
- **Camera Extrinsics** (`camera_extrinsic`) - Camera pose transformation matrix (4x4)
- **Voxel Volume** (`voxel_volume`) - Individual voxel volume in mm³
- **Baseline** (`baseline`) - Stereo camera baseline distance (if stereo cameras used)
- **README** - Documentation explaining coordinate systems and units

## Recording Frequencies

1. **Main Synchronized Data**: ~50Hz
   - Controlled by `queue_size=50` in TimeSynchronizer
   - Includes synchronized camera images, depth data, and poses

2. **High-Frequency Poses**: ~2kHz
   - Separate high-frequency capture for drill, camera, and anatomy poses
   - Not synchronized with main data stream

3. **Event-Driven Data**:
   - **Voxel Removal**: Triggered when drilling occurs
   - **Force Feedback**: Triggered when force/torque changes
   - **Burr Changes**: Triggered when drill size changes

## File Structure & Organization

### Data Format
- **File Format**: HDF5 with gzip compression
- **File Naming**: `YYYYMMDD_HHMMSS.hdf5`
- **Pose Format**: 7-element arrays [x, y, z, qx, qy, qz, qw]
  - Positions scaled by conversion factor (meters)
  - Quaternions in [x, y, z, w] order
- **Timestamps**: ROS timestamps converted to seconds since epoch

### Writing Strategy
- **Chunked Writing**: Data written to disk every 500 samples (configurable via `--chunk_size`)
- **Threading**: Separate thread handles HDF5 writing to prevent blocking data collection
- **Memory Management**: Data containers reset after each write to free memory
- **Real-time Processing**: 2ms sleep interval in timer callback for real-time performance

### Coordinate Systems
- **Position Units**: Meters (scaled from simulation units)
- **Pose Convention**: T_world_obj (world to object transformation)
- **Camera Convention**: Depth data corrected to OpenCV convention using extrinsic matrix
- **Quaternion Format**: [qx, qy, qz, qw] normalized quaternions

## Key Features

### Synchronization
- **Approximate Time Synchronizer**: Used for multi-modal data alignment
- **Configurable Sync**: Can toggle between exact and approximate time synchronization
- **Queue Management**: Prevents data loss with configurable queue sizes

### Data Integrity
- **Thread Safety**: Mutex locks protect shared data structures
- **Error Handling**: Graceful handling of missing topics or failed subscriptions
- **Validation**: Dimension checking for voxel data consistency

### Scalability
- **Multi-object Support**: Configurable object tracking via `--objects` parameter
- **Flexible Topics**: All ROS topics configurable via command-line arguments
- **Extensible Structure**: Easy to add new data types to existing HDF5 groups

## Usage Notes
- Script is called from `gui_setup.yaml` configuration file
- Requires AMBF simulation to be running before data recording starts
- All ROS topics must be active for successful data recording
- Camera calibration data loaded from YAML configuration files