# New Data Structure Format

## Overview
This document describes the proposed reorganized data structure for the FIVRS surgical simulator system. The new structure divides data into three main categories based on data type and collection frequency, improving organization and data access patterns.

## Main Data Categories

### 1. Metadata (v3) (`/metadata_v3/`)
**Purpose**: Static configuration and calibration data
- **Camera Intrinsics** (`camera_intrinsic`) - Camera calibration matrix (3x3)
- **Camera Extrinsics** (`camera_extrinsic`) - Camera pose transformation matrix (4x4)
- **Voxel Volume** (`voxel_volume`) - Individual voxel volume in mm³
- **Baseline** (`baseline`) - Stereo camera baseline distance (if stereo cameras used)
- **README** - Documentation explaining coordinate systems and units

### 2. Vision Data (`/vision_data/`)
**Purpose**: All visual and spatial sensing information

#### 2.1 Intermittent Data (`/vision_data/intermittent_data/`)
**Purpose**: Event-driven or sparse vision-related data
- *Currently empty - reserved for future expansion*

#### 2.2 Continuous Data (`/vision_data/continuous_data/`)
**Purpose**: Regularly sampled visual and pose data (~50Hz synchronized)
- **Left Stereo Image** (`l_img`) - RGB camera images from left stereo camera
- **Right Stereo Image** (`r_img`) - RGB camera images from right stereo camera
- **Depth Data** (`depth`) - 3D point cloud data converted to depth maps
- **Segmentation Images** (`segm`) - Segmented image data for object identification
- **Object Poses** (`pose_<object_name>`) - Position and orientation of tracked objects
  - Format: 7-element arrays [x, y, z, qx, qy, qz, qw]
  - Includes drill, camera, anatomy poses from synchronized data stream
- **Volume Pose** (`pose_mastoidectomy_volume`) - Static volume position/orientation
- **Timestamps** (`time`) - Synchronized timestamps for all vision data

### 3. Physics Data (`/physics_data/`)
**Purpose**: All physical interaction and simulation data

#### 3.1 Intermittent Data (`/physics_data/intermittent_data/`)
**Purpose**: Event-driven physical interactions
- **Voxels Removed** (`voxels_removed/`)
  - `voxel_removed` - 3D coordinates of removed voxels during drilling
  - `voxel_color` - RGBA color values of removed voxels
  - `voxel_time_stamp` - When each voxel group was removed
  - `README` - Documentation explaining voxel data structure
- **Burr Changes** (`burr_change/`)
  - `burr_size` - Drill burr size changes during simulation
  - `time_stamp` - When burr size changed
- **Drill Force Feedback** (`drill_force_feedback/`)
  - `wrench` - 6DOF force and torque values from haptic device
    - Format: [force_x, force_y, force_z, torque_x, torque_y, torque_z]
  - `time_stamp` - When each force measurement was taken (event-driven)

#### 3.2 Continuous Data (`/physics_data/continuous_data/`)
**Purpose**: High-frequency continuously sampled pose data
- **High-Frequency Poses** (`high_frequency_poses/`)
  - **Drill Pose** (`drill_pose/`)
    - `time_stamp` - Timestamps for each pose sample (~2kHz)
    - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]
  - **Camera Pose** (`camera_pose/`)
    - `time_stamp` - Timestamps for each pose sample (~2kHz)
    - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]
  - **Anatomy Pose** (`anatomy_pose/`)
    - `time_stamp` - Timestamps for each pose sample (~2kHz)
    - `pose` - 7-element arrays [x, y, z, qx, qy, qz, qw]

## Data Migration Mapping

### From Old Structure → New Structure
```
/metadata/ → /metadata_v3/
/data/l_img → /vision_data/continuous_data/l_img
/data/r_img → /vision_data/continuous_data/r_img
/data/depth → /vision_data/continuous_data/depth
/data/segm → /vision_data/continuous_data/segm
/data/pose_* → /vision_data/continuous_data/pose_*
/data/pose_mastoidectomy_volume → /vision_data/continuous_data/pose_mastoidectomy_volume
/data/time → /vision_data/continuous_data/time
/high_frequency_poses/ → /physics_data/continuous_data/high_frequency_poses/
/drill_force_feedback/ → /physics_data/intermittent_data/drill_force_feedback/
/voxels_removed/ → /physics_data/intermittent_data/voxels_removed/
/burr_change/ → /physics_data/intermittent_data/burr_change/
```

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