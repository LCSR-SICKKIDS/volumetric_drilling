"""
Script Name: Data Record v3
Authors: Oren Wei and Jonathan Wang
Date Created: 2024-11-15
Last Modified: 2024-12-11
Version: 1.0

Description:
    This script records the data in the background from both ROS and AMBF
    while running the virtual reality simulator. Previous iteration of this 
    script have already incorporated the overarching data groups "burr_change", 
    "data", "drill_force_feedback", "metadata", and "voxels_removed". This iteration
    adds the "high_frequency_poses" data group, saving the high frequency drill, camera, and anatomy poses.
    It also attempts to clean up the code for more efficiency.
    
Updated HDF5 File Structure:
    root/
      ├── burr_change/
      ├── data/
      ├── drill_force_feedback/
      ├── high_frequency_poses/
      ├── metadata/
      └── voxels_removed/

Usage:
    Called from within the gui_setup.yaml file
"""

import logging
import math
import os
import pathlib
import pickle
import sys
import time
from argparse import ArgumentParser
from collections import OrderedDict
from threading import Thread, Lock

import h5py
import numpy as np
import yaml

if sys.version_info[0] >= 3:
    from queue import Empty, Full, Queue
else:
    from Queue import Empty, Full, Queue

import message_filters
from msg_synchronizer import TimeSynchronizer
import ros_numpy
import rospy
from ambf_msgs.msg import RigidBodyState, CameraState # from https://github.com/WPI-AIM/ambf/tree/ambf-2.0/ambf_ros_modules
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2 # from https://github.com/ros/common_msgs
from geometry_msgs.msg import WrenchStamped # from https://github.com/ros/common_msgs

try:
    from volumetric_drilling_msgs.msg import Voxels, DrillSize, VolumeInfo
except ImportError:
    print(
        "\nvolumetric_drilling_msgs.msg: cannot open shared message file. "
        + "Please source <volumetric_plugin_path>/build/devel/setup.bash \n"
    )

# Global variables
voxel_data_lock = Lock()
drill_force_data_lock = Lock()
drill_pose_data_lock = Lock()
log = logging.getLogger()

def rpy_to_quat(roll, pitch, yaw):
    """
    Converts roll, pitch, and yaw (Euler angles) into a quaternion representation.

    Parameters:
    - roll (float): Rotation angle around the X-axis, in radians.
    - pitch (float): Rotation angle around the Y-axis, in radians.
    - yaw (float): Rotation angle around the Z-axis, in radians.

    Returns:
    - tuple: A tuple (x, y, z, w) representing the quaternion components.

    Notes:
    - The quaternion is normalized for consistency.
    """
    # Compute half-angles for efficiency
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    # Compute trigonometric terms
    cr, sr = np.cos(half_roll), np.sin(half_roll)
    cp, sp = np.cos(half_pitch), np.sin(half_pitch)
    cy, sy = np.cos(half_yaw), np.sin(half_yaw)
    # Calculate quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def depth_gen(depth_msg):
    """
    Generates a depth map from a ROS depth message.

    Parameters:
    - depth_msg: The input ROS PointCloud2 message containing depth information.

    Returns:
    - numpy.ndarray: A (H x W) array of z-values representing the depth map.

    Notes:
    - The depth map is scaled and reshaped for compatibility with the AMBF simulation format.
    - The output is converted to half-precision (float16) to optimize storage.
    """
   # Convert PointCloud2 message to structured numpy array
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
    # Extract and scale x, y, z coordinates
    xcol = xyz_array["x"][:, None] * scale
    ycol = xyz_array["y"][:, None] * scale
    zcol = xyz_array["z"][:, None] * scale
    # Combine scaled coordinates into a single array
    scaled_depth = np.concatenate([xcol, ycol, zcol], axis=-1)
    # Convert to half-precision (float16) to save storage
    scaled_depth = scaled_depth.astype(np.float16)
    # Reshape depth data to match AMBF convention, reversing height direction
    scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
    # Project using extrinsic matrix and extract z-values (last column)
    scaled_depth = np.einsum("ab,hwb->hwa", extrinsic[:3, :3], scaled_depth)[..., -1]
    return scaled_depth


def image_gen(image_msg):
    """
    Converts a ROS image message to an OpenCV-compatible format.

    Parameters:
    - image_msg: The input ROS sensor_msgs/Image message containing image data.

    Returns:
    - numpy.ndarray: The image in BGR format as a numpy array (OpenCV compatible), or None if conversion fails.
    """
    try:
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        return cv2_img
    except CvBridgeError as e:
        print(e)
        return None


def pose_gen(pose_msg):
    """
    Generates a numpy array representation of a pose from a ROS Pose message.

    Parameters:
    - pose_msg: The input ROS geometry_msgs/Pose message containing position and orientation.

    Returns:
    - numpy.ndarray: A 7-element numpy array containing the scaled position (x, y, z) and orientation (x, y, z, w).

    Notes:
    - The position values are scaled by a specified factor (`scale`).
    """
    # Extract position and orientation from the Pose message
    pose = pose_msg.pose
    # Create a numpy array with the scaled position (x, y, z) and the orientation (x, y, z, w)
    pose_np = np.array(
        [
            pose.position.x * scale,
            pose.position.y * scale,
            pose.position.z * scale,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    return pose_np


def load_yaml_file(file_path):
    """Helper function to load YAML files."""
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def calculate_camera_intrinsics(main_camera):
    """Calculate and return camera intrinsic matrix."""
    fva = main_camera["field view angle"]
    img_height = main_camera["publish image resolution"]["height"]
    img_width = main_camera["publish image resolution"]["width"]
    focal = img_height / (2 * math.tan(fva / 2))
    c_x = img_width / 2
    c_y = img_height / 2
    intrinsic = np.array([[focal, 0, c_x], [0, focal, c_y], [0, 0, 1]])
    return intrinsic, img_height, img_width


def calculate_conversion_factor(args, world_params):
    """Calculate and return the conversion factor based on input."""
    if sys.version_info[0] >= 3:
        with open(args.nrrd_header, "rb") as nrrd_header:
            header = pickle.load(nrrd_header)
            directions = header["space directions"]
            sizes = header["sizes"]
            largest_dim = np.argmax(sizes)
            s = np.linalg.norm(directions[largest_dim]) * sizes[largest_dim] / 1000.0
    else:
        s = world_params["conversion factor"]
    return s


def get_volume_pose(args, s):
    """Generate volume pose from ADF file."""
    volume_params = load_yaml_file(args.volume_adf)
    volume_name = volume_params["volumes"][0]
    volume_loc = volume_params[volume_name]["location"]
    
    volume_position = [
        volume_loc["position"]["x"] * s,
        volume_loc["position"]["y"] * s,
        volume_loc["position"]["z"] * s,
    ]
    volume_orientation = rpy_to_quat(
        volume_loc["orientation"]["r"],
        volume_loc["orientation"]["p"],
        volume_loc["orientation"]["y"],
    )
    return np.concatenate([volume_position, volume_orientation])


def create_hdf5_file(args, intrinsic, extrinsic, s, volume_pose):
    """Create and initialize the HDF5 file with the appropriate metadata."""
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    time_str = time.strftime("%Y%m%d_%H%M%S")
    file = h5py.File(os.path.join(args.output_dir, f"{time_str}.hdf5"), "w")
    
    metadata = file.create_group("metadata")
    metadata.create_dataset("camera_intrinsic", data=intrinsic)
    metadata.create_dataset("camera_extrinsic", data=extrinsic)
    metadata.create_dataset("README", data=(
        "All position information is in meters unless specified otherwise. \n"
        "Quaternion is a list in the order of [qx, qy, qz, qw]. \n"
        "Poses are defined to be T_world_obj. \n"
        "Depth in CV convention (corrected by extrinsic, T_cv_ambf). \n"
    ))

    return file, metadata


def init_hdf5(args):
    """Main function to initialize HDF5 file with camera, volume, and metadata."""
    # Load world parameters
    world_params = load_yaml_file(args.world_adf)
    main_camera = world_params["main_camera"]
    # Calculate camera intrinsics
    intrinsic, img_height, img_width = calculate_camera_intrinsics(main_camera)
    # Calculate conversion factor
    s = calculate_conversion_factor(args, world_params)
    # Get volume pose
    volume_pose = get_volume_pose(args, s)
    # Create the HDF5 file
    file, metadata = create_hdf5_file(args, intrinsic, extrinsic, s, volume_pose)
    # Optionally add stereo baseline info
    if args.stereo:
        stereo_params = load_yaml_file(args.stereo_adf)
        baseline = abs(stereo_params["stereoL"]["location"]["y"] - stereo_params["stereoR"]["location"]["y"]) * s
        metadata.create_dataset("baseline", data=baseline)
    # Create other necessary groups
    file.create_group("data")
    file.create_group("voxels_removed")
    file.create_group("burr_change")
    file.create_group("drill_force_feedback")
    file.create_group("high_frequency_poses")

    return file, img_height, img_width, s, volume_pose


def callback(*inputs):
    """
    Callback function to process inputs and store them in a data queue.
    The inputs are expected to be in a strict order: l_img, depth, r_img, segm, pose_A, pose_B, ...
    
    This function processes the input data, applies the corresponding transformation
    (such as image generation or depth processing), and stores the processed data in a queue.

    :param inputs: A list of input data, where the first element contains the timestamp,
                   and the last element contains a dictionary with data keys.
    :return: None
    """
    log.log(logging.DEBUG, "Callback function triggered")
    # Extract keys and initialize data dictionary with timestamp
    keys = list(inputs[-1])
    data = dict(time=inputs[0].header.stamp.to_sec())
    # Print progress every 5th data point
    if num_data % 5 == 0:
        print("Recording data: " + "#" * (num_data // 10))
    # Define a mapping from data keys to their respective processing functions
    data_processing_map = {
        "l_img": image_gen,
        "r_img": image_gen,
        "segm": image_gen,
        "depth": depth_gen,
        "pose_": pose_gen
    }
    # Process each input and add it to the data dictionary
    for idx, key in enumerate(keys[1:]):  # skip the first element (time)
        for data_key, processing_func in data_processing_map.items():
            if key.startswith(data_key):
                data[key] = processing_func(inputs[idx])
    # Try to add processed data to the queue, catch the Full exception
    try:
        data_queue.put_nowait(data)
    except Full:
        log.log(logging.DEBUG, "Queue is full, data not added")


def create_and_store_dataset(group, key, data, compression="gzip"):
    """
    Helper function to create and store a dataset in the HDF5 group.
    
    :param group: HDF5 group where the dataset will be created.
    :param key: Key for the dataset.
    :param data: Data to be stored in the dataset.
    :param compression: Compression type for the dataset.
    """
    if len(data) > 0:
        print(f"key {key}")
        group.create_dataset(key, data=np.stack(data, axis=0), compression=compression)
        log.log(logging.INFO, (key, group[key].shape))


def write_voxel_data(collisions):
    """
    Process and write voxel data to HDF5.
    
    :param collisions: Dictionary containing voxel-related data.
    :return: None
    """
    voxel_idx, voxel_color = [], []

    try:
        assert len(collisions["voxel_color"]) == len(collisions["voxel_removed"]) == len(collisions["voxel_time_stamp"]), \
            "Dimension mismatch in voxel data"
    except AssertionError:
        print("Voxel data dimension mismatch:", collisions)
        raise

    for idx in range(len(collisions["voxel_time_stamp"])):
        num_removed = collisions["voxel_removed"][idx].shape[0]
        if num_removed > 0:
            idx_column = np.ones((num_removed, 1)) * idx
            voxel_idx.append(np.hstack((idx_column, collisions["voxel_removed"][idx])))
            voxel_color.append(np.hstack((idx_column, collisions["voxel_color"][idx])))

    # Combine the voxel data and write it to the HDF5 file
    try:
        voxel_idx = np.vstack(voxel_idx)
        voxel_color = np.vstack(voxel_color)
        voxel_data = {
            "voxel_time_stamp": collisions["voxel_time_stamp"],
            "voxel_removed": voxel_idx,
            "voxel_color": voxel_color,
        }
        for key, value in voxel_data.items():
            print(f"key {key}")
            f["voxels_removed"].create_dataset(key, data=value, compression="gzip")
            log.log(logging.INFO, (key, f["voxels_removed"][key].shape))
            collisions[key] = []  # Reset to free memory
        f["voxels_removed"].create_dataset("README", data = "voxels_removed contains a group of voxels (Gv) removed. \n"
                                           "The voxel_time_stamp contains the time that the Gv was removed. The voxels_removed contains the voxels that comprise the Gv. \n"
                                           "The voxel_color contains the color of voxels that comprise the Gv. \n")
    except Exception as e:
        print("INFO! No voxels removed in this batch due to exception:", str(e))


def write_high_frequency_pose_data(high_frequency_pose_data):
    """
    Process and write high-frequency pose data to HDF5.

    :param high_frequency_pose_data: Dictionary containing pose data for multiple objects.
    :return: None
    """
    try:
        for obj_name, data in high_frequency_pose_data.items():
            print(f"Storing high-frequency pose data for object: {obj_name}")
            group = f["high_frequency_poses"].create_group(obj_name)
            for key in ["pose", "time_stamp"]:
                print(f"Storing {key} data for {obj_name}")
                group.create_dataset(
                    key, data=np.stack(data[key], axis=0), compression="gzip"
                )
                log.log(logging.INFO, (key, group[key].shape))
            # Reset the data
            high_frequency_pose_data[obj_name] = {"time_stamp": [], "pose": []}
    except Exception as e:
        print("INFO! High-frequency pose data recording failed due to EXCEPTION:", str(e))


def write_volume_pose(volume_pose, num_samples):
    """
    Write volume pose data to the HDF5 file.
    
    :param volume_pose: The volume pose data.
    :param num_samples: The number of samples to write.
    :return: None
    """
    try:
        key = "pose_mastoidectomy_volume"
        f["data"].create_dataset(
            key, data=np.stack([volume_pose] * num_samples, axis=0), compression="gzip"
        )
        log.log(logging.INFO, (key, f["data"][key].shape))
    except Exception as e:
        print('INFO! No data recorded in this batch due to exception:', str(e))


def write_to_hdf5():
    """
    Main function to write data to an HDF5 file. It stores voxel data, drill pose data,
    and volume pose data in their respective HDF5 datasets.
    
    :return: None
    """
    # Create and store voxel volume dataset
    try:
        hdf5_vox_vol = f["metadata"].create_dataset("voxel_volume", data=voxel_volume)
        hdf5_vox_vol.attrs["units"] = "mm^3, millimeters cubed"
    except Exception:
        f.close()
        print("File writing interrupted.")
        return
    # Save image data and burr_change data
    containers = [(f["data"], container), (f["burr_change"], burr_change), (f["drill_force_feedback"], drill_force_feedback)]
    for group, data in containers:
        if group.name == "/drill_force_feedback":
            drill_force_data_lock.acquire()
        for key, value in data.items():
            create_and_store_dataset(group, key, value)
            data[key] = []  # Reset to free memory
        if group.name == "/drill_force_feedback":
            drill_force_data_lock.release()
    # Save voxel data
    write_voxel_data(collisions)
    try:
        num_samples = len(f["data"][list(f["data"].keys())[0]])  # Number of samples in data
        write_volume_pose(volume_pose, num_samples)
    except Exception as e:
        print('INFO! No data recorded in this batch since EXCEPTION:', str(e))
    
    # Store high-frequency pose data
    high_frequency_pose_data_lock.acquire()
    write_high_frequency_pose_data(high_frequency_pose_data)
    high_frequency_pose_data_lock.release()

    # Close the file after all data is written
    f.close()
    print("Finished writing and closing HDF5 file")


def write_and_reinitialize_hdf5(f, args):
    """
    Helper function to write data to HDF5 and reinitialize the file.
    This function is called when the chunk size is reached or when the recording is done.
    """
    log.log(logging.INFO, "\nWrite data to disk")
    write_to_hdf5()  # Write the data to disk
    f, _, _, _, _ = init_hdf5(args)  # Re-initialize the HDF5 file
    return f, 0  # Reset num_data to 0 after writing


def process_data(data_dict, container):
    """
    Helper function to process and store data in containers.
    This function appends the incoming data to the respective containers.
    """
    for key, data in data_dict.items():
        container[key].append(data)


def timer_callback():
    """
    Callback function that continuously checks for new data in the queue and writes it to an HDF5 file
    when a specified chunk size is reached. This function is designed to handle data in real-time,
    minimizing any potential performance overhead while ensuring data integrity.
    """
    global terminate_recording, finished_recording, num_data, f
    terminate_recording = False
    finished_recording = False
    while not terminate_recording:
        log.log(logging.NOTSET, "Timer callback - Checking for data")
        try:
            data_dict = data_queue.get_nowait()  # Non-blocking call to get data
            # Process and store data
            process_data(data_dict, container)
            num_data += 1
            if num_data >= chunk:
                f, num_data = write_and_reinitialize_hdf5(f, args)
        except Empty:
            log.log(logging.NOTSET, "Queue is empty, waiting for data")
        # Dynamically adjust sleep time based on the load or any external factors
        time.sleep(0.002)  # Sleep for 2ms (adjustable based on real-time needs)
    # Ensure that any remaining data is written to disk after recording finishes
    write_to_hdf5()
    finished_recording = True
    log.log(logging.INFO, "Finished recording and data written to disk")


def rm_vox_callback(rm_vox_msg):
    global voxel_data_lock
    voxel_data_lock.acquire()

    # Convert voxel removed and voxel color to numpy
    voxels_colors = []
    voxels_indices = []
    for idx in range(len(rm_vox_msg.indices)):
        vcolor = rm_vox_msg.colors[idx]
        vidx = rm_vox_msg.indices[idx]
        voxels_colors.append([vcolor.r, vcolor.g, vcolor.b, vcolor.a])
        voxels_indices.append([vidx.x, vidx.y, vidx.z])
    voxels_colors = np.array(voxels_colors) * 255
    voxels_indices = np.array(voxels_indices)

    collisions["voxel_time_stamp"].append(rm_vox_msg.header.stamp.to_sec())
    collisions["voxel_removed"].append(voxels_indices)
    collisions["voxel_color"].append(voxels_colors)
    voxel_data_lock.release()


def drill_force_feedback_callback(wrench_msg):
    global drill_force_data_lock
    drill_force_data_lock.acquire()
    wrench = [wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z,
              wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z]
    drill_force_feedback['time_stamp'].append(wrench_msg.header.stamp.to_sec())
    drill_force_feedback['wrench'].append(wrench)
    drill_force_data_lock.release()


def burr_change_callback(burr_change_msg):
    global burr_change
    burr_change["time_stamp"].append(burr_change_msg.header.stamp.to_sec())
    burr_change["burr_size"].append(burr_change_msg.size.data)


def volume_prop_callback(volume_prop_msg):
    global voxel_volume
    dimensions = volume_prop_msg.dimensions
    voxel_count = volume_prop_msg.voxel_count
    resolution = np.divide(dimensions, voxel_count) * 1000
    voxel_volume = np.prod(resolution) * scale ** 3

def high_frequency_pose_callback(pose_msg, name):
    """
    Callback function to capture high-frequency pose data for multiple objects.

    Args:
        pose_msg: The message containing pose information.
        name: The name of the object (e.g., 'mastoidectomy_drill', 'main_camera', etc.).
    """
    # Extract position and orientation based on message type
    if hasattr(pose_msg, 'pose'):
        pose = pose_msg.pose
    else:
        # Handle other message types if necessary
        return

    # Create a numpy array with the scaled position (x, y, z) and the orientation (x, y, z, w)
    pose_np = np.array(
        [
            pose.position.x * scale,
            pose.position.y * scale,
            pose.position.z * scale,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )

    timestamp = pose_msg.header.stamp.to_sec()  # Use the timestamp from the message header

    # Store in the high_frequency_pose_data dictionary
    with high_frequency_pose_data_lock:
        if name not in high_frequency_pose_data:
            high_frequency_pose_data[name] = {"time_stamp": [], "pose": []}
        high_frequency_pose_data[name]["time_stamp"].append(timestamp)
        high_frequency_pose_data[name]["pose"].append(pose_np)
    
def setup_subscriber(args):
    active_topics = [n for [n, _] in rospy.get_published_topics()]
    subscribers = []
    topics = []

    if active_topics == ["/rosout_agg", "/rosout"]:
        log.log(logging.CRITICAL, "CRITICAL! Launch simulation before recording!")
        exit()

    if args.stereoL_topic != "None":
        if args.stereoL_topic in active_topics:
            stereoL_sub = message_filters.Subscriber(args.stereoL_topic, Image)
            subscribers += [stereoL_sub]
            container["l_img"] = []
            topics += [args.stereoL_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.stereoL_topic)
            exit()

    if args.depth_topic != "None":
        if args.depth_topic in active_topics:
            depth_sub = message_filters.Subscriber(args.depth_topic, PointCloud2)
            subscribers += [depth_sub]
            container["depth"] = []
            topics += [args.depth_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.depth_topic)
            exit()

    if args.stereoR_topic != "None":
        if args.stereoR_topic in active_topics:
            stereoR_sub = message_filters.Subscriber(args.stereoR_topic, Image)
            subscribers += [stereoR_sub]
            container["r_img"] = []
            topics += [args.stereoR_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.stereoR_topic)
            exit()

    if args.segm_topic != "None":
        if args.segm_topic in active_topics:
            segm_sub = message_filters.Subscriber(args.segm_topic, Image)
            subscribers += [segm_sub]
            container["segm"] = []
            topics += [args.segm_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.segm_topic)
            exit()

    if args.rm_vox_topic != "None":
        if args.rm_vox_topic in active_topics:
            rospy.Subscriber(args.rm_vox_topic, Voxels, rm_vox_callback)
            collisions["voxel_time_stamp"] = []
            collisions["voxel_removed"] = []
            collisions["voxel_color"] = []
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.rm_vox_topic)
            exit()

    if args.burr_change_topic != "None":
        if args.burr_change_topic in active_topics:
            rospy.Subscriber(args.burr_change_topic, DrillSize, burr_change_callback)
            burr_change["time_stamp"] = []
            burr_change["burr_size"] = []
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.burr_change_topic)
            exit()

    if args.volume_prop_topic != "None":
        if args.volume_prop_topic in active_topics:
            rospy.Subscriber(args.volume_prop_topic, VolumeInfo, volume_prop_callback)
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.volume_prop_topic)
            exit()

    if args.drill_force_feedback_topic != 'None':
        if args.drill_force_feedback_topic in active_topics:
            rospy.Subscriber(args.drill_force_feedback_topic, WrenchStamped, drill_force_feedback_callback)
            # Can I just use omni_force here or do I have to use a different variable?
            drill_force_feedback['time_stamp'] = []
            drill_force_feedback['wrench'] =[]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.force_topic)
            exit()
            
    # poses
    for name in args.objects:
        if "camera" in name: # this is for Main Camera State
            topic = "/ambf/env/" + "cameras/" + name + "/State"
            pose_sub = message_filters.Subscriber(topic, CameraState)
        else: # this is for Mastoidectomy Drill State
            topic = "/ambf/env/" + name + "/State"
            pose_sub = message_filters.Subscriber(topic, RigidBodyState)
            pose_sub.registerCallback(lambda msg, name=name: drill_pose_callback(msg, name))

        if topic in active_topics:
            container["pose_" + name] = []
            subscribers += [pose_sub]
            topics += [topic]
        else:
            print("Failed to subscribe to", topic)
            exit()

    log.log(logging.INFO, "\n".join(["Subscribed to the following topics:"] + topics))
    return subscribers


def main(args):
    container["time"] = []

    # Setup ROS node and subscribers
    rospy.init_node("data_recorder")
    subscribers = setup_subscriber(args)

    # High-frequency pose subscribers
    for name in args.objects:
        if "camera" in name:
            topic = "/ambf/env/" + "cameras/" + name + "/State"
            rospy.Subscriber(topic, CameraState, high_frequency_pose_callback, callback_args=name)
        else:
            topic = "/ambf/env/" + name + "/State"
            rospy.Subscriber(topic, RigidBodyState, high_frequency_pose_callback, callback_args=name)

    print("Synchronous? : ", args.sync)
    # NOTE: don't set queue size to a large number (e.g. 1000).
    # Otherwise, the time taken to compute synchronization becomes very long and no more message will be spit out.
    if args.sync is False:
        ats = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=50, slop=0.01)
        ats.registerCallback(callback, container.keys())
    else:
        ats = TimeSynchronizer(subscribers, queue_size=50)
        ats.registerCallback(callback, container.keys())

    # separate thread for writing to hdf5 to release memory
    # rospy.Timer(rospy.Duration(0, 500000), timer_callback)  # set to 2Khz such that we don't miss pose data
    global terminate_recording, finished_recording
    timer_thread = Thread(target=timer_callback)
    timer_thread.start()

    print("Writing to HDF5 every chunk of %d data" % args.chunk_size)

    rospy.spin()
    terminate_recording = True

    while not finished_recording:
        print('Waiting for recording to finish')
        time.sleep(1.0)

    print("Terminating ", __file__)
    # write_to_hdf5()  # save when user exits


def verify_cv_bridge():
    arr = np.zeros([480, 640])
    msg = bridge.cv2_to_imgmsg(arr)
    try:
        bridge.imgmsg_to_cv2(msg)
    except ImportError:
        log.log(
            logging.WARNING,
            "libcv_bridge.so: cannot open shared object file. Please source ros env first.",
        )
        return False

    return True


if __name__ == "__main__":
    parser = ArgumentParser()

    parser.add_argument("--output_dir", default="data", type=str)

    resolved_path = str(pathlib.Path(os.path.dirname(__file__)).resolve())

    #fmt: off
    parser.add_argument("--world_adf", default=resolved_path + "/../ADF/world/world.yaml", type=str)
    parser.add_argument("--volume_adf", default=resolved_path + "/../ADF/volume_171.yaml", type=str)
    parser.add_argument("--stereo_adf", default=resolved_path + "/../ADF/stereo_cameras.yaml", type=str)
    parser.add_argument("--nrrd_header", default=resolved_path + "/../resources/volumes/nrrd_header.pkl", type=str)

    ambf_prefix = "/ambf/env"
    parser.add_argument("--stereoL_topic", default=ambf_prefix + "/cameras/stereoL/ImageData", type=str)
    parser.add_argument("--depth_topic", default=ambf_prefix + "/cameras/segmentation_camera/DepthData", type=str)
    parser.add_argument("--stereoR_topic", default=ambf_prefix + "/cameras/stereoR/ImageData", type=str)
    parser.add_argument("--segm_topic", default=ambf_prefix + "/cameras/segmentation_camera/ImageData", type=str)
    parser.add_argument("--rm_vox_topic", default=ambf_prefix + "/plugin/volumetric_drilling/voxels_removed", type=str,)
    parser.add_argument("--burr_change_topic", default=ambf_prefix + "/plugin/volumetric_drilling/drill_size", type=str,)
    parser.add_argument("--volume_prop_topic", default=ambf_prefix + "/plugin/volumetric_drilling/volume_info", type=str,)
    parser.add_argument("--objects", default=["mastoidectomy_drill", "main_camera"], type=str, nargs="+")
    parser.add_argument("--drill_force_feedback_topic", default=ambf_prefix + "/plugin/volumetric_drilling/drill_force_feedback", type=str,)

    parser.add_argument("--sync", action="store_true")
    parser.add_argument(
        "--chunk_size", type=int, default=500, help="Write to disk every chunk size"
    )
    #fmt: on

    parser.add_argument("--debug", action="store_true")

    args = parser.parse_args()
    print("Provided args: \n", args)
    # init cv bridge for data conversion
    bridge = CvBridge()
    valid = verify_cv_bridge()
    if not valid:
        exit()

    # init logger
    log = logging.getLogger("logger")
    log.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(message)s")
    ch = logging.StreamHandler()
    if args.debug:
        ch.setLevel(logging.DEBUG)
    else:
        ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    log.addHandler(ch)

    # camera extrinsics, the transformation that pre-multiplies recorded poses to match opencv convention
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])  # T_cv_ambf

    # check topics and see if we need to read stereo adf for baseline
    if args.stereoL_topic is not None and args.stereoR_topic is not None:
        args.stereo = True
    else:
        args.stereo = False

    terminate_recording = False
    finished_recording = True
    voxel_data_lock = Lock()
    drill_force_data_lock = Lock()
    drill_pose_data_lock = Lock()

    f, h, w, scale, volume_pose = init_hdf5(args)

    # initialize queue for multi-threading
    chunk = args.chunk_size
    data_queue = Queue(chunk * 2)
    num_data = 0
    container = OrderedDict()
    collisions = OrderedDict()
    burr_change = OrderedDict()
    drill_force_feedback = OrderedDict()
    voxel_volume = 0
    drill_pose_data = OrderedDict()

    # Initialize high-frequency pose data
    high_frequency_pose_data = OrderedDict()
    high_frequency_pose_data_lock = Lock()

    main(args)
