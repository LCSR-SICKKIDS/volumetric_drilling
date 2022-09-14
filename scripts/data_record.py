import logging
import math
import os
import pathlib
import pickle
import sys
import time
from argparse import ArgumentParser
from collections import OrderedDict

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
from ambf_msgs.msg import RigidBodyState, CameraState
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

try:
    from vdrilling_msgs.msg import points, UInt8Stamped, VolumeProp
except ImportError:
    print("\nvdrilling_msgs.msg: cannot open shared message file. " +
          "Please source <volumetric_plugin_path>/vdrilling_msgs/build/devel/setup.bash \n")


def rpy_to_quat(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def depth_gen(depth_msg):
    """
    generate depth
    :param depth_msg:
    :return: HxW, z-values
    """
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
    xcol = xyz_array['x'][:, None] * scale
    ycol = xyz_array['y'][:, None] * scale
    zcol = xyz_array['z'][:, None] * scale

    scaled_depth = np.concatenate([xcol, ycol, zcol], axis=-1)
    # halve precision to save storage
    scaled_depth = scaled_depth.astype(np.float16)
    # reverse height direction due to AMBF reshaping
    scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
    # convert to cv convention
    scaled_depth = np.einsum(
        'ab,hwb->hwa', extrinsic[:3, :3], scaled_depth)[..., -1]

    return scaled_depth


def image_gen(image_msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        return cv2_img
    except CvBridgeError as e:
        print(e)
        return None


def pose_gen(pose_msg):
    pose = pose_msg.pose
    pose_np = np.array([
        pose.position.x * scale,
        pose.position.y * scale,
        pose.position.z * scale,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

    return pose_np


def init_hdf5(args, stereo):
    world_adf = open(args.world_adf, "r")
    world_params = yaml.safe_load(world_adf)
    world_adf.close()

    main_camera = world_params["main_camera"]

    # perspective camera intrinsics
    fva = main_camera["field view angle"]
    img_height = main_camera["publish image resolution"]["height"]
    img_width = main_camera["publish image resolution"]["width"]

    focal = img_height / (2 * math.tan(fva / 2))
    c_x = img_width / 2
    c_y = img_height / 2
    intrinsic = np.array([[focal, 0, c_x], [0, focal, c_y], [0, 0, 1]])

    # conversion factor
    if sys.version_info[0] >= 3:
        nrrd_header = open(args.nrrd_header, "rb")
        header = pickle.load(nrrd_header)
        directions = header['space directions']
        sizes = header['sizes']
        largest_dim = np.argmax(sizes)
        s = np.linalg.norm(directions[largest_dim]) * sizes[largest_dim] / 1000.0
    else:
        s = world_params["conversion factor"]

    # volume pose, which is fixed
    volume_adf = open(args.volume_adf, "r")
    volume_params = yaml.safe_load(volume_adf)
    volume_adf.close()
    volume_name = volume_params["volumes"][0]
    volume_loc = volume_params[volume_name]["location"]
    volume_position = [volume_loc["position"]["x"] * s, volume_loc["position"]["y"] * s,
                       volume_loc["position"]["z"] * s]
    volume_orientation = rpy_to_quat(volume_loc["orientation"]["r"], volume_loc["orientation"]["p"],
                                     volume_loc["orientation"]["y"])  # ZYX needs to be capitalized
    volume_pose = np.concatenate([volume_position, volume_orientation])

    # Create hdf5 file with date
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    time_str = time.strftime("%Y%m%d_%H%M%S")
    file = h5py.File(args.output_dir + '/' + time_str + ".hdf5", "w")

    metadata = file.create_group("metadata")
    metadata.create_dataset("camera_intrinsic", data=intrinsic)
    metadata.create_dataset("camera_extrinsic", data=extrinsic)
    metadata.create_dataset("README", data="All position information is in meters unless specified otherwise. \n"
                                           "Quaternion is a list in the order of [qx, qy, qz, qw]. \n"
                                           "Poses are defined to be T_world_obj. \n"
                                           "Depth in CV convention (corrected by extrinsic, T_cv_ambf). \n")

    # baseline info from stereo adf
    if stereo:
        adf = args.stereo_adf
        stereo_adf = open(adf, "r")
        stereo_params = yaml.safe_load(stereo_adf)
        baseline = math.fabs(
            stereo_params['stereoL']['location']['y'] - stereo_params['stereoR']['location']['y']) * s
        metadata.create_dataset("baseline", data=baseline)

    file.create_group("data")
    file.create_group("voxels_removed")
    file.create_group("burr_change")

    return file, img_height, img_width, s, volume_pose


def callback(*inputs):
    """
    Current implementation strictly enforces the ordering
    ordering - l_img, depth, r_img, segm, pose_A, pose_B, ..., data_keys

    :param inputs:
    :return:
    """
    log.log(logging.DEBUG, "msg callback")

    keys = list(inputs[-1])
    data = dict(time=inputs[0].header.stamp.to_sec())

    if num_data % 5 == 0:
        print("Recording data: " + '#' * (num_data // 10))

    for idx, key in enumerate(keys[1:]):  # skip time
        if 'l_img' == key or 'r_img' == key or 'segm' == key:
            data[key] = image_gen(inputs[idx])
        if 'depth' == key:
            # print("depth")
            data[key] = depth_gen(inputs[idx])
        if 'pose_' in key:
            # print("pose")
            data[key] = pose_gen(inputs[idx])

    try:
        data_queue.put_nowait(data)
    except Full:
        log.log(logging.DEBUG, "Queue full")


def write_to_hdf5():
    hdf5_vox_vol = f['metadata'].create_dataset("voxel_volume", data=voxel_volume)
    hdf5_vox_vol.attrs['units'] = "mm^3, millimeters cubed"
    containers = [(f["data"], container), (f["voxels_removed"], collisions), (f["burr_change"], burr_change)]
    for group, data in containers:
        for key, value in data.items():
            if len(value) > 0:
                group.create_dataset(
                    key, data=np.stack(value, axis=0), compression='gzip')  # write to disk
                log.log(logging.INFO, (key, group[key].shape))
            data[key] = []  # reset list to empty memory

    # write volume pose
    key = "pose_mastoidectomy_volume"
    num_samples = len(f["data"][f["data"].keys()[0]])
    f["data"].create_dataset(key, data=np.stack([volume_pose] * num_samples, axis=0),
                             compression='gzip')  # write to disk
    log.log(logging.INFO, (key, f["data"][key].shape))

    f.close()

    return


def timer_callback(_):
    log.log(logging.NOTSET, "timer callback")
    try:
        data_dict = data_queue.get_nowait()
    except Empty:
        log.log(logging.NOTSET, "Empty queue")
        return

    global num_data, f
    for key, data in data_dict.items():
        container[key].append(data)

    num_data = num_data + 1
    if num_data >= chunk:
        log.log(logging.INFO, "\nWrite data to disk")
        write_to_hdf5()
        f, _, _, _, _ = init_hdf5(args, stereo)
        num_data = 0


def rm_vox_callback(rm_vox_msg):
    voxel = [rm_vox_msg.voxel_removed.x, rm_vox_msg.voxel_removed.y, rm_vox_msg.voxel_removed.z]
    collisions['time_stamp'].append(rm_vox_msg.header.stamp.to_sec())
    collisions['voxel_removed'].append(voxel)
    int_vox_color = [round(elem * 255) for elem in rm_vox_msg.voxel_color]
    collisions['voxel_color'].append(int_vox_color)


def burr_change_callback(burr_change_msg):
    global burr_change
    burr_change['time_stamp'].append(burr_change_msg.header.stamp.to_sec())
    burr_change['burr_size'].append(burr_change_msg.number.data)


def volume_prop_callback(volume_prop_msg):
    global voxel_volume
    dimensions = volume_prop_msg.dimensions
    voxelCount = volume_prop_msg.voxelCount
    resolution = np.divide(dimensions, voxelCount) * 1000
    voxel_volume = np.prod(resolution) * scale ** 3


def setup_subscriber(args):
    active_topics = [n for [n, _] in rospy.get_published_topics()]
    subscribers = []
    topics = []

    if active_topics == ['/rosout_agg', '/rosout']:
        log.log(logging.CRITICAL, 'CRITICAL! Launch simulation before recording!')
        exit()

    if args.stereoL_topic != 'None':
        if args.stereoL_topic in active_topics:
            stereoL_sub = message_filters.Subscriber(args.stereoL_topic, Image)
            subscribers += [stereoL_sub]
            container['l_img'] = []
            topics += [args.stereoL_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.stereoL_topic)
            exit()

    if args.depth_topic != 'None':
        if args.depth_topic in active_topics:
            depth_sub = message_filters.Subscriber(args.depth_topic, PointCloud2)
            subscribers += [depth_sub]
            container['depth'] = []
            topics += [args.depth_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.depth_topic)
            exit()

    if args.stereoR_topic != 'None':
        if args.stereoR_topic in active_topics:
            stereoR_sub = message_filters.Subscriber(args.stereoR_topic, Image)
            subscribers += [stereoR_sub]
            container['r_img'] = []
            topics += [args.stereoR_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.stereoR_topic)
            exit()

    if args.segm_topic != 'None':
        if args.segm_topic in active_topics:
            segm_sub = message_filters.Subscriber(args.segm_topic, Image)
            subscribers += [segm_sub]
            container['segm'] = []
            topics += [args.segm_topic]
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.segm_topic)
            exit()

    if args.rm_vox_topic != 'None':
        if args.rm_vox_topic in active_topics:
            rospy.Subscriber(args.rm_vox_topic, points, rm_vox_callback)
            collisions['time_stamp'] = []
            collisions['voxel_removed'] = []
            collisions['voxel_color'] = []
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.rm_vox_topic)
            exit()

    if args.burr_change_topic != 'None':
        if args.burr_change_topic in active_topics:
            rospy.Subscriber(args.burr_change_topic, UInt8Stamped, burr_change_callback)
            burr_change['time_stamp'] = []
            burr_change['burr_size'] = []
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.burr_change_topic)
            exit()

    if args.volume_prop_topic != 'None':
        if args.volume_prop_topic in active_topics:
            rospy.Subscriber(args.volume_prop_topic, VolumeProp, volume_prop_callback)
        else:
            log.log(logging.CRITICAL, "CRITICAL! Failed to subscribe to " + args.volume_prop_topic)
            exit()

    # poses
    for name in args.objects:
        if 'camera' in name:
            topic = '/ambf/env/' + 'cameras/' + name + '/State'
            pose_sub = message_filters.Subscriber(topic, CameraState)
        else:
            topic = '/ambf/env/' + name + '/State'
            pose_sub = message_filters.Subscriber(topic, RigidBodyState)

        if topic in active_topics:
            container['pose_' + name] = []
            subscribers += [pose_sub]
            topics += [topic]
        else:
            print("Failed to subscribe to", topic)
            exit()

    log.log(logging.INFO, '\n'.join(["Subscribed to the following topics:"] + topics))
    return subscribers


def main(args):
    container['time'] = []

    # setup ros node and subscribers
    rospy.init_node('data_recorder')
    subscribers = setup_subscriber(args)

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
    rospy.Timer(rospy.Duration(0, 500000), timer_callback)  # set to 2Khz such that we don't miss pose data
    print("Writing to HDF5 every chunk of %d data" % args.chunk_size)

    rospy.spin()
    print("Terminating ", __file__)
    write_to_hdf5()  # save when user exits


def verify_cv_bridge():
    arr = np.zeros([480, 640])
    msg = bridge.cv2_to_imgmsg(arr)
    try:
        bridge.imgmsg_to_cv2(msg)
    except ImportError:
        log.log(logging.WARNING, "libcv_bridge.so: cannot open shared object file. Please source ros env first.")
        return False

    return True


if __name__ == '__main__':
    parser = ArgumentParser()

    parser.add_argument(
        '--output_dir', default='data', type=str)

    resolved_path = str(pathlib.Path(os.path.dirname(__file__)).resolve())

    parser.add_argument(
        '--world_adf', default=resolved_path + '/../ADF/world/world.yaml', type=str)
    parser.add_argument(
        '--volume_adf', default=resolved_path + '/../ADF/volume_171.yaml', type=str)
    parser.add_argument(
        '--stereo_adf', default=resolved_path + '/../ADF/stereo_cameras.yaml', type=str)
    parser.add_argument(
        '--nrrd_header', default=resolved_path + '/../resources/volumes/nrrd_header.pkl', type=str)

    parser.add_argument(
        '--stereoL_topic', default='/ambf/env/cameras/stereoL/ImageData', type=str)
    parser.add_argument(
        '--depth_topic', default='/ambf/env/cameras/segmentation_camera/DepthData', type=str)
    parser.add_argument(
        '--stereoR_topic', default='/ambf/env/cameras/stereoR/ImageData', type=str)
    parser.add_argument(
        '--segm_topic', default='/ambf/env/cameras/segmentation_camera/ImageData', type=str)
    parser.add_argument(
        '--rm_vox_topic', default='/ambf/volumetric_drilling/voxels_removed', type=str)
    parser.add_argument(
        '--burr_change_topic', default='/ambf/volumetric_drilling/burr_change', type=str)
    parser.add_argument(
        '--volume_prop_topic', default='/ambf/volumetric_drilling/volume_prop', type=str)
    parser.add_argument(
        '--objects', default=['mastoidectomy_drill', 'main_camera'], type=str, nargs='+')

    parser.add_argument('--sync', action='store_true')
    parser.add_argument('--chunk_size', type=int, default=500,
                        help='Write to disk every chunk size')
    parser.add_argument('--debug', action='store_true')

    args = parser.parse_args()
    print("Provided args: \n", args)
    # init cv bridge for data conversion
    bridge = CvBridge()
    valid = verify_cv_bridge()
    if not valid:
        exit()

    # init logger
    log = logging.getLogger('logger')
    log.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(message)s')
    ch = logging.StreamHandler()
    if args.debug:
        ch.setLevel(logging.DEBUG)
    else:
        ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    log.addHandler(ch)

    # camera extrinsics, the transformation that pre-multiplies recorded poses to match opencv convention
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]])  # T_cv_ambf

    # check topics and see if we need to read stereo adf for baseline
    if args.stereoL_topic is not None and args.stereoR_topic is not None:
        stereo = True
    else:
        stereo = False
    f, h, w, scale, volume_pose = init_hdf5(args, stereo)

    # initialize queue for multi-threading
    chunk = args.chunk_size
    data_queue = Queue(chunk * 2)
    num_data = 0
    container = OrderedDict()
    collisions = OrderedDict()
    burr_change = OrderedDict()
    voxel_volume = 0

    main(args)
