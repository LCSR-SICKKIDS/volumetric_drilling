# TODO: currently only works with python 2.7 due to cv_bridge, we should migrate to python3 but not sure best way yet

import math
import os
import time
from argparse import ArgumentParser
from collections import OrderedDict

import h5py
import numpy as np
import yaml
from Queue import Empty, Full  # TODO: this is python2 syntax
from Queue import Queue

import message_filters
import ros_numpy
import rospy
from ambf_msgs.msg import RigidBodyState
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from utils import init_ambf


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
    scaled_depth = scaled_depth.astype(np.float16)  # halve precision to save storage
    # reverse height direction due to AMBF reshaping
    scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
    # convert to cv convention
    scaled_depth = np.einsum('ab,hwb->hwa', extrinsic[:3, :3], scaled_depth)[..., -1]

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
    pose_np = np.array(
        [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w])

    return pose_np


def init_hdf5(adf, camera_name, stereo):
    # perspective camera intrinsics
    camera_adf = open(adf, "r")
    camera_params = yaml.safe_load(camera_adf)

    fva = camera_params[camera_name]["field view angle"]
    img_height = camera_params[camera_name]["publish image resolution"]["height"]
    img_width = camera_params[camera_name]["publish image resolution"]["width"]

    focal = img_height / (2 * math.tan(fva / 2))
    c_x = img_width / 2
    c_y = img_height / 2
    intrinsic = np.array([[focal, 0, c_x], [0, focal, c_y], [0, 0, 1]])

    # Create hdf5 file with date
    if not os.path.exists('data'): os.makedirs('data')
    time_str = time.strftime("%Y%m%d_%H%M%S")
    file = h5py.File("./data/" + time_str + ".hdf5", "w")

    metadata = file.create_group("metadata")
    metadata.create_dataset("camera_intrinsic", data=intrinsic)
    metadata.create_dataset("camera_extrinsic", data=extrinsic)
    if stereo:
        baseline = math.fabs(camera_params['stereoL']['location']['y'] - camera_params['stereoR']['location']['y'])
        metadata.create_dataset("baseline", data=baseline)
    metadata.create_dataset("README", data="All position information is in meters unless specified otherwise \n"
                                           "Quaternion is a list in the order of [qx, qy, qz, qw] \n"
                                           "Extrinsic (T_cv_ambf) should be pre-multiplied to intrinsic")

    file.create_group("data")

    return file, img_height, img_width


def callback(*inputs):
    """
    Current implementation strictly enforces the ordering
    ordering - l_img, depth, r_img, segm, pose_A, pose_B, ..., data_keys

    :param inputs:
    :return:
    """
    keys = inputs[-1]
    data = dict(time=inputs[0].header.stamp.secs)

    # for inp in inputs[:-1]:
    #     print(inp.header.stamp.secs)

    for idx, key in enumerate(keys[1:]):  # skip time
        if 'l_img' == key or 'r_img' == key or 'segm' == key:
            data[key] = image_gen(inputs[idx])
        if 'depth' == key:
            data[key] = depth_gen(inputs[idx])
        if 'pose_' in key:
            data[key] = pose_gen(inputs[idx])

    try:
        data_queue.put_nowait(data)
    except Full:
        print('full')


def write_to_hdf5():
    data_group = f["data"]
    for key, value in container.items():
        if len(value) > 0:
            data_group.create_dataset(key, data=np.stack(value, axis=0))  # write to disk
            print(key, f["data"][key].shape)
        container[key] = []  # reset list to empty memory
    f.close()

    return


def timer_callback(_):
    try:
        data_dict = data_queue.get_nowait()
    except Empty:
        return

    global data_id, f
    for key, data in data_dict.items():
        container[key].append(data)

    data_id = data_id + 1
    if data_id >= chunk:
        print("write")
        write_to_hdf5()
        f, _, _ = init_hdf5(args.camera_adf, args.camera_name, args.stereo)
        data_id = 0


def main(args):
    print("started")

    container['time'] = []

    stereoL_sub = message_filters.Subscriber(args.stereoL_topic, Image)
    subscribers = [stereoL_sub]
    container['l_img'] = []

    if args.depth_topic != 'None':
        depth_sub = message_filters.Subscriber(args.depth_topic, PointCloud2)
        subscribers += [depth_sub]
        container['depth'] = []

    if args.stereoR_topic != 'None':
        stereoR_sub = message_filters.Subscriber(args.stereoR_topic, Image)
        subscribers += [stereoR_sub]
        container['r_img'] = []

    if args.segm_topic != 'None':
        segm_sub = message_filters.Subscriber(args.segm_topic, Image)
        subscribers += [segm_sub]
        container['segm'] = []

    # poses
    for name, _ in objects.items():
        container['pose_' + name] = []
        if 'camera' in name:
            name = 'cameras/' + name
        pose_sub = message_filters.Subscriber('/ambf/env/' + name + '/State', RigidBodyState)
        subscribers += [pose_sub]

    ats = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=10, slop=0.1)
    ats.registerCallback(callback, container.keys())

    rospy.Timer(rospy.Duration(0, 100000000), timer_callback)  # separate thread for writing to hdf5 to release memory
    # TODO: add an argument to set duration to be ~ 2x of publishing rate so we don't miss data

    rospy.spin()
    write_to_hdf5()  # save when user exits


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera_adf', default='../ADF/segmentation_camera.yaml', type=str)
    parser.add_argument('--camera_name', default=None, type=str)
    parser.add_argument('--stereo', action='store_true')

    parser.add_argument('--scale', type=float, default=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis')
    parser.add_argument('--chunk_size', type=int, default=500, help='Write to disk every chunk size')

    parser.add_argument('--stereoL_topic', default='/ambf/env/cameras/stereoL/ImageData', type=str)
    parser.add_argument('--depth_topic', default='/ambf/env/cameras/segmentation_camera/DepthData', type=str)
    parser.add_argument('--stereoR_topic', default='/ambf/env/cameras/stereoR/ImageData', type=str)
    parser.add_argument('--segm_topic', default='/ambf/env/cameras/segmentation_camera/ImageData', type=str)

    # TODO: record voxels (either what has been removed, or what is the current voxels) as asked by pete?

    args = parser.parse_args()

    bridge = CvBridge()
    _client, objects = init_ambf('data_record')
    _client.clean_up()
    chunk = args.chunk_size
    scale = args.scale

    # camera extrinsics, the transformation that pre-multiplies recorded poses to match opencv convention
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])  # T_cv_ambf

    f, h, w = init_hdf5(args.camera_adf, args.camera_name, args.stereo)

    data_queue = Queue(chunk)
    data_id = 0
    container = OrderedDict()

    main(args)
