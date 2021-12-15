# TODO: currently only works with python 2.7 due to cv_bridge, we should migrate to python3 but not sure best way yet

import math
import os
import time
from argparse import ArgumentParser

import h5py
import numpy as np
import yaml
from Queue import Empty, Full  # TODO: this is python2 syntax
from Queue import Queue

import message_filters
import ros_numpy
import rospy
from ambf_client import Client
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2


def object_info(data):
    for name, obj in objects.items():
        pos_temp = np.array([obj.get_pos().x, obj.get_pos().y, obj.get_pos().z]) * scale
        rot_temp = np.array([obj.get_rot().x, obj.get_rot().y, obj.get_rot().z, obj.get_rot().w])
        pose = np.concatenate([pos_temp, rot_temp], axis=0)
        data['pose_' + name] = pose


def depth_gen(depth_msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
    xcol = xyz_array['x'][:, None] * scale
    ycol = xyz_array['y'][:, None] * scale
    zcol = xyz_array['z'][:, None] * scale

    scaled_depth = np.concatenate([xcol, ycol, zcol], axis=-1)
    scaled_depth = scaled_depth.astype(np.float16)  # halve precision to save storage
    scaled_depth = scaled_depth.reshape([h, w, 3])

    return scaled_depth


def image_gen(image_msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        return cv2_img
    except CvBridgeError as e:
        print(e)
        return None


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
    metadata.create_dataset("camera_intrinsics", data=intrinsic)
    if stereo:
        baseline = math.fabs(camera_params['stereoL']['location']['y'] - camera_params['stereoR']['location']['y'])
        metadata.create_dataset("baseline", data=baseline)
    metadata.create_dataset("README", data="All position information is in meters unless specified otherwise \n"
                                           "Quaternion is a list in the order of [qx, qy, qz, qw] ")

    file.create_group("data")

    return file, img_height, img_width


def init_ambf():
    # Generate all objects in scene, even ones that may not be needed
    _client = Client()
    _client.connect()
    objects_dict = {}

    # TODO: maybe this can be an argument?
    ignore_list = ['World', 'light']  # remove world and light as we don't need them

    # Create python instances of AMBF objects
    obj_names = _client.get_obj_names()
    for obj_name in obj_names:
        obj_name = os.path.basename(os.path.normpath(obj_name))  # Get the last part of file path
        for ignore in ignore_list:
            if ignore in obj_name:
                break
        else:
            objects_dict[obj_name] = _client.get_obj_handle(obj_name)

    return _client, objects_dict


def callback(stereoL, depth=None, stereoR=None, segm=None):
    data = dict(time=stereoL.header.stamp.secs, l_img=image_gen(stereoL))

    if depth is not None:
        data['depth'] = depth_gen(depth)
    if stereoR is not None:
        data['r_img'] = image_gen(stereoR)
    if segm is not None:
        data['segm'] = image_gen(segm)

    object_info(data)

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

    stereoL_sub = message_filters.Subscriber(args.stereoL_topic, Image)
    depth_sub = message_filters.Subscriber(args.depth_topic, PointCloud2)
    # stereoR_sub = message_filters.Subscriber(stereoR_topic, Image)
    # segm_sub = message_filters.Subscriber(segm_topic, Image)

    ats = message_filters.ApproximateTimeSynchronizer([stereoL_sub, depth_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.Timer(rospy.Duration(0, 100000000), timer_callback)  # separate thread for writing to hdf5 to release memory
    # TODO: add an argument to set duration to be ~ 2x of publishing rate so we don't miss data

    rospy.spin()

    _client.clean_up()
    write_to_hdf5()  # save when user exits


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera_adf', default='../ADF/segmentation_camera.yaml', type=str)
    parser.add_argument('--camera_name', default=None, type=str)
    parser.add_argument('--stereo', action='store_true')

    parser.add_argument('--scale', type=float, default=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis')
    parser.add_argument('--chunk_size', type=int, default=200, help='Write to disk every chunk size')

    parser.add_argument('--stereoL_topic', default='/ambf/env/cameras/stereoL/ImageData', type=str)
    parser.add_argument('--stereoR_topic', default='/ambf/env/cameras/stereoR/ImageData', type=str)
    parser.add_argument('--depth_topic', default='/ambf/env/cameras/segmentation_camera/DepthData', type=str)
    parser.add_argument('--segm_topic', default='/ambf/env/cameras/segmentation_camera/ImageData', type=str)

    # TODO: record voxels (either what has been removed, or what is the current voxels) as asked by pete?

    args = parser.parse_args()

    bridge = CvBridge()
    _client, objects = init_ambf()
    chunk = args.chunk_size
    scale = args.scale

    f, h, w = init_hdf5(args.camera_adf, args.camera_name, args.stereo)

    data_queue = Queue(chunk)
    data_id = 0
    container = dict(depth=[], segm=[], l_img=[], r_img=[], time=[])
    for name, _ in objects.items():
        container['pose_' + name] = []

    main(args)
