# TODO: currently only works with python 2.7 due to cv_bridge, we should migrate to python3 but not sure best way yet

import math
from argparse import ArgumentParser
from collections import OrderedDict

import numpy as np
import yaml
import sys
if sys.version_info[0] >= 3:
    from queue import Queue
else:
    from Queue import Queue

import message_filters
import ros_numpy
import rospy
from ambf_msgs.msg import RigidBodyState
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from utils import *
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

global intrinsic


def pose_to_matrix(pose):
    quat_norm = np.linalg.norm(pose[3:])
    assert np.isclose(quat_norm, 1.0)
    r = R.from_quat(pose[3:]).as_matrix()
    t = pose[:3]
    tau = np.identity(4)
    tau[:3, :3] = r
    tau[:3, -1] = t
    return tau


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
    pose_np = np.array(
        [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w])

    return pose_np


def init_camera_params(adf, camera_name):
    global intrinsic
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

    return img_height, img_width


def verify_sphere(depth, K, RT, pose_cam, pose_primitive, time_stamp):
    global last_depth
    # simple test, querying a point on the sphere
    query_point = np.array([1, 0, 0, 1])[None, :, None]  # homo, Nx4x1
    query_point_c = np.linalg.inv(pose_cam) @ (pose_primitive @ query_point)
    query_point_c = RT @ query_point_c
    uvz = K @ query_point_c[..., :3, :]
    u = np.rint((uvz[..., 0, :] / uvz[..., -1, :]).squeeze()).astype(int)
    v = np.rint((uvz[..., 1, :] / uvz[..., -1, :]).squeeze()).astype(int)
    z = uvz[..., -1, :].squeeze()

    h, w = depth.shape

    # make sure point is within image
    if not (0 <= u, u < w) or not (0 <= v, v < h):
        print("u: ", u,", v: ", v, " out of bounds, ignoring")
        return

    depth_output = depth[v, u]
    z_act = z
    last_depth = depth_output
    z_mea = depth_output
    ts = time_stamp
    err = z_act - z_mea
    time_str = INFO_STR("t: " + "{:10.6f}".format(ts))
    pose_str = "Cam X: " + toStr(pose_cam[0:3, 3][0])
    error_str = "Anal_z: " + \
        toStr(z_act) + " Depth_z: " + toStr(z_mea) + " Diff: "
    lag_lead_str = ""
    if abs(err) < 0.01:
        error_str = error_str + " " + OK_STR(err)
    else:
        error_str = error_str + " " + FAIL_STR(err)
        if cb_cntr > 0:
            if z_mea == last_depth:
                lag_lead_str = WARN_STR("LAG")
            else:
                lag_lead_str = WARN2_STR("LEAD")
    print(time_str, pose_str, error_str, lag_lead_str)

    return


def callback(*inputs):
    """
    Current implementation strictly enforces the ordering
    ordering - l_img, depth, r_img, segm, pose_A, pose_B, ..., data_keys

    :param inputs:
    :return:
    """
    global cb_cntr
    keys = list(inputs[-1])
    time_stamp = inputs[0].header.stamp.to_sec()
    data = dict(time=time_stamp)

    # for inp in inputs[:-1]:
    #     print(inp.header.stamp.secs)
    for idx, key in enumerate(keys[1:]):  # skip time
        if 'l_img' == key or 'r_img' == key or 'segm' == key:
            data[key] = image_gen(inputs[idx])
        if 'depth' == key:
            data[key] = depth_gen(inputs[idx])
        if 'pose_' in key:
            data[key] = pose_gen(inputs[idx])

    T_cam = pose_to_matrix(data['pose_main_camera'])
    T_sphere = pose_to_matrix(data['pose_Sphere'])
    verify_sphere(data['depth'], intrinsic, extrinsic, T_cam, T_sphere, time_stamp)
    cb_cntr = cb_cntr + 1


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
        pose_sub = message_filters.Subscriber(
            '/ambf/env/' + name + '/State', RigidBodyState)
        subscribers += [pose_sub]

    print("Synchronous? : ", args.sync)
    if args.sync is False:
        ats = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=100, slop=0.01)
        ats.registerCallback(callback, container.keys())
    else:
        ats = message_filters.TimeSynchronizer(subscribers, queue_size=50)
        ats.registerCallback(callback, container.keys())

    rospy.spin()


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        '--camera_adf', default='../ADF/segmentation_camera.yaml', type=str)
    parser.add_argument('--camera_name', default=None, type=str)
    parser.add_argument('--stereo', action='store_true')

    parser.add_argument('--scale', type=float, default=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis')
    parser.add_argument('--chunk_size', type=int, default=500,
                        help='Write to disk every chunk size')

    parser.add_argument(
        '--stereoL_topic', default='/ambf/env/cameras/stereoL/ImageData', type=str)
    parser.add_argument(
        '--depth_topic', default='/ambf/env/cameras/segmentation_camera/DepthData', type=str)
    parser.add_argument(
        '--stereoR_topic', default='/ambf/env/cameras/stereoR/ImageData', type=str)
    parser.add_argument(
        '--segm_topic', default='/ambf/env/cameras/segmentation_camera/ImageData', type=str)
    parser.add_argument(
        '--sync', type=str, default='True')

    # TODO: record voxels (either what has been removed, or what is the current voxels) as asked by pete?

    args = parser.parse_args()
    print(args)

    bridge = CvBridge()
    _client, objects = init_ambf('data_record')
    _client.clean_up()
    chunk = args.chunk_size
    scale = args.scale
    if args.sync in ['True', 'true', '1']:
        args.sync = True
    else:
        args.sync = False

    # camera extrinsics, the transformation that pre-multiplies recorded poses to match opencv convention
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                         [-1, 0, 0, 0], [0, 0, 0, 1]])  # T_cv_ambf

    h, w = init_camera_params(args.camera_adf, args.camera_name)
    last_depth = 0.0
    cb_cntr = 0
    data_queue = Queue(chunk)
    data_id = 0
    container = OrderedDict()

    main(args)
