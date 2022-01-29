from argparse import ArgumentParser

import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R

from utils import *


def quaternion_multiply(q1, q2):
    """
    rotation multiplication as quaternion
    """
    x1, y1, z1, w1 = np.split(q1, 4, axis=-1)
    x2, y2, z2, w2 = np.split(q2, 4, axis=-1)
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.concatenate([x, y, z, w], axis=-1)


def verify_xyz(depth, K):
    h, w = depth.shape[1:3]
    u, v = np.meshgrid(np.arange(0, w), np.arange(0, h))
    uv = np.stack([u, v], axis=-1)
    uv_one = np.concatenate([uv, np.ones([h, w, 1])], axis=-1)[None]  # 1xHxWx3
    uvz = uv_one * depth[..., -1][..., None]  # NxHxWx3
    K_inv = np.linalg.inv(K)  # 3x3
    xyz = np.einsum('ab,nhwb->nhwa', K_inv, uvz)

    assert np.all(np.isclose(xyz, depth, rtol=0.01, atol=1e-3)
                  ), "Analytical result doesn't match emperical result"


def pose_to_matrix(pose):
    quat_norm = np.linalg.norm(pose[:, 3:], axis=-1)
    assert np.all(np.isclose(quat_norm, 1.0))
    r = R.from_quat(pose[:, 3:]).as_matrix()
    t = pose[:, :3]
    tau = np.identity(4)[None].repeat(pose.shape[0], axis=0)
    tau[:, :3, :3] = r
    tau[:, :3, -1] = t

    return tau


def verify_sphere(depth, K, RT, pose_cam, pose_primitive, time_stamps):
    # simple test, querying a point on the sphere
    query_point = np.array([1, 0, 0, 1])[None, :, None]  # homo, Nx4x1
    query_point_c = np.linalg.inv(pose_cam) @ (pose_primitive @ query_point)
    query_point_c = RT @ query_point_c
    uvz = K @ query_point_c[..., :3, :]
    u = np.rint((uvz[..., 0, :] / uvz[..., -1, :]).squeeze()).astype(int)
    v = np.rint((uvz[..., 1, :] / uvz[..., -1, :]).squeeze()).astype(int)
    z = uvz[..., -1, :].squeeze()

    h, w = depth.shape[1:3]

    # make sure point is within image
    valid_u = np.logical_and(0 <= u, u < w)
    valid_v = np.logical_and(0 <= v, v < h)
    valid = np.logical_and(valid_u, valid_v)
    depth_output = np.array([depth[idx, v_, u_]
                             for idx, (v_, u_) in enumerate(zip(v, u))])
    depth_output = depth_output[valid]
    time_stamps = time_stamps[valid]
    pose_cam = pose_cam[valid]
    for i in range(len(depth_output)):
        z_act = z[i]
        z_mea = depth_output[i]
        ts = time_stamps[i]
        err = z_act - z_mea
        time_str = INFO_STR("t: " + "{:10.6f}".format(ts))
        cam_xyz = pose_cam[i][0:3, 3]
        lag_lead_str = ""
        cam_pose_str = "Cam Z: " + toStr(cam_xyz[0])
        error_str = "Anal_z: " + \
                    toStr(z_act) + " Depth_z: " + toStr(z_mea) + " Diff: "
        if abs(err) < 0.01:
            error_str = error_str + " " + OK_STR(err)
        else:
            error_str = error_str + " " + FAIL_STR(err)
            if i > 0:
                if z_mea == depth_output[i - 1]:
                    lag_lead_str = WARN_STR("LAG")
                else:
                    lag_lead_str = WARN2_STR("LEAD")
        print(time_str, cam_pose_str, error_str, lag_lead_str)
    # print(depth_output)
    assert np.all(np.isclose(z, depth_output, rtol=0.01)
                  ), "fail, largest error %f" % (np.max(np.abs(z - depth_output)))

    print("pass")

    return


def pose_depth_test(K, pose, depth, segm, u, v, target_color):
    for i in range(depth.shape[0] - 1):
        # iproj
        d = depth[i, v, u]
        X0 = np.linalg.inv(K) @ np.array([u, v, 1]) * d  # in camera coordinate

        # transform
        X0_one = np.concatenate([X0, np.ones([1])], axis=-1)[..., None]
        X1_one = np.linalg.inv(pose[i + 1]) @ pose[i] @ X0_one  # T_i+1,wc^-1 @ T_i,wc
        X1 = X1_one[:3, 0]
        uvz = K @ X1
        uv = uvz[:2] / uvz[2]

        # update uv
        v_new = np.rint(uv[1]).astype(int)
        u_new = np.rint(uv[0]).astype(int)

        if not np.all(pose[i + 1] == pose[i]):
            # check if within image
            if v_new < 0 or v_new >= 480:
                print("out of window", i)
                break
            if u_new < 0 or u_new >= 640:
                print("out of window", i)
                break
            if not np.all(segm[i + 1, v_new, u_new] == target_color, axis=-1):
                if np.all(segm[i + 1, v_new + 1, u_new] == target_color, axis=-1):
                    v_new += 1
                elif np.all(segm[i + 1, v_new, u_new + 1] == target_color, axis=-1):
                    u_new += 1
                elif np.all(segm[i + 1, v_new - 1, u_new] == target_color, axis=-1):
                    v_new -= 1
                elif np.all(segm[i + 1, v_new, u_new - 1] == target_color, axis=-1):
                    u_new -= 1
                print("can't find target class any more")
                break

            d_new = depth[i + 1, v_new, u_new]
            assert np.isclose(uvz[2], d_new, rtol=0.01)

            u = u_new
            v = v_new


def verify_drilling(K, pose_cam, pose_drill, segm, depth):
    # tool
    tool = np.all(segm[0] == np.array([33, 32, 34]), axis=-1)
    y, x = np.where(tool)
    while True:
        u = np.random.randint(np.min(x), np.max(x))
        v = np.random.randint(np.min(y), np.max(y))
        if tool[v, u]:
            break
    poses = np.linalg.inv(pose_drill) @ pose_cam
    pose_depth_test(K, poses, depth, segm, u, v, target_color=np.array([33, 32, 34]))

    if args.no_drilling:
        # mastoid (only valid when there is no drilling)
        mastoid = np.all(segm[0] == np.array([219, 249, 255]), axis=-1)
        y, x = np.where(mastoid)
        while True:
            u = np.random.randint(np.min(x), np.max(x))
            v = np.random.randint(np.min(y), np.max(y))
            if mastoid[v, u]:
                break
        pose_depth_test(K, pose_cam, depth, segm, u, v, target_color=np.array([219, 249, 255]))
    print("All test passed :)")
    return


def verify_cube(depth, K, RT, poses):
    # TODO
    return


def verify_cylinder(depth, K, RT, poses):
    # TODO
    return


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--setting', choices=['sphere', 'drilling'], type=str, default='drilling')
    parser.add_argument('--file', type=str, default=None)
    parser.add_argument('--no_drilling', action='store_true')

    np.set_printoptions(suppress=True, formatter={'float_kind': '{:f}'.format})
    np.random.seed(32)

    args = parser.parse_args()
    if args.file is not None:
        f = h5py.File(args.file, 'r')
        intrinsic = f['metadata']['camera_intrinsic'][()]
        extrinsic = f['metadata']['camera_extrinsic'][()]
        extrinsic_quat_inv = R.from_matrix(np.linalg.inv(extrinsic)[:3, :3]).as_quat()

        depth = f['data']['depth'][()]
        time_stamps = f['data']['time'][()]

        pose_cam = f['data']['pose_main_camera'][()]
        # apply extrinsic inv to camera poses
        pose_cam[:, 3:] = quaternion_multiply(pose_cam[:, 3:], extrinsic_quat_inv)
        pose_cam = pose_to_matrix(pose_cam)

        if args.setting == 'sphere':
            pose_sphere = pose_to_matrix(f['data']['pose_Sphere'][()])
            verify_sphere(depth, intrinsic, extrinsic, pose_cam, pose_sphere, time_stamps)
        elif args.setting == 'drilling':
            segm = f['data']['segm'][()]
            pose_drill = pose_to_matrix(f['data']['pose_mastoidectomy_drill'][()])
            verify_drilling(intrinsic, pose_cam, pose_drill, segm, depth)
