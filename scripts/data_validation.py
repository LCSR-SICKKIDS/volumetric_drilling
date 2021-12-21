import h5py
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def verify_xyz(depth, K):
    h, w = depth.shape[1:3]
    u, v = np.meshgrid(np.arange(0, w), np.arange(0, h))
    uv = np.stack([u, v], axis=-1)
    uv_one = np.concatenate([uv, np.ones([h, w, 1])], axis=-1)[None]  # 1xHxWx3
    uvz = uv_one * depth[..., -1][..., None]  # NxHxWx3
    K_inv = np.linalg.inv(K)  # 3x3
    xyz = np.einsum('ab,nhwb->nhwa', K_inv, uvz)

    assert np.all(np.isclose(xyz, depth, rtol=0.01, atol=1e-3)), "Analytical result doesn't match emperical result"


def pose_to_matrix(pose):
    quat_norm = np.linalg.norm(pose[:, 3:], axis=-1)
    assert np.all(np.isclose(quat_norm, 1.0))
    r = R.from_quat(pose[:, 3:]).as_matrix()
    t = pose[:, :3]
    tau = np.identity(4)[None].repeat(pose.shape[0], axis=0)
    tau[:, :3, :3] = r
    tau[:, :3, -1] = t

    return tau

def toStr(f):
    str
    return f"{f:.3f}"

def verify_sphere(depth, K, RT, pose_cam, pose_primitive):
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

    depth_output = np.array([depth[idx, v_, u_] for idx, (v_, u_) in enumerate(zip(v, u))])
    depth_output = depth_output[valid]
    for i in range(len(depth_output)):
        z_act = z[i]
        z_mea = depth_output[i]
        err = z_act - z_mea
        error_str = "Analytical z: " + toStr(z_act) + " Captured z: " + toStr(z_mea) + " Difference: "
        if abs(err) < 0.01:
            error_str = error_str + " " + bcolors.OKGREEN + \
            toStr(err) + bcolors.ENDC
        else:
            error_str = error_str + " " + bcolors.FAIL + \
            toStr(err) + bcolors.ENDC
        print(error_str)
    # print(depth_output)
    assert np.all(np.isclose(z, depth_output, rtol=0.01)), "fail, largest error %f" % (np.max(np.abs(z - depth_output)))

    print("pass")

    return


def verify_cube(depth, K, RT, poses):
    # TODO
    return


def verify_cylinder(depth, K, RT, poses):
    # TODO
    return


f = h5py.File('/home/max/git_ws/volumetric_drilling/scripts/data/test.hdf5', 'r')
intrinsic = f['metadata']['camera_intrinsic'][()]
extrinsic = f['metadata']['camera_extrinsic'][()]

depth = f['data']['depth'][()]
# verify_xyz(depth, intrinsic)
plt.imshow(depth[0].astype(np.float32))
plt.show()

img = f['data']['l_img'][()]
# plt.imshow(img[0])
# plt.show()

pose_cam = pose_to_matrix(f['data']['pose_main_camera'][()])
pose_sphere = pose_to_matrix(f['data']['pose_Sphere'][()])

verify_sphere(depth, intrinsic, extrinsic, pose_cam, pose_sphere)
# print(intrinsics)
# print(depth.shape)
#
# plt.imshow(depth[0, ..., -1].astype(np.float32))
# plt.show()
