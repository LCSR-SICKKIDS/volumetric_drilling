from argparse import ArgumentParser

import cv2
import h5py
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from data_validation import pose_to_matrix


def view_data():
    if args.idx is None:
        for i in range(l_img.shape[0]):
            plt.subplot(221)
            plt.imshow(l_img[i])
            plt.subplot(222)
            plt.imshow(r_img[i])
            plt.subplot(223)
            plt.imshow(depth[i], vmax=1)
            plt.subplot(224)
            plt.imshow(segm[i])

            plt.show()
    else:
        i = int(args.idx[0])
        j = int(args.idx[1])
        plt.subplot(221)
        plt.imshow(l_img[i])
        plt.subplot(222)
        plt.imshow(l_img[j])
        plt.subplot(223)
        plt.imshow(depth[i], vmax=1)
        plt.subplot(224)
        plt.imshow(depth[j], vmax=1)
        plt.show()

        # print(np.linalg.inv(pose_cam[i]) @ pose_cam[j])
        d_pose_rhs = np.linalg.inv(pose_drill[i]) @ pose_drill[j]
        rot = R.from_matrix(d_pose_rhs[:3, :3]).as_euler('XYZ') / np.pi * 180
        print(rot, d_pose_rhs[:3, 3] * 1000)

        d_pose_lhs = pose_drill[j] @ np.linalg.inv(pose_drill[i])
        rot = R.from_matrix(d_pose_lhs[:3, :3]).as_euler('XYZ') / np.pi * 180
        print(rot, d_pose_lhs[:3, 3] * 1000)

        # j @ i.inv --> large translation
        print(np.linalg.inv(pose_cam[j]) @ pose_drill[j] @ np.linalg.inv(np.linalg.inv(pose_cam[i]) @ pose_drill[i]))
        # j.inv @ i --> small translation
        print(np.linalg.inv(np.linalg.inv(pose_cam[j]) @ pose_drill[j]) @ (np.linalg.inv(pose_cam[i]) @ pose_drill[i]))


def generate_video():
    cmap = plt.get_cmap()
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (640 * 2, 480 * 2))

    for i in tqdm(range(l_img.shape[0])):
        d = (cmap(depth[i])[..., :3] * 255).astype(np.uint8)
        rgb = np.concatenate([l_img[i], r_img[i]], axis=1)
        depth_segm = np.concatenate([d, segm[i]], axis=1)
        frame = np.concatenate([rgb, depth_segm], axis=0)
        out.write(frame)

    out.release()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--file', type=str, default=None, action='Iterate through all data frame by frame')
    parser.add_argument('--idx', nargs='+', default=None, action='View the data of provided index')
    parser.add_argument('--generate_video', action='store_true', help="create a video of recorded data")
    args = parser.parse_args()

    if args.file is not None:
        file = h5py.File(args.file, 'r')
        l_img = file["data"]["l_img"]
        r_img = file["data"]["r_img"]
        depth = file["data"]["depth"]
        segm = file["data"]["segm"]
        K = file['metadata']["camera_intrinsic"]
        extrinsic = file['metadata']['camera_extrinsic']

        pose_cam = pose_to_matrix(file['data']['pose_main_camera'])
        pose_cam = np.matmul(pose_cam, np.linalg.inv(extrinsic)[None])  # update pose so world directly maps to CV
        pose_drill = pose_to_matrix(file['data']['pose_mastoidectomy_drill'])

        if args.generate_video:
            generate_video()
        else:
            view_data()
