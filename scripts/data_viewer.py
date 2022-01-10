from argparse import ArgumentParser

import h5py
import matplotlib.pyplot as plt


def view_data():
    for i in range(l_img.shape[0]):
        plt.subplot(221)
        plt.imshow(l_img[i])
        plt.subplot(222)
        plt.imshow(r_img[i])
        plt.subplot(223)
        plt.imshow(depth[i])
        plt.subplot(224)
        plt.imshow(segm[i])

        plt.show()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--file', type=str, default=None)
    args = parser.parse_args()

    if args.file is not None:
        file = h5py.File(args.file, 'r')
        l_img = file["data"]["l_img"][()]
        r_img = file["data"]["r_img"][()]
        depth = file["data"]["depth"][()]
        segm = file["data"]["segm"][()]

        view_data()
