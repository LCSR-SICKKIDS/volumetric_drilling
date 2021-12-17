# This file can be used to sync camera parameters for cameras being used. Desired camera parameters should be
# placed in the segmenation_camera.yaml ADF and the desired baseline should be edited in this camera_sync file on
# line 8. When the program is run, stereo_cameras.yaml and single_stereo_camera.yaml will be edited to reflect 
# the same parameters used in segmentation_camera.yaml

# TODO: update the above description

import math
from argparse import ArgumentParser

import ruamel.yaml

SENSOR_SIZE = [0.036, 0.024]  # meters


def sync_mono_param(params, args, name, stereo_r=False):
    camera = params[name]

    camera['location']['x'] = args.location[0] / args.scale
    camera['location']['y'] = args.location[1] / args.scale
    if stereo_r:
        camera['location']['y'] = (args.location[1] + args.baseline) / args.scale
    camera['location']['z'] = args.location[2] / args.scale

    # compute look at location based on look at vector and location
    camera['look at']['x'] = args.look_at[0] + args.location[0]
    camera['look at']['y'] = args.look_at[1] + args.location[1]
    if stereo_r:
        camera['look at']['y'] = args.look_at[1] + args.baseline + args.location[1]
    camera['look at']['z'] = args.look_at[2] + + args.location[2]

    camera['up']['x'] = args.up[0]
    camera['up']['y'] = args.up[1]
    camera['up']['z'] = args.up[2]

    camera['clipping plane']['near'] = args.clipping_plane[0] / args.scale
    camera['clipping plane']['far'] = args.clipping_plane[1] / args.scale

    if args.fov is not None:
        camera['field view angle'] = args.fov
    else:
        focal = args.focal / args.scale
        # convert focal length in meters to fov
        fov = 2 * math.atan(SENSOR_SIZE[1] / 2 / focal)
        camera['field view angle'] = fov

    # TODO: this is not available in all cameras, why?
    # camera['publish image resolution'] = dict(width=args.res[0], height=args.res[1])

    return params


def sync(file, args, stereo=False, name=None):
    yaml = ruamel.yaml.YAML()
    yaml.boolean_representation = ['False',
                                   'True']  # TODO: there is mixd upper case and lower case, which one is right?
    # TODO: there is mixed path with quotation and without, which one is right?
    #  for example,
    #  without quotation: path: ../../../ambf_shaders/depth,
    #  with quotation: vertex: "shader.vs"

    # TODO: in drilling.yaml, the dictionary are formatted without brackets.
    #  but in cameras, they are not. which one is right?
    #  for example:
    #  in camera: location: {x: 10.0, y: 0.0, z: 0.0}
    #  in drill:
    #       location:
    #           position:
    #               x: 0.01
    #               y: 0.01
    #               z: 0.01

    with open(file, 'r') as f:
        params = yaml.load(f)
        if stereo:
            params = sync_mono_param(params, args, 'stereoL')
            params = sync_mono_param(params, args, 'stereoR', stereo)
        else:
            params = sync_mono_param(params, args, name)

    with open(file, 'w') as f:
        yaml.dump(params, f)

    return


def main(args):
    if args.camera_name is None and not args.stereo:
        raise Exception('Specify camera name')
    else:
        sync(args.camera_adf, args, stereo=args.stereo, name=args.camera_name)


if __name__ == '__main__':
    # TODO: once all above todos are checked off, verify the generated yaml files can be launched
    parser = ArgumentParser()
    parser.add_argument('--baseline', type=float, default=0.065, help='meters')
    parser.add_argument('--scale', type=float, default=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis')
    parser.add_argument('--fov', type=float, default=None, help='vertical field of view in radians')
    parser.add_argument('--focal', type=float, default=0.25, help='focal length in meters (fx)')
    parser.add_argument('--location', nargs='+', default=[0.0, 0.0, 0.0], help='camera center location (meters)')
    parser.add_argument('--up', nargs='+', default=[0.0, 0.0, 1.0],
                        help='vertical up direction of camera (unit vector), negative y of pixel coordinate')
    parser.add_argument('--look_at', nargs='+', default=[-1.0, 0.0, 0.0],
                        help='outwards direction of camera (unit vector), positive z of pixel coordinate')
    parser.add_argument('--clipping_plane', nargs='+', default=[0.10, 10.0], help='[near, far], meters, for accuracy\n'
                                                                                  'set far plane as far as possible')

    parser.add_argument('--res', nargs='+', default=[640, 480], help='[width, height]')
    # TODO: remove parenting to "main_camera" as it is not needed (redundant).
    #  Currently main camera is initialized to be identity at origin, which makes it pointless
    # TODO: double check all units in meters are properly scaled by the scale value
    # TODO: save scale parameter in somewhere so we can read (either NRRD or ADF)??

    parser.add_argument('--camera_adf', type=str, default=None)
    parser.add_argument('--stereo', action='store_true')
    parser.add_argument('--camera_name', type=str, default=None, help='name of  cameras')

    args = parser.parse_args()
    main(args)
