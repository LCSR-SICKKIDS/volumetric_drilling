'''
This file can be used to sync camera parameters for cameras being used. When calling the script from command line
specify the world/volume/camera AMBF description files (ADF) you want to change. A full list of parameters that can
be changed are found by using the --help argument.

The following is an example of using the script in the command line:
python3 adf_configs.py --camera_adf ../ADF/segmentation_camera.yaml --location 0.5,0.5,1.0 --fov 0.85
'''

import math
from argparse import ArgumentParser

import ruamel.yaml

yaml = ruamel.yaml.YAML()
yaml.boolean_representation = [u'false', u'true']

SENSOR_SIZE = [0.0048, 0.0036]  # meters


def camera_config(params, args, name):
    camera = params[name]
    main_cam = name == 'main_camera'
    stereo_r = name == 'stereoR'

    if not main_cam:
        args.location = [0.0, 0.0, 0.0]  # all cameras are parented to main, thus setting to 0.0 offset
    camera['location']['x'] = args.location[0] / args.scale
    camera['location']['y'] = args.location[1] / args.scale
    if stereo_r:
        camera['location']['y'] = (args.location[1] + args.baseline) / args.scale
    camera['location']['z'] = args.location[2] / args.scale

    # compute look at location based on look at vector and location
    if main_cam:
        # if no look at is specified, set to volume location
        if args.look_at is None:
            args.look_at = args.volume_location
        camera['look at']['x'] = args.look_at[0] / args.scale
        camera['look at']['y'] = args.look_at[1] / args.scale
        camera['look at']['z'] = args.look_at[2] / args.scale
    else:
        args.look_at = [-1.0, 0.0, 0.0]  # all cameras are parented to main, thus setting rotation to identity
        camera['look at']['x'] = args.look_at[0] + args.location[0] / args.scale
        camera['look at']['y'] = args.look_at[1] + args.location[1] / args.scale
        if stereo_r:
            camera['look at']['y'] = args.look_at[1] + (args.baseline + args.location[1]) / args.scale
        camera['look at']['z'] = args.look_at[2] + args.location[2] / args.scale

    if not main_cam:
        args.up = [0.0, 0.0, 1.0]  # all cameras are parented to main, thus setting rotation to identity
    camera['up']['x'] = args.up[0]
    camera['up']['y'] = args.up[1]
    camera['up']['z'] = args.up[2]

    camera['clipping plane']['near'] = args.clipping_plane[0] / args.scale
    camera['clipping plane']['far'] = args.clipping_plane[1] / args.scale

    if args.focal is None:
        camera['field view angle'] = args.fov
    else:
        focal = args.focal
        sensor_height = SENSOR_SIZE[1]
        # convert focal length in meters to fov
        fov = 2 * math.atan(sensor_height / 2 / focal)
        camera['field view angle'] = fov

    camera['publish image resolution']['width'] = args.res[0]
    camera['publish image resolution']['height'] = args.res[1]
    camera['publish image interval'] = args.image_interval
    if 'publish depth interval' in camera:
        camera['publish depth interval'] = args.image_interval

    return params


def save_scale(args):
    world = args.world_adf
    with open(world, 'r') as FILE:
        world_yaml = yaml.load(FILE)
    if args.scale is None:
        args.scale = world_yaml['conversion factor']
    else:
        with open(world, 'w') as FILE:
            world_yaml['conversion factor'] = args.scale
            yaml.dump(world_yaml, FILE)


def sync_cameras(args):
    for file in args.camera_adfs:
        with open(file, 'r') as f:
            params = yaml.load(f)
            cameras = params['cameras']
            if len(cameras) > 1:
                params = camera_config(params, args, 'stereoL')
                params = camera_config(params, args, 'stereoR')
            else:
                params = camera_config(params, args, cameras[0])
        with open(file, 'w') as f:
            yaml.dump(params, f)
        f.close()

    return


def world_config(args):
    '''
    set main camera parameters
    set AMBF to volume conversion unit
    '''
    with open(args.world_adf, 'r') as f:
        params = yaml.load(f)
        f.close()

    params['conversion factor'] = args.scale
    params = camera_config(params, args, 'main_camera')

    with open(args.world_adf, 'w') as f:
        yaml.dump(params, f)
        f.close()


def sync_volumes(args):
    '''
    set volume config
    '''
    for file in args.volume_adfs:
        with open(file, 'r') as f:
            params = yaml.load(f)
            for k, _ in params.items():
                if 'VOLUME' in k:
                    volume_name = k
                    break
            params[volume_name]['location']['position']['x'] = args.volume_location[0] / args.scale
            params[volume_name]['location']['position']['y'] = args.volume_location[1] / args.scale
            params[volume_name]['location']['position']['z'] = args.volume_location[2] / args.scale
            f.close()
        with open(file, 'w') as f:
            yaml.dump(params, f)
            f.close()


def main(args):
    if args.fov is not None and args.focal is not None:
        raise Exception('Specify either focal length or vertical field of view angle, not both')
    else:
        world_config(args)
        sync_volumes(args)
        sync_cameras(args)

        print("Configuration was set successfully!")


if __name__ == '__main__':
    parser = ArgumentParser()

    # path to adfs
    parser.add_argument('--world_adf', type=str, default="../ADF/world/world.yaml",
                        help='world adf is used to set main camera param and scale')
    parser.add_argument('--volume_adfs', type=str, nargs='+',
                        default=["../ADF/volume_171.yaml", "../ADF/volume_256.yaml",
                                 "../ADF/volume_512.yaml"],
                        help='volume adf')
    parser.add_argument('--camera_adfs', type=str, nargs='+',
                        default=["../ADF/stereo_cameras.yaml", "../ADF/segmentation_camera.yaml",
                                 "../ADF/single_stereo_camera.yaml"],
                        help='relative paths to camera AMBF Description '
                             'Files (ADF) that need to be synchronized')

    # world parameters
    parser.add_argument('--scale', type=float, default=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis'
                             'AMBF unit = Meter / scale')

    # volume parameters
    parser.add_argument('--volume_location', nargs='?', default=[-0.3, 0.0, 0.0], help='volume location (meters)')

    # stereo parameters
    parser.add_argument('--baseline', type=float, nargs='?', default=0.025, help='meters')

    # main camera parameters
    parser.add_argument('--fov', type=float, nargs='+', default=None, help='vertical field of view in radians')
    parser.add_argument('--focal', type=float, nargs='+', default=0.02, help='focal length in meters (fx)')
    parser.add_argument('--location', nargs='+', default=[0.0, 0.0, 0.0], help='camera center location (meters)')
    parser.add_argument('--up', nargs='+', default=[0.0, 0.0, 1.0],
                        help='vertical up direction of camera (unit vector), negative y of pixel coordinate')
    parser.add_argument('--look_at', nargs='+', default=None,
                        help='outwards direction of camera (unit vector), positive z of pixel coordinate.\n'
                             'Default set to the location of the volume.')
    parser.add_argument('--clipping_plane', nargs='+', default=[0.1, 50.0], help='[near, far], meters, for accuracy\n'
                                                                                 'set far plane as far as possible')
    parser.add_argument('--res', nargs='+', default=[640, 480], help='image [width, height]')
    parser.add_argument('--image_interval', type=int, default=5, help='Publish every nth scene update')

    args = parser.parse_args()
    main(args)
