# This file can be used to sync camera parameters for cameras being used. When calling the script from command line
# first specify the camera AMBF description file (ADF) you want to change with --camera_adf. Then specify if the ADF
# is describing stereo cameras with --stereo flag. If the ADF is not describing stereo cameras, specify the camera name
# of the camera you wish to change with --camera_name. Now that the script knows which cameras to edit, you can change
# any of the camera parameters by calling its flag and providing an argument (i.e. --fov 0.52). Any parameters that are
# not called will stay untouched. A full list of parameters that can be changed are found by using the --help argument.
# Note that all camera parameters can be reset to default by calling the --reset argument.

# The following is an example of using the script in the command line:
#  python3 sync_cam_params.py --camera_adf ../ADF/stereo_cameras.yaml --stereo --location [0.5,0.5,1.0] --fov 0.85

import math
from argparse import ArgumentParser, Namespace

import ruamel.yaml
yaml = ruamel.yaml.YAML()

SENSOR_SIZE = [0.036, 0.024]  # meters

def sync_mono_param(params, args, name, stereo_r=False):
    if args.reset:
        args = Namespace(baseline=0.065, clipping_plane=[0.005, 10.0], focal=0.25, fov=None, image_int=100,
                         location=[0.0, 0.0, 0.0], look_at= [-1.0, 0.0, 0.0], res=[640, 480], scale=0.049664,
                         up=[0.0, 0.0, 1.0])
        save_scale(args)
    camera = params[name]

    if isinstance(args.location, str):
        args.location = str_to_list(args.location)
    camera['location']['x'] = args.location[0] / args.scale
    camera['location']['y'] = args.location[1] / args.scale
    if stereo_r:
        camera['location']['y'] = (args.location[1] + args.baseline) / args.scale
    camera['location']['z'] = args.location[2] / args.scale

    # compute look at location based on look at vector and location
    if isinstance(args.look_at, str):
        args.look_at = str_to_list(args.look_at)
    camera['look at']['x'] = (args.look_at[0] + args.location[0]) / args.scale
    camera['look at']['y'] = (args.look_at[1] + args.location[1]) / args.scale
    if stereo_r:
        camera['look at']['y'] = (args.look_at[1] + args.baseline + args.location[1]) / args.scale
    camera['look at']['z'] = (args.look_at[2] + args.location[2]) / args.scale

    if isinstance(args.up, str):
        args.up = str_to_list(args.up)
    camera['up']['x'] = args.up[0]
    camera['up']['y'] = args.up[1]
    camera['up']['z'] = args.up[2]

    if isinstance(args.clipping_plane, str):
        args.clipping_plane = str_to_list(args.clipping_plane)
    camera['clipping plane']['near'] = args.clipping_plane[0] / args.scale
    camera['clipping plane']['far'] = args.clipping_plane[1] / args.scale

    if args.focal is None:
        camera['field view angle'] = args.fov
    else:
        focal = args.focal / args.scale
        sensor_height = SENSOR_SIZE[1] / args.scale
        # convert focal length in meters to fov
        fov = 2 * math.atan(sensor_height / 2 / focal)
        camera['field view angle'] = fov

    if isinstance(args.res, str):
        args.res = str_to_list(args.res)
    camera['publish image resolution']['width'] = args.res[0]
    camera['publish image resolution']['height'] = args.res[1]
    camera['publish image interval'] = args.image_int

    return params

def fill_args(file, args, name, stereo=False):
    if stereo: name = 'stereoL'
    with open(file, 'r') as f:
        params = yaml.load(f)
    save_scale(args)

    if args.baseline is None and stereo:
        args.baseline = params['stereoR']['location']['y'] - params['stereoL']['location']['y']
        args.baseline = args.baseline * args.scale
    if args.fov is None:
        args.fov = params[name]['field view angle']
    if args.location is None:
        args.location = list(params[name]['location'].values())
        args.location = [x * args.scale for x in args.location]
    if args.up is None:
        args.up = list(params[name]['up'].values())
    if args.look_at is None:
        args.look_at = list(params[name]['look at'].values())
        args.look_at = [x * args.scale for x in args.look_at]
    if args.clipping_plane is None:
        args.clipping_plane = list(params[name]['clipping plane'].values())
        args.clipping_plane = [x * args.scale for x in args.clipping_plane]
    if args.res is None:
        args.res = list(params[name]['publish image resolution'].values())
    if args.image_int is None:
        args.image_int = params[name]['publish image interval']

def str_to_list(strings):
    numbers = [float(item) for item in strings.strip(" []").split(',')]
    return numbers

def save_scale(args):
    world = "../ADF/world/world.yaml"  # this file is where scale is being stored
    with open(world, 'r') as FILE:
        world_yaml = yaml.load(FILE)
    if args.scale is None:
        args.scale = world_yaml['conversion factor']
    else:
        with open(world, 'w') as FILE:
            world_yaml['conversion factor'] = args.scale
            yaml.dump(world_yaml, FILE)


def sync(file, args, stereo=False, name=None):
    yaml.boolean_representation = [u'false', u'true']
    with open(file, 'r') as f:
        params = yaml.load(f)
        if stereo:
            params = sync_mono_param(params, args, 'stereoL')
            params = sync_mono_param(params, args, 'stereoR', stereo)
        else:
            params = sync_mono_param(params, args, name)

    with open(file, 'w') as f:
        yaml.dump(params, f)
    f.close()

    return


def main(args):
    if args.camera_name is None and not args.stereo:
        raise Exception('Specify camera name')
    if args.fov is not None and args.focal is not None:
        raise Exception('Specify either focal length or vertical field of view angle, not both')
    else:
        fill_args(args.camera_adf, args, stereo=args.stereo, name=args.camera_name)
        sync(args.camera_adf, args, stereo=args.stereo, name=args.camera_name)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera_adf', type=str, required=True, default=None)
    parser.add_argument('--stereo', action='store_true')
    parser.add_argument('--camera_name', type=str, default=None, help='name of camera')
    parser.add_argument('--reset', action='store_true', help='reset cameras to default parameters')

    parser.add_argument('--baseline', type=float, nargs='?', const=0.065, help='meters')
    parser.add_argument('--scale', type=float, nargs='?', const=0.049664,
                        help='Scale factor is the dimension of the volume in 1 axis')
    parser.add_argument('--fov', type=float, nargs='?', const=None, help='vertical field of view in radians')
    parser.add_argument('--focal', type=float, nargs='?', const=0.25, help='focal length in meters (fx)')
    parser.add_argument('--location', nargs='?', const=[0.0, 0.0, 0.0], help='camera center location (meters)')
    parser.add_argument('--up', nargs='?', const=[0.0, 0.0, 1.0],
                        help='vertical up direction of camera (unit vector), negative y of pixel coordinate')
    parser.add_argument('--look_at', nargs='?', const=[-1.0, 0.0, 0.0],
                        help='outwards direction of camera (unit vector), positive z of pixel coordinate')
    parser.add_argument('--clipping_plane', nargs='?', const=[0.005, 10.0], help='[near, far], meters, for accuracy\n'
                                                                                  'set far plane as far as possible')
    parser.add_argument('--res', nargs='?', const=[640, 480], help='image [width, height]')
    parser.add_argument('--image_int', nargs='?', type=int, const=100, help='Publish every nth scene update')

    args = parser.parse_args()
    main(args)
