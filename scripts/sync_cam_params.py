'''
This file can be used to sync camera parameters for cameras being used. When calling the script from command line
specify the camera AMBF description files (ADF) you want to change with --camera_adfs. Now that the
script knows which cameras to edit, you can change any of the camera parameters by calling its flag and providing an
argument (i.e. --fov 0.52). Any parameters that are not called will stay untouched. A full list of parameters that can
be changed are found by using the --help argument.
Note that all camera parameters can be reset to default by calling the --reset argument.

The following is an example of using the script in the command line:
python3 sync_cam_params.py --camera_adf ../ADF/segmentation_camera.yaml ../ADF/stereo_cameras.yaml ../ADF/world/world.yaml --location [0.5,0.5,1.0] --fov 0.85

'''

import math
from argparse import ArgumentParser, Namespace

import ruamel.yaml
yaml = ruamel.yaml.YAML()
yaml.boolean_representation = [u'false', u'true']

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

def fill_args(args):
    file = args.camera_adfs[0]
    with open(file, 'r') as f:
        params = yaml.load(f)

    name = params['cameras'][0]
    save_scale(args)

    if '../ADF/stereo_cameras.yaml' in args.camera_adfs:
        with open('../ADF/stereo_cameras.yaml','r') as f_s:
            params_s = yaml.load(f_s)
        if args.baseline is None:
            args.baseline = params_s['stereoR']['location']['y'] - params_s['stereoL']['location']['y']
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

    return args

def str_to_list(strings):
    numbers = [float(item) for item in strings.strip(" []").split(',')]
    return numbers

def save_scale(args):
    world = "../ADF/world/world.yaml"
    with open(world, 'r') as FILE:
        world_yaml = yaml.load(FILE)
    if args.scale is None:
        args.scale = world_yaml['conversion factor']
    else:
        with open(world, 'w') as FILE:
            world_yaml['conversion factor'] = args.scale
            yaml.dump(world_yaml, FILE)


def sync(args):
    for file in args.camera_adfs:
        with open(file, 'r') as f:
            params = yaml.load(f)
            cameras = params['cameras']
            if len(cameras) > 1:
                params = sync_mono_param(params, args, 'stereoL')
                params = sync_mono_param(params, args, 'stereoR', True)
            else:
                params = sync_mono_param(params, args, cameras[0])
        with open(file, 'w') as f:
            yaml.dump(params, f)
        f.close()

    return


def main(args):
    if args.fov is not None and args.focal is not None:
        raise Exception('Specify either focal length or vertical field of view angle, not both')
    else:
        fill_args(args)
        sync(args)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera_adfs', type=str, nargs='+', default=[], help='relative paths to camera AMBF Description '
                                                                               'Files (ADF) that need to be synchronized')
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
