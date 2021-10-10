# volumetric_drilling
This repo provides a realistic simulator for skull-base surgery with VR and haptics integration as well as the ability to generate data for use in downstream algorithm development. Volumetric drilling is a plugin is built on top of Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al. While the application shown in this repo is skull-base surgery, the framework of this plugin can be adapted to other surgical procedures.

![image](https://user-images.githubusercontent.com/61888209/136677737-af8e1a6c-1f76-44d7-bb3c-6a9d99ec08fd.png)

# Installation Instructions:
Lets call the absolute location of this package as **<volumetric_plugin_path>**. E.g. if you cloned this repo in your home folder, **<volumetric_plugin_path>** = `~/volumetric_drilling/` OR `/home/<username>/volumetric_plugin`
## 1. Install and Source AMBF 2.0

Clone and build `ambf-2.0` branch.
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Note that depth and image recording are enabled by default (in camera ADFs) and these features only work on Linux with ROS installed. Additionally, the following packages must be installed prior to building to AMBF:

```bash
cv-bridge # Can be installed via apt install ros-<version>-cv-bridge
image-transport # Can be installed via apt install ros-<version>-image-transport
```

Build and source ambf as per the instructions on [AMBFs wiki on installation](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF).

## 2. Clone and build this plugin
``` bash
git clone https://github.com/LCSR-SICKKIDS/volumetric_drilling
cd <volumetric_plugin_path>
mkdir build
cd build
cmake ..
make
```
If everything went smoothly, we are good to go.

# Running the Plugin with ambf_simulator:
The volumetric drilling simulator is a plugin that is launched on top of the AMBF simulator along with other AMBF multibodies as will be described below. 
Note that the `libvolumetric_drilling.so` plugin is described in the `launch.yaml` file and can be commented out for the purpose of debugging the ADF (AMBF Description Format) files. Below are instructions as to how to load different volume and camera options. The -l tag used below allows user to run indexed multibodies that can also be found in the `launch.yaml` under the `multibody configs:` data block. More info on launching the simulator can be found in the [AMBF Wiki](https://github.com/WPI-AIM/ambf/wiki/Launching-the-Simulator).

## Different Volume Options:
We provide three different volumes to choose from:

#### Option 1:
A low res volume (`1`) and a drill (`0`):
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1
```

#### Option 2:
A medium res volume (`2`) and a drill (`0`):
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,2
```

#### Option 3:
A high res volume (`3`) and a drill (`0`):
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,3
```
#### User-provided volume
Patient specific anatomy may also be used in the simulator. The volumes are an array of images (JPG or PNG) that are rendered via texture-based volume rendering. With segmented images and an ADF for the volume, user specified anatomy can easily be used in the simulator. We provide utility scripts that can convert the data from the NRRD format to an array of images.

## Camera Options:
Different cameras, defined via ADF model files, can be loaded alongside the simulation.

#### Option 1:
You can add `4` to any of the above commands to load a segmentation_camera. This camera publishes a segmented depth point cloud. Launch example:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1,4
```
#### Option 2:
You can add `5` to any of the above commands to load two cameras (one of each stereo eye). Each of these cameras publishes a its video. Launch example:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1,5
```

#### Option 1 and 2 combined:
You can also load both the segmentation_camera and the two stereo_cameras together as:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1,4,5
```
## Changing Scene Parameters
All the relevant ADF scene objects are in the ADF folder and can be modified as needed. For example, camera intrinsics can be adjusted via the field view angle and image resolution parameters of Camera ADFs.

## Recording Data
A python script is provided to generate left and right stereo images, depth point cloud, segmentation mask, and object/camera pose. Data is recorded in a convenient and well-organized hdf5 file. 
