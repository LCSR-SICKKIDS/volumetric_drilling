# volumetric_drilling

Lets call the absolute location of this package as **<volumetric_plugin_path>**. E.g. if you cloned this repo in your home folder, **<volumetric_plugin_path>** = `~/volumetric_drilling/` OR `/home/<username>/volumetric_plugin`

# Instructions:
## 1. Install and Source AMBF 2.0

Clone and build `ambf-2.0` branch.
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Build and source ambf as per the instructions on AMBFs Github page.

## 2. Clone and build this plugin
``` bash
git clone https://github.com/adnanmunawar/volumetric_drilling.git
cd <volumetric_plugin_path>
mkdir build
cd build
cmake ..
make
```
If everything went smoothly, we are good to go.

<strike>
## 3. Add this plugin to ambf plugins path
Only needed once per terminal. In the terminal that you run `ambf_simulator` in, type the following command:

``` bash
export AMBF_PLUGIN_PATH=<volumetric_plugin_path>/build/
```
If you want to run in a different terminal, make sure to set the plugin path again in that terminal. You can alternatively add the above command to you `.bashrc` file.
 </strike>

# Launching the plugin with ambf_simulator:
The `libvolumetric_drilling.so` plugin is described in the `launch.yaml` file. It can be
commented out for the purpose of debugging the ADF models.


# Different Volume Options:
There are three different volumes to choose from:

#### Option 1:
A low res volume and a drill:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1
```

#### Option 2:
A medium res volume and a drill:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,2
```

#### Option 3:
A high res volume and a drill:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,3
```

# Camera Options:
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
You can also load both the segmenration_camera and the two stereo_cameras together as:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <volumetric_plugin_path>/launch.yaml -l 0,1,4,5
```
# Changing Scene Parameters
All the relevant ADF scene objects are in the ADF folder
