# volumetric_drilling

Lets call the absolute location of this package as **<volumetric_plugin_path>**. E.g. if you cloned this repo in your home folder, **<volumetric_plugin_path>** = `~/volumetric_drilling/` OR `/home/<username>/volumetric_plugin`

# Instructions:
## 1. Install and Source AMBF 2.0

Clone and build `ambf-2.0` branch.
```bash
git clone https://github.com/WPI-AIM/ambf.git
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Build and source ambf as per the instructions on AMBFs Github page.

## 2. Clone and build this plugin
``` bash
cd <volumetric_plugin_path>
mkdir build
cd build
cmake ..
make
```
If everything went smoothly, we are good to go.

## 3. Add this plugin to ambf plugins path
Only needed once per terminal. In the terminal that you are going to be running ambf_simulator in, type the following command:

``` bash
export AMBF_PLUGIN_PATH=<volumetric_plugin_path>/build/
```
If you want to run in a different terminal, make sure to set the plugin path again in that terminal. You can alternatively add the above command to you `.bashrc` file.

# Launching the plugin with ambf_simulator:

```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator -l 100 --plugins libvolumetric_drilling.so --images_path <volumetric_plugin_path>/resources/volumes/ear3/ --prefix plane00 --count 500 --shaders_path <volumetric_plugin_path>/resources/shaders/
```

