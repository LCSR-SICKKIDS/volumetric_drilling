#!/bin/bash -i
source ~/.bashrc
source ../../build/devel/setup.bash
gnome-terminal --tab -e "roscore" --tab -e "python3 study_gui.py"
