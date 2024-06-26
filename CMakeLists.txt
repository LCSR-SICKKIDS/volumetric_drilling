#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2021, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev:  $

cmake_minimum_required (VERSION 3.1)
project (volumetric_plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

add_subdirectory(volumetric_drilling_msgs)
find_package(catkin COMPONENTS volumetric_drilling_msgs)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${AMBF_LIBRARY_DIRS})

add_definitions(${AMBF_DEFINITIONS})


message("---> LIBS " ${catkin_INCLUDE_DIRS})
message("---> FOUND ? " ${catkin_vdrilling_msgs_FOUND})

set (vol_plugin_prefix "plugin/volumetric_drilling")

add_library(volumetric_drilling SHARED
    ${vol_plugin_prefix}/common.h
    ${vol_plugin_prefix}/volumetric_drilling.cpp
    ${vol_plugin_prefix}/volumetric_drilling.h
    ${vol_plugin_prefix}/ros_interface.cpp
    ${vol_plugin_prefix}/ros_interface.h
    ${vol_plugin_prefix}/joystick.cpp
    ${vol_plugin_prefix}/joystick.h
    ${vol_plugin_prefix}/footpedal.cpp
    ${vol_plugin_prefix}/footpedal.h
    ${vol_plugin_prefix}/wave_generator.cpp
    ${vol_plugin_prefix}/wave_generator.h
    ${vol_plugin_prefix}/drill_manager.cpp
    ${vol_plugin_prefix}/drill_manager.h
    ${vol_plugin_prefix}/camera_panel_manager.h
    ${vol_plugin_prefix}/camera_panel_manager.cpp
    ${vol_plugin_prefix}/gaze_marker_controller.h
  ${vol_plugin_prefix}/gaze_marker_controller.cpp)
add_dependencies(volumetric_drilling ${catkin_EXPORTED_TARGETS})
target_link_libraries (volumetric_drilling ${Boost_LIBRARIES} ${AMBF_LIBRARIES} ${catkin_LIBRARIES})
set_property(TARGET volumetric_drilling PROPERTY POSITION_INDEPENDENT_CODE TRUE)

add_library(camera_hmd plugin/camera_hmd/hmd.cpp plugin/camera_hmd/hmd.h)
target_link_libraries(camera_hmd ${AMBF_LIBRARIES})
set_property(TARGET camera_hmd PROPERTY POSITION_INDEPENDENT_CODE TRUE)
