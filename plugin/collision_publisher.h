//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <nnaguru1@jh.edu>
    \author    Nimesh Nagururu
*/
//==============================================================================

#ifndef COLLISION_PUBLISHER_H
#define COLLISION_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <vdrilling_msgs/points.h>
#include <vdrilling_msgs/UInt8Stamped.h>
#include <vdrilling_msgs/VolumeProp.h>


class DrillingPublisher{
public:
    DrillingPublisher(std::string a_namespace, std::string a_plugin);
    ~DrillingPublisher();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;

    void voxelsRemoved(double ray[3], float vcolor[4], double time);
    void burrChange(int burrSize, double time);
    void volumeProp(float dimensions[3], int voxelCount[3]);
private:
    ros::Publisher m_voxelsRemovedPub;
    ros::Publisher m_burrChangePub;
    ros::Publisher m_volumePropPub;
    vdrilling_msgs::points voxel_msg;
    vdrilling_msgs::UInt8Stamped burr_msg;
    vdrilling_msgs::VolumeProp volume_msg;

};

#endif //VOLUMETRIC_PLUGIN_COLLISION_PUBLISHER_H
