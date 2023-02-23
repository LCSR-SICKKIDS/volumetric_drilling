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
    \author    Adnan Munawar
*/
//==============================================================================
#ifndef COLLISION_PUBLISHER_H
#define COLLISION_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <volumetric_drilling_msgs/Voxels.h>
#include <volumetric_drilling_msgs/DrillSize.h>
#include <volumetric_drilling_msgs/VolumeInfo.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/ColorRGBA.h>

#include <afFramework.h>

using namespace chai3d;

class DrillingPublisher{
public:
    DrillingPublisher(std::string a_namespace, std::string a_plugin);
    ~DrillingPublisher();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    void publishDrillSize(int burrSize, double time);

    void setVolumeInfo(cTransform& pose, cVector3d& dimensions, cVector3d& voxel_count);

    void publishVolumeInfo(double time);

    void appendToVoxelMsg(cVector3d& index, cColorf& color);

    void clearVoxelMsg();

    void publishVoxelMsg(double time);

    void publishForceFeedback(cVector3d& force, cVector3d& moment, double time);

private:
    ros::Publisher m_voxelsRemovalPub;
    ros::Publisher m_drillSizePub;
    ros::Publisher m_volumeInfoPub;
    ros::Publisher m_forcefeedbackPub;

    volumetric_drilling_msgs::Voxels m_voxel_msg;
    volumetric_drilling_msgs::DrillSize m_drill_size_msg;
    volumetric_drilling_msgs::VolumeInfo m_volume_info_msg;
    geometry_msgs::WrenchStamped m_force_feedback_msg;

};

#endif //VOLUMETRIC_PLUGIN_COLLISION_PUBLISHER_H
