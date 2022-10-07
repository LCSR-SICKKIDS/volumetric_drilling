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
#include "collision_publisher.h"
#include <ambf_server/RosComBase.h>

using namespace std;

DrillingPublisher::DrillingPublisher(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

DrillingPublisher::~DrillingPublisher(){
    m_voxelsRemovedPub.shutdown();
    m_burrChangePub.shutdown();
    m_volumePropPub.shutdown();
}

void DrillingPublisher::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();

    m_voxelsRemovedPub = m_rosNode-> advertise<vdrilling_msgs::points>(a_namespace + "/" + a_plugin + "/voxels_removed", 1);
    m_burrChangePub = m_rosNode -> advertise<vdrilling_msgs::UInt8Stamped>(a_namespace + "/" + a_plugin + "/burr_change", 1, true);
    m_volumePropPub = m_rosNode -> advertise<vdrilling_msgs::VolumeProp>(a_namespace + "/" + a_plugin + "/volume_prop", 1, true);
}

void DrillingPublisher::voxelsRemoved(double vray[3], float vcolor[4], double time){
    voxel_msg.header.stamp.fromSec(time);

    std::vector<float> vec(vcolor, vcolor + 4);
    voxel_msg.voxel_color = vec;

    voxel_msg.voxel_removed.x = vray[0];
    voxel_msg.voxel_removed.y = vray[1];
    voxel_msg.voxel_removed.z = vray[2];

    m_voxelsRemovedPub.publish(voxel_msg);
}

void DrillingPublisher::burrChange(int burrSize, double time){
    burr_msg.header.stamp.fromSec(time);
    burr_msg.number.data = burrSize;

    m_burrChangePub.publish(burr_msg);
}

void DrillingPublisher::volumeProp(float dimensions[3], int voxelCount[3]){
    std::vector<float> dim(dimensions, dimensions + 3);
    volume_msg.dimensions = dim;

    std::vector<int> vox(voxelCount, voxelCount + 3);
    volume_msg.voxelCount = vox;

    m_volumePropPub.publish(volume_msg);
}
