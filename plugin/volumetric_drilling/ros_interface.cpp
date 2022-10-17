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
#include "ros_interface.h"
#include <ambf_server/RosComBase.h>

using namespace std;

DrillingPublisher::DrillingPublisher(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

DrillingPublisher::~DrillingPublisher(){
    m_voxelsRemovalPub.shutdown();
    m_drillSizePub.shutdown();
    m_volumeInfoPub.shutdown();
}

void DrillingPublisher::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();

    m_voxelsRemovalPub = m_rosNode->advertise<volumetric_drilling_msgs::Voxels>(a_namespace + "/" + a_plugin + "/voxels_removed", 1);
    m_drillSizePub = m_rosNode->advertise<volumetric_drilling_msgs::DrillSize>(a_namespace + "/" + a_plugin + "/drill_size", 1, true);
    m_volumeInfoPub = m_rosNode->advertise<volumetric_drilling_msgs::VolumeInfo>(a_namespace + "/" + a_plugin + "/volume_info", 1, true);
}

void DrillingPublisher::publishDrillSize(int burrSize, double time){
    m_drill_size_msg.header.stamp.fromSec(time);
    m_drill_size_msg.size.data = burrSize;

    m_drillSizePub.publish(m_drill_size_msg);
}

void DrillingPublisher::setVolumeInfo(cTransform &pose, cVector3d& dimensions, cVector3d& voxel_count)
{

    cQuaternion quat;
    quat.fromRotMat(pose.getLocalRot());

    m_volume_info_msg.pose.position.x = pose.getLocalPos().x();
    m_volume_info_msg.pose.position.y = pose.getLocalPos().y();
    m_volume_info_msg.pose.position.z = pose.getLocalPos().z();

    m_volume_info_msg.pose.orientation.z = quat.z;
    m_volume_info_msg.pose.orientation.x = quat.x;
    m_volume_info_msg.pose.orientation.y = quat.y;
    m_volume_info_msg.pose.orientation.w = quat.w;

    m_volume_info_msg.dimensions.resize(3);
    m_volume_info_msg.dimensions[0] = dimensions.x();
    m_volume_info_msg.dimensions[1] = dimensions.y();
    m_volume_info_msg.dimensions[2] = dimensions.z();

    m_volume_info_msg.voxel_count.resize(3);
    m_volume_info_msg.voxel_count[0] = voxel_count.x();
    m_volume_info_msg.voxel_count[1] = voxel_count.y();
    m_volume_info_msg.voxel_count[2] = voxel_count.z();
}

void DrillingPublisher::publishVolumeInfo(double time)
{
    m_volume_info_msg.header.stamp.fromSec(time);
    m_volumeInfoPub.publish(m_volume_info_msg);
}

void DrillingPublisher::appendToVoxelMsg(cVector3d &index, cColorf &color)
{
    geometry_msgs::Point p;
    p.x = index.x(); p.y = index.y(); p.z = index.z();
    m_voxel_msg.indices.push_back(p);
    std_msgs::ColorRGBA col;
    col.r = color.getR(); col.g = color.getG(); col.b = color.getB(); col.a = color.getA();
    m_voxel_msg.colors.push_back(col);
}

void DrillingPublisher::clearVoxelMsg()
{
    // Clear the message for next itr.
    m_voxel_msg.indices.clear();
    m_voxel_msg.colors.clear();
}

void DrillingPublisher::publishVoxelMsg(double time)
{
    m_voxel_msg.header.stamp.fromSec(time);
    m_voxelsRemovalPub.publish(m_voxel_msg);
}
