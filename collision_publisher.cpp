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
    m_removeVoxelsSub = m_rosNode->subscribe(a_namespace + "/" + a_plugin + "/remove_voxels", 1, &DrillingPublisher::removeVoxelsCallback, this);

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

void DrillingPublisher::removeVoxelsCallback(vdrilling_msgs::points voxel_msg){
    m_voxelRemoving_idx[0] = voxel_msg.voxel_removed.x;
    m_voxelRemoving_idx[1] = voxel_msg.voxel_removed.y;
    m_voxelRemoving_idx[2] = voxel_msg.voxel_removed.z;

    m_removingVoxel = true;

    // voxelRemovingColor = voxel_msg.voxel_color;
}

bool DrillingPublisher::getRemoveVoxelsIdx(double* vector){
    if (m_removingVoxel){
        vector[0] = m_voxelRemoving_idx[0];
        vector[1] = m_voxelRemoving_idx[1];
        vector[2] = m_voxelRemoving_idx[2];
    }
    return m_removingVoxel;
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
