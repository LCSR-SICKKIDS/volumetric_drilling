#include "collision_publisher.h"
#include <ambf_server/RosComBase.h>

using namespace std;

DrillingPublisher::DrillingPublisher(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

DrillingPublisher::~DrillingPublisher(){
    m_voxelsRemovedPub.shutdown();
    m_burrChangePub.shutdown();
}

void DrillingPublisher::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();

    m_voxelsRemovedPub = m_rosNode-> advertise<vdrilling_msgs::points>(a_namespace + "/" + a_plugin + "/voxels_removed", 1);
    m_burrChangePub = m_rosNode -> advertise<vdrilling_msgs::UInt8Stamped>(a_namespace + "/" + a_plugin + "/burr_change", 1, true);
}

void DrillingPublisher::voxelsRemoved(double ray[3], float vcolor[4], double time){
    voxel_msg.header.stamp.fromSec(time);

    std::vector<float> vec(vcolor, vcolor + 4);
    voxel_msg.voxel_color = vec;

    voxel_msg.voxel_removed.x = ray[0];
    voxel_msg.voxel_removed.y = ray[1];
    voxel_msg.voxel_removed.z = ray[2];

    m_voxelsRemovedPub.publish(voxel_msg);
}

void DrillingPublisher::burrChange(int burrSize, double time){
    burr_msg.header.stamp.fromSec(time);
    burr_msg.number.data = burrSize;

    m_burrChangePub.publish(burr_msg);
}
