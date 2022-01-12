#include "collision_publisher.h"
#include <ambf_server/RosComBase.h>

using namespace std;

DrillingPublisher::DrillingPublisher(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

DrillingPublisher::~DrillingPublisher(){
    m_voxelsRemovedPub.shutdown();
}

void DrillingPublisher::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();
    m_voxelsRemovedPub = m_rosNode-> advertise<vdrilling_msgs::points>(a_namespace + "/" + a_plugin + "/voxels_removed", 1);

}

void DrillingPublisher::voxelsRemoved(double ray[3], int vcolor[4], double time){
    msg.sim_time = time;

    std::vector<int> vec(vcolor, vcolor + 3);
    msg.voxel_color = vec;

    msg.voxel_removed.x = ray[0];
    msg.voxel_removed.y = ray[1];
    msg.voxel_removed.z = ray[2];

    m_voxelsRemovedPub.publish(msg);

}

