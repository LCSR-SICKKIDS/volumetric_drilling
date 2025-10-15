#ifndef COLLISION_PUBLISHER_H
#define COLLISION_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <vdrilling_msgs/points.h>
#include <vdrilling_msgs/UInt8Stamped.h>
#include <vdrilling_msgs/VolumeProp.h>
#include <vector>

using namespace std;

class DrillingPublisher{
public:
    DrillingPublisher(std::string a_namespace, std::string a_plugin);
    ~DrillingPublisher();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;

    void voxelsRemoved(double ray[3], float vcolor[4], double time);
    void removeVoxelsCallback(vdrilling_msgs::points voxel_msg);
    void burrChange(int burrSize, double time);
    void volumeProp(float dimensions[3], int voxelCount[3]);
    bool getRemoveVoxelsIdx(double* vector);

private:
    ros::Publisher m_voxelsRemovedPub;
    ros::Publisher m_burrChangePub;
    ros::Publisher m_volumePropPub;
    ros::Subscriber m_removeVoxelsSub;

    vdrilling_msgs::points voxel_msg;
    vdrilling_msgs::UInt8Stamped burr_msg;
    vdrilling_msgs::VolumeProp volume_msg;

    double m_voxelRemoving_idx[3];
    bool m_removingVoxel = false;

};

#endif //VOLUMETRIC_PLUGIN_COLLISION_PUBLISHER_H
