#ifndef COLLISION_PUBLISHER_H
#define COLLISION_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <vdrilling_msgs/points.h>
#include <vdrilling_msgs/UInt8Stamped.h>


class DrillingPublisher{
public:
    DrillingPublisher(std::string a_namespace, std::string a_plugin);
    ~DrillingPublisher();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;

    void voxelsRemoved(double ray[3], float vcolor[4], double time);
    void burrChange(int burrSize, double time);
private:
    ros::Publisher m_voxelsRemovedPub;
    ros::Publisher m_burrChangePub;
    vdrilling_msgs::points voxel_msg;
    vdrilling_msgs::UInt8Stamped burr_msg;

};

#endif //VOLUMETRIC_PLUGIN_COLLISION_PUBLISHER_H
