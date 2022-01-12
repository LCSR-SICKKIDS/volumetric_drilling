#ifndef COLLISION_PUBLISHER_H
#define COLLISION_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <vdrilling_msgs/points.h>


class DrillingPublisher{
public:
    DrillingPublisher(std::string a_namespace, std::string a_plugin);
    ~DrillingPublisher();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;

    void voxelsRemoved(double ray[3], int vcolor[4], double time);
private:
    ros::Publisher m_voxelsRemovedPub;
    vdrilling_msgs::points msg;
};

#endif //VOLUMETRIC_PLUGIN_COLLISION_PUBLISHER_H

/*
std_msgs/Header header
float64 sim_time
geometry_msgs/Point voxel_removed
int32[] voxel_color
*/
