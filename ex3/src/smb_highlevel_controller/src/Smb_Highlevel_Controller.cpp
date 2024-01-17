#include"smb_highlevel_controller/Smb_Highlevel_Controller.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace smb_highlevel_controller 
{
    Smb_Highlevel_Controller::Smb_Highlevel_Controller(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle)
    {
        //load param
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        scan_subscriber_ = nodeHandle_.subscribe(scan_subscriberTopic_, subscriberQueueSize_, &Smb_Highlevel_Controller::scan_callback, this);

    }
    Smb_Highlevel_Controller::~Smb_Highlevel_Controller()
    {
    }

    void Smb_Highlevel_Controller::scan_callback(const sensor_msgs::LaserScan& message)
    {
        auto range_min = message.range_min;
        auto range_max = message.range_max;
        
        auto min_dist = range_max;
        auto min_index = 0;
        for(int i=0;i<message.ranges.size();i++)
        {
            if(message.ranges[i]>=range_min&&message.ranges[i] < min_dist)
            {
                min_dist = message.ranges[i];
                min_index = i;
            }
        }
        
        auto pillar_dist = min_dist;
        auto pillar_angle = message.angle_min + min_index*message.angle_increment;

        ROS_INFO_STREAM("pillar_dist: "<<pillar_dist<<" pillar_angle: "<<pillar_angle<<std::endl);
    }
    


    bool Smb_Highlevel_Controller::readParameters()
    {
        if (!nodeHandle_.getParam("laser_subscriber_topic", scan_subscriberTopic_))
            return false;
        if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_))
            return false;
        return true;
    }

}