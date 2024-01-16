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
        for(auto i: message.ranges)
        {
            if(i>=range_min&&i < min_dist)
                min_dist = i;
        }
        ROS_INFO("min_dist: %f\n", min_dist);
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