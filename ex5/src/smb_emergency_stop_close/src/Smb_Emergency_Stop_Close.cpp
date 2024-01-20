#include "smb_emergency_stop_close/Smb_Emergency_Stop_Close.h"
#include <std_srvs/SetBool.h>

namespace smb_emergency_stop_close 
{
    Smb_Emergency_Stop_Close::Smb_Emergency_Stop_Close(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle)
    {
        //load param
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
        scan_subscriber_ = nodeHandle_.subscribe(scan_subscriberTopic_, subscriberQueueSize_, &Smb_Emergency_Stop_Close::scan_callback, this);
        stop_and_start_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(start_and_stop_src_name_);
    }
    Smb_Emergency_Stop_Close::~Smb_Emergency_Stop_Close()
    {
    }

    void Smb_Emergency_Stop_Close::scan_callback(const sensor_msgs::LaserScan& message)
    {
        auto range_min = message.range_min;
        auto range_max = message.range_max;
        
        auto min_dist = range_max;
        for(int i=0;i<message.ranges.size();i++)
        {
            if(message.ranges[i]>=range_min&&message.ranges[i] < min_dist)
            {
                min_dist = message.ranges[i];
            }
        }
        
        auto pillar_dist = min_dist;

        if(pillar_dist<1)
        {
            call_stop();
        }
        
    }

    void Smb_Emergency_Stop_Close::call_stop()
    {
        std_srvs::SetBool srv;
        srv.request.data = false;
        if(stop_and_start_client_.call(srv))
        {
            ROS_INFO("call stop");
        }
        else
        {
            ROS_ERROR("Failed to call service stop_and_start");
        }
    }


    bool Smb_Emergency_Stop_Close::readParameters()
    {
        if (!nodeHandle_.getParam("laser_subscriber_topic", scan_subscriberTopic_))
            return false;
        if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_))
            return false;
        if (!nodeHandle_.getParam("start_and_stop_src_name", start_and_stop_src_name_))
            return false;
        return true;
    }

}