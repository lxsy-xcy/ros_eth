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


        // algorithm_ = Algorithm();
        scan_subscriber_ = nodeHandle_.subscribe(scan_subscriberTopic_, subscriberQueueSize_, &Smb_Highlevel_Controller::scan_callback, this);
        pc_subscriber_ = nodeHandle_.subscribe(pc_subscriberTopic_, subscriberQueueSize_, &Smb_Highlevel_Controller::pc_callback, this);

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
    
    void Smb_Highlevel_Controller::pc_callback(const sensor_msgs::PointCloud2& message)
    {
        auto point_step = message.point_step;
        auto len_data = message.data.size();
        auto num_points = len_data/point_step;
        ROS_INFO_STREAM("num_points:"<<num_points<<"\n");
    }

    bool Smb_Highlevel_Controller::readParameters()
    {
        if (!nodeHandle_.getParam("laser_subscriber_topic", scan_subscriberTopic_))
            return false;
        if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_))
            return false;
        if(!nodeHandle_.getParam("pc_subscriber_topic",pc_subscriberTopic_))
            return false;
        return true;
    }



}