#include "smb_emergency_stop_crash/Smb_Emergency_Stop_Crash.h"
#include <std_srvs/SetBool.h>

namespace smb_emergency_stop_crash 
{
    Smb_Emergency_Stop_Crash::Smb_Emergency_Stop_Crash(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle)
    {
        //load param
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
        imu_subscriber_ = nodeHandle_.subscribe(imu_subscriberTopic_, subscriberQueueSize_, &Smb_Emergency_Stop_Crash::imu_callback, this);
        stop_and_start_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(start_and_stop_src_name_);
    }
    Smb_Emergency_Stop_Crash::~Smb_Emergency_Stop_Crash()
    {
    }

    void Smb_Emergency_Stop_Crash::imu_callback(const sensor_msgs::Imu& message)
    {
        auto linear_x = message.linear_acceleration.x;
        auto linear_y = message.linear_acceleration.y;
        // ROS_INFO_STREAM("linear_x: "<<linear_x<<" linear_y: "<<linear_y<<std::endl);
        // ROS_INFO_STREAM("time: "<<ros::Time::now().toSec()<<std::endl);
        // when the simulator is started, the imu will have a large linear_x and linear_y
        if (abs(linear_x)+abs(linear_y)>imu_threshold_&&ros::Time::now().toSec()>10)
        {
            ROS_INFO_STREAM("[imu warning] linear_x: "<<linear_x<<" linear_y: "<<linear_y<<std::endl);
            ROS_INFO_STREAM("Crash!!!!!!!!!!!!!!!!=============================="<<std::endl);
            call_stop();
        }
    }

    void Smb_Emergency_Stop_Crash::call_stop()
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


    bool Smb_Emergency_Stop_Crash::readParameters()
    {
        if (!nodeHandle_.getParam("imu_subscriber_topic", imu_subscriberTopic_))
            return false;
        if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_))
            return false;
        if (!nodeHandle_.getParam("start_and_stop_src_name", start_and_stop_src_name_))
            return false;
        if (!nodeHandle_.getParam("imu_threshold", imu_threshold_))
            return false;
        return true;
    }

}