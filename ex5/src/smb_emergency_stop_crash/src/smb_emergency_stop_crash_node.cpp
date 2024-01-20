#include <ros/ros.h>
#include "smb_emergency_stop_crash/Smb_Emergency_Stop_Crash.h"
int main(int argc,char**argv)
{
    ros::init(argc,argv,"smb_emergency_stop_crash");
    ros::NodeHandle nodeHandle("~");
    smb_emergency_stop_crash::Smb_Emergency_Stop_Crash smb_emergency_stop_crash(nodeHandle);

    ros::spin();
    return 0;
}