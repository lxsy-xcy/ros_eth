#include <ros/ros.h>
#include "smb_emergency_stop_close/Smb_Emergency_Stop_Close.h"

int main(int argc,char**argv)
{
    ros::init(argc,argv,"smb_emergency_stop_close");
    ros::NodeHandle nodeHandle("~");
    smb_emergency_stop_close::Smb_Emergency_Stop_Close smb_emergency_stop_close(nodeHandle);

    ros::spin();
    return 0;
}