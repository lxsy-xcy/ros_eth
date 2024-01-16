#include <ros/ros.h>
#include "smb_highlevel_controller/Smb_Highlevel_Controller.h"

int main(int argc,char**argv)
{
    ros::init(argc,argv,"smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    smb_highlevel_controller::Smb_Highlevel_Controller smb_highlevel_controller(nodeHandle);

    ros::spin();
    return 0;
}