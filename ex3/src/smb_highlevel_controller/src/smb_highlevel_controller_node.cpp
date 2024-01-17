#include <ros/ros.h>
#include "smb_highlevel_controller/Smb_Highlevel_Controller.h"

int main(int argc,char**argv)
{
    ros::init(argc,argv,"smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    smb_highlevel_controller::Smb_Highlevel_Controller smb_highlevel_controller(nodeHandle,tfBuffer);

    ros::spin();
    return 0;
}