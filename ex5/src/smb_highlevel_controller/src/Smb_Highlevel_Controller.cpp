#include"smb_highlevel_controller/Smb_Highlevel_Controller.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller 
{
    Smb_Highlevel_Controller::Smb_Highlevel_Controller(ros::NodeHandle& nodeHandle,tf2_ros::Buffer& tfBuffer):nodeHandle_(nodeHandle),tfBuffer_(tfBuffer)
    {
        //load param
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
        scan_subscriber_ = nodeHandle_.subscribe(scan_subscriberTopic_, subscriberQueueSize_, &Smb_Highlevel_Controller::scan_callback, this);
        cmd_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        vis_marker_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
        stop_and_start = nodeHandle_.advertiseService(start_and_stop_src_name,&Smb_Highlevel_Controller::stop_and_start_callback,this);
    }
    Smb_Highlevel_Controller::~Smb_Highlevel_Controller()
    {
    }

    double Smb_Highlevel_Controller::P_controller(double cur, double Kp,double target)
    {
        auto error = target - cur;
        return Kp*error;
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
        
        pub_vel(pillar_dist,pillar_angle);
        pub_vis_marker(pillar_dist,pillar_angle);
        
    }

    void Smb_Highlevel_Controller::pub_vel(double pillar_dist,double pillar_angle)
    {
    
        geometry_msgs::Twist cmd_vel;

        if(!running)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }
        else
        {
            cmd_vel.linear.x = -P_controller(pillar_dist, kp_linear_,0);
            cmd_vel.linear.x = cmd_vel.linear.x>1?1:cmd_vel.linear.x;
            cmd_vel.angular.z = -P_controller(pillar_angle, kp_angular_,0);
        }
        
        cmd_vel_publisher_.publish(cmd_vel);
    }
    
    void Smb_Highlevel_Controller::pub_vis_marker(double pillar_dist,double pillar_angle)
    {
        auto x = pillar_dist*cos(pillar_angle);
        auto y = pillar_dist*sin(pillar_angle);
        visualization_msgs::Marker marker;
        
        // rslidar_frame
        // marker.header.frame_id = "rslidar";
        // marker.pose.position.x = x;
        // marker.pose.position.y = y;
        // marker.pose.position.z = 0;

        // odom_frame
        geometry_msgs::PointStamped laser_point;
        geometry_msgs::PointStamped odom_point;
        laser_point.header.frame_id = "rslidar";
        laser_point.header.stamp = ros::Time();
        laser_point.point.x = x;
        laser_point.point.y = y;
        laser_point.point.z = 0;
        
        try
        {
            auto transformStamped = tfBuffer_.lookupTransform("odom","rslidar", ros::Time(0));
            tf2::doTransform(laser_point, odom_point, transformStamped);
            marker.header.frame_id = "odom";
            marker.pose.position.x = odom_point.point.x;
            marker.pose.position.y = odom_point.point.y;
            marker.pose.position.z = odom_point.point.z;

            
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.header.stamp = ros::Time();
            marker.ns = "pillar";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;

            vis_marker_publisher_.publish(marker);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
    }

    bool Smb_Highlevel_Controller::stop_and_start_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
    {
        if(request.data)
        {
            ROS_INFO("start");
            running = true;
            response.message = "start to run";
        }
        else
        {
            ROS_INFO("stop");
            running = false;
            response.message = "stop to run";
        }
        response.success = true;
        return true;
    }

    bool Smb_Highlevel_Controller::readParameters()
    {
        if (!nodeHandle_.getParam("laser_subscriber_topic", scan_subscriberTopic_))
            return false;
        if (!nodeHandle_.getParam("queue_size", subscriberQueueSize_))
            return false;
        if (!nodeHandle_.getParam("kp_linear", kp_linear_))
            return false;
        if (!nodeHandle_.getParam("kp_angular", kp_angular_))
            return false;
        if (!nodeHandle_.getParam("start_and_stop_src_name", start_and_stop_src_name))
            return false;
        return true;
    }

}