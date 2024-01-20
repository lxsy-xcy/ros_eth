#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller
{
    class Smb_Highlevel_Controller
    {
        public:
            Smb_Highlevel_Controller(ros::NodeHandle &nodeHandle,tf2_ros::Buffer &tfBuffer);
            virtual ~Smb_Highlevel_Controller();

        private:
            ros::NodeHandle& nodeHandle_;
            ros::Subscriber scan_subscriber_;
            std::string scan_subscriberTopic_;
            std::string start_and_stop_src_name;
            tf2_ros::Buffer& tfBuffer_;
            float kp_linear_;
            float kp_angular_;
            ros::Publisher cmd_vel_publisher_;
            ros::Publisher vis_marker_publisher_;
            ros::ServiceServer stop_and_start;
            int subscriberQueueSize_;

            bool running=true;
            
            void scan_callback(const sensor_msgs::LaserScan& message);
            void pub_vel(double pillar_dist,double pillar_angle);
            void pub_vis_marker(double pillar_dist,double pillar_angle);
            bool readParameters();
            double P_controller(double cur, double Kp,double target);

            bool stop_and_start_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    };
}