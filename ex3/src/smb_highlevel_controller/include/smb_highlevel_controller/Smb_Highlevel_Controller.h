#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace smb_highlevel_controller
{
    class Smb_Highlevel_Controller
    {
        public:
            Smb_Highlevel_Controller(ros::NodeHandle &nodeHandle);
            virtual ~Smb_Highlevel_Controller();

        private:
            ros::NodeHandle& nodeHandle_;
            ros::Subscriber scan_subscriber_;
            std::string scan_subscriberTopic_;
            int subscriberQueueSize_;
            void scan_callback(const sensor_msgs::LaserScan& message);
            bool readParameters();
    };
}