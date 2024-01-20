#include<ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace smb_emergency_stop_close
{
    class Smb_Emergency_Stop_Close
    {
        public:
            Smb_Emergency_Stop_Close(ros::NodeHandle& nodeHandle);
            virtual ~Smb_Emergency_Stop_Close();
        private:
            ros::NodeHandle& nodeHandle_;
            ros::Subscriber scan_subscriber_;
            std::string scan_subscriberTopic_;
            std::string start_and_stop_src_name_;
            ros::ServiceClient stop_and_start_client_;

            int subscriberQueueSize_;
            void scan_callback(const sensor_msgs::LaserScan& message);
            bool readParameters();
            void call_stop();
    };
}