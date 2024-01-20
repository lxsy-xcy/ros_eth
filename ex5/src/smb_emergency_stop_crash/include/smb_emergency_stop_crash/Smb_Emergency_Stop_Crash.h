#include<ros/ros.h>
#include<sensor_msgs/Imu.h>

namespace smb_emergency_stop_crash
{
    class Smb_Emergency_Stop_Crash
    {
        public:
            Smb_Emergency_Stop_Crash(ros::NodeHandle& nodeHandle);
            virtual ~Smb_Emergency_Stop_Crash();
        private:
            ros::NodeHandle& nodeHandle_;
            ros::Subscriber imu_subscriber_;
            std::string imu_subscriberTopic_;
            std::string start_and_stop_src_name_;
            ros::ServiceClient stop_and_start_client_;
            float imu_threshold_;

            int subscriberQueueSize_;
            void imu_callback(const sensor_msgs::Imu& message);
            bool readParameters();
            void call_stop();
    };
}