#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <ros/console.h>
#include <cmath>

class RotateService
{
public:
    RotateService(){
        // Initialize service server
        serviceServer_ = nh_.advertiseService("/rotate_robot",&RotateService::handleRequest,this);
                                                                               
        // Initialize subscriber
        sub_ = nh_.subscribe("/odom", 1000, &RotateService::odomCallback, this);

        // Initialize publisher
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Service server /rotate_robot ready.");
    }

    bool handleRequest(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {

        ROS_INFO("Rotating %d degrees.", req.degrees);
        try {
            int current_angle = quaternionZToDegrees(z_, w_);
            int degrees_to_rotate = std::abs(req.degrees);
            cmd_.angular.z = (req.degrees > 0) ? 0.4 : -0.4;        
            pub_.publish(cmd_);
            while (degrees_to_rotate > 0) {
                int new_angle = quaternionZToDegrees(z_, w_);
                if (new_angle != current_angle) {
                    degrees_to_rotate -= 1;
                    current_angle = new_angle;
                    ROS_DEBUG("Current angle: %d", current_angle);
                }
                ros::spinOnce();
            }
            ROS_INFO("Finished successfully.");
            res.result = "Service Finished Successfully";
            cmd_.angular.z = 0.0;
            pub_.publish(cmd_);
        } catch (...) {
            res.result = "Failed";
        }
        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        w_ = msg->pose.pose.orientation.w;
        z_ = msg->pose.pose.orientation.z;
    }

    int quaternionZToDegrees(double z, double w) {
        // Calculate the rotation angle in radians
        double theta = 2 * std::atan2(z, w);
        // Convert the angle to degrees
        int angle = theta * (180.0 / M_PI);
        return angle;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer serviceServer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    float z_;
    float w_;
    geometry_msgs::Twist cmd_;
};

int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "rotate_service_server");
    RotateService server;

    ros::spin();

    return 0;
}