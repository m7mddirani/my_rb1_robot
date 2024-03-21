#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class RotateService
{
public:
    RotateService() : current_theta(0.0) {
        serviceServer_ = nh_.advertiseService("/rotate_robot", &RotateService::handleRequest, this);
        sub_ = nh_.subscribe("/odom", 1000, &RotateService::odomCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        ROS_INFO("Service server /rotate_robot ready.");
    }

    bool handleRequest(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
        ROS_INFO("Rotating %d degrees.", req.degrees);
        double radians = (req.degrees % 360) * (M_PI / 180.0); // Convert degrees to radians
        double target_theta = fmod(current_theta + radians + 2 * M_PI, 2 * M_PI); // Calculate target in radians
        ros::Rate rate(10);
        while (ros::ok()) {
            double orientation_error = target_theta - current_theta;
            orientation_error = atan2(sin(orientation_error), cos(orientation_error)); // Normalize error
            if (fabs(orientation_error) < 0.05) { // Check if within margin
                cmd_.angular.z = 0.0; // Stop rotation
                pub_.publish(cmd_);
                ROS_INFO("Rotation completed successfully.");
                res.result = "Rotation completed successfully";
                return true;
            }
            cmd_.angular.z = (orientation_error > 0) ? 0.1 : -0.1;
            pub_.publish(cmd_);
            ros::spinOnce();
            rate.sleep();
        }
        res.result = "Rotation NOT complete";
        return false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta = atan2(siny_cosp, cosy_cosp);
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer serviceServer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::Twist cmd_;
    double current_theta; // Track the current orientation in radians
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service_server");
    RotateService server;
    ros::spin();
    return 0;
}
