#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "robotController.h"

RobotController robotController;

void on_controller(const ds4_driver::Status& msg)
{
    robotController.onControllerInput(msg);
}

void on_rcout(const geometry_msgs::Twist::ConstPtr& msg) {
    robotController.onRCOut(msg);
}

void on_state(const mavros_msgs::State::ConstPtr& msg) {
    robotController.onPX4State(msg);
}

void on_lidar_risk(const std_msgs::Float32& msg) {
    robotController.onLidarRisk(msg);
}

int main(int argc, char **argv){
    // bootup of the node
    ros::init(argc, argv, "robotcontrol");
    ros::NodeHandle node_handler;

    robotController.init(node_handler);

    // Subscribing and advertising
    robotController.drive_publisher = node_handler.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    robotController.ds4_publisher = node_handler.advertise<ds4_driver::Feedback>("set_feedback", 1000);
    ros::Subscriber px4_state = node_handler.subscribe("mavros/state", 1000, on_state);
    ros::Subscriber controller_subcriber = node_handler.subscribe("status", 1000, on_controller);
    ros::Subscriber drive_sub = node_handler.subscribe("drive_vel", 1000, on_rcout);
    ros::Subscriber lidar_risk_sub = node_handler.subscribe("lidar_calculated_risk", 1000, on_lidar_risk);

    ROS_INFO("Node is now ready for driving");
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        robotController.handle();

        ros::spinOnce();
        loop_rate.sleep();
    }
}