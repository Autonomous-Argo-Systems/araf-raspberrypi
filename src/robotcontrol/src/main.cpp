#include <ros/ros.h>
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

int main(int argc, char **argv){
    // bootup of the node
    ros::init(argc, argv, "robotcontrol");
    ros::NodeHandle node_handler;

    robotController.init();

    // Subscribing and advertising
    robotController.drive_publisher = node_handler.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    robotController.ds4_publisher = node_handler.advertise<ds4_driver::Feedback>("set_feedback", 1000);
    ros::Subscriber controller_subcriber = node_handler.subscribe("status", 1000, on_controller);
    ros::Subscriber drive_sub = node_handler.subscribe("drive_vel", 1000, on_rcout);

    ROS_INFO("Node is now ready for driving");
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        robotController.handle();

        ros::spinOnce();
        loop_rate.sleep();
    }
}