#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "robotcontrol");
    ros::NodeHandle nh;

            ROS_INFO("Help me robot control");

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ROS_INFO("Hello robotcontrol");
        ros::spinOnce();
        loop_rate.sleep();
    }
}