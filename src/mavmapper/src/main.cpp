#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "mavmapper");
    ros::NodeHandle nh;

            ROS_INFO("Help me mavmapper");

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ROS_INFO("Hello mavmapper");
        ros::spinOnce();
        loop_rate.sleep();
    }
}