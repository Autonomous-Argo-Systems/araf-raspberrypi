#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <time.h>

int main(int argc, char **argv){
    // bootup of the node
    ros::init(argc, argv, "gpscontrol");
    ros::NodeHandle node_handler;

    ROS_INFO("GPS node has booted");
    ros::Rate loop_rate(1000);  
    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }
}