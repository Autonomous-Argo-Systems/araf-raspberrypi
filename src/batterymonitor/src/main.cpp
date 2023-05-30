#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "mavmapper");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}