#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ds4_driver/Status.h>

void on_input_ps4(const ds4_driver::Status& msg){
     ROS_INFO("Published x: %f, y: %f", msg.axis_left_x, msg.axis_left_y);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "robotcontrol");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("status", 1000, on_input_ps4);

    ROS_INFO("We are now subbed");

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}