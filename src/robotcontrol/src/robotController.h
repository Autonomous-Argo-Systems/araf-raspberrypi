#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/Twist.h>
#include <ds4_driver/Status.h>
#include <ds4_driver/Feedback.h>

class State;

class RobotController
{
  
public:
    ros::Publisher drive_publisher;
    ros::Publisher ds4_publisher;
    State* currentState = nullptr;

    void init(ros::NodeHandle node_handler);
    void handle();
    void onControllerInput(const ds4_driver::Status& msg);
    void onRCOut(const geometry_msgs::Twist::ConstPtr& msg);
    void switchState(State* nextState);
    bool setCommandToPX4(uint16_t cmdint);
    bool setPX4Mode(uint8_t mode);
};

#endif