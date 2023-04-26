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

    void init();
    void handle();
    void onControllerInput(const ds4_driver::Status& msg);
    void onRCOut(const geometry_msgs::Twist::ConstPtr& msg);
    void switchState(State* nextState);
};

#endif