#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ds4_driver/Status.h>

class State;

class RobotController
{
  
public:
    ros::Publisher drive_publisher;
    State* currentState = nullptr;

    void init();
    void handle();
    void onControllerInput(const ds4_driver::Status& msg);
    void switchState(State* nextState);
};

#endif