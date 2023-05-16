#ifndef STATE_H
#define STATE_H

#include "../robotController.h"
#include <ds4_driver/Status.h>

class State
{
public:
    virtual void onEnter(RobotController* controller) = 0;
    virtual void onExit(RobotController* controller) = 0;
    virtual void update(RobotController* controller) = 0;
    virtual void onControllerData(const ds4_driver::Status& msg, RobotController* controller) = 0;
    virtual void onRCOut(const geometry_msgs::Twist::ConstPtr& msg, RobotController* controller) = 0;
    virtual void onPX4State(const mavros_msgs::State::ConstPtr& msg, RobotController* controller) = 0;
};

#endif