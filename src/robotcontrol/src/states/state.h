#ifndef STATE_H
#define STATE_H

#include "../robotController.h"
#include <ds4_driver/Status.h>

class State
{
public:
    virtual void onEnter() = 0;
    virtual void onExit() = 0;
    virtual void update(RobotController* controller) = 0;
    virtual void onControllerData(const ds4_driver::Status& msg, RobotController* controller) = 0;
};

#endif