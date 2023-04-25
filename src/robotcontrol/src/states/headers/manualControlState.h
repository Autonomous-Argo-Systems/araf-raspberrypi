#ifndef MANUAL_STATE_H
#define MANUAL_STATE_H

#include "../state.h"

class ManualControlState : public State
{
    void set_drive(int forward, int rotate, RobotController* controller);

public:
    void onEnter(RobotController* controller);
    void onExit(RobotController* controller);
    void update(RobotController* controller);
    void onControllerData(const ds4_driver::Status& msg, RobotController* controller);
};

#endif