#ifndef AUTONOMOUS_STATE_H
#define AUTONOMOUS_STATE_H

#include "../state.h"

class AutonomousControlState : public State
{
public:
    void onEnter();
    void onExit();
    void update(RobotController* controller);
    void onControllerData(const ds4_driver::Status& msg, RobotController* controller);
};

#endif