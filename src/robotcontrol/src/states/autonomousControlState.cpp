#include "state.h"
#include <geometry_msgs/Twist.h>

class AutonomousControlState : public State
{
    void update(RobotController* controller){}

    void onEnter(){}
    void onExit(){}

    void onControllerData(const ds4_driver::Status& msg, RobotController* controller){}
};