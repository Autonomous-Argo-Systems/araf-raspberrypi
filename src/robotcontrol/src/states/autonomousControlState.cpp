#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>

#include "states.h"

void AutonomousControlState::update(RobotController* controller){}

void AutonomousControlState::onEnter(RobotController* controller){ROS_INFO("Autonomous onEnter called");}
void AutonomousControlState::onExit(RobotController* controller){}

void AutonomousControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller)
{
    if(msg.button_r1) controller->switchState(manualControlState);
}
