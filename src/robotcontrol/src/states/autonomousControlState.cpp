#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>


void AutonomousControlState::update(RobotController* controller){}

void AutonomousControlState::onEnter(){}
void AutonomousControlState::onExit(){}

void AutonomousControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller){}
