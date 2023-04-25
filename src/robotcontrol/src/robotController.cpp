#include "robotController.h"

#include "states/state.h"
#include "states/states.h"

void RobotController::onControllerInput(const ds4_driver::Status& msg)
{
    currentState->onControllerData(msg, this);
}

void RobotController::init()
{
    ROS_INFO("Init RoboController called");

    // Default manual state on init
    currentState = manualControlState;

    ROS_INFO("Finished RobotController init");
}

void RobotController::handle()
{
    currentState->update(this);
}

void RobotController::switchState(State* newState)
{
    currentState->onExit();
    currentState = newState;
    currentState->onEnter();
}