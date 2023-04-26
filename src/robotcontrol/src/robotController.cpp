#include "robotController.h"

#include "states/state.h"
#include "states/states.h"

const uint32_t switchCooldown = 500000000; // Cooldown time in nanoseconds

void RobotController::onControllerInput(const ds4_driver::Status& msg)
{
    currentState->onControllerData(msg, this);
}

void RobotController::onRCOut(const geometry_msgs::Twist::ConstPtr &msg)
{
    currentState->onRCOut(msg, this);
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

ros::Time last_state_switch_time;

void RobotController::switchState(State* newState)
{
    ros::Time now = ros::Time::now();
    if (now.nsec - last_state_switch_time.nsec < switchCooldown) return;
    last_state_switch_time = now;

    ROS_INFO("Switching to new state");
    currentState->onExit(this);
    currentState = newState;
    currentState->onEnter(this);
}