#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>

#include "states.h"

void AutonomousControlState::update(RobotController* controller){}

void AutonomousControlState::onEnter(RobotController* controller)
{
    ROS_INFO("Autonomous onEnter called");

    auto msg = ds4_driver::Feedback();
    msg.set_led = true;
    msg.led_r = 255;
    msg.led_g = 50;
    msg.led_b = 100;

    controller->ds4_publisher.publish(msg);
}

void AutonomousControlState::onExit(RobotController* controller){}

void AutonomousControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller)
{
    if(msg.button_r1) controller->switchState(manualControlState);
}
