#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>

#include "states.h"

void AutonomousControlState::update(RobotController *controller) {}

void AutonomousControlState::onEnter(RobotController *controller)
{
    ROS_INFO("Autonomous onEnter called");

    auto msg = ds4_driver::Feedback();
    msg.set_led = true;
    msg.led_r = 255;
    msg.led_g = 50;
    msg.led_b = 100;

    controller->ds4_publisher.publish(msg);
}

void AutonomousControlState::onExit(RobotController *controller) {}

void AutonomousControlState::onControllerData(const ds4_driver::Status &msg, RobotController *controller)
{
    if (msg.button_r1) {
        // TODO handle error in setting of mode
        controller->setPX4Mode(192 /*MAV_MODE_MANUAL_ARMED */);
        controller->switchState(manualControlState);
    }
    if (msg.button_triangle) {
        controller->setCommandToPX4(300 /*MISSION_START*/);
    }
}

void AutonomousControlState::onRCOut(const geometry_msgs::Twist::ConstPtr &msg, RobotController *controller)
{
    controller->drive_publisher.publish(msg);
}
