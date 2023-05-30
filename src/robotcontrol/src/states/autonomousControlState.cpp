#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>

#include "states.h"
#include "headers/autonomousControlState.h"

void AutonomousControlState::update(RobotController *controller) {}

void AutonomousControlState::onEnter(RobotController *controller)
{
    ROS_INFO("Autonomous onEnter called");

    bool stateChangeSent = controller->setPX4Mode("AUTO");
    if (!stateChangeSent) {
        ROS_INFO("Failed sending px4state to auto");
    }
    controller->setLedColor(255, 50, 100, false);
}

void AutonomousControlState::onExit(RobotController *controller) {}

void AutonomousControlState::onControllerData(const ds4_driver::Status &msg, RobotController *controller)
{
    if (msg.button_r1) {
        ROS_INFO("Switching to manual mode");
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

void AutonomousControlState::onPX4State(const mavros_msgs::State::ConstPtr &msg, RobotController *controller)
{
    if (msg->mode != "AUTO") {
        ROS_INFO("Switching PX4 to auto mode");
        bool stateChangeSuccess = controller->setPX4Mode("AUTO");
        if (!stateChangeSuccess) {
            ROS_INFO("Failed changing state to auto");
        }
    }
}

void AutonomousControlState::onLidarRisk(const std_msgs::Float32& msg, RobotController* controller)
{
    
}
