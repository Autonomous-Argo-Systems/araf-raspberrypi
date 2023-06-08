#include "headers/autonomousControlState.h"
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>

#include "states.h"

void AutonomousControlState::update(RobotController *controller) {}

void AutonomousControlState::onEnter(RobotController *controller)
{
    ROS_INFO("Autonomous onEnter called");

    bool stateChangeSent = controller->setPX4Mode("AUTO");
    if (!stateChangeSent) {
        ROS_INFO("Failed sending px4state to auto");
    }
    controller->setLedColor(255, 50, 100, false);

    stopTwistMsg.linear.x = 0;
    stopTwistMsg.linear.y = 0;
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
    if (directStopActive) return;

    geometry_msgs::Twist twist;
    twist.linear.x = msg->linear.x * speedFactor;
    twist.linear.y = msg->linear.y * speedFactor;

    if (twist.linear.x != AutonomousControlState::previous_left || twist.linear.y != AutonomousControlState::previous_right)
    {
        controller->drive_publisher.publish(twist);

    }
    AutonomousControlState::previous_left = twist.linear.x;
    AutonomousControlState::previous_right = twist.linear.y; 
        
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

bool isSlowing = false;

void AutonomousControlState::onLidarRisk(const std_msgs::Float32& msg, RobotController* controller)
{
    if (msg.data >= directStopTreshold)
    {
        directStopActive = true;
        controller->drive_publisher.publish(stopTwistMsg);

        controller->setLedColor(255, 0, 0, false);

        return;
    } else
    {
        if (directStopActive) controller->setLedColor(255, 50, 100, false);
        directStopActive = false;
    }

    if (msg.data > slowThreshold)
    {
        isSlowing = true;
        float overSpeedFactor = (msg.data - slowThreshold) / (directStopTreshold - slowThreshold);
        float speedPercentage = ((1.0f - msg.data) * (100.0f - slowThreshold)) + slowThreshold;
        speedFactor = speedFactor / 100.0f;

        controller->setLedColor(255, 50, 50, false);
    } else
    {
        if(isSlowing) controller->setLedColor(255, 50, 100, false);
        speedFactor = 1.0f;
    }
}
