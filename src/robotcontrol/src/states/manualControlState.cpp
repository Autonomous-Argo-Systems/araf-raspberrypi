#include "headers/manualControlState.h"
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>

#include "states.h"
#include "headers/manualControlState.h"

int previous_y;
int previous_x;
int last_controller_time;

void ManualControlState::update(RobotController* controller)
{
    if (time(NULL) - last_controller_time > 2) {
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = 0;
        outmsg.linear.y = 0;
        controller->drive_publisher.publish(outmsg);
    }
}

void ManualControlState::onEnter(RobotController* controller)
{
    controller->setLedColor(0, 0, 255, false);
    bool stateChangeSent = controller->setPX4Mode("MANUAL");
    if (!stateChangeSent) {
        ROS_INFO("Failed sending px4state to manual");
    }
}

void ManualControlState::onExit(RobotController* controller)
{
    ROS_INFO("Manual onExit called");
}

void ManualControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller)
{
    last_controller_time = time(NULL);

    if (msg.button_r1)  {
        ROS_INFO("Switching to auto mode");
        controller->switchState(autonomousControlState);
    }

    if (msg.axis_left_y != previous_y || msg.axis_left_x != previous_x)
    {
        ROS_INFO("Set drive forward: %f, rotate: %f", msg.axis_left_y, msg.axis_left_x);
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = msg.axis_left_x;
        outmsg.linear.y = msg.axis_left_y;
        controller->drive_publisher.publish(outmsg);

        previous_y = msg.axis_left_y;
        previous_x = msg.axis_left_x;
    }
}

void ManualControlState::onRCOut(const geometry_msgs::Twist::ConstPtr &msg, RobotController *controller)
{
    // DO NOTHING
}

void ManualControlState::onPX4State(const mavros_msgs::State::ConstPtr &msg, RobotController *controller)
{
    if (msg->mode != "MANUAL") {
        ROS_INFO("Switching PX4 to manual mode");
        bool stateChangeSent = controller->setPX4Mode("MANUAL");
        if (!stateChangeSent) {
            ROS_INFO("Failed sending state to manual");
        }
    }
}
