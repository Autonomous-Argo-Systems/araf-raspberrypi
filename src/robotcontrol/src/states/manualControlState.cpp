#include "headers/manualControlState.h"
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>

#include "states.h"
#include "headers/manualControlState.h"

int previous_right, previous_left;
int last_controller_time;

void ManualControlState::update(RobotController* controller)
{
    if ((time(NULL) - last_controller_time) > 2) {
        last_controller_time = time(NULL);
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = 0.0;
        outmsg.linear.y = 0.0;
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

    int input_left = (int)(msg.axis_left_y * 100.0);
    int input_right = (int)(msg.axis_right_y * 100.0);

    if (input_right != previous_right || input_left != previous_left)
    {
        ROS_INFO("Set drive forward: %d, rotate: %d", input_left, previous_right);
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = ((double)input_left / 100.0);
        outmsg.linear.y = ((double)input_right / 100.0);
        controller->drive_publisher.publish(outmsg);

        previous_right = input_right;
        previous_left = input_left;
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
