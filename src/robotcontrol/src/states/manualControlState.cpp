#include "headers/manualControlState.h"
#include <geometry_msgs/Twist.h>

#include "states.h"

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
    auto msg = ds4_driver::Feedback();
    msg.set_led = true;
    msg.led_r = 0;
    msg.led_g = 0;
    msg.led_b = 255;

    controller->ds4_publisher.publish(msg);
}

void ManualControlState::onExit(RobotController* controller)
{
    ROS_INFO("Manual onExit called");
}

void ManualControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller)
{
    last_controller_time = time(NULL);

    if (msg.button_r1)  {
        // TODO handle error in setting of mode
        controller->setPX4Mode(216 /*MAV_MODE_GUIDED_ARMED*/);
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
