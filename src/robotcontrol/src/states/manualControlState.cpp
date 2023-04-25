#include "headers/manualControlState.h"
#include <geometry_msgs/Twist.h>

#include "states.h"

int previous_linear;
int previous_rotation;
int last_controller_time;

void ManualControlState::set_drive(int forward, int rotate, RobotController* controller){
    if (forward != previous_linear || rotate != previous_rotation){
        ROS_INFO("Set drive forward: %d, rotate: %d", forward, rotate);
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = forward;
        outmsg.angular.z = rotate;
        controller->drive_publisher.publish(outmsg);

        previous_linear = forward;
        previous_rotation = rotate;
    }
}

void ManualControlState::update(RobotController* controller)
{
    if (time(NULL) - last_controller_time > 2) {
        set_drive(0, 0, controller);
    }
}

void ManualControlState::onEnter(){}
void ManualControlState::onExit(){}

void ManualControlState::onControllerData(const ds4_driver::Status& msg, RobotController* controller)
{
    last_controller_time = time(NULL);

    if (msg.axis_left_x > 0.9) set_drive(0, 2, controller);
    else if(msg.axis_left_x < -0.9) set_drive(0, -2, controller);
    else if(msg.axis_left_y > 0.9) set_drive(1, 0, controller);
    else if(msg.axis_left_y < -0.9) set_drive(-1, 0, controller);
    else set_drive(0, 0, controller);

    if(msg.button_r1) controller->switchState(autonomousControlState);    
}