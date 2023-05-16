#include "robotController.h"

#include "states/state.h"
#include "states/states.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>

ros::ServiceClient setmode_ser;
ros::ServiceClient cmd_ser;
const uint32_t switchCooldown = 500000000; // Cooldown time in nanoseconds

void RobotController::onControllerInput(const ds4_driver::Status& msg)
{
    currentState->onControllerData(msg, this);
}

void RobotController::onRCOut(const geometry_msgs::Twist::ConstPtr &msg)
{
    currentState->onRCOut(msg, this);
}

void RobotController::onPX4State(const mavros_msgs::State::ConstPtr& msg)
{
    currentState->onPX4State(msg, this);
}

void RobotController::init(ros::NodeHandle node_handler)
{
    ROS_INFO("Init RoboController called");

    // Default manual state on init
    currentState = manualControlState;

    setmode_ser = node_handler.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    cmd_ser = node_handler.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

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

/**
 * sendCommand sends command to ardurover (ardupilot)
 **/
bool RobotController::setCommandToPX4(uint16_t cmdint)
{
    mavros_msgs::CommandLong cmd;
    cmd.request.command = cmdint;
    if (cmd_ser.call(cmd)) {
        return cmd.response.success;
    }
    return false;
}

/**
 * setNewMode attempts to set the new mode in ardurover (ardupilot)
 **/
bool RobotController::setPX4Mode(char* mode) {
    mavros_msgs::SetMode newMode;
    newMode.request.custom_mode = mode;
    if (setmode_ser.call(newMode)) {
        return newMode.response.mode_sent;
    }
    return false;
}

void RobotController::setLedColor(float r, float g, float b, bool blink)
{
    auto msg = ds4_driver::Feedback();
    msg.set_led = true;
    msg.led_r = r;
    msg.led_g = g;
    msg.led_b = b;
    msg.led_flash_off = 0.25;
    msg.led_flash_on = 0.25;
    msg.set_led_flash = blink;
    
    this->ds4_publisher.publish(msg);
}
