#ifndef AUTONOMOUS_STATE_H
#define AUTONOMOUS_STATE_H

#include "../state.h"

class AutonomousControlState : public State
{
public:
    void onEnter(RobotController* controller);
    void onExit(RobotController* controller);
    void update(RobotController* controller);
    void onControllerData(const ds4_driver::Status& msg, RobotController* controller);
    void onRCOut(const geometry_msgs::Twist::ConstPtr& msg, RobotController* controller);
    void onPX4State(const mavros_msgs::State::ConstPtr& msg, RobotController* controller);
    void onLidarRisk(const std_msgs::Float32& msg, RobotController* controller);
};

#endif