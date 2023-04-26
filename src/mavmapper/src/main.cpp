#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/Twist.h>

#define RCAVG 1500
#define RCVAR 500.0

ros::ServiceClient setmode_ser;
ros::Publisher drive_pub;

mavros_msgs::State current_state;
/**
 * state_cb is the callback function triggered by the mavros/state event
 * This function handles reading and storing the State message from mavros/state
 **/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/**
 * rcout_cb is the callback function triggered by the mavros/rc/out event
 * This function handles reading the RCOut message from mavros/rc/out and publishing new drive_vel Twist message
 **/
void rcout_cb(const mavros_msgs::RCOut::ConstPtr& msg) {
    geometry_msgs::Twist tw;
    tw.linear.x = ((double)(msg->channels[0]-RCAVG))/RCVAR;
    tw.linear.y = ((double)(msg->channels[1]-RCAVG))/RCVAR;
    tw.linear.z = ((double)(msg->channels[2]-RCAVG))/RCVAR;
    drive_pub.publish(tw);
}

/**
 * setNewMode attempts to set the new mode in ardurover (ardupilot)
 **/
bool setNewMode(uint8_t mode) {
    mavros_msgs::SetMode newMode;
    newMode.request.base_mode = mode;
    if (setmode_ser.call(newMode)) {
        return newMode.response.mode_sent;
    }
    return false;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavmapper");
    ros::NodeHandle nh;
    setmode_ser = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    drive_pub = nh.advertise<geometry_msgs::Twist>("drive_vel", 1000);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber rcin_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 10, rcout_cb);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ROS_INFO("MavROS connected %i", current_state.connected);
        ROS_INFO("MavROS header %i", current_state.header.seq);
        ros::spinOnce();
        loop_rate.sleep();
    }
}