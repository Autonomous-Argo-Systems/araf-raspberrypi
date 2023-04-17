#include <ros/ros.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavmapper");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ROS_INFO("MavROS connected %i", current_state.connected);
        ROS_INFO("MavROS header %i", current_state.header.seq);
        ros::spinOnce();
        loop_rate.sleep();
    }
}