#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ds4_driver/Status.h>
#include <geometry_msgs/Twist.h>
#include <time.h>

ros::Publisher drive_publisher;

int previous_linear;
int previous_rotation;
int previous_time;

void set_drive(int forward, int rotate){
    if (forward != previous_linear || rotate != previous_rotation){
        auto outmsg = geometry_msgs::Twist();
        outmsg.linear.x = forward;
        outmsg.angular.z = rotate;
        drive_publisher.publish(outmsg);

        previous_linear = forward;
        previous_rotation = rotate;
    }
}

void on_controller(const ds4_driver::Status& msg){
    previous_time = time(NULL);
    //  ROS_INFO("Published x: %f, y: %f", msg.axis_left_x, msg.axis_left_y);
     if (msg.axis_left_x > 0.9){
        set_drive(0, 2);
     } else if(msg.axis_left_x < -0.9) {
         set_drive(0, -2);
     } else if(msg.axis_left_y > 0.9) {
        set_drive(1, 0);
     } else if(msg.axis_left_y < -0.9) {
        set_drive(-1, 0);
     }else {
        set_drive(0, 0);
     }
}

int main(int argc, char **argv){
    // bootup of the node
    ros::init(argc, argv, "robotcontrol");
    ros::NodeHandle node_handler;

    // Subscribing and advertising
    drive_publisher = node_handler.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber controller_subcriber = node_handler.subscribe("status", 1000, on_controller);

    ROS_INFO("Node is now ready for driving");
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        if (time(NULL) - previous_time > 2) {
            set_drive(0, 0);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}