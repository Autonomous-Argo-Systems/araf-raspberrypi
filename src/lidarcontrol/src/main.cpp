#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


ros::Publisher drive_pub;

void onLidarData(const sensor_msgs::PointCloud2 poinscloud)
{
    // Check message

    // Filter usefull points

    // Create 'Usefull data' cloud message

    // Sends
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidarcontrol");
    ros::NodeHandle nh;
    
    ros::Subscriber rcin_sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, onLidarData);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}