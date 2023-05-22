#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "datapoint/dataPoint.h"
#include "validate/headers/boxValidator.h"

sensor_msgs::PointCloud2 pointcloud_valid;
ros::Publisher points_pub;

void onLidarData(const sensor_msgs::PointCloud2 pointcloud)
{
    // Filter out all points not on route
    BoxValidator bbVal;
    bbVal.Validate(pointcloud, pointcloud_valid);

    points_pub.publish(pointcloud_valid);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidarcontrol");
    ros::NodeHandle nh;
    
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("validated_points", 1000);
    ros::Subscriber rcin_sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, onLidarData);

    ROS_INFO("Initialized lidarcontrol");

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}