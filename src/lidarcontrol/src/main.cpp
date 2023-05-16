#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "dataPoint.h"

sensor_msgs::PointCloud2 pointcloud_valid;
ros::Publisher points_pub;

bool isDataPointValid(DataPoint* point)
{
    return point->x < 2 && point->y < 0;
}

void onLidarData(const sensor_msgs::PointCloud2 pointcloud)
{
    // Filter usefull points
    uint8_t point_data[pointcloud.point_step];
    pointcloud_valid.data.clear();
    
    for(int i = 0; i < pointcloud.row_step; i += pointcloud.point_step)
    {
        // Copying to temp buffer
        for (size_t j = 0; j < pointcloud.point_step; j++)
            point_data[j] = pointcloud.data[i + j];
        
        // Casting pointers
        DataPoint* datapoint_ptr = (DataPoint*) point_data;

        if (isDataPointValid(datapoint_ptr)){
            // Copying to result buffer
            for (size_t j = 0; j < pointcloud.point_step; j++)
                pointcloud_valid.data.push_back(point_data[j]);
        }
    }

    // Setup Validated pointcloud message
    pointcloud_valid.height = pointcloud.height;
    pointcloud_valid.fields = pointcloud.fields;
    pointcloud_valid.is_bigendian = pointcloud.is_bigendian;
    pointcloud_valid.point_step = pointcloud.point_step;
    pointcloud_valid.is_dense = pointcloud.is_dense;
    pointcloud_valid.header.frame_id = pointcloud.header.frame_id;

    pointcloud_valid.row_step = pointcloud_valid.data.size();
    pointcloud_valid.width = pointcloud_valid.row_step / pointcloud_valid.point_step;

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