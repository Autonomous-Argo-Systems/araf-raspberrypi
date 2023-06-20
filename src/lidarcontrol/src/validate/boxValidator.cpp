#include "headers/boxValidator.h"
#include <ros/ros.h>


// Default settings for the bounding box, value's in m
float leftOffset = 0.5f;
float rightOffset = 0.5f;
float topOffset = 4.0f;
float bottomOffset = 1.0f;
float distance = 3.0f;
float back = 0.0f;

/// @brief The algotithm that determines if the datapoint is usefull for further analyses.
/// For now this returns a bounding box in front of the lidar. Could be reworkt to use different algorithms based on 
/// the situation
/// @param point The point to check
/// @return True if valid point.
bool BoxValidator::isDataPointValid(DataPoint* point)
{
    return point->x <= distance && point->x >= (back * -1.0f) &&
        point->y <= leftOffset && point->y >= (rightOffset * -1.0f) &&
        point->z <= topOffset && point->z >= (bottomOffset * -1.0f);
}

void BoxValidator::Validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &valid_pointcloud)
{
    // Filter usefull points
    valid_pointcloud.data.clear();
    
    for(int i = 0; i < pointcloud.row_step; i += pointcloud.point_step)
    {
        DataPoint datapoint = fetchDatapoint(pointcloud.data, i);
        if (isDataPointValid(&datapoint)){
            // Pushing datapoint
           pushDatapoint(valid_pointcloud.data, datapoint);
        }
    }

    //Setup Validated pointcloud message
    valid_pointcloud.height = pointcloud.height;
    valid_pointcloud.fields = pointcloud.fields;
    valid_pointcloud.is_bigendian = pointcloud.is_bigendian;
    valid_pointcloud.point_step = pointcloud.point_step;
    valid_pointcloud.is_dense = pointcloud.is_dense;
    valid_pointcloud.header.frame_id = pointcloud.header.frame_id;

    valid_pointcloud.row_step = valid_pointcloud.data.size();
    valid_pointcloud.width = valid_pointcloud.row_step / valid_pointcloud.point_step;
}


void BoxValidator::Configure(std::vector<float> configuration)
{
    leftOffset = configuration.at(0);
    rightOffset = configuration.at(1);
    topOffset = configuration.at(2);
    bottomOffset = configuration.at(3);
    distance = configuration.at(4);
    back = configuration.at(5);
}