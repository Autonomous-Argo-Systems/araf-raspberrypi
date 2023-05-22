#include "headers/BoundingBoxValidator.h"

/// @brief The algotithm that determines if the datapoint is usefull for further analyses.
/// For now this returns a bounding box in front of the lidar. Could be reworkt to use different algorithms based on 
/// the situation
/// @param point The point to check
/// @return True if valid point.
bool BoundingBoxValidator::isDataPointValid(DataPoint* point)
{
    // Square boundingbox in m
    const float leftOffset = 1.0f;
    const float rightOffset = 1.0f;
    const float topOffset = 4.0f;
    const float bottomOffset = 1.0f;
    const float distance = 3.0f;
    const float back = 0.0f;

    return point->x <= distance && point->x >= (back * -1.0f) &&
        point->y <= leftOffset && point->y >= (rightOffset * -1.0f) &&
        point->z <= topOffset && point->z >= (bottomOffset * -1.0f);
}

void BoundingBoxValidator::validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &pointcloud_valid)
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
}