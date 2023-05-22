#ifndef BOUNDING_BOX_VALIDATOR_H
#define BOUDNING_BOX_VALIDATOR_H

#include <sensor_msgs/PointCloud2.h>
#include "../dataPoint.h"

class BoundingBoxValidator
{
public:
void validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &pointcloud_valid);

private:
bool isDataPointValid(DataPoint* point);

};

#endif