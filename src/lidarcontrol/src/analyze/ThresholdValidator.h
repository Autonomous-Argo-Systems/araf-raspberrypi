#ifndef THRESHOLD_VALIDATOR_H
#define THRESHOLD_VALIDATOR_H

#include <sensor_msgs/PointCloud2.h>

class ThresholdValidator
{
public:
void validate(sensor_msgs::PointCloud2 &pointcloud);

};

#endif