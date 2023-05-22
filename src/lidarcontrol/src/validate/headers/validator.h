#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <sensor_msgs/PointCloud2.h>

class Validator
{
public:
   virtual void Validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &valid_pointcloud) = 0;
   virtual void Configure(std::vector<float> configuration) = 0;
};

#endif