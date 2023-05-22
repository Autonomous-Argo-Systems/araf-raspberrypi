#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <sensor_msgs/PointCloud2.h>

class Validator
{
public:
   /// @brief Validate given pointcloud based on validator type and configuration. Copies valid points from pointcloud to valid_pointcloud
   /// @param pointcloud Pointcloud with all points
   /// @param valid_pointcloud Modified cloud with only the valid points
   virtual void Validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &valid_pointcloud) = 0;
   
   /// @brief Configuration for the validator
   /// @param configuration float vector to configure validator, meaning of data dependend on validator implementation
   virtual void Configure(std::vector<float> configuration) = 0;
};

#endif