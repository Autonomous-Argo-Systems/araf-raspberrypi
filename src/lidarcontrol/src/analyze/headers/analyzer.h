#ifndef ANALYZER_H
#define ANALYZER_H

#include <sensor_msgs/PointCloud2.h>

class Analyzer
{
public:
    /// @brief Analyzes given pointcloud and calculates a risk value based on the points.
    /// @param valid_pointcloud The pointcloud to analyze
    /// @return A risk value based on the analyzer implementation, value between 0 and 100
    virtual float Analyze(sensor_msgs::PointCloud2 &valid_pointcloud) = 0;

    /// @brief Configure the analyzer, meaning specific to implementation
    /// @param configuration Float vector with configuration
    virtual void Configure(std::vector<float> configuration) = 0;
};

#endif