#ifndef BASIC_ANALYZER_H
#define BASIC_ANALYZER_H

#include "analyzer.h"

class BasicAnalyzer : public Analyzer
{
public:
    float Analyze(sensor_msgs::PointCloud2 &valid_pointcloud);
    void Configure(std::vector<float> configuration);
};

#endif