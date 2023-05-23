#ifndef BASIC_ANALYZER_H
#define BASIC_ANALYZER_H

#include "../../datapoint/dataPoint.h"
#include "analyzer.h"

class BasicAnalyzer : public Analyzer
{
public:
    /// @brief Analyzes pointcloud using basic analyzer
    /// Analyzes 3 features:
    /// - Distance from the lidar to a point
    /// - Intensity of a point
    /// - Amount of points in de pointcloud
    /// These three features have linear and exponential weights assigned to them
    /// Risk float is calculated as precentage based on upper and lower threshold
    /// @param configuration 
    float Analyze(sensor_msgs::PointCloud2 &valid_pointcloud);
    void Configure(std::vector<float> configuration);
private:
    float CalculateDistance(DataPoint* point);
    float CalculatePointRisk(DataPoint* point);
};

#endif