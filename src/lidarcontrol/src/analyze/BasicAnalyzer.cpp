#include "headers/basicAnalyser.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

// multipliers
float linearDistanceWeight = 2.25f;
float linearIntensityWeight = 1.0f;
float linearAmountWeight = 1.0f;

// Exponentials
float exponentialDistanceWeight = 3.5f;
float exponentialIntensityWeight = 1.0f; 
float exponentialAmountWeight = 1.0f;

float lowerRiskThreshold = 0.0f;
float upperRiskThreshold = 2.0f;

/// @brief Calculate distance from the lidar to the given data 
/// @param point The point to calculate distance from
/// @return Distane in meters to centre point
float BasicAnalyzer::CalculateDistance(DataPoint* point)
{
    return sqrt( 
        (point->x * point->x) +
        (point->y * point->y) +
        (point->z * point->z));
}

float BasicAnalyzer::CalculatePointRisk(DataPoint* point)
{
    float distance = CalculateDistance(point);

    // Distance: value in meters from the sensor
    float distanceRisk = pow((1.0f / distance), exponentialDistanceWeight) * linearDistanceWeight;
    
    // Intensity high 100 - low 0;
    float intensityRisk = pow((point->intensity / 100.0f), (1.0f / exponentialIntensityWeight)) * linearDistanceWeight;

    return distanceRisk * intensityRisk;
}

float BasicAnalyzer::Analyze(sensor_msgs::PointCloud2 &valid_pointcloud)
{
    double avg = 0;

    for(int i = 0; i < valid_pointcloud.row_step; i += valid_pointcloud.point_step)
    {
        DataPoint datapoint = fetchDatapoint(valid_pointcloud.data, i);

        float pointRisk = CalculatePointRisk(&datapoint);

        avg += (double)pointRisk / valid_pointcloud.width;
    }

    double calculatedAmountRisk = pow(avg, exponentialAmountWeight) * linearAmountWeight;

    // Calculate percentage from amount risk
    double risk = ((calculatedAmountRisk - lowerRiskThreshold) / upperRiskThreshold) * 100.0f;
    risk = risk >= 100.0f ? 100.0f : risk; // Limit risk to 100

    ROS_INFO("Point risk avg: %f, total risk: %f", avg, risk);

    return risk;
}

void BasicAnalyzer::Configure(std::vector<float> configuration)
{

}
