#ifndef BOX_VALIDATOR_H
#define BOX_VALIDATOR_H

#include "../../datapoint/dataPoint.h"
#include "validator.h"

class BoxValidator : public Validator
{
public:
    void Validate(const sensor_msgs::PointCloud2 &pointcloud, sensor_msgs::PointCloud2 &valid_pointcloud);

    /// @brief Takes 6 floats to configure a box. with following order:
    /// @param configuration LeftOffset, RightOffset, TopOffset, BottomOffset, Distance, Back
    void Configure(std::vector<float> configuration);
private:
    bool isDataPointValid(DataPoint* point);
};

#endif