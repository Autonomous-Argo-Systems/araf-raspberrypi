#ifndef DATA_POINT_CPP
#define DATA_POINT_CPP

#include <vector>
#include <stdint.h>

typedef struct __attribute__((packed))
{
    float x,y,z;
    float intensity;
    uint16_t ring;
    float time;
} DataPoint;

DataPoint fetchDatapoint(const std::vector<uint8_t> &data, uint32_t datapoint_offset);
void pushDatapoint(std::vector<uint8_t> &data, DataPoint &datapoint);

#endif